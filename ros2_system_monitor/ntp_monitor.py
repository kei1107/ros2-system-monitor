#!/usr/bin/env python3

#################################################################################
# Copyright 2009, Willow Garage, Inc.
# Copyright 2013 by Ralf Kaestner
# Copyright 2013 by Jerome Maye
# Copyright 2023 by kei1107
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#################################################################################

import re
import threading
from subprocess import PIPE, Popen

import diagnostic_updater
import rclpy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from rclpy.node import Node


class NtpMonitor(Node):
    def __init__(self, hostname, diag_hostname):
        super().__init__("ntp_monitor")
        self._mutex = threading.Lock()

        self._ntp_hostname = self.declare_parameter(
            "reference_host", "ntp.ubuntu.com").get_parameter_value().string_value
        self._offset = self.declare_parameter(
            "offset_tolerance", 500.0).get_parameter_value().double_value
        self._error_offset = self.declare_parameter(
            "error_offset_tolerance", 5000000.0).get_parameter_value().double_value

        # updater
        self.updater = diagnostic_updater.Updater(self)
        self.updater.setHardwareID(hostname)

        # Memory Stat
        self._ntp_stat = DiagnosticStatus()
        self._ntp_stat.level = DiagnosticStatus.OK
        self._ntp_stat.message = "OK"
        self._ntp_stat.values: list[KeyValue] = []
        self.updater.add(
            f"NTP offset from {diag_hostname} to {self._ntp_hostname}", self.update_ntp_stat)

        # Start checking everything
        self._ntp_timer = None
        self.check_ntp()

    def cancel_timers(self):
        """Must have the lock to cancel everything."""
        if self._ntp_timer:
            self._ntp_timer.cancel()

    def check_ntp(self):
        if not rclpy.ok():
            with self._mutex:
                self.cancel_timers()
            return

        st = DiagnosticStatus()
        for host, off in [(self._ntp_hostname, self._offset)]:
            try:
                p = Popen(["ntpdate", "-q", host],
                          stdout=PIPE, stdin=PIPE, stderr=PIPE)
                res = p.wait()
                (o, e) = p.communicate()
                o = o.decode()
                e = e.decode()
            except OSError as e:
                if e.errno == 4:
                    break  # ctrl-c interrupt
                else:
                    raise
            if (res == 0):
                if "offset" in o:
                    measured_offset = float(
                        re.search("offset (.*),", o).group(1))*1000000
                else:
                    # Newer ntpdate versions output the following
                    #   YYYY-MM-DD HH:MM:SS.SSSSSS (UTC Offset) OFFSET +/ DELAY HOST IP STRATUM LEAP
                    # Instead of using regex to get the offset, we can
                    # split on spaces and use the fourth value.
                    measured_offset = float(o.split()[3])*1000000
                st.level = DiagnosticStatus.OK
                st.message = "OK"
                st.values = [KeyValue(key="Offset (us)", value=str(measured_offset)),
                             KeyValue(key="Offset tolerance (us)",
                                      value=str(off)),
                             KeyValue(key="Offset tolerance (us) for Error",
                                      value=str(self._error_offset))]

                if (abs(measured_offset) > off):
                    st.level = DiagnosticStatus.WARN
                    st.message = "NTP Offset Too High"
                if (abs(measured_offset) > self._error_offset):
                    st.level = DiagnosticStatus.ERROR
                    st.message = "NTP Offset Too High"

            else:
                st.level = DiagnosticStatus.ERROR
                st.message = f"Error Running ntpdate. Returned {res}"
                st.values = [KeyValue(key="Offset (us)", value="N/A"),
                             KeyValue(key="Offset tolerance (us)",
                                      value=str(off)),
                             KeyValue(key="Offset tolerance (us) for Error",
                                      value=str(self._error_offset)),
                             KeyValue(key="Output", value=o),
                             KeyValue(key="Errors", value=e)]

        # Update status
        with self._mutex:
            self._ntp_stat.level = st.level
            self._ntp_stat.values = st.values
            self._ntp_stat.message = st.message
            if rclpy.ok():
                self._ntp_timer = threading.Timer(1.0, self.check_ntp)
                self._ntp_timer.start()
            else:
                self.cancel_timers()

    def update_ntp_stat(self, stat: diagnostic_updater.DiagnosticStatusWrapper):
        with self._mutex:
            stat.summary(self._ntp_stat.level, self._ntp_stat.message)
            value: KeyValue
            for value in self._ntp_stat.values:
                stat.add(value.key, value.value)
        return stat

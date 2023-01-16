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
import subprocess
import threading
import traceback

import diagnostic_updater
import rclpy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.time import Time
from ros2_system_monitor.utilities import (NetDict, StatDict,
                                           update_status_stale)


def get_sys_net_stat(iface, sys):
    cmd = f"cat /sys/class/net/{iface}/statistics/{sys}"
    p = subprocess.Popen(cmd,
                         stdout=subprocess.PIPE,
                         stderr=subprocess.PIPE, shell=True)
    stdout, _ = p.communicate()
    return (p.returncode, stdout.decode().strip())


def get_sys_net(iface, sys):
    cmd = f"cat /sys/class/net/{iface}/{sys}"
    p = subprocess.Popen(cmd,
                         stdout=subprocess.PIPE,
                         stderr=subprocess.PIPE, shell=True)
    stdout, _ = p.communicate()
    return (p.returncode, stdout.decode().strip())


class NetMonitor(Node):
    def __init__(self, hostname, diag_hostname):
        super().__init__("net_monitor")
        self._mutex = threading.Lock()

        self._net_level_warn = self.declare_parameter(
            "net_level_warn", 0.95).get_parameter_value().double_value
        self._net_capacity = self.declare_parameter(
            "net_capacity", 128.0).get_parameter_value().double_value

        # updater
        self.updater = diagnostic_updater.Updater(self)
        self.updater.setHardwareID(hostname)

        # Network Stat
        self._usage_stat = DiagnosticStatus()
        self._usage_stat.level = DiagnosticStatus.WARN
        self._usage_stat.message = 'No Data'
        self._usage_stat.values = [KeyValue(key='Update Status', value='No Data'),
                                   KeyValue(key='Time Since Last Update', value='N/A')]
        self.updater.add(
            f"Network Usage ({diag_hostname})", self.update_usage_status)

        self._last_usage_time = Time(seconds=0.0)

        # Start checking everything
        self._usage_timer = None
        self.check_usage()

    def cancel_timers(self):
        """Must have the lock to cancel everything."""
        if self._usage_timer:
            self._usage_timer.cancel()

    def check_network(self):
        values: list[KeyValue] = []

        try:
            p = subprocess.Popen('ifstat -q -S 1 1',
                                 stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE, shell=True)
            stdout, _ = p.communicate()
            stdout = stdout.decode()
            retcode = p.returncode
            if retcode != 0:
                values.append(KeyValue(key="\"ifstat -q -S 1 1\" Call Error",
                                       value=str(retcode)))
                return DiagnosticStatus.ERROR, str("Call Error"), values
            rows = stdout.split('\n')
            data = rows[0].split()
            ifaces = []
            for i in range(0, len(data)):
                ifaces.append(data[i])
            data = rows[2].split()
            kb_in = []
            kb_out = []
            for i in range(0, len(data), 2):
                kb_in.append(data[i])
                kb_out.append(data[i + 1])
            level = DiagnosticStatus.OK
            for i in range(0, len(ifaces)):
                values.append(KeyValue(key='Interface Name',
                                       value=ifaces[i]))
                (retcode, cmd_out) = get_sys_net(ifaces[i], 'operstate')
                if retcode == 0:
                    values.append(KeyValue(key='State', value=cmd_out))
                    ifacematch = re.match('eth[0-9]+', ifaces[i])
                    if ifacematch and (cmd_out == 'down' or cmd_out == 'dormant'):
                        level = DiagnosticStatus.ERROR
                values.append(KeyValue(key='Input Traffic',
                                       value=str(float(kb_in[i]) / 1024) + " (MB/s)"))
                values.append(KeyValue(key='Output Traffic',
                                       value=str(float(kb_out[i]) / 1024) + " (MB/s)"))
                net_usage_in = float(kb_in[i]) / 1024 / self._net_capacity
                net_usage_out = float(kb_out[i]) / 1024 / self._net_capacity
                if net_usage_in > self._net_level_warn or net_usage_out > self._net_level_warn:
                    level = DiagnosticStatus.WARN
                (retcode, cmd_out) = get_sys_net(ifaces[i], 'mtu')
                if retcode == 0:
                    values.append(KeyValue(key='MTU', value=cmd_out))
                (retcode, cmd_out) = get_sys_net_stat(ifaces[i], 'rx_bytes')
                if retcode == 0:
                    values.append(KeyValue(key='Total received MB',
                                           value=str(float(cmd_out) / 1024 / 1024)))
                (retcode, cmd_out) = get_sys_net_stat(ifaces[i], 'tx_bytes')
                if retcode == 0:
                    values.append(KeyValue(key='Total transmitted MB',
                                           value=str(float(cmd_out) / 1024 / 1024)))
                (retcode, cmd_out) = get_sys_net_stat(ifaces[i], 'collisions')
                if retcode == 0:
                    values.append(KeyValue(key='Collisions', value=cmd_out))
                (retcode, cmd_out) = get_sys_net_stat(ifaces[i], 'rx_errors')
                if retcode == 0:
                    values.append(KeyValue(key='Rx Errors', value=cmd_out))
                (retcode, cmd_out) = get_sys_net_stat(ifaces[i], 'tx_errors')
                if retcode == 0:
                    values.append(KeyValue(key='Tx Errors', value=cmd_out))
        except Exception as e:
            self.get_logger().error(traceback.format_exc())
            msg = 'Network Usage Check Error'
            values.append(KeyValue(key=msg, value=str(e)))
            level = DiagnosticStatus.ERROR

        return level, NetDict[level], values

    def check_usage(self):
        if not rclpy.ok():
            with self._mutex:
                self.cancel_timers()
            return

        diag_level = DiagnosticStatus.OK
        diag_vals = [KeyValue(key='Update Status', value='OK'),
                     KeyValue(key='Time Since Last Update', value=str(Time(seconds=0.0)))]
        diag_msgs: list[str] = []

        # Check network
        net_level, net_msg, net_vals = self.check_network()

        diag_vals.extend(net_vals)
        if net_level != DiagnosticStatus.OK:
            diag_msgs.append(net_msg)
        diag_level = max(diag_level, net_level)
        if diag_msgs and diag_level != DiagnosticStatus.OK:
            usage_msg = ', '.join(set(diag_msgs))
        else:
            usage_msg = StatDict[diag_level]

        # Update status
        with self._mutex:
            self._last_usage_time = self.get_clock().now()
            self._usage_stat.level = diag_level
            self._usage_stat.values = diag_vals
            self._usage_stat.message = usage_msg
            if rclpy.ok():
                self._usage_timer = threading.Timer(5.0, self.check_usage)
                self._usage_timer.start()
            else:
                self.cancel_timers()

    def update_usage_status(self, stat: diagnostic_updater.DiagnosticStatusWrapper):
        with self._mutex:
            update_status_stale(stat=self._usage_stat,
                                clock=self.get_clock(),
                                last_update_time=self._last_usage_time)
            stat.summary(self._usage_stat.level, self._usage_stat.message)
            value: KeyValue
            for value in self._usage_stat.values:
                stat.add(value.key, value.value)
        return stat

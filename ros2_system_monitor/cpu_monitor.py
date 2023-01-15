#!/usr/bin/env python3

#################################################################################
# Copyright (C) 2009, Willow Garage, Inc.                                       #
# Copyright (C) 2013 by Ralf Kaestner                                           #
# Copyright (C) 2013 by Jerome Maye                                             #
#                                                                               #
# All rights reserved.                                                          #
#                                                                               #
# Redistribution and use in source and binary forms, with or without            #
# modification, are permitted provided that the following conditions are met:   #
#                                                                               #
#    * Redistributions of source code must retain the above copyright           #
#      notice, this list of conditions and the following disclaimer.            #
#                                                                               #
#    * Redistributions in binary form must reproduce the above copyright        #
#      notice, this list of conditions and the following disclaimer in the      #
#      documentation and/or other materials provided with the distribution.     #
#                                                                               #
#    * Neither the name of the copyright holder nor the names of its            #
#      contributors may be used to endorse or promote products derived from     #
#      this software without specific prior written permission.                 #
#                                                                               #
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"   #
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE     #
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE    #
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE     #
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR           #
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF          #
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS      #
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN       #
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)       #
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE    #
# POSSIBILITY OF SUCH DAMAGE.                                                   #
#################################################################################

import multiprocessing
import socket
import subprocess
import sys
import threading
import traceback

import diagnostic_updater
import rclpy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.time import Time
from utilities import (MpstatLoadDict, StatDict, UptimeLoadDict,
                       update_status_stale)


class CPUMonitor(Node):
    def __init__(self, hostname, diag_hostname):
        super().__init__("cpu_monitor")
        self._mutex = threading.Lock()

        self._check_core_temps = self.declare_parameter(
            "check_core_temps", True).get_parameter_value().bool_value
        self._cpu_load_warn = self.declare_parameter(
            "cpu_load_warn", 0.9).get_parameter_value().double_value
        self._cpu_load_error = self.declare_parameter(
            "cpu_load_error", 1.1).get_parameter_value().double_value
        self._cpu_load1_warn = self.declare_parameter(
            "cpu_load1_warn", 0.9).get_parameter_value().double_value
        self._cpu_load5_warn = self.declare_parameter(
            "cpu_load5_warn", 0.8).get_parameter_value().double_value
        self._cpu_temp_warn = self.declare_parameter(
            "cpu_temp_warn", 85.0).get_parameter_value().double_value
        self._cpu_temp_error = self.declare_parameter(
            "cpu_temp_error", 90.0).get_parameter_value().double_value

        self._num_cores = multiprocessing.cpu_count()

        # Get temp_input files
        self._temp_vals = self.get_core_temp_names()

        # updater
        self.updater = diagnostic_updater.Updater(self)
        self.updater.setHardwareID(hostname)

        # CPU stats
        self._temp_stat = DiagnosticStatus()
        self._temp_stat.level = DiagnosticStatus.WARN
        self._temp_stat.message = "No Data"
        self._temp_stat.values = [KeyValue(key='Update Status', value='No Data'),
                                  KeyValue(key='Time Since Last Update', value='N/A')]
        self.updater.add(
            f"CPU Temperature ({diag_hostname})", self.update_temp_status)

        self._usage_stat = DiagnosticStatus()
        self._usage_stat.level = DiagnosticStatus.WARN
        self._usage_stat.message = 'No Data'
        self._usage_stat.values = [KeyValue(key='Update Status', value='No Data'),
                                   KeyValue(key='Time Since Last Update', value='N/A')]
        self.updater.add(
            f"CPU Usage ({diag_hostname})", self.update_usage_status)

        self._last_temp_time = Time(seconds=0.0)
        self._last_usage_time = Time(seconds=0.0)

        self._usage_old = 0.0
        self._has_warned_mpstat = False
        self._has_error_core_count = False

        # Start checking everything
        self._temps_timer = None
        self._usage_timer = None
        self.check_temps()
        self.check_usage()

    def cancel_timers(self):
        """Must have the lock to cancel everything."""
        if self._temps_timer:
            self._temps_timer.cancel()

        if self._usage_timer:
            self._usage_timer.cancel()

    def check_core_temps(self, sys_temp_strings):
        """Check CPU core temps.

        Use 'find /sys -name temp1_input' to find cores
        Read from every core, divide by 1000
        """
        diag_vals: list[KeyValue] = []
        diag_msgs: list[str] = []
        diag_level = DiagnosticStatus.OK

        for index, temp_str in enumerate(sys_temp_strings):
            if len(temp_str) < 5:
                continue

            cmd = f"cat {temp_str}"
            p = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE, shell=True)
            stdout, stderr = p.communicate()
            stdout = stdout.decode()
            stderr = stderr.decode()
            retcode = p.returncode

            if retcode != 0:
                diag_level = DiagnosticStatus.ERROR
                diag_msgs = ['Core Temperature Error']
                diag_vals = [KeyValue(key='Core Temperature Error', value=stderr),
                             KeyValue(key='Output', value=stdout)]
                return diag_vals, diag_msgs, diag_level

            tmp = stdout.strip()
            if tmp.isnumeric():
                temp = float(tmp) / 1000
                diag_vals.append(
                    KeyValue(key=f"Core {index} Temperature", value=str(temp)+"DegC"))

                if temp >= self._cpu_temp_warn:
                    diag_level = max(diag_level, DiagnosticStatus.WARN)
                    diag_msgs.append('Warm')
                elif temp >= self._cpu_temp_error:
                    diag_level = max(diag_level, DiagnosticStatus.ERROR)
                    diag_msgs.append('Hot')
            else:
                # Error if not numeric value
                diag_level = max(diag_level, DiagnosticStatus.ERROR)
                diag_vals.append(
                    KeyValue(key=f"Core {index} Temperature", value=tmp))

        return diag_vals, diag_msgs, diag_level

    def check_clock_speed(self):
        """Checks clock speed from reading from CPU info."""
        vals: list[KeyValue] = []
        msgs: list[str] = []
        lvl = DiagnosticStatus.OK

        try:
            p = subprocess.Popen('cat /proc/cpuinfo | grep MHz',
                                 stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE, shell=True)
            stdout, stderr = p.communicate()
            stdout = stdout.decode()
            stderr = stderr.decode()
            retcode = p.returncode

            if retcode != 0:
                lvl = DiagnosticStatus.ERROR
                msgs = ['Clock speed error']
                vals = [KeyValue(key='Clock speed error', value=stderr),
                        KeyValue(key='Output', value=stdout)]

                return (vals, msgs, lvl)

            for index, ln in enumerate(stdout.split('\n')):
                words = ln.split(':')
                if len(words) < 2:
                    continue

                # Conversion to float doesn't work with decimal
                speed = words[1].strip().split('.')[0]
                vals.append(
                    KeyValue(key=f"Core {index} Clock Speed", value=speed+"MHz"))

        except Exception:
            self.get_logger().error(traceback.format_exc())
            lvl = DiagnosticStatus.ERROR
            msgs.append('Exception')
            vals.append(KeyValue(key='Exception',
                        value=traceback.format_exc()))

        return vals, msgs, lvl

    def check_uptime(self):
        """Uses 'uptime' to see load average."""
        level = DiagnosticStatus.OK
        vals: list[KeyValue] = []

        try:
            p = subprocess.Popen('uptime', stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE, shell=True)
            stdout, stderr = p.communicate()
            stdout = stdout.decode()
            stderr = stderr.decode()
            retcode = p.returncode

            if retcode != 0:
                vals.append(KeyValue(key='uptime Failed', value=stderr))
                return DiagnosticStatus.ERROR, vals

            upvals = stdout.split()
            load1 = float(upvals[-3].rstrip(','))/self._num_cores
            load5 = float(upvals[-2].rstrip(','))/self._num_cores
            load15 = float(upvals[-1])/self._num_cores

            # Give warning if we go over load limit
            if load1 > self._cpu_load1_warn or load5 > self._cpu_load5_warn:
                level = DiagnosticStatus.WARN

            vals.append(KeyValue(key='Load Average Status',
                        value=UptimeLoadDict[level]))
            vals.append(KeyValue(key='Load Average (1min)',
                        value=str(load1*1e2)+"%"))
            vals.append(KeyValue(key='Load Average (5min)',
                        value=str(load5*1e2)+"%"))
            vals.append(KeyValue(key='Load Average (15min)',
                        value=str(load15*1e2)+"%"))

        except Exception:
            self.get_logger().error(traceback.format_exc())
            level = DiagnosticStatus.ERROR
            vals.append(KeyValue(key='Load Average Status',
                        value=traceback.format_exc()))

        return level, UptimeLoadDict[level], vals

    def check_mpstat(self):
        """Use mpstat to find CPU usage."""
        vals: list[KeyValue] = []
        mp_level = DiagnosticStatus.OK

        try:
            p = subprocess.Popen('mpstat -P ALL 1 1',
                                 stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE, shell=True)
            stdout, _ = p.communicate()
            stdout = stdout.decode()
            retcode = p.returncode

            if retcode != 0:
                if not self._has_warned_mpstat:
                    self.get_logger().error(
                        f"mpstat failed to run for cpu_monitor. Return code {retcode}.")
                    self._has_warned_mpstat = True

                mp_level = DiagnosticStatus.ERROR
                vals.append(
                    KeyValue(key='\"mpstat\" Call Error', value=str(retcode)))
                return mp_level, 'Unable to Check CPU Usage', vals

            # Check which column '%idle' is, #4539
            # mpstat output changed between 8.06 and 8.1
            rows = stdout.split('\n')
            col_names = rows[2].split()
            idle_col = -1 if (len(col_names) >
                              2 and col_names[-1] == '%idle') else -2

            num_cores = 0
            cores_loaded = 0
            for index, row in enumerate(stdout.split('\n')):
                if index < 3:
                    continue

                # Skip row containing 'all' data
                if row.find('all') > -1:
                    continue

                lst = row.split()
                if len(lst) < 8:
                    continue

                # Ignore 'Average: ...' data
                if lst[0].startswith('Average'):
                    continue

                cpu_name = str(num_cores)
                idle = lst[idle_col]
                user = lst[3]
                nice = lst[4]
                system = lst[5]

                core_level = DiagnosticStatus.OK
                usage = (float(user)+float(nice))*1e-2
                if usage > 10.0:  # wrong reading, use old reading instead
                    self.get_logger().warning(
                        f"Read CPU usage of {usage} percent. "
                        f"Reverting to previous reading of {self._usage_old} percent")
                    usage = self._usage_old
                self._usage_old = usage

                if usage >= self._cpu_load_warn:
                    cores_loaded += 1
                    core_level = DiagnosticStatus.WARN
                elif usage >= self._cpu_load_error:
                    core_level = DiagnosticStatus.ERROR

                vals.append(
                    KeyValue(key=f"Core {cpu_name} Status", value=MpstatLoadDict[core_level]))
                vals.append(
                    KeyValue(key=f"Core {cpu_name} User", value=user+"%"))
                vals.append(
                    KeyValue(key=f"Core {cpu_name} Nice", value=nice+"%"))
                vals.append(
                    KeyValue(key=f"Core {cpu_name} System", value=system+"%"))
                vals.append(
                    KeyValue(key=f"Core {cpu_name} Idle", value=idle+"%"))

                num_cores += 1

            # Warn for high load only if we have <= 2 cores that aren't loaded
            if num_cores - cores_loaded <= 2 and num_cores > 2:
                mp_level = DiagnosticStatus.WARN

            if not self._num_cores:
                self._num_cores = num_cores

            # Check the number of cores if self._num_cores > 0, #4850
            if self._num_cores != num_cores:
                mp_level = DiagnosticStatus.ERROR
                if not self._has_error_core_count:
                    self.get_logger().error(
                        f"Error checking number of cores. Expected {self._num_cores}"
                        f"got {num_cores}. Computer may have not booted properly.")
                    self._has_error_core_count = True
                return DiagnosticStatus.ERROR, 'Incorrect number of CPU cores', vals
        except Exception as e:
            print(traceback.print_exc())
            mp_level = DiagnosticStatus.ERROR
            vals.append(KeyValue(key='mpstat Exception', value=str(e)))

        return mp_level, MpstatLoadDict[mp_level], vals

    def get_core_temp_names(self):
        """
        Returns names for core temperature files
        Returns list of names as each name can be read like file
        """
        temp_vals: list[str] = []
        try:
            p = subprocess.Popen('find /sys/devices -name temp1_input',
                                 stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE, shell=True)
            stdout, stderr = p.communicate()
            stdout = stdout.decode()
            stderr = stderr.decode()
            retcode = p.returncode

            if retcode != 0:
                self.get_logger().error(
                    f"Error find core temp locations: {stderr}")
                return []

            for ln in stdout.split('\n'):
                temp_vals.append(ln.strip())

            return temp_vals
        except Exception:
            self.get_logger().error(
                f"Exception finding temp vals: {traceback.format_exc()}")
            return []

    def check_temps(self):
        if not rclpy.ok():
            with self._mutex:
                self.cancel_timers()
            return

        diag_vals = [KeyValue(key='Update Status', value='OK'),
                     KeyValue(key='Time Since Last Update', value=str(Time(seconds=0.0)))]
        diag_msgs: list[str] = []
        diag_level = DiagnosticStatus.OK

        if self._check_core_temps:
            core_vals, core_msgs, core_level = self.check_core_temps(
                self._temp_vals)
            diag_vals.extend(core_vals)
            diag_msgs.extend(core_msgs)
            diag_level = max(diag_level, core_level)

        diag_log = set(diag_msgs)
        if len(diag_log) > 0:
            message = ', '.join(diag_log)
        else:
            message = StatDict[diag_level]

        # update status
        with self._mutex:
            self._last_temp_time = self.get_clock().now()
            self._temp_stat.level = diag_level
            self._temp_stat.message = message
            self._temp_stat.values = diag_vals
            if rclpy.ok():
                self._temps_timer = threading.Timer(5.0, self.check_temps)
                self._temps_timer.start()
            else:
                self.cancel_timers()

    def check_usage(self):
        if not rclpy.ok():
            with self._mutex:
                self.cancel_timers()
            return

        diag_vals = [KeyValue(key='Update Status', value='OK'),
                     KeyValue(key='Time Since Last Update', value=str(Time(seconds=0.0)))]
        diag_msgs: list[str] = []
        diag_level = DiagnosticStatus.OK

        # Check clock speed
        clock_vals, clock_msgs, clock_level = self.check_clock_speed()
        diag_vals.extend(clock_vals)
        diag_msgs.extend(clock_msgs)
        diag_level = max(diag_level, clock_level)

        # Check mpstat
        mp_level, mp_msg, mp_vals = self.check_mpstat()
        diag_vals.extend(mp_vals)
        if mp_level != DiagnosticStatus.OK:
            diag_msgs.append(mp_msg)
        diag_level = max(diag_level, mp_level)

        # Check uptime
        uptime_level, up_msg, up_vals = self.check_uptime()
        diag_vals.extend(up_vals)
        if uptime_level != DiagnosticStatus.OK:
            diag_msgs.append(up_msg)
        diag_level = max(diag_level, uptime_level)

        if diag_msgs and diag_level != DiagnosticStatus.OK:
            usage_msg = ', '.join(set(diag_msgs))
        else:
            usage_msg = StatDict[diag_level]

        # update status
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

    def update_temp_status(self, stat: diagnostic_updater.DiagnosticStatusWrapper):
        with self._mutex:
            update_status_stale(stat=self._temp_stat,
                                clock=self.get_clock(),
                                last_update_time=self._last_temp_time)
            stat.summary(self._temp_stat.level, self._temp_stat.message)
            value: KeyValue
            for value in self._temp_stat.values:
                stat.add(value.key, value.value)
        return stat

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


if __name__ == '__main__':
    rclpy.init(args=sys.argv)

    hostname = socket.gethostname()
    hostname = hostname.replace('-', '_')

    import optparse
    parser = optparse.OptionParser(
        usage="usage: cpu_monitor.py [--diag-hostname=cX]")
    parser.add_option("--diag-hostname", dest="diag_hostname",
                      help="Computer name in diagnostics output (ex: 'c1')",
                      metavar="DIAG_HOSTNAME",
                      action="store", default=hostname)
    from rclpy.utilities import remove_ros_args
    options, args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    try:
        cpu_node = CPUMonitor(hostname, options.diag_hostname)
        rclpy.spin(cpu_node)
    except KeyboardInterrupt:
        pass
    except Exception:
        from rclpy.logging import get_logger
        get_logger("cpu_monitor_node").error(traceback.format_exc())

    cpu_node.cancel_timers()
    sys.exit(0)

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

import subprocess
import threading
import traceback

import diagnostic_updater
import rclpy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.time import Time
from ros2_system_monitor.utilities import (MemDict, StatDict,
                                           update_status_stale)


class MemMonitor(Node):
    def __init__(self, hostname, diag_hostname):
        super().__init__("mem_monitor")
        self._mutex = threading.Lock()

        self._mem_level_warn = self.declare_parameter(
            "mem_level_warn", 0.95).get_parameter_value().double_value
        self._mem_level_error = self.declare_parameter(
            "mem_level_error", 0.99).get_parameter_value().double_value

        # updater
        self.updater = diagnostic_updater.Updater(self)
        self.updater.setHardwareID(hostname)

        # Memory Stat
        self._usage_stat = DiagnosticStatus()
        self._usage_stat.level = DiagnosticStatus.WARN
        self._usage_stat.message = 'No Data'
        self._usage_stat.values = [KeyValue(key='Update Status', value='No Data'),
                                   KeyValue(key='Time Since Last Update', value='N/A')]
        self.updater.add(
            f"Memory Usage ({diag_hostname})", self.update_usage_status)

        self._last_usage_time = Time(seconds=0.0)

        # Start checking everything
        self._usage_timer = None
        self.check_usage()

    def cancel_timers(self):
        """Must have the lock to cancel everything."""
        if self._usage_timer:
            self._usage_timer.cancel()

    def check_memory(self):
        values: list[KeyValue] = []
        level = DiagnosticStatus.OK
        msg = ""

        try:
            p = subprocess.Popen('free -tm',
                                 stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE, shell=True)
            stdout, _ = p.communicate()
            stdout = stdout.decode()
            retcode = p.returncode

            if retcode != 0:
                values.append(
                    KeyValue(key="\"free -tm\" Call Error", value=str(retcode)))
                return DiagnosticStatus.ERROR, values

            rows = stdout.split('\n')
            data = rows[1].split()
            total_mem_physical = data[1]
            used_mem_physical = data[2]
            free_mem_physical = data[3]
            data = rows[2].split()
            total_mem_swap = data[1]
            used_mem_swap = data[2]
            free_mem_swap = data[3]
            data = rows[3].split()
            total_mem = data[1]
            used_mem = data[2]
            free_mem = data[3]

            level = DiagnosticStatus.OK
            mem_usage = float(used_mem_physical)/float(total_mem_physical)
            if (mem_usage < self._mem_level_warn):
                level = DiagnosticStatus.OK
            elif (mem_usage < self._mem_level_error):
                level = DiagnosticStatus.WARN
            else:
                level = DiagnosticStatus.ERROR

            values.append(KeyValue(key='Memory Status', value=MemDict[level]))
            values.append(KeyValue(key='Total Memory (Physical)',
                          value=total_mem_physical+"M"))
            values.append(KeyValue(key='Used Memory (Physical)',
                          value=used_mem_physical+"M"))
            values.append(KeyValue(key='Free Memory (Physical)',
                          value=free_mem_physical+"M"))
            values.append(KeyValue(key='Total Memory (Swap)',
                          value=total_mem_swap+"M"))
            values.append(KeyValue(key='Used Memory (Swap)',
                          value=used_mem_swap+"M"))
            values.append(KeyValue(key='Free Memory (Swap)',
                          value=free_mem_swap+"M"))
            values.append(KeyValue(key='Total Memory', value=total_mem+"M"))
            values.append(KeyValue(key='Used Memory', value=used_mem+"M"))
            values.append(KeyValue(key='Free Memory', value=free_mem+"M"))

            msg = MemDict[level]
        except Exception as e:
            self.get_logger().error(traceback.format_exc())
            msg = 'Memory Usage Check Error'
            values.append(KeyValue(key=msg, value=str(e)))
            level = DiagnosticStatus.ERROR

        return level, MemDict[level], values

    def check_usage(self):
        if not rclpy.ok():
            with self._mutex:
                self.cancel_timers()
            return

        diag_level = DiagnosticStatus.OK
        diag_vals = [KeyValue(key='Update Status', value='OK'),
                     KeyValue(key='Time Since Last Update', value=str(Time(seconds=0.0)))]
        diag_msgs: list[str] = []

        # Check memory
        mem_level, mem_msg, mem_vals = self.check_memory()
        diag_vals.extend(mem_vals)
        if mem_level != DiagnosticStatus.OK:
            diag_msgs.append(mem_msg)
        diag_level = max(diag_level, mem_level)

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

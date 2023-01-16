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

import json
import subprocess
import threading
import traceback
from typing import Tuple

import diagnostic_updater
import rclpy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.time import Time
from ros2_system_monitor.utilities import (StatDict, TempDict, UsageDict,
                                           update_status_stale)


class HDDMonitor(Node):
    def __init__(self, hostname, diag_hostname):
        super().__init__("hdd_monitor")
        self._mutex = threading.Lock()

        self._no_temp = self.declare_parameter(
            "no_hw_temp", False).get_parameter_value().bool_value
        self._no_temp_warn = self.declare_parameter(
            "no_hw_temp_warn", False).get_parameter_value().bool_value
        self._hdd_level_warn = self.declare_parameter(
            "hdd_level_warn", 0.95).get_parameter_value().double_value
        self._hdd_level_error = self.declare_parameter(
            "hdd_level_error", 0.99).get_parameter_value().double_value
        self._hdd_temp_warn = self.declare_parameter(
            "hdd_temp_warn", 55.0).get_parameter_value().double_value
        self._hdd_temp_error = self.declare_parameter(
            "hdd_temp_error", 70.0).get_parameter_value().double_value

        # updater
        self.updater = diagnostic_updater.Updater(self)
        self.updater.setHardwareID(hostname)

        # Temp Stat
        if not self._no_temp:
            self._temp_stat = DiagnosticStatus()
            self._temp_stat.level = DiagnosticStatus.WARN
            self._temp_stat.message = "No Data"
            self._temp_stat.values = [KeyValue(key="Update Status", value="No Data"),
                                      KeyValue(key="Time Since Last Update", value="N/A")]
            self.updater.add(
                f"HW Temperature ({diag_hostname})", self.update_temp_status)

        # Usage Stat
        self._usage_stat = DiagnosticStatus()
        self._usage_stat.level = DiagnosticStatus.WARN
        self._usage_stat.name = "HDD Usage (%s)" % diag_hostname
        self._usage_stat.values = [KeyValue(key="Update Status", value="No Data"),
                                   KeyValue(key="Time Since Last Update", value="N/A")]
        self.updater.add(
            f"HDD Usage ({diag_hostname})", self.update_usage_status)

        self._last_temp_time = Time(seconds=0.0)
        self._last_usage_time = Time(seconds=0.0)

        self._has_warned_sensors = False

        self._temp_timer = None
        self._usage_timer = None
        if not self._no_temp:
            self.check_temps()
        self.check_disk_usage()

    def cancel_timers(self):
        """Must have the lock to cancel everything."""
        if self._temp_timer:
            self._temp_timer.cancel()

        if self._usage_timer:
            self._usage_timer.cancel()

    def get_temp_input(self, obj: dict) -> Tuple[list[str], list[float]]:
        ret_paths = []
        ret_temps = []
        for k, v in obj.items():
            if isinstance(v, float):
                if k.endswith("input"):
                    ret_paths.append(k)
                    ret_temps.append(v)
            elif isinstance(v, dict):
                paths, temps = self.get_temp_input(v)
                paths = [k + "/" + path for path in paths]

                ret_paths.extend(paths)
                ret_temps.extend(temps)

        return (ret_paths, ret_temps)

    def check_temps(self):
        if not rclpy.ok():
            with self._mutex:
                self.cancel_timers()
            return

        diag_strs = [KeyValue(key="Update Status", value="OK"),
                     KeyValue(key="Time Since Last Update", value=str(Time(seconds=0.0)))]
        diag_msgs: list[str] = []
        diag_level = DiagnosticStatus.OK

        sensors_ok = True

        try:
            p = subprocess.Popen('sensors -j',
                                 stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE, shell=True)
            stdout, _ = p.communicate()
            stdout = stdout.decode()
            retcode = p.returncode
            if retcode != 0:
                if not self._has_warned_sensors:
                    self.get_logger().error(
                        f"'sensors' failed to run for hdd_monitor. Return code {retcode}.")
                    self._has_warned_sensors = True

                diag_level = DiagnosticStatus.ERROR
                diag_strs.append(
                    KeyValue(key='\"sensors -j\" Call Error', value=str(retcode)))
                diag_msgs.append("Unable to Check HW Temp")
                sensors_ok = False
            else:
                sensors_data: dict = json.loads(stdout)
                hw_paths, temps = self.get_temp_input(sensors_data)
                assert len(hw_paths) == len(temps)
                for index in range(0, len(hw_paths)):
                    hw_path = hw_paths[index]
                    temp = temps[index]

                    temp_level = DiagnosticStatus.OK
                    if float(temp) >= self._hdd_temp_warn:
                        temp_level = DiagnosticStatus.WARN
                    if float(temp) >= self._hdd_temp_error:
                        temp_level = DiagnosticStatus.ERROR

                    diag_level = max(diag_level, temp_level)
                    diag_strs.append(
                        KeyValue(key=f"{hw_path} Temperature Status", value=TempDict[temp_level]))
                    diag_strs.append(
                        KeyValue(key=f"{hw_path} Temperature", value=str(temp)+"DegC"))

        except Exception as e:
            self.get_logger().error(traceback.print_exc())
            diag_level = DiagnosticStatus.ERROR
            diag_strs.append(KeyValue(key="Sensors Exception", value=str(e)))
            sensors_ok = False

        diag_log = set(diag_msgs)
        if len(diag_log) > 0:
            message = ', '.join(diag_log)
        else:
            message = TempDict[diag_level]

        if self._no_temp_warn and sensors_ok:
            diag_level = DiagnosticStatus.OK

        with self._mutex:
            self._last_temp_time = self.get_clock().now()
            self._temp_stat.values = diag_strs
            self._temp_stat.message = message
            self._temp_stat.level = diag_level
            if rclpy.ok():
                self._temp_timer = threading.Timer(10.0, self.check_temps)
                self._temp_timer.start()
            else:
                self.cancel_timers()

    def check_disk_usage(self):
        if not rclpy.ok():
            with self._mutex:
                self.cancel_timers()
            return

        diag_vals = [KeyValue(key="Update Status", value="OK"),
                     KeyValue(key="Time Since Last Update", value=str(Time(seconds=0.0)))]
        diag_level = DiagnosticStatus.OK
        diag_message = "OK"

        try:
            p = subprocess.Popen(["df", "-Pht", "ext4"],
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            stdout, _ = p.communicate()
            stdout = stdout.decode()
            retcode = p.returncode

            if (retcode == 0 or retcode == 1):
                diag_vals.append(
                    KeyValue(key="Disk Space Reading", value="OK"))
                rows = stdout.split("\n")
                del rows[0]
                row_count = 0

                for row in rows:
                    if len(row.split()) < 2:
                        continue
                    if row.split()[0] == "none":
                        continue

                    row_count += 1
                    g_available = row.split()[-3]
                    g_use = row.split()[-2]
                    name = row.split()[0]
                    size = row.split()[1]
                    mount_pt = row.split()[-1]

                    hdd_usage = float(g_use.replace("%", ""))*1e-2
                    if (hdd_usage < self._hdd_level_warn):
                        level = DiagnosticStatus.OK
                    elif (hdd_usage < self._hdd_level_error):
                        level = DiagnosticStatus.WARN
                    else:
                        level = DiagnosticStatus.ERROR

                    diag_vals.append(
                        KeyValue(key=f"Disk {row_count} Name", value=name))
                    diag_vals.append(
                        KeyValue(key=f"Disk {row_count} Size", value=size))
                    diag_vals.append(
                        KeyValue(key=f"Disk {row_count} Available", value=g_available))
                    diag_vals.append(
                        KeyValue(key=f"Disk {row_count} Use", value=g_use))
                    diag_vals.append(
                        KeyValue(key=f"Disk {row_count} Status", value=StatDict[level]))
                    diag_vals.append(
                        KeyValue(key=f"Disk {row_count} Mount Point", value=mount_pt))

                    diag_level = max(diag_level, level)
                    diag_message = UsageDict[diag_level]

            else:
                diag_vals.append(
                    KeyValue(key="Disk Space Reading", value="Failed"))
                diag_level = DiagnosticStatus.ERROR
                diag_message = StatDict[diag_level]

        except Exception:
            self.get_logger().error(traceback.print_exc())
            diag_vals.append(
                KeyValue(key="Disk Space Reading", value="Exception"))
            diag_vals.append(KeyValue(key="Disk Space Ex",
                             value=traceback.format_exc()))

            diag_level = DiagnosticStatus.ERROR
            diag_message = StatDict[diag_level]

        # Update status
        with self._mutex:
            self._last_usage_time = self.get_clock().now()
            self._usage_stat.values = diag_vals
            self._usage_stat.message = diag_message
            self._usage_stat.level = diag_level
            if rclpy.ok():
                self._usage_timer = threading.Timer(5.0, self.check_disk_usage)
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

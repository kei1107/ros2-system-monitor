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

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from rclpy.clock import Clock
from rclpy.duration import Duration
from rclpy.time import Time

NetDict = {
    DiagnosticStatus.OK: 'OK',
    DiagnosticStatus.WARN: 'High Network Usage',
    DiagnosticStatus.ERROR: 'Network Down',
}
MemDict = {
    DiagnosticStatus.OK: 'OK',
    DiagnosticStatus.WARN: 'Low Memory',
    DiagnosticStatus.ERROR: 'Very Low Memory'
}
MpstatLoadDict = {
    DiagnosticStatus.OK: 'OK',
    DiagnosticStatus.WARN: 'High Load',
    DiagnosticStatus.ERROR: 'Error'
}
StatDict = {
    DiagnosticStatus.OK: 'OK',
    DiagnosticStatus.WARN: 'Warning',
    DiagnosticStatus.ERROR: 'Error'
}
TempDict = {
    DiagnosticStatus.OK: 'OK',
    DiagnosticStatus.WARN: 'Hot',
    DiagnosticStatus.ERROR: 'Critical Hot'
}
UptimeLoadDict = {
    DiagnosticStatus.OK: 'OK',
    DiagnosticStatus.WARN: 'High Load',
    DiagnosticStatus.ERROR: 'Very High Load'
}
UsageDict = {
    DiagnosticStatus.OK: 'OK',
    DiagnosticStatus.WARN: 'Low Disk Space',
    DiagnosticStatus.ERROR: 'Very Low Disk Space'
}


def update_status_stale(stat: DiagnosticStatus, clock: Clock, last_update_time: Time):
    time_since_update = clock.now() - last_update_time

    stale_status = 'OK'
    if time_since_update > Duration(seconds=20.0) and time_since_update <= Duration(seconds=35.0):
        stale_status = 'Lagging'
        if stat.level == DiagnosticStatus.OK:
            stat.message = stale_status
        elif stat.message.find(stale_status) < 0:
            stat.message = ', '.join([stat.message, stale_status])
        stat.level = max(stat.level, DiagnosticStatus.WARN)
    if time_since_update > Duration(seconds=35.0):
        stale_status = 'Stale'
        if stat.level == DiagnosticStatus.OK:
            stat.message = stale_status
        elif stat.message.find(stale_status) < 0:
            stat.message = ', '.join([stat.message, stale_status])
        stat.level = max(stat.level, DiagnosticStatus.ERROR)

    stat.values.pop(0)
    stat.values.pop(0)
    stat.values.insert(0, KeyValue(key='Update Status', value=stale_status))
    stat.values.insert(1, KeyValue(key='Time Since Last Update',
                       value=str(time_since_update)))

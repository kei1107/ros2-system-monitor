#!/usr/bin/env python3

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from rclpy.clock import Clock
from rclpy.duration import Duration
from rclpy.time import Time

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

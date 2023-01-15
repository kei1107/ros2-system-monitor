#!/usr/bin/env python3

from diagnostic_msgs.msg import DiagnosticStatus

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

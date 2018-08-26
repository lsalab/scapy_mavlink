#!/usr/bin/env python3
'''MAVLink message definitions'''

from scapy.packet import Packet
from scapy.fields import *
from .fields import *
from .enums import *

# MESSAGE ID: 0
class Heartbeat(Packet):
    '''
    Message ID:     0 -> HEARTBEAT

    The heartbeat message shows that a system is present and responding.
    '''
    name = 'HEARTBEAT'
    fields_desc = [
        ByteEnumField('type', None, MAV_TYPE),
        ByteEnumField('autopilot', None, MAV_AUTOPILOT),
        XByteField('base_mode', None),
        LEIntField('custom_mode', None),
        ByteEnumField('system_status', None, MAV_STATE),
        ByteField('mavlink_version', None)
    ]

# MESSAGE ID: 1
class SysStatus(Packet):
    '''
    Message ID:     1 -> SYS_STATUS

    The general system state.
    '''
    name = 'SYS_STATUS'
    fields_desc = [
        LEIntField('onboard_control_sensors_present', None),
        LEIntField('onboard_control_sensors_enabled', None),
        LEIntField('onboard_control_sensors_health', None),
        LEShortField('load', None),
        LEShortField('voltage_battery', None),
        LESignedShortField('current_battery', None),
        SignedByteField('battery_remaining', None),
        LEShortField('drop_rate_comm', None),
        LEShortField('errors_comm', None),
        LEShortField('errors_count1', None),
        LEShortField('errors_count2', None),
        LEShortField('errors_count3', None),
        LEShortField('errors_count4', None),
    ]

# MESSAGE ID: 2
class SystemTime(Packet):
    '''
    Message ID:     2 -> SYSTEM_TIME

    The system time is the time of the master clock, typically the computer clock of the main onboard computer.
    '''
    name = 'SYSTEM_TIME'
    fields_desc = [
        LELongField('time_unix_usec', None),
        LEIntField('time_boot_ms', None)
    ]

# MESSAGE ID: 4
class Ping(Packet):
    '''
    Message ID:     4 -> PING

    A ping message either requesting or responding to a ping.
    This allows to measure the system latencies, including serial port, radio modem and UDP connections.
    '''
    name = 'PING'
    fields_desc = [
        LELongField('time_usec', None),
        LEIntField('seq', None),
        ByteField('target_system', None),
        ByteField('target_component', None)
    ]

# MESSAGE ID: 5
class ChangeOperatorControl(Packet):
    '''
    Message ID:     5 -> CHANGE_OPERATOR_CONTROL

    Request to control this MAV
    '''
    name = 'CHANGE_OPERATOR_CONTROL'
    fields_desc = [
        ByteField('target_system', None),
        ByteField('control_request', None),
        ByteField('version', None),
        StrFixedLenField('passkey', None, length=25),
    ]

# MESSAGE ID: 6
class ChangeOperatorControlAck(Packet):
    '''
    Message ID:     6 -> CHANGE_OPERATOR_CONTROL_ACK

    Accept / deny control of this MAV
    '''
    name = 'CHANGE_OPERATOR_CONTROL_ACK'
    fields_desc = [
        ByteField('gcs_system_id', None),
        ByteField('control_request', None),
        ByteEnumField('ack', None, {0: 'ACK', 1: 'NACK: Wrong passkey', 2: 'NACK: Unsupported passkey encryption method', 3: 'NACK: Already under control'}),
    ]

MESSAGES = {
    0: Heartbeat,
    1: SysStatus,
    2: SystemTime,
    4: Ping,
    5: ChangeOperatorControl,
    6: ChangeOperatorControlAck,
}
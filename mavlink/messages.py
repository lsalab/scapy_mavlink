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

# MESSAGE ID: 7
class AuthKey(Packet):
    '''
    Message ID:     7 -> AUTH_KEY

    Emit an encrypted signature / key identifying this system.
    '''
    name = 'AUTH_KEY'
    fields_desc = [
        StrFixedLenField('key', None, length=32),
    ]

# MESSAGE ID: 11
class SetMode(Packet):
    '''
    Message ID:    11 -> SET_MODE

    Set the system mode, as defined by enum MAV_MODE.
    '''
    name = 'SET_MODE'
    fields_desc = [
        XByteField('target_system', None),
        ByteEnumField('base_mode', None, MAV_MODE),
        LEIntField('custom_mode', None)
    ]

# MESSAGE ID: 20
class ParamRequestRead(Packet):
    '''
    Message ID:    20 -> PARAM_REQUEST_READ

    Request to read the onboard parameter with the param_id string id.
    '''
    name = 'PARAM_REQUEST_READ'
    fields_desc = [
        XByteField('target_system', None),
        XByteField('target_component', None),
        StrFixedLenField('param_id', None, length=16),
        LESignedShortField('param_index', None),
    ]

# MESSAGE ID: 21
class ParamRequestList(Packet):
    '''
    Message ID:    21 -> PARAM_REQUEST_LIST

    Request all parameters of this component. After this request, all parameters are emitted.
    '''
    name = 'PARAM_REQUEST_LIST'
    fields_desc = [
        XByteField('target_system', None),
        XByteField('target_component', None),
    ]

# MESSAGE ID: 22
class ParamValue(Packet):
    '''
    Message ID:    22 -> PARAM_VALUE

    Emit the value of a onboard parameter.
    '''
    name = 'PARAM_VALUE'
    fields_desc = [
        StrFixedLenField('param_id', None, length=16),
        LEFloatField('param_value', None),
        ByteEnumField('param_type', None, MAV_PARAM_TYPE),
        LEShortField('param_count', None),
        LEShortField('param_index', None),
    ]

# MESSAGE ID: 23
class ParamSet(Packet):
    '''
    Message ID:    23 -> PARAM_SET

    Set a parameter value TEMPORARILY to RAM. It will be reset to default on system reboot.
    '''
    name = 'PARAM_SET'
    fields_desc = [
        XByteField('target_system', None),
        XByteField('target_component', None),
        StrFixedLenField('param_id', None, length=16),
        LEFloatField('param_value', None),
        ByteEnumField('param_type', None, MAV_PARAM_TYPE),
    ]

# MESSAGE ID: 24
class GPSRawInt(Packet):
    '''
    Message ID:    24 -> GPS_RAW_INT

    The global position, as returned by the Global Positioning System (GPS).
    This is NOT the global position estimate of the system, but rather a RAW sensor value.
    '''
    name = 'GPS_RAW_INT'
    fields_desc = [
        LELongField('time_usec', None),
        ByteEnumField('fix_type', None, GPS_FIX_TYPE),
        LESignedIntField('lat', None),
        LESignedIntField('lon', None),
        LESignedIntField('alt', None),
        LEShortField('eph', None),
        LEShortField('epv', None),
        LEShortField('vel', None),
        LEShortField('cog', None),
        ByteField('satellites_visible', None),
        LESignedIntField('alt_ellipsoid', None),
        LEIntField('h_acc', None),
        LEIntField('v_acc', None),
        LEIntField('vel_acc', None),
        LEIntField('hdg_acc', None),
    ]

# MESSAGE ID: 25
class GPSStatus(Packet):
    '''
    Message ID:    25 -> GPS_STATUS

    The positioning status, as reported by GPS.
    This message is intended to display status information about each satellite visible to the receiver.
    '''
    name = 'GPS_STATUS'
    fields_desc = [
        ByteField('satellites_visible', None),
        FieldListField('satellite_prn', [], ByteField('', None), length_from=lambda pkt: 20),
        FieldListField('satellite_used', [], ByteField('', None), length_from=lambda pkt: 20),
        FieldListField('satellite_elevation', [], ByteField('', None), length_from=lambda pkt: 20),
        FieldListField('satellite_azimuth', [], ByteField('', None), length_from=lambda pkt: 20),
        FieldListField('satellite_snr', [], ByteField('', None), length_from=lambda pkt: 20),
    ]

# MESSAGE ID: 26
class ScaledIMU(Packet):
    '''
    Message ID:    26 -> SCALED_IMU

    The RAW IMU readings for the usual 9DOF sensor setup.
    This message should contain the scaled values to the described units
    '''
    name = 'SCALED_IMU'
    fields_desc = [
        LEIntField('time_boot_ms', None),
        LESignedShortField('xacc', None),
        LESignedShortField('yacc', None),
        LESignedShortField('zacc', None),
        LESignedShortField('xgyro', None),
        LESignedShortField('ygyro', None),
        LESignedShortField('zgyro', None),
        LESignedShortField('xmag', None),
        LESignedShortField('ymag', None),
        LESignedShortField('zmag', None),
    ]

# MESSAGE ID: 27
class RawIMU(Packet):
    '''
    MessageID:     27 -> RAW_IMU

    The RAW IMU readings for the usual 9DOF sensor setup.
    This message should always contain the true raw values without any scaling to allow data capture and system debugging.
    '''
    name = 'RAW_IMU'
    fields_desc = [
        LELongField('time_usec', None),
        LESignedShortField('xacc', None),
        LESignedShortField('yacc', None),
        LESignedShortField('zacc', None),
        LESignedShortField('xgyro', None),
        LESignedShortField('ygyro', None),
        LESignedShortField('zgyro', None),
        LESignedShortField('xmag', None),
        LESignedShortField('ymag', None),
        LESignedShortField('zmag', None),
    ]

# MESSAGE ID: 28
class RawPressure(Packet):
    '''
    Message ID:    28 -> RAW_PRESSURE

    The RAW pressure readings for the typical setup of one absolute pressure and one differential pressure sensor.
    The sensor values should be the raw, UNSCALED ADC values.
    '''
    name = 'RAW_PRESSURE'
    fields_desc = [
        LELongField('time_usec', None),
        LESignedShortField('press_abs', None),
        LESignedShortField('press_diff1', None),
        LESignedShortField('press_diff2', None),
        LESignedShortField('temperature', None),
    ]

# MESSAGE ID: 29
class ScaledPressure(Packet):
    '''
    Message ID:    29 -> SCALED_PRESSURE

    The pressure readings for the typical setup of one absolute and differential pressure sensor.
    '''
    name = 'SCALED_PRESSURE'
    fields_desc = [
        LEIntField('time_boot_ms', None),
        LEFloatField('press_abs', None),
        LEFloatField('press_diff', None),
        LESignedShortField('temperature', None),
    ]

# MESSAGE ID: 30
class Attitude(Packet):
    '''
    Message ID:    30 -> ATTITUDE

    The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
    '''
    name = 'ATTITUDE'
    fields_desc = [
        LEIntField('time_boot_ms', None),
        LEFloatField('roll', None),
        LEFloatField('pitch', None),
        LEFloatField('yaw', None),
        LEFloatField('rollspeed', None),
        LEFloatField('pitchspeed', None),
        LEFloatField('yawspeed', None),
    ]

# MESSAGE ID: 31
class AttitudeQuaternion(Packet):
    '''
    Message ID:    31 -> ATTITUDE_QUATERION

    The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion.
    Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).
    '''
    name = 'ATTITUDE_QUATERNION'
    fields_desc = [
        LEIntField('time_boot_ms', None),
        LEFloatField('q1', None),
        LEFloatField('q2', None),
        LEFloatField('q3', None),
        LEFloatField('q4', None),
        LEFloatField('rollspeed', None),
        LEFloatField('pitchspeed', None),
        LEFloatField('yawspeed', None),
    ]

MESSAGES = {
    0: Heartbeat,
    1: SysStatus,
    2: SystemTime,
    4: Ping,
    5: ChangeOperatorControl,
    6: ChangeOperatorControlAck,
    7: AuthKey,
    11: SetMode,
    20: ParamRequestRead,
    21: ParamRequestList,
    22: ParamValue,
    23: ParamSet,
    24: GPSRawInt,
    25: GPSStatus,
    26: ScaledIMU,
    27: RawIMU,
    28: RawPressure,
    29: ScaledPressure,
    30: Attitude,
    31:AttitudeQuaternion,
}
#!/usr/bin/env python3
# pylint: disable=C0302
'''MAVLink message definitions'''

from scapy.packet import Packet
from scapy.fields import FlagsField, ByteEnumField, ByteField, FieldListField, LEIntField, LELongField, LEShortEnumField, LEShortField, LESignedIntField, SignedByteField, StrField, StrLenField, StrFixedLenField, XByteField, ConditionalField
from .fields import LEFloatField, XLELongField, LESignedShortField, XLEShortField, WGS84, TagField
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
        XByteField('target_system', None),
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
        XByteField('target_system', None),
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

# MESSAGE ID: 32
class LocalPositionNED(Packet):
    '''
    Messaged ID:   32 -> LOCAL_POSITION_NED

    The filtered local position (e.g. fused computer vision and accelerometers).
    Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
    '''
    name = 'LOCAL_POSITION_NED'
    fields_desc = [
        LEIntField('time_boot_ms', None),
        LEFloatField('x', None),
        LEFloatField('y', None),
        LEFloatField('z', None),
        LEFloatField('vx', None),
        LEFloatField('vy', None),
        LEFloatField('vz', None),
    ]

# MESSAGE ID: 33
class GlobalPositionInt(Packet):
    '''
    Message ID:    33 -> GLOBAL_POSITION_INT

    The filtered global position (e.g. fused GPS and accelerometers).
    The position is in GPS-frame (right-handed, Z-up).
    It is designed as scaled integer message since the resolution of float is not sufficient.
    '''
    name = 'GLOBAL_POSITION_INT'
    fields_desc = [
        LEIntField('time_boot_ms', None),
        LESignedIntField('lat', None),
        LESignedIntField('lon', None),
        LESignedIntField('alt', None),
        LESignedIntField('relative_alt', None),
        LESignedShortField('vx', None),
        LESignedShortField('vy', None),
        LESignedShortField('vz', None),
        LEShortField('hdg', None),
    ]

# MESSAGE ID: 34
class RCChannelsScaled(Packet):
    '''
    Message ID:    34 -> RC_CHANNELS_SCALED

    The scaled values of the RC channels received: (-100%) -10000, (0%) 0, (100%) 10000.
    Channels that are inactive should be set to UINT16_MAX.
    '''
    name = 'RC_CHANNELS_SCALED'
    fields_desc = [
        LEIntField('time_boot_ms', None),
        ByteField('port', None),
        LESignedShortField('chan1_scaled', None),
        LESignedShortField('chan2_scaled', None),
        LESignedShortField('chan3_scaled', None),
        LESignedShortField('chan4_scaled', None),
        LESignedShortField('chan5_scaled', None),
        LESignedShortField('chan6_scaled', None),
        LESignedShortField('chan7_scaled', None),
        LESignedShortField('chan8_scaled', None),
        ByteField('rssi', None),
    ]

# MESSAGE ID: 35
class RCChannelsRaw(Packet):
    '''
    Message ID:    35 -> RC_CHANNELS_RAW

    The RAW values of the RC channels received.
    The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%.
    A value of UINT16_MAX implies the channel is unused.
    Individual receivers/transmitters might violate this specification.
    '''
    name = 'RC_CHANNELS_RAW'
    fields_desc = [
        LEIntField('time_boot_ms', None),
        ByteField('port', None),
        LEShortField('chan1_raw', None),
        LEShortField('chan2_raw', None),
        LEShortField('chan3_raw', None),
        LEShortField('chan4_raw', None),
        LEShortField('chan5_raw', None),
        LEShortField('chan6_raw', None),
        LEShortField('chan7_raw', None),
        LEShortField('chan8_raw', None),
        ByteField('rssi', None),
    ]

# MESSAGE ID: 36
class ServoOutputRaw(Packet):
    '''
    Message ID:    36 -> SERVO_OUTPUT_RAW

    The RAW values of the servo outputs (for RC input from the remote, use the RC_CHANNELS messages).
    The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%.
    '''
    name = 'SERVO_OUTPUT_RAW'
    fields_desc = [
        LEIntField('time_usec', None),
        ByteField('port', None),
        LEShortField('servo1_raw', None),
        LEShortField('servo2_raw', None),
        LEShortField('servo3_raw', None),
        LEShortField('servo4_raw', None),
        LEShortField('servo5_raw', None),
        LEShortField('servo6_raw', None),
        LEShortField('servo7_raw', None),
        LEShortField('servo8_raw', None),
        LEShortField('servo9_raw', None),
        LEShortField('servo10_raw', None),
        LEShortField('servo11_raw', None),
        LEShortField('servo12_raw', None),
        LEShortField('servo13_raw', None),
        LEShortField('servo14_raw', None),
        LEShortField('servo15_raw', None),
        LEShortField('servo16_raw', None),
    ]

# MESSAGE ID: 37
class MissionRequestPartialList(Packet):
    '''
    Message ID:    37 -> MISSION_REQUEST_PARTIAL_LIST

    Request a partial list of mission items from the system/component.
    https://mavlink.io/en/protocol/mission.html.
    If start and end index are the same, just send one waypoint.
    '''
    name = 'MISSION_REQUEST_PARTIAL_LIST'
    fields_desc = [
        XByteField('target_system', None),
        XByteField('target_component', None),
        LESignedShortField('start_index', 0x0000),
        LESignedShortField('end_index', None),
        ByteEnumField('mission_type', None, MAV_MISSION_TYPE),
    ]

# MESSAGE ID: 38
class MissionWritePartialList(Packet):
    '''
    Message ID:    38 -> MISSION_WRITE_PARTIAL_LIST

    This message is sent to the MAV to write a partial list.
    If start index == end index, only one item will be transmitted / updated.
    If the start index is NOT 0 and above the current list size, this request should be REJECTED!
    '''
    name = 'MISSION_WRITE_PARTIAL_LIST'
    fields_desc = [
        XByteField('target_system', None),
        XByteField('target_component', None),
        LESignedShortField('start_index', 0x0000),
        LESignedShortField('end_index', None),
        ByteEnumField('mission_type', None, MAV_MISSION_TYPE)
    ]

# MESSAGE ID: 39
class MissionItem(Packet):
    '''
    Message ID:    39 -> MISSION_ITEM

    Message encoding a mission item.
    This message is emitted to announce the presence of a mission item and to set a mission item on the system.
    The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude.
    Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU).
    See also https://mavlink.io/en/protocol/mission.html.
    '''
    name = 'MISSION_ITEM'
    fields_desc = [
        XByteField('target_system', None),
        XByteField('target_component', None),
        LEShortField('seq', None),
        ByteEnumField('frame', None, MAV_FRAME),
        LEShortEnumField('command', None, MAV_CMD),
        ByteField('current', None),
        ByteField('autocontinue', None),
        LEFloatField('param1', None),
        LEFloatField('param2', None),
        LEFloatField('param3', None),
        LEFloatField('param4', None),
        LEFloatField('x', None),
        LEFloatField('y', None),
        LEFloatField('z', None),
    ]

# MESSAGE ID: 40
class MissionRequest(Packet):
    '''
    Message ID:    40 ->  MISSION_REQUEST

    Request the information of the mission item with the sequence number seq.
    The response of the system to this message should be a MISSION_ITEM message.
    https://mavlink.io/en/protocol/mission.html
    '''
    name = 'MISSION_REQUEST'
    fields_desc = [
        XByteField('target_system', None),
        XByteField('target_component', None),
        LEShortField('seq', None),
        ByteEnumField('mission_type', None, MAV_MISSION_TYPE),
    ]

# MESSAGE ID: 41
class MissionSetCurrent(Packet):
    '''
    Message ID:    41 -> MISSION_SET_CURRENT

    Set the mission item with sequence number seq as current item.
    This means that the MAV will continue to this mission item on the shortest path (not following the mission items in-between).
    '''
    name = 'MISSION_SET_CURRENT'
    fields_desc = [
        XByteField('target_system', None),
        XByteField('target_component', None),
        LEShortField('seq', None),
    ]

# MESSAGE ID: 42
class MissionCurrent(Packet):
    '''
    Message ID:    42 -> MISSION_CURRENT

    Message that announces the sequence number of the current active mission item.
    The MAV will fly towards this mission item.
    '''
    name = 'MISSION_CURRENT'
    fields_desc = [
        LEShortField('seq', None),
    ]

# MESSAGE ID: 43
class MissionRequestList(Packet):
    '''
    Message ID:    43 -> MISSION_REQUEST_LIST

    Request the overall list of mission items from the system/component.
    '''
    name = 'MISSION_REQUEST_LIST'
    fields_desc = [
        XByteField('target_system', None),
        XByteField('target_component', None),
        ByteEnumField('mission_type', None, MAV_MISSION_TYPE),
    ]

# MESSAGE ID: 44
class MissionCount(Packet):
    '''
    Message ID:    44 -> MISSION_COUNT

    This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction.
    The GCS can then request the individual mission item based on the knowledge of the total number of waypoints.
    '''
    name = 'MISSION_COUNT'
    fields_desc = [
        XByteField('target_system', None),
        XByteField('targer_component', None),
        LEShortField('count', None),
        ByteEnumField('mission_type', None, MAV_MISSION_TYPE),
    ]

# MESSAGE ID: 45
class MissionClearAll(Packet):
    '''
    Message ID:    45 -> MISSION_CLEAR_ALL

    Delete all mission items at once.
    '''
    name = 'MISSION_CLEAR_ALL'
    fields_desc = [
        XByteField('target_system', None),
        XByteField('target_component', None),
        ByteEnumField('mission_type', None, MAV_MISSION_TYPE),
    ]

# MESSAGE ID: 46
class MissionItemReached(Packet):
    '''
    Message ID:    46 -> MISSION_ITEM_REACHED

    A certain mission item has been reached.
    The system will either hold this position (or circle on the orbit) or
    (if the autocontinue on the WP was set) continue to the next waypoint.
    '''
    name = 'MISSION_ITEM_REACHED'
    fields_desc = [
        LEShortField('seq', None),
    ]

# MESSAGE ID: 47
class MissionAck(Packet):
    '''
    Message ID:    47 -> MISSION_ACK

    Acknowledgment message during waypoint handling.
    The type field states if this message is a positive ack (type=0) or if an error happened (type=non-zero).
    '''
    name = 'MISSION_ACK'
    fields_desc = [
        XByteField('target_system', None),
        XByteField('target_component', None),
        ByteEnumField('type', None, MAV_MISSION_RESULT),
        ByteEnumField('mission_type', None, MAV_MISSION_TYPE),
    ]

# MESSAGE ID: 48
class SetGPSGlobalOrigin(Packet):
    '''
    Message ID:    48 -> SET_GPS_GLOBAL_ORIGIN

    As local waypoints exist, the global waypoint reference allows to transform between the local coordinate frame and the global (GPS) coordinate frame.
    This can be necessary when e.g. in- and outdoor settings are connected and the MAV should move from in- to outdoor.
    '''
    name = 'SET_GPS_GLOBAL_ORIGIN'
    fields_desc = [
        XByteField('target_system', None),
        LESignedIntField('latitude', None),
        LESignedIntField('longitude', None),
        LESignedIntField('altitude', None),
        LELongField('time_usec', None),
    ]

# MESSAGE ID: 49
class GPSGlobalOrigin(Packet):
    '''
    Message ID:    49 -> GPS_GLOBAL_ORIGIN

    Once the MAV sets a new GPS-Local correspondence, this message announces the origin (0,0,0) position
    '''
    name = 'GPS_GLOBAL_ORIGIN'
    fields_desc = [
        LESignedIntField('latitude', None),
        LESignedIntField('longitude', None),
        LESignedIntField('altitude', None),
        LELongField('time_usec', None),
    ]

# MESSAGE ID: 50
class ParamMapRC(Packet):
    '''
    Message ID:    50 -> PARAM_MAP_RC

    Bind a RC channel to a parameter.
    The parameter should change according to the RC channel value.
    '''
    name = 'PARAM_MAP_RC'
    fields_desc = [
        XByteField('target_system', None),
        XByteField('target_component', None),
        StrFixedLenField('param_id', None, length=16),
        LESignedShortField('param_index', None),
        ByteField('parameter_rc_channel_index', None),
        LEFloatField('param_value0', None),
        LEFloatField('scale', None),
        LEFloatField('param_value_min', None),
        LEFloatField('param_value_max', None),
    ]

# MESSAGE ID: 51
class MissionRequestInt(Packet):
    '''
    Message ID:    51 -> MISSION_REQUEST_INT

    Request the information of the mission item with the sequence number seq.
    The response of the system to this message should be a MISSION_ITEM_INT message. https://mavlink.io/en/protocol/mission.html
    '''
    name = 'MISSION_REQUEST_INT'
    fields_desc = [
        XByteField('target_system', None),
        XByteField('target_component', None),
        LEShortField('seq', None),
        ByteEnumField('mission_type', None, MAV_MISSION_TYPE)
    ]

# MESSAGE ID: 54
class SafetySetAllowedArea(Packet):
    '''
    Message ID:    54 -> SAFETY_SET_ALLOWED_AREA

    Set a safety zone (volume), which is defined by two corners of a cube.
    This message can be used to tell the MAV which setpoints/waypoints to accept and which to reject.
    Safety areas are often enforced by national or competition regulations.
    '''
    name = 'SAFETY_SET_ALLOWED_AREA'
    fields_desc = [
        XByteField('target_system', None),
        XByteField('target_component', None),
        ByteEnumField('frame', None, MAV_FRAME),
        LEFloatField('p1x', None),
        LEFloatField('p1y', None),
        LEFloatField('p1z', None),
        LEFloatField('p2x', None),
        LEFloatField('p2y', None),
        LEFloatField('p2z', None),
    ]

# MESSAGE ID: 55
class SafetyAllowedArea(Packet):
    '''
    Message ID:    55 -> SAFETY_ALLOWED_AREA

    Read out the safety zone the MAV currently assumes.
    '''
    name = 'SAFETY_ALLOWED_AREA'
    fields_desc = [
        ByteEnumField('frame', None, MAV_FRAME),
        LEFloatField('p1x', None),
        LEFloatField('p1y', None),
        LEFloatField('p1z', None),
        LEFloatField('p2x', None),
        LEFloatField('p2y', None),
        LEFloatField('p2z', None),
    ]

# MESSAGE ID: 61
class AttitudeQuaternionCOV(Packet):
    '''
    Message ID:    61 -> ATTITUDE_QUATERNION_COV

    The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion.
    Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).
    '''
    name = 'ATTITUDE_QUATERNION_COV'
    fields_desc = [
        LELongField('time_usec', None),
        FieldListField('q', None, LEFloatField, count_from=lambda pkt: 4),
        LEFloatField('rollspeed', None),
        LEFloatField('pitchspeed', None),
        LEFloatField('yawspeed', None),
        FieldListField('covariance', None, LEFloatField, count_from=lambda pkt: 9),
    ]

# MESSAGE ID: 62
class NAVControllerOutput(Packet):
    '''
    Message ID:    62 -> NAV_CONTROLLER_OUTPUT

    The state of the fixed wing navigation and position controller.
    '''
    name = 'NAV_CONTROLLER_OUTPUT'
    fields_desc = [
        LEFloatField('nav_roll', None),
        LEFloatField('nav_pitch', None),
        LESignedShortField('nav_bearing', None),
        LESignedShortField('target_bearing', None),
        LEShortField('wp_dist', None),
        LEFloatField('alt_error', None),
        LEFloatField('aspd_error', None),
        LEFloatField('xtrack_error', None),
    ]

# MESSAGE ID: 63
class GlobalPositionIntCOV(Packet):
    '''
    Message ID:    63 -> GLOBAL_POSITION_INT_COV

    The filtered global position (e.g. fused GPS and accelerometers).
    The position is in GPS-frame (right-handed, Z-up).
    It is designed as scaled integer message since the resolution of float is not sufficient.

    NOTE: This message is intended for onboard networks / companion computers and higher-bandwidth links and optimized for accuracy and completeness.
    Please use the GLOBAL_POSITION_INT message for a minimal subset.
    '''
    name = 'GLOBAL_POSITION_INT_COV'
    fields_desc = [
        LELongField('time_usec', None),
        ByteEnumField('estimator_type', None, MAV_ESTIMATOR_TYPE),
        LESignedIntField('lat', None),
        LESignedIntField('lon', None),
        LESignedIntField('alt', None),
        LESignedIntField('relative_alt', None),
        LEFloatField('vx', None),
        LEFloatField('vy', None),
        LEFloatField('vz', None),
        FieldListField('covariance', None, LEFloatField, count_from=lambda pkt: 36),
    ]

# MESSAGE ID: 64
class LocalPositionNEDCOV(Packet):
    '''
    Message ID:    64 -> LOCAL_POSITION_NED_COV

    The filtered local position (e.g. fused computer vision and accelerometers).
    Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
    '''
    name = 'LOCAL_POSITION_NED_COV'
    fields_desc = [
        LELongField('time_usec', None),
        ByteEnumField('estimator_type', None, MAV_ESTIMATOR_TYPE),
        LEFloatField('x', None),
        LEFloatField('y', None),
        LEFloatField('z', None),
        LEFloatField('vx', None),
        LEFloatField('vy', None),
        LEFloatField('vz', None),
        LEFloatField('ax', None),
        LEFloatField('ay', None),
        LEFloatField('az', None),
        FieldListField('covariance', None, LEFloatField, count_from=lambda pkt: 45)
    ]

# MESSAGE ID: 65
class RCChannels(Packet):
    '''
    Message ID:    65 -> RC_CHANNELS

    The PPM values of the RC channels received.
    The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%.
    A value of UINT16_MAX implies the channel is unused.
    Individual receivers/transmitters might violate this specification.
    '''
    name = 'RC_CHANNELS'
    fields_desc = [
        LEIntField('time_boot_ms', None),
        ByteField('chancount', None),
        LEShortField('chan1_raw', None),
        LEShortField('chan2_raw', None),
        LEShortField('chan3_raw', None),
        LEShortField('chan4_raw', None),
        LEShortField('chan5_raw', None),
        LEShortField('chan6_raw', None),
        LEShortField('chan7_raw', None),
        LEShortField('chan8_raw', None),
        LEShortField('chan9_raw', None),
        LEShortField('chan10_raw', None),
        LEShortField('chan11_raw', None),
        LEShortField('chan12_raw', None),
        LEShortField('chan13_raw', None),
        LEShortField('chan14_raw', None),
        LEShortField('chan15_raw', None),
        LEShortField('chan16_raw', None),
        LEShortField('chan17_raw', None),
        LEShortField('chan18_raw', None),
        ByteField('rssi', None),
    ]

# MESSAGE ID: 66
class RequestDataStream(Packet):
    '''
    Message ID:    66 -> REQUEST_DATA_STREAM

    Request a data stream.
    '''
    name = 'REQUEST_DATA_STREAM'
    fields_desc = [
        XByteField('target_system', None),
        XByteField('target_component', None),
        XByteField('req_stream_id', None),
        LEShortField('req_message_rate', None),
        ByteField('start_stop', None),
    ]

# MESSAGE ID: 67
class DataStream(Packet):
    '''
    Message ID:    67 -> DATA_STREAM

    Data stream status information.
    '''
    name = 'DATA_STREAM'
    fields_desc = [
        XByteField('stream_id', None),
        LEShortField('message_rate', None),
        ByteField('on_off', None),
    ]

# MESSAGE ID: 69
class ManualControl(Packet):
    '''
    Message ID:    69 -> MANUAL_CONTROL

    This message provides an API for manually controlling the vehicle using standard
    joystick axes nomenclature, along with a joystick-like input device.
    '''
    name = 'MANUAL_CONTROL'
    fields_desc = [
        XByteField('target', None),
        LESignedShortField('x', None),
        LESignedShortField('y', None),
        LESignedShortField('z', None),
        LESignedShortField('r', None),
        LEShortField('buttons', None),
    ]

# MESSAGE ID: 70
class RCChanelsOverride(Packet):
    '''
    Message ID:    70 -> RC_CHANNELS_OVERRIDE

    The RAW values of the RC channels sent to the MAV to override info received from the RC radio.
    A value of UINT16_MAX means no change to that channel.
    A value of 0 means control of that channel should be released back to the RC radio.
    The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%.
    Individual receivers/transmitters might violate this specification.
    '''
    name = 'RC_CHANNELS_OVERRIDE'
    fields_desc = [
        XByteField('target_system', None),
        XByteField('target_component', None),
        LEShortField('chan1_raw', None),
        LEShortField('chan2_raw', None),
        LEShortField('chan3_raw', None),
        LEShortField('chan4_raw', None),
        LEShortField('chan5_raw', None),
        LEShortField('chan6_raw', None),
        LEShortField('chan7_raw', None),
        LEShortField('chan8_raw', None),
        LEShortField('chan9_raw', None),
        LEShortField('chan10_raw', None),
        LEShortField('chan11_raw', None),
        LEShortField('chan12_raw', None),
        LEShortField('chan13_raw', None),
        LEShortField('chan14_raw', None),
        LEShortField('chan15_raw', None),
        LEShortField('chan16_raw', None),
        LEShortField('chan17_raw', None),
        LEShortField('chan18_raw', None),
    ]

# MESSAGE ID: 73
class MissionItemInt(Packet):
    '''
    Message ID:    73 -> MISSION_ITEM_INT

    Message encoding a mission item.
    This message is emitted to announce the presence of a mission item and to set a mission item on the system.
    The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude.
    Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU).
    See also https://mavlink.io/en/protocol/mission.html.
    '''
    name = 'MISSION_ITEM_INT'
    fields_desc = [
        XByteField('target_system', None),
        XByteField('target_component', None),
        LEShortField('seq', None),
        ByteEnumField('frame', None, MAV_FRAME),
        LEShortEnumField('command', None, MAV_CMD),
        ByteField('current', None),
        ByteField('autocontinue', None),
        LEFloatField('param1', None),
        LEFloatField('param2', None),
        LEFloatField('param3', None),
        LEFloatField('param4', None),
        LESignedIntField('x', None),
        LESignedIntField('y', None),
        LEFloatField('z', None),
        ByteEnumField('mission_type', None, MAV_MISSION_TYPE),
    ]

# MESSAGE ID: 74
class VFRHUD(Packet):
    '''
    Message ID:    74 -> VFR_HUD

    Metrics typically displayed on a HUD for fixed wing aircraft.
    '''
    name = 'VFR_HUD'
    fields_desc = [
        LEFloatField('airspeed', None),
        LEFloatField('groundspeed', None),
        LESignedShortField('heading', None),
        LEShortField('throttle', None),
        LEFloatField('alt', None),
        LEFloatField('climb', None),
    ]

# MESSAGE ID: 75
class CommandInt(Packet):
    '''
    Message ID:    75 -> COMMAND_INT

    Message encoding a command with parameters as scaled integers. Scaling depends on the actual command value.
    '''
    name = 'COMMAND_INT'
    fields_desc = [
        XByteField('target_system', None),
        XByteField('target_component', None),
        ByteEnumField('frame', None, MAV_FRAME),
        LEShortEnumField('command', None, MAV_CMD),
        ByteField('current', None),
        ByteField('autocontinue', None),
        LEFloatField('param1', None),
        LEFloatField('param2', None),
        LEFloatField('param3', None),
        LEFloatField('param4', None),
        LESignedIntField('x', None),
        LESignedIntField('y', None),
        LEFloatField('z', None),
    ]

# MESSAGE ID: 76
class CommandLong(Packet):
    '''
    Message ID:    76 -> COMMAND_LONG

    Send a command with up to seven parameters to the MAV
    '''
    name = 'COMMAND_LONG'
    fields_desc = [
        XByteField('target_system', None),
        XByteField('target_component', None),
        LEShortEnumField('command', None, MAV_CMD),
        ByteField('confirmation', None),
        LEFloatField('param1', None),
        LEFloatField('param2', None),
        LEFloatField('param3', None),
        LEFloatField('param4', None),
        LEFloatField('param5', None),
        LEFloatField('param6', None),
        LEFloatField('param7', None),
    ]

# MESSAGE ID: 77
class CommandAck(Packet):
    '''
    Message ID:    77 -> COMMAND_ACK

    Report status of a command. Includes feedback whether the command was executed.
    '''
    name = 'COMMAND_ACK'
    fields_desc = [
        LEShortEnumField('command', None, MAV_CMD),
        ByteEnumField('result', None, MAV_RESULT),
        ByteField('progress', None),
        LEIntField('result_param2', None),
        XByteField('target_system', None),
        XByteField('target_component', None)
    ]

# MESSAGE ID: 81
class ManualSetpoint(Packet):
    '''
    Message ID:    81 -> MANUAL_SETPOINT

    Setpoint in roll, pitch, yaw and thrust from the operator
    '''
    name = 'MANUAL_SETPOINT'
    fields_desc = [
        LEIntField('time_boot_ms', None),
        LEFloatField('roll', None),
        LEFloatField('pitch', None),
        LEFloatField('yaw', None),
        LEFloatField('thrust', None),
        ByteField('mode_switch', None),
        ByteField('manual_override_switch', None),
    ]

# MESSAGE ID: 82
class SetAttitudeTarget(Packet):
    '''
    Message ID:    82 -> SET_ATTITUDE_TARGET

    Sets a desired vehicle attitude.
    Used by an external controller to command the vehicle (manual controller or other system).
    '''
    name = 'SET_ATTITUDE_TARGET'
    fields_desc = [
        LEIntField('time_boot_ms', None),
        XByteField('target_system', None),
        XByteField('target_component', None),
        XByteField('type_mask', 0x00),
        FieldListField('q', None, LEFloatField, count_from=lambda pkt: 4),
        LEFloatField('body_roll_rate', None),
        LEFloatField('body_pitch_rate', None),
        LEFloatField('body_raw_rate', None),
        LEFloatField('thrust', None),
    ]

# MESSAGE ID: 83
class AttitudeTarget(Packet):
    '''
    Message ID:    83 -> ATTITUDE_TARGET

    Reports the current commanded attitude of the vehicle as specified by the autopilot.
    This should match the commands sent in a SET_ATTITUDE_TARGET message if the vehicle is being controlled this way.
    '''
    name = 'ATTITUDE_TARGET'
    fields_desc = [
        LEIntField('time_boot_ms', None),
        XByteField('type_mask', 0x00),
        FieldListField('q', None, LEFloatField, count_from=lambda pkt: 4),
        LEFloatField('body_roll_rate', None),
        LEFloatField('body_pitch_rate', None),
        LEFloatField('body_raw_rate', None),
        LEFloatField('thrust', None),
    ]

# MESSAGE ID: 84
class SetPositionTargetLocalNED(Packet):
    '''
    Message ID:    84 -> SET_POSITION_TARGET_LOCAL_NED

    Sets a desired vehicle position in a local north-east-down coordinate frame.
    Used by an external controller to command the vehicle (manual controller or other system).
    '''
    name = 'SET_POSITION_TARGET_LOCAL_NED'
    fields_desc = [
        LEIntField('time_boot_ms', None),
        XByteField('target_system', None),
        XByteField('target_component', None),
        ByteEnumField('coordinate_frame', 0x01, MAV_FRAME),
        XLEShortField('type_mask', 0x0000),
        LEFloatField('x', None),
        LEFloatField('y', None),
        LEFloatField('z', None),
        LEFloatField('vx', None),
        LEFloatField('vy', None),
        LEFloatField('vz', None),
        LEFloatField('afx', None),
        LEFloatField('afy', None),
        LEFloatField('afz', None),
        LEFloatField('yaw', None),
        LEFloatField('yaw_rate', None),
    ]

# MESSAGE ID: 85
class PositionTargetLocalNED(Packet):
    '''
    Message ID:    85 -> POSITION_TARGET_LOCAL_NED

    Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot.
    This should match the commands sent in SET_POSITION_TARGET_LOCAL_NED if the vehicle is being controlled this way.
    '''
    name = 'POSITION_TARGET_LOCAL_NED'
    fields_desc = [
        LEIntField('time_boot_ms', None),
        ByteEnumField('coordinate_frame', None, MAV_FRAME),
        XLEShortField('type_mask', 0x0000),
        LEFloatField('x', None),
        LEFloatField('y', None),
        LEFloatField('z', None),
        LEFloatField('vx', None),
        LEFloatField('vy', None),
        LEFloatField('vz', None),
        LEFloatField('afx', None),
        LEFloatField('afy', None),
        LEFloatField('afz', None),
        LEFloatField('yaw', None),
        LEFloatField('yaw_rate', None),
    ]

# MESSAGE ID: 86
class SetPositionTargetGlobalInt(Packet):
    '''
    Message ID:    86 -> SET_POSITION_TARGET_GLOBAL_INT

    Sets a desired vehicle position, velocity, and/or acceleration in a global coordinate system (WGS84).
    Used by an external controller to command the vehicle (manual controller or other system).
    '''
    name = 'SET_POSITION_TARGET_GLOBAL_INT'
    fields_desc = [
        LEIntField('time_boot_ms', None),
        XByteField('target_system', None),
        XByteField('target_component', None),
        ByteEnumField('coordinate_frame', 0x05, MAV_FRAME),
        XLEShortField('type_mask', 0x0000),
        LESignedIntField('lat_int', None),
        LESignedIntField('lon_int', None),
        LEFloatField('alt', None),
        LEFloatField('vx', None),
        LEFloatField('vy', None),
        LEFloatField('vz', None),
        LEFloatField('afx', None),
        LEFloatField('afy', None),
        LEFloatField('afz', None),
        LEFloatField('yaw', None),
        LEFloatField('yaw_rate', None),
    ]

# MESSAGE ID: 87
class PositionTargetGlobalInt(Packet):
    '''
    Message ID:    87 -> POSITION_TARGET_GLOBAL_INT

    Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot.
    This should match the commands sent in SET_POSITION_TARGET_GLOBAL_INT if the vehicle is being controlled this way.
    '''
    name = 'POSITION_TARGET_GLOBAL_INT'
    fields_desc = [
        LEIntField('time_boot_ms', None),
        ByteEnumField('coordinate_frame', 0x05, MAV_FRAME),
        XLEShortField('type_mask', 0x0000),
        LESignedIntField('lat_int', None),
        LESignedIntField('lon_int', None),
        LEFloatField('alt', None),
        LEFloatField('vx', None),
        LEFloatField('vy', None),
        LEFloatField('vz', None),
        LEFloatField('afx', None),
        LEFloatField('afy', None),
        LEFloatField('afz', None),
        LEFloatField('yaw', None),
        LEFloatField('yaw_rate', None),
    ]

# MESSAGE ID: 89
class LocalPositionNEDSystemGlobalOffset(Packet):
    '''
    Message ID:    89 -> LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET

    The offset in X, Y, Z and yaw between the LOCAL_POSITION_NED messages of MAV X and the global coordinate frame in NED coordinates.
    Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
    '''
    name = 'LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET'
    fields_desc = [
        LEIntField('time_boot_ms', None),
        LEFloatField('x', None),
        LEFloatField('y', None),
        LEFloatField('z', None),
        LEFloatField('roll', None),
        LEFloatField('pitch', None),
        LEFloatField('yaw', None),
    ]

# MESSAGE ID: 90
class HILState(Packet):
    '''
    Message ID:    90 -> HIL_STATE

    Sent from simulation to autopilot.
    This packet is useful for high throughput applications such as hardware in the loop simulations.
    '''
    name = 'HIL_STATE'
    fields_desc = [
        LELongField('time_usec', None),
        LEFloatField('roll', None),
        LEFloatField('pitch', None),
        LEFloatField('yaw', None),
        LEFloatField('rollspeed', None),
        LEFloatField('pitchspeed', None),
        LEFloatField('yawspeed', None),
        LESignedIntField('lat', None),
        LESignedIntField('lon', None),
        LESignedIntField('alt', None),
        LESignedShortField('vx', None),
        LESignedShortField('vy', None),
        LESignedShortField('vz', None),
        LESignedShortField('xacc', None),
        LESignedShortField('yacc', None),
        LESignedShortField('zacc', None),
    ]

# MESSAGE ID: 91
class HILControls(Packet):
    '''
    Message ID:    91 -> HIL_CONTROLS

    Sent from autopilot to simulation.
    Hardware in the loop control outputs
    '''
    name = 'HIL_CONTROLS'
    fields_desc = [
        LELongField('time_usec', None),
        LEFloatField('roll_ailerons', None),
        LEFloatField('pitch_elevator', None),
        LEFloatField('yaw_rudder', None),
        LEFloatField('throttle', None),
        LEFloatField('aux1', None),
        LEFloatField('aux2', None),
        LEFloatField('aux3', None),
        LEFloatField('aux4', None),
        ByteEnumField('mode', None, MAV_MODE),
        XByteField('nav_mode', None),
    ]

# MESSAGE ID: 92
class HILRCInputsRaw(Packet):
    '''
    Message ID:    92 -> HIL_RC_INPUTS_RAW

    Sent from simulation to autopilot.
    The RAW values of the RC channels received.
    The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%.
    Individual receivers/transmitters might violate this specification.
    '''
    name = 'HIL_RC_INPUTS_RAW'
    fields_desc = [
        LELongField('time_usec', None),
        LEShortField('chan1_raw', None),
        LEShortField('chan2_raw', None),
        LEShortField('chan3_raw', None),
        LEShortField('chan4_raw', None),
        LEShortField('chan5_raw', None),
        LEShortField('chan6_raw', None),
        LEShortField('chan7_raw', None),
        LEShortField('chan8_raw', None),
        LEShortField('chan9_raw', None),
        LEShortField('chan10_raw', None),
        LEShortField('chan11_raw', None),
        LEShortField('chan12_raw', None),
        ByteField('rssi', None),
    ]

# MESSAGE ID: 93
class HILActuatorControls(Packet):
    '''
    Message ID:    93 -> HIL_ACTUATOR_CONTROLS

    Sent from autopilot to simulation.
    Hardware in the loop control outputs (replacement for HIL_CONTROLS)
    '''
    name = 'HIL_ACTUATOR_CONTROLS'
    fields_desc = [
        LELongField('time_usec', None),
        FieldListField('controls', None, LEFloatField, count_from=lambda pkt: 16),
        ByteEnumField('mode', None, MAV_MODE),
        XLELongField('flags', None),
    ]

# MESSAGE ID: 100
class OpticalFlow(Packet):
    '''
    Message ID:   100 -> OPTICAL_FLOW

    Optical flow from a flow sensor (e.g. optical mouse sensor)
    '''
    name = 'OPTICAL_FLOW'
    fields_desc = [
        LELongField('time_usec', None),
        ByteField('sensor_id', None),
        LESignedShortField('flow_x', None),
        LESignedShortField('flow_y', None),
        LEFloatField('flow_comp_m_x', None),
        LEFloatField('flow_comp_m_y', None),
        ByteField('quality', None),
        LEFloatField('ground_distance', None),
        LEFloatField('flow_rate_x', None),
        LEFloatField('flow_rate_y', None),
    ]

# MESSAGE ID: 101
class GlobalVisionPositionEstimate(Packet):
    '''
    Message ID:   101 -> GLOBAL_VISION_POSITION_ESTIMATE
    '''
    name = 'GLOBAL_VISION_POSITION_ESTIMATE'
    fields_desc = [
        LELongField('usec', None),
        LEFloatField('x', None),
        LEFloatField('y', None),
        LEFloatField('z', None),
        LEFloatField('roll', None),
        LEFloatField('pitch', None),
        LEFloatField('yaw', None),
        FieldListField('covariance', None, LEFloatField, count_from=lambda pkt: 21),
    ]

# MESSAGE ID: 102
class VisionPositionEstimate(Packet):
    '''
    Message ID:   102 -> VISION_POSITION_ESTIMATE
    '''
    name = 'VISION_POSITION_ESTIMATE'
    fields_desc = [
        LELongField('usec', None),
        LEFloatField('x', None),
        LEFloatField('y', None),
        LEFloatField('z', None),
        LEFloatField('roll', None),
        LEFloatField('pitch', None),
        LEFloatField('yaw', None),
        FieldListField('covariance', None, LEFloatField, count_from=lambda pkt: 21),
    ]

# MESSAGE ID: 103
class VisionSpeedEstimate(Packet):
    '''
    Message ID:   103 -> VISION_SPEED_ESTIMATE

    Speed estimate from a vision source.
    '''
    name = 'VISION_SPEED_ESTIMATE'
    fields_desc = [
        LELongField('usec', None),
        LEFloatField('x', None),
        LEFloatField('y', None),
        LEFloatField('z', None),
        FieldListField('covariance', None, LEFloatField, count_from=lambda pkt: 9),
    ]

# MESSAGE ID: 104
class ViconPositionEstimate(Packet):
    '''
    Message ID:   104 -> VICON_POSITION_ESTIMATE

    Global position estimate from a Vicon motion system source.
    '''
    name = 'VICON_POSITION_ESTIMATE'
    fields_desc = [
        LELongField('usec', None),
        LEFloatField('x', None),
        LEFloatField('y', None),
        LEFloatField('z', None),
        LEFloatField('roll', None),
        LEFloatField('pitch', None),
        LEFloatField('yaw', None),
        FieldListField('covariance', None, LEFloatField, count_from=lambda pkt: 21)
    ]

# MESSAGE ID: 105
class HighresIMU(Packet):
    '''
    Message ID:   105 -> HIGHRES_IMU

    The IMU readings in SI units in NED body frame.
    '''
    name = 'HIGHRES_IMU'
    fields_desc = [
        LELongField('time_usec', None),
        LEFloatField('xacc', None),
        LEFloatField('yacc', None),
        LEFloatField('zacc', None),
        LEFloatField('xgyro', None),
        LEFloatField('ygyro', None),
        LEFloatField('zgyro', None),
        LEFloatField('xmag', None),
        LEFloatField('ymag', None),
        LEFloatField('zmag', None),
        LEFloatField('abs_pressure', None),
        LEFloatField('diff_pressure', None),
        LEFloatField('pressure_alt', None),
        LEFloatField('temperature', None),
        XLEShortField('fields_updated', None),
    ]

# MESSAGE ID: 106
class OpticalFlowRAD(Packet):
    '''
    Message ID:   106 -> OPTICAL_FLOW_RAD

    Optical flow from an angular rate flow sensor (e.g. PX4FLOW or mouse sensor)
    '''
    name = 'OPTICAL_FLOW_RAD'
    fields_desc = [
        LELongField('time_usec', None),
        XByteField('sensor_id', None),
        LEIntField('integration_time_us', None),
        LEFloatField('integrated_x', None),
        LEFloatField('integrated_y', None),
        LEFloatField('integrated_xgyro', None),
        LEFloatField('integrated_ygyro', None),
        LEFloatField('integrated_zgyro', None),
        LESignedShortField('temperature', None),
        ByteField('quality', None),
        LEIntField('time_delta_distance_us', None),
        LEFloatField('distance', None),
    ]

# MESSAGE ID: 107
class HILSensor(Packet):
    '''
    Message ID:   107 -> HIL_SENSOR

    The IMU readings in SI units in NED body frame
    '''
    name = 'HIL_SENSOR'
    fields_desc = [
        LELongField('time_usec', None),
        LEFloatField('xacc', None),
        LEFloatField('yacc', None),
        LEFloatField('zacc', None),
        LEFloatField('xgyro', None),
        LEFloatField('ygyro', None),
        LEFloatField('zgyro', None),
        LEFloatField('xmag', None),
        LEFloatField('ymag', None),
        LEFloatField('zmag', None),
        LEFloatField('abs_pressure', None),
        LEFloatField('diff_pressure', None),
        LEFloatField('pressure_alt', None),
        LEFloatField('temperature', None),
        XLEShortField('fields_updated', None),
    ]

# MESSAGE ID: 108
class SIMState(Packet):
    '''
    Message ID:   108 -> SIM_STATE

    Status of simulation environment, if used.
    '''
    name = 'SIM_STATE'
    fields_desc = [
        LEFloatField('q1', None),
        LEFloatField('q2', None),
        LEFloatField('q3', None),
        LEFloatField('q4', None),
        LEFloatField('roll', None),
        LEFloatField('pitch', None),
        LEFloatField('yaw', None),
        LEFloatField('xacc', None),
        LEFloatField('yacc', None),
        LEFloatField('zacc', None),
        LEFloatField('xgyro', None),
        LEFloatField('ygyro', None),
        LEFloatField('zgyro', None),
        LEFloatField('lat', None),
        LEFloatField('lon', None),
        LEFloatField('alt', None),
        LEFloatField('std_dev_horz', None),
        LEFloatField('std_dev_vert', None),
        LEFloatField('vn', None),
        LEFloatField('ve', None),
        LEFloatField('vd', None),
    ]

# MESSAGE ID: 109
class RadioStatus(Packet):
    '''
    Message ID:   109 -> RADIO_STATUS

    Status generated by radio and injected into MAVLink stream.
    '''
    name = 'RADIO_STATUS',
    fields_desc = [
        ByteField('rssi', None),
        ByteField('remrssi', None),
        ByteField('txbuf', None),
        ByteField('noise', None),
        ByteField('remnoise', None),
        LEShortField('rxerrors', None),
        LEShortField('fixed', None),
    ]

# MESSAGE ID: 110
class FileTransferProtocol(Packet):
    '''
    Message ID:   100 -> FILE_TRANSFER_PROTOCOL

    File transfer message.
    '''
    name = 'FILE_TRANSFER_PROTOCOL'
    fields_desc = [
        ByteField('target_network', None),
        ByteField('target_system', None),
        ByteField('target_component', None),
        FieldListField('payload', None, ByteField, count_from=lambda pkt: 251),
    ]

# MESSAGE ID: 111
class Timesync(Packet):
    '''
    Message ID:   111 -> TIMESYNC

    Time synchronization message.
    '''
    name = 'TIMESYNC'
    fields_desc = [
        LELongField('tc1', None),
        LELongField('ts1', None),
    ]

# MESSAGE ID: 112
class CameraTrigger(Packet):
    '''
    Message ID:   112 -> CAMERA_TRIGGER

    Camera-IMU triggering and synchronisation message.
    '''
    name = 'CAMERA_TRIGGER'
    fields_desc = [
        LELongField('time_usec', None),
        LEIntField('seq', None),
    ]

# MESSAGE ID: 113
class HILGPS(Packet):
    '''
    Message ID:   113 -> HIL_GPS

    The global position, as returned by the Global Positioning System (GPS).
    This is NOT the global position estimate of the sytem, but rather a RAW sensor value.
    See message GLOBAL_POSITION for the global position estimate.
    '''
    name = 'HIL_GPS'
    fields_desc = [
        LELongField('time_usec', None),
        ByteField('fix_type', None),
        WGS84('lat', None),
        WGS84('lon', None),
        LESignedIntField('alt', None),
        LEShortField('eph', None),
        LEShortField('epv', None),
        LEShortField('vel', None),
        LESignedShortField('vn', None),
        LESignedShortField('ve', None),
        LESignedShortField('vd', None),
        LEShortField('cog', None),
        ByteField('satellites_visible', None),
    ]

# MESSAGE ID: 114
class HILOpticalFlow(Packet):
    '''
    Message ID:   114 -> HIL_OPTICAL_FLOW

    Simulated optical flow from a flow sensor
    (e.g. PX4FLOW or optical mouse sensor)
    '''
    name = 'HIL_OPTICAL_FLOW'
    fields_desc = {
        LELongField('time_usec', None),
        XByteField('sensor_id', None),
        LEIntField('integration_time_us', None),
        LEFloatField('integrated_x', None),
        LEFloatField('integrated_y', None),
        LEFloatField('integrated_xgyro', None),
        LEFloatField('integrated_ygyro', None),
        LEFloatField('integrated_zgyro', None),
        LESignedShortField('temperature', None),
        ByteField('quality', None),
        LEIntField('time_delta_distance_us', None),
        LEFloatField('distance', None),
    }

# MESSAGE ID: 115
class HILStateQuaternion(Packet):
    '''
    Message ID:   115 -> HIL_STATE_QUATERNION

    Sent from simulation to autopilot, avoids in contrast to HIL_STATE singularities.
    This packet is useful for high throughput applications such as hardware in the loop simulations.
    '''
    name = 'HIL_STATE_QUATERNION'
    fields_desc = {
        LELongField('time_usec', None),
        FieldListField('attitude_quaternion', None, LEFloatField, count_from=lambda pkt: 4),
        LEFloatField('rollspeed', None),
        LEFloatField('pitchspeed', None),
        LEFloatField('yawspeed', None),
        WGS84('lat', None),
        WGS84('lat', None),
        LESignedIntField('alt', None),
        LESignedShortField('vx', None),
        LESignedShortField('vy', None),
        LESignedShortField('vz', None),
        LEShortField('ind_airspeed', None),
        LEShortField('true_airspeed', None),
        LESignedShortField('xacc', None),
        LESignedShortField('yacc', None),
        LESignedShortField('zacc', None),
    }

# MESSAGE ID: 116
class ScaledIMU2(Packet):
    '''
    Message ID:   116 -> SCALED_IMU2

    The RAW IMU readings for secondary 9DOF sensor setup.
    This message should contain the scaled values to the described units.
    '''
    name = 'SCALED_IMU2'
    fields_desc = {
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
    }

# MESSAGE ID: 117
class LogRequestList(Packet):
    '''
    Message ID:   117 -> LOG_REQUEST_LIST

    Request a list of available logs.
    On some systems calling this may stop on-board logging until LOG_REQUEST_END is called.
    '''
    name = 'LOG_REQUEST_LIST'
    fields_desc = {
        XByteField('target_system', None),
        XByteField('target_component', None),
        LEShortField('start', None),
        LEShortField('end', None),
    }

# MESSAGE ID: 118
class LogEntry(Packet):
    '''
    Message ID:   118 -> LOG_ENTRY

    Reply to LOG_REQUEST_LIST
    '''
    name = 'LOG_ENTRY'
    fields_desc = {
        LEShortField('id', None),
        LEShortField('num_logs', None),
        LEShortField('last_log_num', None),
        LEIntField('time_utc', None),
        LEIntField('size', None),
    }

# MESSAGE ID: 119
class LogRequestData(Packet):
    '''
    Message ID:   119 -> LOG_REQUEST_DATA

    Request a chunk of a log.
    '''
    name = 'LOG_REQUEST_DATA'
    fields_desc = {
        XByteField('target_system', None),
        XByteField('target_component', None),
        LEShortField('id', None),
        LEIntField('ofs', None),
        LEIntField('count', None),
    }

# MESSAGE ID: 120
class LogData(Packet):
    '''
    Message ID:   120 -> LOG_DATA

    Reply to LOG_REQUEST_DATA.
    '''
    name = 'LOG_DATA'
    fields_desc = {
        LEShortField('id', None),
        LEIntField('ofs', None),
        ByteField('count', None),
        FieldListField('data', None, ByteField, count_from=lambda pkt: 90),
    }

# MESSAGE ID: 121
class LogErase(Packet):
    '''
    Message ID:   121 -> LOG_ERASE

    Erase all logs.
    '''
    name = 'LOG_ERASE'
    fields_desc = {
        XByteField('target_system', None),
        XByteField('target_component', None),
    }

# MESSAGE ID: 122
class LogRequestEnd(Packet):
    '''
    Message ID:   121 -> LOG_REQUEST_END

    Erase all logs.
    '''
    name = 'LOG_REQUEST_END'
    fields_desc = {
        XByteField('target_system', None),
        XByteField('target_component', None),
    }

# MESSAGE ID: 123
class GPSInjectData(Packet):
    '''
    Message ID:   123 -> GPS_INJECT_DATA
    
    Data for injecting into the onboard GPS (used for DGPS)
    '''
    name = 'GPS_INJECT_DATA'
    fields_desc = {
        XByteField('target_system', None),
        XByteField('target_component', None),
        ByteField('len', None),
        FieldListField('data', None, ByteField, count_from=lambda pkt: 110),
    }

# MESSAGE ID: 124
class GPS2Raw(Packet):
    '''
    Message ID:   124 -> GPS2_RAW

    Second GPS data.
    '''
    name = 'GPS2_RAW'
    fields_desc = {
        LELongField('time_usec', None),
        ByteEnumField('fix_type', None, GPS_FIX_TYPE),
        WGS84('lat', None),
        WGS84('lon', None),
        LESignedIntField('alt', None),
        LEShortField('eph', None),
        LEShortField('epv', None),
        LEShortField('vel', None),
        LEShortField('cog', None),
        ByteField('satellites_visible', None),
        ByteField('dgps_numch', None),
        ByteField('dgps_age', None),
    }

# MESSAGE ID: 125
class PowerStatus(Packet):
    '''
    Message ID:   125 -> POWER_STATUS

    Power supply status
    '''
    name = 'POWER_STATUS'
    fields_desc = {
        LEShortField('Vcc', None),
        LEShortField('Vservo', None),
        LEShortField('flags', None),
    }

# MESSAGE ID: 126
class SerialControl(Packet):
    '''
    Message ID:   126 -> SERIAL_CONTROL

    Control a serial port.
    This can be used for raw access to an onboard serial peripheral such as a GPS or telemetry radio.
    It is designed to make it possible to update the devices firmware via MAVLink messages or change the devices settings.
    A message with zero bytes can be used to change just the baudrate.
    '''
    name = 'SERIAL_CONTROL',
    fields_desc = {
        ByteEnumField('device', None, SERIAL_CONTROL_DEV),
        ByteEnumField('flags', None, SERIAL_CONTROL_FLAG),
        LEShortField('timeout', None),
        LEIntField('baudrate', None),
        ByteField('count', None),
        FieldListField('data', None, ByteField, count_from=lambda pkt: 70),
    }

# MESSAGE ID: 127
class GPSRTK(Packet):
    '''
    Message ID:   127 -> GPS_RTK

    RTK GPS data.
    Gives information on the relative baseline calculation the GPS is reporting.
    '''
    name = 'GPS_RTK'
    fields_desc = {
        LEIntField('time_last_baseline_ms', None),
        XByteField('rtk_receiver_id', None),
        LEShortField('wn', None),
        LEIntField('tow', None),
        ByteField('rtk_health', None),
        ByteField('rtk_rate', None),
        ByteField('nsats', None),
        ByteEnumField('baseline_coords_type', None, RTK_BASELINE_COORDINATE_SYSTEM),
        LESignedIntField('baseline_a_mm', None),
        LESignedIntField('baseline_b_mm', None),
        LESignedIntField('baseline_c_mm', None),
        LEIntField('accuracy', None),
        LESignedIntField('iar_num_hypotheses', None),
    }

# MESSAGE ID: 128
class GPS2RTK(Packet):
    '''
    Message ID:   128 -> GPS2_RTK

    RTK GPS data.
    Gives information on the relative baseline calculation the GPS is reporting
    '''
    name = 'GPS2_RTK'
    fields_desc = {
        LEIntField('time_last_baseline_ms', None),
        XByteField('rtk_receiver_id', None),
        LEShortField('wn', None),
        LEIntField('tow', None),
        ByteField('rtk_health', None),
        ByteField('rtk_rate', None),
        ByteField('nsats', None),
        ByteEnumField('baseline_coords_type', None, RTK_BASELINE_COORDINATE_SYSTEM),
        LESignedIntField('baseline_a_mm', None),
        LESignedIntField('baseline_b_mm', None),
        LESignedIntField('baseline_c_mm', None),
        LEIntField('accuracy', None),
        LESignedIntField('iar_num_hypotheses', None),
    }

# MESSAGE ID: 129
class ScaledIMU3(Packet):
    '''
    Message ID:   129 -> SCALED_IMU3

    The RAW IMU readings for 3rd 9DOF sensor setup.
    This message should contain the scaled values to the described units.
    '''
    name = 'SCALED_IMU3'
    fields_desc = {
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
    }

# MESSAGE ID: 130
class DataTransmissionHandshake(Packet):
    '''
    Message ID:   130 -> DATA_TRANSMISSION_HANDSHAKE

    Handshake message to initiate, control and stop image streaming when using the Image Transmission Protocol:
    https://mavlink.io/en/protocol/image_transmission.html.
    '''
    name = 'DATA_TRANSMISSION_PROTOCOL'
    fields_desc = {
        XByteField('type', None),
        LEIntField('soze', None),
        LEShortField('width', None),
        LEShortField('height', None),
        LEShortField('packets', None),
        ByteField('payload', None),
        ByteField('jpg_quality', None),
    }

# MESSAGE ID: 131
class EncapsulatedData(Packet):
    '''
    Message ID:   131 -> ENCAPSULATED_DATA

    Data packet for images sent using the Image Transmission Protocol:
    https://mavlink.io/en/protocol/image_transmission.html.
    '''
    name = 'ENCAPSULATED_DATA'
    fields_desc = {
        LEShortField('seqnr', None),
        FieldListField('data', None, ByteField, count_from=lambda pkt: 253),
    }

# MESSAGE ID: 132
class DistanceSensor(Packet):
    '''
    Message ID:   132 -> DISTANCE_SENSOR

    Distance sensor information for an onboard rangefinder.
    '''
    name = 'DISTANCE_SENSOR'
    fields_desc = {
        LEIntField('time_boot_ms', None),
        LEShortField('min_distance', None),
        LEShortField('max_distance', None),
        LEShortField('current_distance', None),
        ByteEnumField('type', None, MAV_DISTANCE_SENSOR),
        XByteField('id', None),
        ByteEnumField('orientation', None, MAV_SENSOR_ORIENTATION),
        ByteField('covariance', None),
    }

# MESSAGE ID: 133
class TerrainRequest(Packet):
    '''
    Message ID:   133 -> TERRAIN_REQUEST

    Request for terrain data and terrain status.
    '''
    name = 'TERRAIN_REQUEST'
    fields_desc = {
        LESignedIntField('lat', None),
        LESignedIntField('lon', None),
        LEShortField('grid_spacing', None),
        XLELongField('mask', None),
    }

# MESSAGE ID: 134
class TerrainData(Packet):
    '''
    Message ID:   134 -> TERRAIN_DATA

    Terrain data sent from GCS.
    The lat/lon and grid_spacing must be the same as a lat/lon from a TERRAIN_REQUEST.
    '''
    name = 'TERRAIN_DATA'
    fields_desc = {
        LESignedIntField('lat', None),
        LESignedIntField('lon', None),
        LEShortField('grid_spacing', None),
        XByteField('gridbit', None),
        FieldListField('data', None, LESignedShortField, lambda pkt: 16),
    }

# MESSAGE ID: 135
class TerrainCheck(Packet):
    '''
    Message ID:   135 -> TERRAIN_CHECK

    Request that the vehicle report terrain height at the given location.
    Used by GCS to check if vehicle has all terrain data needed for a mission.
    '''
    name = 'TERRAIN_CHECK'
    fields_desc = {
        LESignedIntField('lat', None),
        LESignedIntField('lon', None),
    }

# MESSAGE ID: 136
class TerrainReport(Packet):
    '''
    Message ID:   136 -> TERRAIN_REPORT

    Response from a TERRAIN_CHECK request.
    '''
    name = 'TERRAIN_REPORT'
    fields_desc = {
        LESignedIntField('lat', None),
        LESignedIntField('lon', None),
        LEShortField('spacing', None),
        LEFloatField('terrain_height', None),
        LEFloatField('current_height', None),
        LEShortField('pending', None),
        LEShortField('loaded', None),
    }

# MESSAGE ID: 137
class ScaledPressure2(Packet):
    '''
    Message ID:   137 -> SCALED_PRESSURE2

    Barometer readings for 2nd barometer.
    '''
    name = 'SCALED_PRESSURE2'
    fields_desc = {
        LEIntField('time_boot_ms', None),
        LEFloatField('press_abs', None),
        LEFloatField('press_diff', None),
        LESignedShortField('temperature', None),
    }

# MESSAGE ID: 138
class ATTPosMocap(Packet):
    '''
    Message ID:   138 -> ATT_POS_MOCAP

    Motion capture attitude and position.
    '''
    name = 'ATT_POS_MOCAP'
    fields_desc = {
        LELongField('time_usec', None),
        FieldListField('q', None, LEFloatField, count_from=lambda pkt: 4),
        LEFloatField('x', None),
        LEFloatField('y', None),
        LEFloatField('z', None),
        ConditionalField(FieldListField('covariance', None, LEFloatField, count_from=lambda pkt: 21), lambda pkt: pkt.magic == 0xfd)
    }

# MESSAGE ID: 139
class SetActuatorControlTarget(Packet):
    '''
    Message ID:   139 -> SET_ACTUATOR_CONTROL_TARGET

    Set the vehicle attitude and body angular rates.
    '''
    name = 'SET_ACTUATOR_CONTROL_TARGET'
    fields_desc = {
        LELongField('time_usec', None),
        XByteField('group_mlx', None),
        XByteField('target_system', None),
        XByteField('target_component', None),
        FieldListField('controls', None, LEFloatField, count_from=lambda pkt: 8),
    }

# MESSAGE ID: 140
class ActuatorControlTarget(Packet):
    '''
    Message ID:   140 -> ACTUATOR_CONTROL_TARGET

    Set the vehicle attitude and body angular rates.
    '''
    name = 'ACTUATOR_CONTROL_TARGET'
    fields_desc = {
        LELongField('time_usec', None),
        XByteField('group_mlx', None),
        FieldListField('controls', None, LEFloatField, count_from=lambda pkt: 8),
    }

# MESSAGE ID: 141
class Altitude(Packet):
    '''
    Message ID:   141 -> ALTITUDE

    The current system altitude.
    '''
    name = 'ALTITUDE'
    fields_desc = {
        LELongField('time_usec', None),
        LEFloatField('altitude_monotonic', None),
        LEFloatField('altitude_amsl', None),
        LEFloatField('altitude_local', None),
        LEFloatField('altitude_relative', None),
        LEFloatField('altitude_terrain', None),
        LEFloatField('bottom_clearance', None),
    }

# MESSAGE ID: 142
class ResourceRequest(Packet):
    '''
    Message ID:   142 -> RESOURCE_REQUEST

    The autopilot is requesting a resource (file, binary, other type of data).
    '''
    name = 'RESOURCE_REQUEST'
    fields_desc = {
        XByteField('request_id', None),
        XByteField('uri_type', None),
        FieldListField('uri', None, XByteField, count_from=lambda pkt: 120),
        XByteField('transfer_type', None),
        FieldListField('storage', None, XByteField, count_from=lambda pkt: 120),
    }

# MESSAGE ID: 143
class ScaledPressure3(Packet):
    '''
    Message ID:   143 -> SCALED_PRESSURE3

    Barometer readings for 3rd barometer.
    '''
    name = 'SCALED_PRESSURE3'
    fields_desc = {
        LEIntField('time_boot_ms', None),
        LEFloatField('press_abs', None),
        LEFloatField('press_diff', None),
        LESignedShortField('temperature', None),
    }

# MESSAGE ID: 144
class FollowTarget(Packet):
    '''
    Message ID:   144 -> FOLLOW_TARGET

    Current motion information from a designated system.
    '''
    name = 'FOLLOW_TARGET'
    fields_desc = {
        LELongField('time_usec', None),
        ByteField('est_capabilities', None),
        WGS84('lat', None),
        WGS84('lon', None),
        LEFloatField('alt', None),
        FieldListField('vel', None, LEFloatField, count_from=lambda pkt: 3),
        FieldListField('acc', None, LEFloatField, count_from=lambda pkt: 3),
        FieldListField('attitude_q', None, LEFloatField, count_from=lambda pkt: 4),
        FieldListField('rates', None, LEFloatField, count_from=lambda pkt: 3),
        FieldListField('position_cov', None, LEFloatField, count_from=lambda pkt: 3),
        XLELongField('custom_state', None),
    }

# MESSAGE ID: 146
class ControlSystemState(Packet):
    '''
    Message ID:   146 -> CONTROL_SYSTEM_STATE

    The smoothed, monotonic system state used to feed the control loops of the system.
    '''
    name = 'CONTROL_SYSTEM_STATE'
    fields_desc = {
        LELongField('time_usec', None),
        LEFloatField('x_acc', None),
        LEFloatField('y_acc', None),
        LEFloatField('z_acc', None),
        LEFloatField('x_vel', None),
        LEFloatField('y_vel', None),
        LEFloatField('z_vel', None),
        LEFloatField('x_pos', None),
        LEFloatField('y_pos', None),
        LEFloatField('z_pos', None),
        LEFloatField('airspeed', None),
        FieldListField('vel_variance', None, LEFloatField, count_from=lambda pkt: 3),
        FieldListField('pos_variance', None, LEFloatField, count_from=lambda pkt: 3),
        FieldListField('q', None, LEFloatField, count_from=lambda pkt: 4),
        LEFloatField('roll_rate', None),
        LEFloatField('pitch_rate', None),
        LEFloatField('yaw_rate', None),
    }

# MESSAGE ID: 147
class BatteryStatus(Packet):
    '''
    Message ID:   147 -> BATTERY_STATUS

    Battery information.
    '''
    name = 'BATTERY_STATUS'
    fields_desc = {
        XByteField('id', None),
        ByteEnumField('battery_function', None, MAV_BATTERY_FUNCTION),
        ByteEnumField('type', None, MAV_BATTERY_TYPE),
        LESignedShortField('temperature', None),
        FieldListField('voltages', None, LEShortField, count_from=lambda pkt: 10),
        LESignedShortField('current_battery', None),
        LESignedIntField('current_consumed', None),
        LESignedIntField('energy_consumed', None),
        SignedByteField('battery_remaining', None),
        ConditionalField(LESignedIntField('time_remaining', None), lambda pkt: pkt.magic == 0xfd),
        ConditionalField(ByteEnumField('charge_state', None, MAV_BATTERY_CHARGE_STATE), lambda pkt: pkt.magic == 0xfd),
    }

# MESSAGE ID: 148
class AutopilotVersion(Packet):
    '''
    Message ID:   148 -> AUTOPILOT_VERSION

    Version and capability of autopilot software.
    This should be emitted in response to a MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES command.
    '''
    name = 'AUTOPILOT_VERSION'
    fields_desc = {
        FlagsField('capabilities', 0x0, 64, list(MAV_PROTOCOL_CAPABILITY.values()) + 47*['-']),
        LEIntField('flight_sw_version', None),
        LEIntField('middleware_sw_version', None),
        LEIntField('os_sw_version', None),
        LEIntField('board_version', None),
        FieldListField('flight_custom_version', None, ByteField, count_from=lambda x: 8),
        FieldListField('middleware_custom_version', None, ByteField, count_from=lambda x: 8),
        FieldListField('os_custom_version', None, ByteField, count_from=lambda x: 8),
        LEShortField('vendor_id', 0x0000),
        LEShortField('product_id', 0x0000),
        LELongField('uid', 0x0000000000000000),
        ConditionalField(FieldListField('uid2', None, ByteField, count_from=lambda x: 18), lambda pkt: len(pkt.original) == 78),
    }

# MESSAGE ID: 149
class LandingTarget(Packet):
    '''
    Message ID:   149 -> LANDING_TARGET

    The location of a landing target.
    See: https://mavlink.io/en/services/landing_target.html
    '''

    name = 'LANDING_TARGET'
    fields_desc = {
        LELongField('time_usec', 0x00),
        ByteField('target_num', 0x00),
        ByteEnumField('frame', 0x00, MAV_FRAME),
        LEFloatField('angle_x', 0x00),
        LEFloatField('angle_y', 0x00),
        LEFloatField('distance', 0x00),
        LEFloatField('size_x', 0x00),
        LEFloatField('size_y', 0x00),
        ConditionalField(LEFloatField('x', 0x00), lambda pkt: len(pkt.original) == 60),
        ConditionalField(LEFloatField('y', 0x00), lambda pkt: len(pkt.original) == 60),
        ConditionalField(LEFloatField('z', 0x00), lambda pkt: len(pkt.original) == 60),
        ConditionalField(LEFloatField('q', 0x00), lambda pkt: len(pkt.original) == 60),
        ConditionalField(ByteEnumField('type', 0x00, LANDING_TARGET_TYPE), lambda pkt: len(pkt.original) == 60),
        ConditionalField(ByteField('position_valid', 0x00), lambda pkt: len(pkt.original) == 60),
    }

# MESSAGE ID: 162
class FenceStatus(Packet):
    '''
    Message ID:   162 -> FENCE_STATUS

    Status of geo-fencing.
    Sent in extended status stream when fencing enabled.
    '''

    name = 'FENCE_STATUS'
    fields_desc = {
        ByteField('breach_status', 0x00),
        LEShortField('breach_count', 0x00),
        ByteEnumField('breach_type', 0x00, FENCE_BREACH),
        LEIntField('breach_time', 0x00),
    }

# MESSAGE ID: 230
class EstimatorStatus(Packet):
    '''
    Message ID:   230 -> ESTIMATOR_STATUS

    Estimator status message including flags, innovation test ratios and estimated accuracies.
    The flags message is an integer bitmask containing information on which EKF outputs are valid.
    See the ESTIMATOR_STATUS_FLAGS enum definition for further information.
    The innovation test ratios show the magnitude of the sensor innovation divided by the innovation check threshold.
    Under normal operation the innovation test ratios should be below 0.5 with occasional values up to 1.0.
    Values greater than 1.0 should be rare under normal operation and indicate that a measurement has been rejected by the filter.
    The user should be notified if an innovation test ratio greater than 1.0 is recorded.
    Notifications for values in the range between 0.5 and 1.0 should be optional and controllable by the user.
    '''

    name = 'ESTIMATOR_STATUS'
    fields_desc = {
        LELongField('time_usec', 0x00),
        FlagsField('flags', 0x0000, 16, list(ESTIMATOR_STATUS_FLAGS.values()) + 4*['-']),
        LEFloatField('vel_ratio', 0x00),
        LEFloatField('pos_horiz_ratio', 0x00),
        LEFloatField('pos_vert_ratio', 0x00),
        LEFloatField('mag_ratio', 0x00),
        LEFloatField('hagl_ratio', 0x00),
        LEFloatField('tas_ratio', 0x00),
        LEFloatField('pos_horiz_accuracy', 0x00),
        LEFloatField('pos_vert_accuracy', 0x00),
    }

# MESSAGE ID: 231
class WindCov(Packet):
    '''
    Message ID:   231 -> WIND_COV

    Wind covariance estimate from vehicle.
    '''

    name = 'WIND_COV'
    fields_desc = {
        LELongField('time_usec', 0x00),
        LEFloatField('wind_x', 0x00),
        LEFloatField('wind_y', 0x00),
        LEFloatField('wind_z', 0x00),
        LEFloatField('var_horiz', 0x00),
        LEFloatField('var_vert', 0x00),
        LEFloatField('wind_alt', 0x00),
        LEFloatField('horiz_accuracy', 0x00),
        LEFloatField('vert_accuracy', 0x00),
    }

# MESSAGE ID: 232
class GPSInput(Packet):
    '''
    Message ID:   232 -> GPS_INPUT

    GPS sensor input message.
    This is a raw sensor value sent by the GPS.
    This is NOT the global position estimate of the system.
    '''

    name = 'GPS_INPUT'
    fields_desc = {
        LELongField('time_usec', 0x00),
        ByteField('gps_id', 0x00),
        FlagsField('ignore_flags', 0x0000, 16, list(GPS_INPUT_INGORE_FLAGS.values()) + (16-len(GPS_INPUT_INGORE_FLAGS))*['-']),
        LEIntField('time_week_ms', 0x00),
        LEShortField('time_week', 0x00),
        ByteEnumField('fix_type', 0x00, {0: 'No fix', 1: 'No fix', 2: '2D', 3: '3D', 4: '3D DGPS', 5: '3D RTK'}),
        WGS84('lat', 0x00),
        WGS84('lon', 0x00),
        LEFloatField('alt', 0x00),
        LEFloatField('hdop', 0x00),
        LEFloatField('vdop', 0x00),
        LEFloatField('vn', 0x00),
        LEFloatField('ve', 0x00),
        LEFloatField('vd', 0x00),
        LEFloatField('speed_accuracy', 0x00),
        LEFloatField('horiz_accuracy', 0x00),
        LEFloatField('vert_accuracy', 0x00),
        ByteField('satellites_visible', 0x00),
    }

# MESSAGE ID: 233
class GPSRTCMData(Packet):
    '''
    Message ID:   233 -> GPS_RTCM_DATA

    RTCM message for injecting into the onboard GPS (used for DGPS)
    '''

    name = 'GPS_RTCM_DATA'
    fields_desc = {
        XByteField('flags', 0x00),
        ByteField('len', 0x00),
        FieldListField('data', None, XByteField, count_from=lambda x: 180),
    }

# MESSAGE ID: 234
class HighLatency(Packet):
    '''
    Message ID:   234 -> HIGH_LATENCY

    Message appropriate for high latency connections like Iridium
    '''

    name = 'HIGH_LATENCY'
    fields_desc = {
        FlagsField('base_mode', 0x01, 8, MAV_MODE_FLAG),
        LEIntField('custom_mode', 0x00),
        ByteEnumField('landed_state', 0x00, MAV_LANDED_STATE),
        LESignedShortField('roll', 0x00),
        LESignedShortField('pitch', 0x00),
        LEShortField('heading', 0x00),
        SignedByteField('throttle', 0x00),
        LESignedShortField('heading_sp', 0x00),
        WGS84('latitude', 0x00),
        WGS84('longitude', 0x00),
        LESignedShortField('altitude_amsl', 0x00),
        LESignedShortField('altitude_sp', 0x00),
        ByteField('airspeed', 0x00),
        ByteField('airspeed_sp', 0x00),
        ByteField('groundspeed', 0x00),
        SignedByteField('climb_rate', 0x00),
        ByteField('gps_nsat', 0x00),
        ByteEnumField('gps_fix_type', 0x00, GPS_FIX_TYPE),
        ByteField('battery_remaining', 0x00),
        SignedByteField('temperature', 0x00),
        SignedByteField('temperature_air', 0x00),
        FlagsField('failsafe', 0x00, 8, ['RC', 'batt', 'GPS', 'GCS', 'fence', '-', '-', '-']),
        ByteField('wp_num', 0x00),
        LEShortField('wp_distance', 0x00),
    }

# MESSAGE ID: 241
class Vibration(Packet):
    '''
    Message ID:  241 -> VIBRATION

    Vibration levels and accelerometer clipping
    '''

    name = 'VIBRATION'
    fields_desc = {
        LELongField('time_usec', 0x00),
        LEFloatField('vibration_x', 0x00),
        LEFloatField('vibration_y', 0x00),
        LEFloatField('vibration_z', 0x00),
        LEIntField('clipping_0', 0x00),
        LEIntField('clipping_1', 0x00),
        LEIntField('clipping_2', 0x00),
    }

# MESSAGE ID: 242
class HomePosition(Packet):
    '''
    Message ID:   242 -> HOME_POSITION

    This message can be requested by sending the MAV_CMD_GET_HOME_POSITION command.
    The position the system will return to and land on.
    The position is set automatically by the system during the takeoff in case it was not explicitly set by the operator before or after.
    The position the system will return to and land on.
    The global and local positions encode the position in the respective coordinate frames, while the q parameter encodes the orientation of the surface.
    Under normal conditions it describes the heading and terrain slope, which can be used by the aircraft to adjust the approach.
    The approach 3D vector describes the point to which the system should fly in normal flight mode and then perform a landing sequence along the vector.
    '''

    name = 'HOME_POSITION'
    fields_desc = {
        WGS84('latitude', 0x00),
        WGS84('longitude', 0x00),
        LESignedIntField('altitude', 0x00),
        LEFloatField('x', 0x00),
        LEFloatField('y', 0x00),
        LEFloatField('z', 0x00),
        FieldListField('q', None, LEFloatField, count_from=lambda x: 4),
        LEFloatField('approach_x', 0x00),
        LEFloatField('approach_y', 0x00),
        LEFloatField('approach_z', 0x00),
        ConditionalField(LELongField('time_usec', 0x00), lambda pkt: len(pkt.original) == 60),
    }

# MESSAGE ID: 243
class SetHomePosition(Packet):
    '''
    Message ID:   243 -> SET_HOME_POSITION

    The position the system will return to and land on.
    The position is set automatically by the system during the takeoff in case it was not explicitly set by the operator before or after.
    The global and local positions encode the position in the respective coordinate frames, while the q parameter encodes the orientation of the surface.
    Under normal conditions it describes the heading and terrain slope, which can be used by the aircraft to adjust the approach.
    The approach 3D vector describes the point to which the system should fly in normal flight mode and then perform a landing sequence along the vector.
    '''

    name = 'SET_HOME_POSITION'
    fields_desc = {
        XByteField('target_system', 0x00),
        WGS84('latitude', 0x00),
        WGS84('longitude', 0x00),
        LESignedIntField('altitude', 0x00),
        LEFloatField('x', 0x00),
        LEFloatField('y', 0x00),
        LEFloatField('z', 0x00),
        FieldListField('q', None, LEFloatField, count_from=lambda x: 4),
        LEFloatField('approach_x', 0x00),
        LEFloatField('approach_y', 0x00),
        LEFloatField('approach_z', 0x00),
        ConditionalField(LELongField('time_usec', 0x00), lambda pkt: len(pkt.original) == 61),
    }

# MESSAGE ID: 244
class MessageInterval(Packet):
    '''
    Message ID:   244 -> MESSAGE_INTERVAL

    The interval between messages for a particular MAVLink message ID.
    This message is the response to the MAV_CMD_GET_MESSAGE_INTERVAL command.
    This interface replaces DATA_STREAM.
    '''

    name = 'MESSAGE_INTERVAL'
    fields_desc = {
        LEShortField('message_id', 0x00),
        LESignedIntField('interval_us', 0x00),
    }

# MESSAGE ID: 245
class ExtendedSysState(Packet):
    '''
    Message ID:   245 -> EXTENDED_SYS_STATE

    Provides state for additional features
    '''

    name = 'EXTENDED_SYS_STATE'
    fields_desc = {
        ByteEnumField('vtol_state', 0x00, MAV_VTOL_STATE),
        ByteEnumField('landed_state', 0x00, MAV_LANDED_STATE),
    }

# MESSAGE ID: 246
class ADSBVehicle(Packet):
    '''
    Message ID:   246 -> ADSB_VEHICLE

    The location and information of an ADSB vehicle
    '''

    name = 'ADSB_VEHICLE'
    fields_desc = {
        LEIntField('ICAO_address', 0x00),
        WGS84('latitude', 0x00),
        WGS84('longitude', 0x00),
        ByteEnumField('altitude_type', 0x00, ADSB_ALTITUDE_TYPE),
        LESignedIntField('altitude', 0x00),
        LEShortField('heading', 0x00),
        LEShortField('hor_velocity', 0x00),
        LESignedShortField('ver_velocity', 0x00),
        StrFixedLenField('callsign', '', length=9),
        ByteEnumField('emitter_type', 0x00, ADSB_EMITTER_TYPE),
        ByteField('tslc', 0x00),
        FlagsField('flags', 0x00, 16, list(ADSB_FLAGS.values()) + (16-len(ADSB_FLAGS))*['-']),
        LEShortField('squawk', 0x00),
    }

# MESSAGE ID: 247
class Collision(Packet):
    '''
    Message ID:   247 -> COLLISION

    Information about a potential collision
    '''

    name = 'COLLISION'
    fields_desc = {
        ByteEnumField('src', 0x00, MAV_COLLISION_SRC),
        LEIntField('id', 0x00),
        ByteEnumField('action', 0x00, MAV_COLLISION_ACTION),
        ByteEnumField('threat_level', 0x00, MAV_COLLISION_THREAT_LEVEL),
        LEFloatField('time_to_minimum_delta', 0x00),
        LEFloatField('altitude_minimum_delta', 0x00),
        LEFloatField('horizontal_minimum_delta', 0x00),
    }

# MESSAGE ID: 248
class V2Extension(Packet):
    '''
    Message ID:   248 -> V2_EXTENSION

    Message implementing parts of the V2 payload specs in V1 frames for transitional support.
    '''

    name = 'V2_EXTENSION'
    fields_desc = {
        XByteField('target_network', 0x00),
        XByteField('target_system', 0x00),
        XByteField('target_component', 0x00),
        LEShortField('message_type', 0x00),
        StrFixedLenField('payload', None, length=249),
    }

# MESSAGE ID: 249
class MemoryVect(Packet):
    '''
    Message ID:   249 -> MEMORY_VECT

    Send raw controller memory.
    The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
    '''

    name = 'MEMORY_VECT'
    fields_desc = {
        LEShortField('address', 0x0000),
        ByteField('ver', 0x00),
        ByteField('type', 0x00),
        StrFixedLenField('value', None, length=32),
    }

# MESSAGE ID: 250
class DebugVect(Packet):
    '''
    Message ID:   250 -> DEBUG_VECT

    To debug something using a named 3D vector.
    '''

    name = 'DEBUG_VECT'
    fields_desc = {
        StrFixedLenField('vname', b'\x00'*10, length=10),
        LELongField('time_usec', 0x00),
        LEFloatField('x', 0x00),
        LEFloatField('y', 0x00),
        LEFloatField('z', 0x00)
    }

# MESSAGE ID: 251
class NamedValueFloat(Packet):
    '''
    Message ID:   251 -> NAMED_VALUE_FLOAT

    Send a key-value pair as float.
    The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
    '''

    name = 'NAMED_VALUE_FLOAT'
    fields_desc = {
        LEIntField('time_boot_ms', 0x00),
        StrFixedLenField('fname', b'\x00'*10, length=10),
        LEFloatField('value', 0x00)
    }

# MESSAGE ID: 252
class NamedValueInt(Packet):
    '''
    Message ID:   252 -> NAMED_VALUE_INT

    Send a key-value pair as integer.
    The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
    '''

    name = 'NAMED_VALUE_INT'
    fields_desc = {
        LEIntField('time_boot_ms', 0x00),
        StrFixedLenField('fname', b'\x00'*10, length=10),
        LESignedIntField('value', 0x00)
    }

# MESSAGE ID: 253
class StatusText(Packet):
    '''
    Message ID:   253 -> STATUSTEXT

    Status text message.
    These messages are printed in yellow in the COMM console of QGroundControl.
    
    WARNING: They consume quite some bandwidth, so use only for important status and error messages.
    If implemented wisely, these messages are buffered on the MCU and sent only at a limited rate (e.g. 10 Hz).
    '''

    name = 'STATUSTEXT'
    fields_desc = {
        ByteEnumField('severity', 0x07, MAV_SEVERITY),
        StrFixedLenField('text', b'\x00'*50, length=50)
    }

# MESSAGE ID: 254
class Debug(Packet):
    '''
    Message ID:   254 -> DEBUG

    Send a debug value.
    The index is used to discriminate between values.
    These values show up in the plot of QGroundControl as DEBUG N.
    '''

    name = 'DEBUG'
    fields_desc = {
        LEIntField('time_boot_ms', 0x00),
        ByteField('ind', 0x00),
        LEFloatField('value', 0x00)
    }

# MESSAGE ID: 256
class SetupSigning(Packet):
    '''
    Message ID:   256 -> SETUP_SIGNING

    (MAVLink 2) Setup a MAVLink2 signing key.
    If called with secret_key of all zero and zero initial_timestamp will disable signing.
    '''

    name = 'SETUP_SIGNING'
    fields_desc = {
        XByteField('target_system', 0x00),
        XByteField('target_component', 0x00),
        StrFixedLenField('secret_key', b'\x00'*32, length=32),
        LELongField('initial_timestamp', 0x00)
    }

# MESSAGE ID: 257
class ButtonChange(Packet):
    '''
    Message ID:   257 -> BUTTON_CHANGE

    (MAVLink 2) Report button state change.
    '''

    name = 'BUTTON_CHANGE'
    fields_desc = {
        LEIntField('time_boot_ms', 0x00),
        LEIntField('last_change_ms', 0x00),
        XByteField('state', 0x00)
    }

# MESSAGE ID: 258
class PlayTune(Packet):
    '''
    Message ID:   258 -> PLAY_TUNE

    (MAVLink 2) Control vehicle tone generation (buzzer)
    '''
    name = 'PLAY_TUNE'
    fields_desc = {
        XByteField('target_system', 0x00),
        XByteField('target_component', 0x00),
        TagField('tune', b'\x00'*30, count=30),
        ConditionalField(TagField('tune2', b'\x00'*200, count=200),lambda pkt: len(pkt.original) > 32)
    }

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
    31: AttitudeQuaternion,
    32: LocalPositionNED,
    33: GlobalPositionInt,
    34: RCChannelsScaled,
    35: RCChannelsRaw,
    36: ServoOutputRaw,
    37: MissionRequestPartialList,
    38: MissionWritePartialList,
    39: MissionItem,
    40: MissionRequest,
    41: MissionSetCurrent,
    42: MissionCurrent,
    43: MissionRequestList,
    44: MissionCount,
    45: MissionClearAll,
    46: MissionItemReached,
    47: MissionAck,
    48: SetGPSGlobalOrigin,
    49: GPSGlobalOrigin,
    50: ParamMapRC,
    51: MissionRequestInt,
    54: SafetySetAllowedArea,
    55: SafetyAllowedArea,
    61: AttitudeQuaternionCOV,
    62: NAVControllerOutput,
    63: GlobalPositionIntCOV,
    64: LocalPositionNEDCOV,
    65: RCChannels,
    66: RequestDataStream,
    67: DataStream,
    69: ManualControl,
    70: RCChanelsOverride,
    73: MissionItemInt,
    74: VFRHUD,
    75: CommandInt,
    76: CommandLong,
    77: CommandAck,
    81: ManualSetpoint,
    82: SetAttitudeTarget,
    83: AttitudeTarget,
    84: SetPositionTargetLocalNED,
    85: PositionTargetLocalNED,
    86: SetPositionTargetGlobalInt,
    87: PositionTargetGlobalInt,
    89: LocalPositionNEDSystemGlobalOffset,
    90: HILState,
    91: HILControls,
    92: HILRCInputsRaw,
    93: HILActuatorControls,
    100: OpticalFlow,
    101: GlobalVisionPositionEstimate,
    102: VisionPositionEstimate,
    103: VisionSpeedEstimate,
    104: ViconPositionEstimate,
    105: HighresIMU,
    106: OpticalFlowRAD,
    107: HILSensor,
    108: SIMState,
    109: RadioStatus,
    110: FileTransferProtocol,
    111: Timesync,
    112: CameraTrigger,
    113: HILGPS,
    114: HILOpticalFlow,
    115: HILStateQuaternion,
    116: ScaledIMU2,
    117: LogRequestList,
    118: LogEntry,
    119: LogRequestData,
    120: LogData,
    121: LogErase,
    122: LogRequestEnd,
    123: GPSInjectData,
    124: GPS2Raw,
    125: PowerStatus,
    126: SerialControl,
    127: GPSRTK,
    128: GPS2RTK,
    129: ScaledIMU3,
    130: DataTransmissionHandshake,
    131: EncapsulatedData,
    132: DistanceSensor,
    133: TerrainRequest,
    134: TerrainData,
    135: TerrainCheck,
    136: TerrainReport,
    137: ScaledPressure2,
    138: ATTPosMocap,
    139: SetActuatorControlTarget,
    140: ActuatorControlTarget,
    141: Altitude,
    142: ResourceRequest,
    143: ScaledPressure3,
    144: FollowTarget,
    146: ControlSystemState,
    147: BatteryStatus,
    148: AutopilotVersion,
    149: LandingTarget,
    162: FenceStatus,
    230: EstimatorStatus,
    231: WindCov,
    232: GPSInput,
    233: GPSRTCMData,
    234: HighLatency,
    241: Vibration,
    242: HomePosition,
    243: SetHomePosition,
    244: MessageInterval,
    245: ExtendedSysState,
    246: ADSBVehicle,
    247: Collision,
    248: V2Extension,
    249: MemoryVect,
    250: DebugVect,
    251: NamedValueFloat,
    252: NamedValueInt,
    253: StatusText,
    254: Debug,
    256: SetupSigning,
    257: ButtonChange,
}

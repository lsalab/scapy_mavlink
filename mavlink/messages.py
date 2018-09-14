#!/usr/bin/env python3
'''MAVLink message definitions'''

from scapy.packet import Packet
from scapy.fields import ByteEnumField, ByteField, FieldListField, LEIntField, LELongField, LEShortEnumField, LEShortField, LESignedIntField, SignedByteField, StrFixedLenField, XByteField
from .fields import LEFloatField, LESignedShortField
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
}

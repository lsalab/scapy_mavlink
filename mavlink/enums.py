#!/usr/bin/env python3
'''MAVLink protocol common enumerations'''

ADSB_ALTITUDE_TYPE = {
    0: 'ADSB_ALTITUDE_TYPE_PRESSURE_QNH',
    1: 'ADSB_ALTITUDE_TYPE_GEOMETRIC',
}

ADSB_EMITTER_TYPE = {
    0: 'ADSB_EMITTER_TYPE_NO_INFO',
    1: 'ADSB_EMITTER_TYPE_LIGHT',
    2: 'ADSB_EMITTER_TYPE_SMALL',
    3: 'ADSB_EMITTER_TYPE_LARGE',
    4: 'ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE',
    5: 'ADSB_EMITTER_TYPE_HEAVY',
    6: 'ADSB_EMITTER_TYPE_HIGHLY_MANUV',
    7: 'ADSB_EMITTER_TYPE_ROTOCRAFT',
    8: 'ADSB_EMITTER_TYPE_UNASSIGNED',
    9: 'ADSB_EMITTER_TYPE_GLIDER',
    10: 'ADSB_EMITTER_TYPE_LIGHTER_AIR',
    11: 'ADSB_EMITTER_TYPE_PARACHUTE',
    12: 'ADSB_EMITTER_TYPE_ULTRA_LIGHT',
    13: 'ADSB_EMITTER_TYPE_UNASSIGNED2',
    14: 'ADSB_EMITTER_TYPE_UAV',
    15: 'ADSB_EMITTER_TYPE_SPACE',
    16: 'ADSB_EMITTER_TYPE_UNASSGINED3',
    17: 'ADSB_EMITTER_TYPE_EMERGENCY_SURFACE',
    18: 'ADSB_EMITTER_TYPE_SERVICE_SURFACE',
    19: 'ADSB_EMITTER_TYPE_POINT_OBSTACLE',
}

ADSB_FLAGS = {
    1: 'ADSB_FLAGS_VALID_COORDS',
    2: 'ADSB_FLAGS_VALID_ALTITUDE',
    4: 'ADSB_FLAGS_VALID_HEADING',
    8: 'ADSB_FLAGS_VALID_VELOCITY',
    16: 'ADSB_FLAGS_VALID_CALLSIGN',
    32: 'ADSB_FLAGS_VALID_SQUAWK',
    64: 'ADSB_FLAGS_SIMULATED',
}

CAMERA_CAP_FLAGS = {
    1: 'CAMERA_CAP_FLAGS_CAPTURE_VIDEO',
    2: 'CAMERA_CAP_FLAGS_CAPTURE_IMAGE',
    4: 'CAMERA_CAP_FLAGS_HAS_MODES',
    8: 'CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE',
    16: 'CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE',
    32: 'CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE',
}

CAMERA_MODE = {
    0: 'CAMERA_MODE_IMAGE',
    1: 'CAMERA_MODE_VIDEO',
    2: 'CAMERA_MODE_IMAGE_SURVEY',
}

ESTIMATOR_STATUS_FLAGS = {
    1: 'ESTIMATOR_ATTITUDE',
    2: 'ESTIMATOR_VELOCITY_HORIZ',
    4: 'ESTIMATOR_VELOCITY_VERT',
    8: 'ESTIMATOR_POS_HORIZ_REL',
    16: 'ESTIMATOR_POS_HORIZ_ABS',
    32: 'ESTIMATOR_POS_VERT_ABS',
    64: 'ESTIMATOR_POS_VERT_AGL',
    128: 'ESTIMATOR_CONST_POS_MODE',
    256: 'ESTIMATOR_PRED_POS_HORIZ_REL',
    512: 'ESTIMATOR_PRED_POS_HORIZ_ABS',
    1024: 'ESTIMATOR_GPS_GLITCH',
    2048: 'ESTIMATOR_ACCEL_ERROR',
}

FENCE_ACTION = {
    0: 'FENCE_ACTION_NONE',
    1: 'FENCE_ACTION_GUIDED',
    2: 'FENCE_ACTION_REPORT',
    3: 'FENCE_ACTION_GUIDED_THR_PASS',
    4: 'FENCE_ACTION_RTL',
}

FENCE_BREACH = {
    0: 'FENCE_BREACH_NONE',
    1: 'FENCE_BREACH_MINALT',
    2: 'FENCE_BREACH_MAXALT',
    3: 'FENCE_BREACH_BOUNDARY',
}

FIRMWARE_VERSION_TYPE = {
    0: 'FIRMWARE_VERSION_TYPE_DEV',
    64: 'FIRMWARE_VERSION_TYPE_ALPHA',
    128: 'FIRMWARE_VERSION_TYPE_BETA',
    192: 'FIRMWARE_VERSION_TYPE_RC',
    255: 'FIRMWARE_VERSION_TYPE_OFFICIAL',
}

GPS_FIX_TYPE = {
    0: 'GPS_FIX_TYPE_NO_GPS',
    1: 'GPS_FIX_TYPE_NO_FIX',
    2: 'GPS_FIX_TYPE_2D_FIX',
    3: 'GPS_FIX_TYPE_3D_FIX',
    4: 'GPS_FIX_TYPE_DGPS',
    5: 'GPS_FIX_TYPE_RTK_FLOAT',
    6: 'GPS_FIX_TYPE_RTK_FIXED',
    7: 'GPS_FIX_TYPE_STATIC',
    8: 'GPS_FIX_TYPE_PPP',
}

GPS_INPUT_INGORE_FLAGS = {
    1: 'GPS_INPUT_IGNORE_FLAG_ALT',
    2: 'GPS_INPUT_IGNORE_FLAG_HDOP',
    4: 'GPS_INPUT_IGNORE_FLAG_VDOP',
    8: 'GPS_INPUT_IGNORE_FLAG_VEL_HORIZ',
    16: 'GPS_INPUT_IGNORE_FLAG_VEL_VERT',
    32: 'GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY',
    64: 'GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY',
    128: 'GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY',
}

HL_FAILURE_FLAG = {
    1: 'HL_FAILURE_FLAG_GPS',
    2: 'HL_FAILURE_FLAG_DIFFERENTIAL_PRESSURE',
    4: 'HL_FAILURE_FLAG_ABSOLUTE_PRESSURE',
    8: 'HL_FAILURE_FLAG_3D_ACCEL',
    16: 'HL_FAILURE_FLAG_3D_GYRO',
    32: 'HL_FAILURE_FLAG_3D_MAG',
    64: 'HL_FAILURE_FLAG_TERRAIN',
    128: 'HL_FAILURE_FLAG_BATTERY',
    256: 'HL_FAILURE_FLAG_RC_RECEIVER',
    512: 'HL_FAILURE_FLAG_OFFBOARD_LINK',
    1024: 'HL_FAILURE_FLAG_ENGINE',
    2048: 'HL_FAILURE_FLAG_GEOFENCE',
    4096: 'HL_FAILURE_FLAG_ESTIMATOR',
    8192: 'HL_FAILURE_FLAG_MISSION',
}

LANDING_TARGET_TYPE = {
    0: 'LANDING_TARGET_TYPE_LIGHT_BEACON',
    1: 'LANDING_TARGET_TYPE_RADIO_BEACON',
    2: 'LANDING_TARGET_TYPE_VISION_FIDUCIAL',
    3: 'LANDING_TARGET_TYPE_VISION_OTHER',
}

MAV_ARM_AUTH_DENIED_REASON = {
    0: 'MAV_ARM_AUTH_DENIED_REASON_GENERIC',
    1: 'MAV_ARM_AUTH_DENIED_REASON_NONE',
    2: 'MAV_ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT',
    3: 'MAV_ARM_AUTH_DENIED_REASON_TIMEOUT',
    4: 'MAV_ARM_AUTH_DENIED_REASON_AIRSPACE_IN_USE',
    5: 'MAV_ARM_AUTH_DENIED_REASON_BAD_WEATHER',
}

MAV_AUTOPILOT = {
    0: 'MAV_AUTOPILOT_GENERIC',
    1: 'MAV_AUTOPILOT_RESERVED',
    2: 'MAV_AUTOPILOT_SLUGS',
    3: 'MAV_AUTOPILOT_ARDUPILOTMEGA',
    4: 'MAV_AUTOPILOT_OPENPILOT',
    5: 'MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY',
    6: 'MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY',
    7: 'MAV_AUTOPILOT_GENERIC_MISSION_FULL',
    8: 'MAV_AUTOPILOT_INVALID',
    9: 'MAV_AUTOPILOT_PPZ',
    10: 'MAV_AUTOPILOT_UDB',
    11: 'MAV_AUTOPILOT_FP',
    12: 'MAV_AUTOPILOT_PX4',
    13: 'MAV_AUTOPILOT_SMACCMPILOT',
    14: 'MAV_AUTOPILOT_AUTOQUAD',
    15: 'MAV_AUTOPILOT_ARMAZILA',
    16: 'MAV_AUTOPILOT_AEROB',
    17: 'MAV_AUTOPILOT_ASLUAV',
    18: 'MAV_AUTOPILOT_SMARTAP',
    19: 'MAV_AUTOPILOT_AIRRAILS'
}

MAV_BATTERY_CHARGE_STATE = {
    0: 'MAV_BATTERY_CHARGE_STATE_UNDEFINED',
    1: 'MAV_BATTERY_CHARGE_STATE_OK',
    2: 'MAV_BATTERY_CHARGE_STATE_LOW',
    3: 'MAV_BATTERY_CHARGE_STATE_CRITICAL',
    4: 'MAV_BATTERY_CHARGE_STATE_EMERGENCY',
    5: 'MAV_BATTERY_CHARGE_STATE_FAILED',
    6: 'MAV_BATTERY_CHARGE_STATE_UNHEALTHY',
}

MAV_BATTERY_FUNCTION = {
    0: 'MAV_BATTERY_FUNCTION_UNKNOWN',
    1: 'MAV_BATTERY_FUNCTION_ALL',
    2: 'MAV_BATTERY_FUNCTION_PROPULSION',
    3: 'MAV_BATTERY_FUNCTION_AVIONICS',
}

MAV_BATTERY_TYPE = {
    0: 'MAV_BATTERY_TYPE_UNKNOWN',
    1: 'MAV_BATTERY_TYPE_LIPO',
    2: 'MAV_BATTERY_TYPE_LIFE',
    3: 'MAV_BATTERY_TYPE_LION',
    4: 'MAV_BATTERY_TYPE_NIMH',
    4: 'MAV_BATTERY_TYPE_PAYLOAD',
}

MAV_CMD = {
    16: 'MAV_CMD_NAV_WAYPOINT',
    17: 'MAV_CMD_NAV_LOITER_UNLIM',
    18: 'MAV_CMD_NAV_LOITER_TURNS',
    19: 'MAV_CMD_NAV_LOITER_TIME',
    20: 'MAV_CMD_NAV_RETURN_TO_LAUNCH',
    21: 'MAV_CMD_NAV_LAND',
    22: 'MAV_CMD_NAV_TAKEOFF',
    23: 'MAV_CMD_NAV_LAND_LOCAL',
    24: 'MAV_CMD_NAV_TAKEOFF_LOCAL',
    25: 'MAV_CMD_NAV_FOLLOW',
    30: 'MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT',
    31: 'MAV_CMD_NAV_LOITER_TO_ALT',
    32: 'MAV_CMD_DO_FOLLOW',
    33: 'MAV_CMD_DO_FOLLOW_REPOSITION',
    34: 'MAV_CMD_DO_ORBIT',
    80: 'MAV_CMD_NAV_ROI',
    81: 'MAV_CMD_NAV_PATHPLANNING',
    82: 'MAV_CMD_NAV_SPLINE_WAYPOINT',
    84: 'MAV_CMD_NAV_VTOL_TAKEOFF',
    85: 'MAV_CMD_NAV_VTOL_LAND',
    92: 'MAV_CMD_NAV_GUIDED_ENABLE',
    93: 'MAV_CMD_NAV_DELAY',
    94: 'MAV_CMD_NAV_PAYLOAD_PLACE',
    95: 'MAV_CMD_NAV_LAST',
    112: 'MAV_CMD_CONDITION_DELAY',
    113: 'MAV_CMD_CONDITION_CHANGE_ALT',
    114: 'MAV_CMD_CONDITION_DISTANCE',
    115: 'MAV_CMD_CONDITION_YAW',
    159: 'MAV_CMD_CONDITION_LAST',
    176: 'MAV_CMD_DO_SET_MODE',
    177: 'MAV_CMD_DO_JUMP',
    178: 'MAV_CMD_DO_CHANGE_SPEED',
    179: 'MAV_CMD_DO_SET_HOME',
    180: 'MAV_CMD_DO_SET_PARAMETER',
    181: 'MAV_CMD_DO_SET_RELAY',
    182: 'MAV_CMD_DO_REPEAT_RELAY',
    183: 'MAV_CMD_DO_SET_SERVO',
    184: 'MAV_CMD_DO_REPEAT_SERVO',
    185: 'MAV_CMD_DO_FLIGHTTERMINATION',
    186: 'MAV_CMD_DO_CHANGE_ALTITUDE',
    189: 'MAV_CMD_DO_LAND_START',
    190: 'MAV_CMD_DO_RALLY_LAND',
    191: 'MAV_CMD_DO_GO_AROUND',
    192: 'MAV_CMD_DO_REPOSITION',
    193: 'MAV_CMD_DO_PAUSE_CONTINUE',
    194: 'MAV_CMD_DO_SET_REVERSE',
    195: 'MAV_CMD_DO_SET_ROI_LOCATION',
    196: 'MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET',
    197: 'MAV_CMD_DO_SET_ROI_NONE',
    200: 'MAV_CMD_DO_CONTROL_VIDEO',
    201: 'MAV_CMD_DO_SET_ROI',
    202: 'MAV_CMD_DO_DIGICAM_CONFIGURE',
    203: 'MAV_CMD_DO_DIGICAM_CONTROL',
    204: 'MAV_CMD_DO_MOUNT_CONFIGURE',
    205: 'MAV_CMD_DO_MOUNT_CONTROL',
    206: 'MAV_CMD_DO_SET_CAM_TRIGG_DIST',
    207: 'MAV_CMD_DO_FENCE_ENABLE',
    208: 'MAV_CMD_DO_PARACHUTE',
    209: 'MAV_CMD_DO_MOTOR_TEST',
    210: 'MAV_CMD_DO_INVERTED_FLIGHT',
    213: 'MAV_CMD_NAV_SET_YAW_SPEED',
    214: 'MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL',
    220: 'MAV_CMD_DO_MOUNT_CONTROL_QUAT',
    221: 'MAV_CMD_DO_GUIDED_MASTER',
    222: 'MAV_CMD_DO_GUIDED_LIMITS',
    223: 'MAV_CMD_DO_ENGINE_CONTROL',
    224: 'MAV_CMD_DO_SET_MISSION_CURRENT',
    240: 'MAV_CMD_DO_LAST',
    241: 'MAV_CMD_PREFLIGHT_CALIBRATION',
    242: 'MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS',
    243: 'MAV_CMD_PREFLIGHT_UAVCAN',
    245: 'MAV_CMD_PREFLIGHT_STORAGE',
    246: 'MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN',
    252: 'MAV_CMD_OVERRIDE_GOTO',
    300: 'MAV_CMD_MISSION_START',
    400: 'MAV_CMD_COMPONENT_ARM_DISARM',
    410: 'MAV_CMD_GET_HOME_POSITION',
    500: 'MAV_CMD_START_RX_PAIR',
    510: 'MAV_CMD_GET_MESSAGE_INTERVAL',
    511: 'MAV_CMD_SET_MESSAGE_INTERVAL',
    519: 'MAV_CMD_REQUEST_PROTOCOL_VERSION',
    520: 'MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES',
    521: 'MAV_CMD_REQUEST_CAMERA_INFORMATION',
    522: 'MAV_CMD_REQUEST_CAMERA_SETTINGS',
    525: 'MAV_CMD_REQUEST_STORAGE_INFORMATION',
    526: 'MAV_CMD_STORAGE_FORMAT',
    527: 'MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS',
    528: 'MAV_CMD_REQUEST_FLIGHT_INFORMATION',
    529: 'MAV_CMD_RESET_CAMERA_SETTINGS',
    530: 'MAV_CMD_SET_CAMERA_MODE',
    2000: 'MAV_CMD_IMAGE_START_CAPTURE',
    2001: 'MAV_CMD_IMAGE_STOP_CAPTURE',
    2002: 'MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE',
    2003: 'MAV_CMD_DO_TRIGGER_CONTROL',
    2500: 'MAV_CMD_VIDEO_START_CAPTURE',
    2501: 'MAV_CMD_VIDEO_STOP_CAPTURE',
    2502: 'MAV_CMD_VIDEO_START_STREAMING',
    2503: 'MAV_CMD_VIDEO_STOP_STREAMING',
    2504: 'MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION',
    2510: 'MAV_CMD_LOGGING_START',
    2511: 'MAV_CMD_LOGGING_STOP',
    2520: 'MAV_CMD_AIRFRAME_CONFIGURATION',
    2600: 'MAV_CMD_CONTROL_HIGH_LATENCY',
    2800: 'MAV_CMD_PANORAMA_CREATE',
    3000: 'MAV_CMD_DO_VTOL_TRANSITION',
    3001: 'MAV_CMD_ARM_AUTHORIZATION_REQUEST',
    4000: 'MAV_CMD_SET_GUIDED_SUBMODE_STANDARD',
    4001: 'MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE',
    4501: 'MAV_CMD_CONDITION_GATE',
    5000: 'MAV_CMD_NAV_FENCE_RETURN_POINT',
    5001: 'MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION',
    5002: 'MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION',
    5003: 'MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION',
    5004: 'MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION',
    5100: 'MAV_CMD_NAV_RALLY_POINT',
    5200: 'MAV_CMD_UAVCAN_GET_NODE_INFO',
    30001: 'MAV_CMD_PAYLOAD_PREPARE_DEPLOY',
    30002: 'MAV_CMD_PAYLOAD_CONTROL_DEPLOY',
    31000: 'MAV_CMD_WAYPOINT_USER_1',
    31001: 'MAV_CMD_WAYPOINT_USER_2',
    31002: 'MAV_CMD_WAYPOINT_USER_3',
    31003: 'MAV_CMD_WAYPOINT_USER_4',
    31004: 'MAV_CMD_WAYPOINT_USER_5',
    31005: 'MAV_CMD_SPATIAL_USER_1',
    31006: 'MAV_CMD_SPATIAL_USER_2',
    31007: 'MAV_CMD_SPATIAL_USER_3',
    31008: 'MAV_CMD_SPATIAL_USER_4',
    31009: 'MAV_CMD_SPATIAL_USER_5',
    31010: 'MAV_CMD_USER_1',
    31011: 'MAV_CMD_USER_2',
    31012: 'MAV_CMD_USER_3',
    31013: 'MAV_CMD_USER_4',
    31014: 'MAV_CMD_USER_5',
}

MAV_CMD_ACK = set([
    'MAV_CMD_ACK_OK',
    'MAV_CMD_ACK_ERR_FAIL',
    'MAV_CMD_ACK_ERR_ACCESS_DENIED',
    'MAV_CMD_ACK_ERR_NOT_SUPPORTED',
    'MAV_CMD_ACK_ERR_COORDINATE_FRAME_NOT_SUPPORTED',
    'MAV_CMD_ACK_ERR_COORDINATES_OUT_OF_RANGE',
    'MAV_CMD_ACK_ERR_X_LAT_OUT_OF_RANGE',
    'MAV_CMD_ACK_ERR_Y_LON_OUT_OF_RANGE',
    'MAV_CMD_ACK_ERR_Z_ALT_OUT_OF_RANGE',
])

MAV_COLLISION_ACTION = {
    0: 'MAV_COLLISION_ACTION_NONE',
    1: 'MAV_COLLISION_ACTION_REPORT',
    2: 'MAV_COLLISION_ACTION_ASCEND_OR_DESCEND',
    3: 'MAV_COLLISION_ACTION_MOVE_HORIZONTALLY',
    4: 'MAV_COLLISION_ACTION_MOVE_PERPENDICULAR',
    5: 'MAV_COLLISION_ACTION_RTL',
    6: 'MAV_COLLISION_ACTION_HOVER',
}

MAV_COLLISION_SRC = {
    0: 'MAV_COLLISION_SRC_ADSB',
    1: 'MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT',
}

MAV_COLLISION_THREAT_LEVEL = {
    0: 'MAV_COLLISION_THREAT_LEVEL_NONE',
    1: 'MAV_COLLISION_THREAT_LEVEL_LOW',
    2: 'MAV_COLLISION_THREAT_LEVEL_HIGH',
}

MAV_COMPONENT = {
    0: 'MAV_COMP_ID_ALL',
    1: 'MAV_COMP_ID_AUTOPILOT1',
    100: 'MAV_COMP_ID_CAMERA',
    101: 'MAV_COMP_ID_CAMERA2',
    102: 'MAV_COMP_ID_CAMERA3',
    103: 'MAV_COMP_ID_CAMERA4',
    104: 'MAV_COMP_ID_CAMERA5',
    105: 'MAV_COMP_ID_CAMERA6',
    140: 'MAV_COMP_ID_SERVO1',
    141: 'MAV_COMP_ID_SERVO2',
    142: 'MAV_COMP_ID_SERVO3',
    143: 'MAV_COMP_ID_SERVO4',
    144: 'MAV_COMP_ID_SERVO5',
    145: 'MAV_COMP_ID_SERVO6',
    146: 'MAV_COMP_ID_SERVO7',
    147: 'MAV_COMP_ID_SERVO8',
    148: 'MAV_COMP_ID_SERVO9',
    149: 'MAV_COMP_ID_SERVO10',
    150: 'MAV_COMP_ID_SERVO11',
    151: 'MAV_COMP_ID_SERVO12',
    152: 'MAV_COMP_ID_SERVO13',
    153: 'MAV_COMP_ID_SERVO14',
    154: 'MAV_COMP_ID_GIMBAL',
    155: 'MAV_COMP_ID_LOG',
    156: 'MAV_COMP_ID_ADSB',
    157: 'MAV_COMP_ID_OSD',
    158: 'MAV_COMP_ID_PERIPHERAL',
    159: 'MAV_COMP_ID_QX1_GIMBAL',
    160: 'MAV_COMP_ID_FLARM',
    180: 'MAV_COMP_ID_MAPPER',
    190: 'MAV_COMP_ID_MISSIONPLANNER',
    195: 'MAV_COMP_ID_PATHPLANNER',
    200: 'MAV_COMP_ID_IMU',
    201: 'MAV_COMP_ID_IMU_2',
    202: 'MAV_COMP_ID_IMU_3',
    220: 'MAV_COMP_ID_GPS',
    221: 'MAV_COMP_ID_GPS2',
    240: 'MAV_COMP_ID_UDP_BRIDGE',
    241: 'MAV_COMP_ID_UART_BRIDGE',
    250: 'MAV_COMP_ID_SYSTEM_CONTROL',
}

MAV_DATA_STREAM = {
    0: 'MAV_DATA_STREAM_ALL',
    1: 'MAV_DATA_STREAM_RAW_SENSORS',
    2: 'MAV_DATA_STREAM_EXTENDED_STATUS',
    3: 'MAV_DATA_STREAM_RC_CHANNELS',
    4: 'MAV_DATA_STREAM_RAW_CONTROLLER',
    6: 'MAV_DATA_STREAM_POSITION',
    10: 'MAV_DATA_STREAM_EXTRA1',
    11: 'MAV_DATA_STREAM_EXTRA2',
    12: 'MAV_DATA_STREAM_EXTRA3',
}

MAV_DISTANCE_SENSOR = {
    0: 'MAV_DISTANCE_SENSOR_LASER',
    1: 'MAV_DISTANCE_SENSOR_ULTRASOUND',
    2: 'MAV_DISTANCE_SENSOR_INFRARED',
    3: 'MAV_DISTANCE_SENSOR_RADAR',
    4: 'MAV_DISTANCE_SENSOR_UNKNOWN',
}

MAV_DO_REPOSITION_FLAGS = {
    1: 'MAV_DO_REPOSITION_FLAGS_CHANGE_MODE',
}

MAV_ESTIMATOR_TYPE = {
    1: 'MAV_ESTIMATOR_TYPE_NAIVE',
    2: 'MAV_ESTIMATOR_TYPE_VISION',
    3: 'MAV_ESTIMATOR_TYPE_VIO',
    4: 'MAV_ESTIMATOR_TYPE_GPS',
    5: 'MAV_ESTIMATOR_TYPE_GPS_INS',
}

MAV_FRAME = {
    0: 'MAV_FRAME_GLOBAL',
    1: 'MAV_FRAME_LOCAL_NED',
    2: 'MAV_FRAME_MISSION',
    3: 'MAV_FRAME_GLOBAL_RELATIVE_ALT',
    4: 'MAV_FRAME_LOCAL_ENU',
    5: 'MAV_FRAME_GLOBAL_INT',
    6: 'MAV_FRAME_GLOBAL_RELATIVE_ALT_INT',
    7: 'MAV_FRAME_LOCAL_OFFSET_NED',
    8: 'MAV_FRAME_BODY_NED',
    9: 'MAV_FRAME_BODY_OFFSET_NED',
    10: 'MAV_FRAME_GLOBAL_TERRAIN_ALT',
    11: 'MAV_FRAME_GLOBAL_TERRAIN_ALT_INT',
    12: 'MAV_FRAME_BODY_FRD',
    13: 'MAV_FRAME_BODY_FLU',
    14: 'MAV_FRAME_MOCAP_NED',
    15: 'MAV_FRAME_MOCAP_ENU',
    16: 'MAV_FRAME_VISION_NED',
    17: 'MAV_FRAME_VISION_ENU',
    18: 'MAV_FRAME_ESTIM_NED',
    19: 'MAV_FRAME_ESTIM_ENU',
}

MAV_GOTO = {
    0: 'MAV_GOTO_DO_HOLD',
    1: 'MAV_GOTO_DO_CONTINUE',
    2: 'MAV_GOTO_HOLD_AT_CURRENT_POSITION',
    3: 'MAV_GOTO_HOLD_AT_SPECIFIED_POSITION',
}

MAV_LANDED_STATE = {
    0: 'MAV_LANDED_STATE_UNDEFINED',
    1: 'MAV_LANDED_STATE_ON_GROUND',
    2: 'MAV_LANDED_STATE_IN_AIR',
    3: 'MAV_LANDED_STATE_TAKEOFF',
    4: 'MAV_LANDED_STATE_LANDING',
}

MAV_MISSION_RESULT = {
    0: 'MAV_MISSION_ACCEPTED',
    1: 'MAV_MISSION_ERROR',
    2: 'MAV_MISSION_UNSUPPORTED_FRAME',
    3: 'MAV_MISSION_UNSUPPORTED',
    4: 'MAV_MISSION_NO_SPACE',
    5: 'MAV_MISSION_INVALID',
    6: 'MAV_MISSION_INVALID_PARAM1',
    7: 'MAV_MISSION_INVALID_PARAM2',
    8: 'MAV_MISSION_INVALID_PARAM3',
    9: 'MAV_MISSION_INVALID_PARAM4',
    10: 'MAV_MISSION_INVALID_PARAM5_X',
    11: 'MAV_MISSION_INVALID_PARAM6_Y',
    12: 'MAV_MISSION_INVALID_PARAM7',
    13: 'MAV_MISSION_INVALID_SEQUENCE',
    14: 'MAV_MISSION_DENIED',
}

MAV_MISSION_TYPE = {
    0: 'MAV_MISSION_TYPE_MISSION',
    1: 'MAV_MISSION_TYPE_FENCE',
    2: 'MAV_MISSION_TYPE_RALLY',
    255: 'MAV_MISSION_TYPE_ALL',
}

MAV_MODE = {
    0: 'MAV_MODE_PREFLIGHT',
    64: 'MAV_MODE_MANUAL_DISARMED',
    66: 'MAV_MODE_TEST_DISARMED',
    80: 'MAV_MODE_STABILIZE_DISARMED',
    88: 'MAV_MODE_GUIDED_DISARMED',
    92: 'MAV_MODE_AUTO_DISARMED',
    192: 'MAV_MODE_MANUAL_ARMED',
    194: 'MAV_MODE_TEST_ARMED',
    208: 'MAV_MODE_STABILIZE_ARMED',
    216: 'MAV_MODE_GUIDED_ARMED',
    220: 'MAV_MODE_AUTO_ARMED',
}

MAV_MODE_FLAG = {
    1: 'MAV_MODE_FLAG_CUSTOM_MODE_ENABLED',
    2: 'MAV_MODE_FLAG_TEST_ENABLED',
    4: 'MAV_MODE_FLAG_AUTO_ENABLED',
    8: 'MAV_MODE_FLAG_GUIDED_ENABLED',
    16: 'MAV_MODE_FLAG_STABILIZE_ENABLED',
    32: 'MAV_MODE_FLAG_HIL_ENABLED',
    64: 'MAV_MODE_FLAG_MANUAL_INPUT_ENABLED',
    128: 'MAV_MODE_FLAG_SAFETY_ARMED',
}

MAV_MODE_FLAG_DECODE_POSITION = {
    1: 'MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE',
    2: 'MAV_MODE_FLAG_DECODE_POSITION_TEST',
    4: 'MAV_MODE_FLAG_DECODE_POSITION_AUTO',
    8: 'MAV_MODE_FLAG_DECODE_POSITION_GUIDED',
    16: 'MAV_MODE_FLAG_DECODE_POSITION_STABILIZE',
    32: 'MAV_MODE_FLAG_DECODE_POSITION_HIL',
    64: 'MAV_MODE_FLAG_DECODE_POSITION_MANUAL',
    128: 'MAV_MODE_FLAG_DECODE_POSITION_SAFETY',
}

MAV_MOUNT_MODE = {
    0: 'MAV_MOUNT_MODE_RETRACT',
    1: 'MAV_MOUNT_MODE_NEUTRAL',
    2: 'MAV_MOUNT_MODE_MAVLINK_TARGETING',
    3: 'MAV_MOUNT_MODE_RC_TARGETING',
    4: 'MAV_MOUNT_MODE_GPS_POINT',
}

MAV PARAM_EXT_TYPE = {
    1: 'MAV_PARAM_EXT_TYPE_UINT8',
    2: 'MAV_PARAM_EXT_TYPE_INT8',
    3: 'MAV_PARAM_EXT_TYPE_UINT16',
    4: 'MAV_PARAM_EXT_TYPE_INT16',
    5: 'MAV_PARAM_EXT_TYPE_UINT32',
    6: 'MAV_PARAM_EXT_TYPE_INT32',
    7: 'MAV_PARAM_EXT_TYPE_UINT64',
    8: 'MAV_PARAM_EXT_TYPE_INT64',
    9: 'MAV_PARAM_EXT_TYPE_REAL32',
    10: 'MAV_PARAM_EXT_TYPE_REAL64',
    11: 'MAV_PARAM_EXT_TYPE_CUSTOM',
}

MAV_PARAM_TYPE = {
    1: 'MAV_PARAM_TYPE_UINT8',
    2: 'MAV_PARAM_TYPE_INT8',
    3: 'MAV_PARAM_TYPE_UINT16',
    4: 'MAV_PARAM_TYPE_INT16',
    5: 'MAV_PARAM_TYPE_UINT32',
    6: 'MAV_PARAM_TYPE_INT32',
    7: 'MAV_PARAM_TYPE_UINT64',
    8: 'MAV_PARAM_TYPE_INT64',
    9: 'MAV_PARAM_TYPE_REAL32',
    10: 'MAV_PARAM_TYPE_REAL64',
}

MAV_POWER_STATUS = {
    1: 'MAV_POWER_STATUS_BRICK_VALID',
    2: 'MAV_POWER_STATUS_SERVO_VALID',
    4: 'MAV_POWER_STATUS_USB_CONNECTED',
    8: 'MAV_POWER_STATUS_PERIPH_OVERCURRENT',
    16: 'MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT',
    32: 'MAV_POWER_STATUS_CHANGED',
}

MAV_PROTOCOL_CAPABILITY = {
    1: 'MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT',
    2: 'MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT',
    4: 'MAV_PROTOCOL_CAPABILITY_MISSION_INT',
    8: 'MAV_PROTOCOL_CAPABILITY_COMMAND_INT',
    16: 'MAV_PROTOCOL_CAPABILITY_PARAM_UNION',
    32: 'MAV_PROTOCOL_CAPABILITY_FTP',
    64: 'MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET',
    128: 'MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED',
    256: 'MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT',
    512: 'MAV_PROTOCOL_CAPABILITY_TERRAIN',
    1024: 'MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET',
    2048: 'MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION',
    4096: 'MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION',
    8192: 'MAV_PROTOCOL_CAPABILITY_MAVLINK2',
    16384: 'MAV_PROTOCOL_CAPABILITY_MISSION_FENCE',
    32768: 'MAV_PROTOCOL_CAPABILITY_MISSION_RALLY',
    65536: 'MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION',
}

MAV_RESULT = {
    0: 'MAV_RESULT_ACCEPTED',
    1: 'MAV_RESULT_TEMPORARILY_REJECTED',
    2: 'MAV_RESULT_DENIED',
    3: 'MAV_RESULT_UNSUPPORTED',
    4: 'MAV_RESULT_FAILED',
    5: 'MAV_RESULT_IN_PROGRESS',
}

MAV_ROI = {
    0: 'MAV_ROI_NONE',
    1: 'MAV_ROI_WPNEXT',
    2: 'MAV_ROI_WPINDEX',
    3: 'MAV_ROI_LOCATION',
    4: 'MAV_ROI_TARGET',
}

MAV_SENSOR_ORIENTATION = {
    0: 'MAV_SENSOR_ROTATION_NONE',
    1: 'MAV_SENSOR_ROTATION_YAW_45',
    2: 'MAV_SENSOR_ROTATION_YAW_90',
    3: 'MAV_SENSOR_ROTATION_YAW_135',
    4: 'MAV_SENSOR_ROTATION_YAW_180',
    5: 'MAV_SENSOR_ROTATION_YAW_225',
    6: 'MAV_SENSOR_ROTATION_YAW_270',
    7: 'MAV_SENSOR_ROTATION_YAW_315',
    8: 'MAV_SENSOR_ROTATION_ROLL_180',
    9: 'MAV_SENSOR_ROTATION_ROLL_180_YAW_45',
    10: 'MAV_SENSOR_ROTATION_ROLL_180_YAW_90',
    11: 'MAV_SENSOR_ROTATION_ROLL_180_YAW_135',
    12: 'MAV_SENSOR_ROTATION_PITCH_180',
    13: 'MAV_SENSOR_ROTATION_ROLL_180_YAW_225',
    14: 'MAV_SENSOR_ROTATION_ROLL_180_YAW_270',
    15: 'MAV_SENSOR_ROTATION_ROLL_180_YAW_315',
    16: 'MAV_SENSOR_ROTATION_ROLL_90',
    17: 'MAV_SENSOR_ROTATION_ROLL_90_YAW_45',
    18: 'MAV_SENSOR_ROTATION_ROLL_90_YAW_90',
    19: 'MAV_SENSOR_ROTATION_ROLL_90_YAW_135',
    20: 'MAV_SENSOR_ROTATION_ROLL_270',
    21: 'MAV_SENSOR_ROTATION_ROLL_270_YAW_45',
    22: 'MAV_SENSOR_ROTATION_ROLL_270_YAW_90',
    23: 'MAV_SENSOR_ROTATION_ROLL_270_YAW_135',
    24: 'MAV_SENSOR_ROTATION_PITCH_90',
    25: 'MAV_SENSOR_ROTATION_PITCH_270',
    26: 'MAV_SENSOR_ROTATION_PITCH_180_YAW_90',
    27: 'MAV_SENSOR_ROTATION_PITCH_180_YAW_270',
    28: 'MAV_SENSOR_ROTATION_ROLL_90_PITCH_90',
    29: 'MAV_SENSOR_ROTATION_ROLL_180_PITCH_90',
    30: 'MAV_SENSOR_ROTATION_ROLL_270_PITCH_90',
    31: 'MAV_SENSOR_ROTATION_ROLL_90_PITCH_180',
    32: 'MAV_SENSOR_ROTATION_ROLL_270_PITCH_180',
    33: 'MAV_SENSOR_ROTATION_ROLL_90_PITCH_270',
    34: 'MAV_SENSOR_ROTATION_ROLL_180_PITCH_270',
    35: 'MAV_SENSOR_ROTATION_ROLL_270_PITCH_270',
    36: 'MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90',
    37: 'MAV_SENSOR_ROTATION_ROLL_90_YAW_270',
    38: 'MAV_SENSOR_ROTATION_ROLL_90_PITCH_68_YAW_293',
    39: 'MAV_SENSOR_ROTATION_PITCH_315',
    40: 'MAV_SENSOR_ROTATION_ROLL_90_PITCH_315',
    100: 'MAV_SENSOR_ROTATION_CUSTOM',
}

MAV_SEVERITY = {
    0: 'MAV_SEVERITY_EMERGENCY',
    1: 'MAV_SEVERITY_ALERT',
    2: 'MAV_SEVERITY_CRITICAL',
    3: 'MAV_SEVERITY_ERROR',
    4: 'MAV_SEVERITY_WARNING',
    5: 'MAV_SEVERITY_NOTICE',
    6: 'MAV_SEVERITY_INFO',
    7: 'MAV_SEVERITY_DEBUG',
}

MAV_STATE = {
    0: 'MAV_STATE_UNINIT',
    1: 'MAV_STATE_BOOT',
    2: 'MAV_STATE_CALIBRATING',
    3: 'MAV_STATE_STANDBY',
    4: 'MAV_STATE_ACTIVE',
    5: 'MAV_STATE_CRITICAL',
    6: 'MAV_STATE_EMERGENCY',
    7: 'MAV_STATE_POWEROFF',
    8: 'MAV_STATE_FLIGHT_TERMINATION',
}

MAV_SYS_STATUS_SENSOR = {
    1: 'MAV_SYS_STATUS_SENSOR_3D_GYRO',
    2: 'MAV_SYS_STATUS_SENSOR_3D_ACCEL',
    4: 'MAV_SYS_STATUS_SENSOR_3D_MAG',
    8: 'MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE',
    16: 'MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE',
    32: 'MAV_SYS_STATUS_SENSOR_GPS',
    64: 'MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW',
    128: 'MAV_SYS_STATUS_SENSOR_VISION_POSITION',
    256: 'MAV_SYS_STATUS_SENSOR_LASER_POSITION',
    512: 'MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH',
    1024: 'MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL',
    2048: 'MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION',
    4096: 'MAV_SYS_STATUS_SENSOR_YAW_POSITION',
    8192: 'MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL',
    16384: 'MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL',
    32768: 'MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS',
    65536: 'MAV_SYS_STATUS_SENSOR_RC_RECEIVER',
    131072: 'MAV_SYS_STATUS_SENSOR_3D_GYRO2',
    262144: 'MAV_SYS_STATUS_SENSOR_3D_ACCEL2',
    524288: 'MAV_SYS_STATUS_SENSOR_3D_MAG2',
    33554432: 'MAV_SYS_STATUS_SENSOR_BATTERY',
    67108864: 'MAV_SYS_STATUS_SENSOR_PROXIMITY',
    134217728: 'MAV_SYS_STATUS_SENSOR_SATCOM',
}

MAV_TYPE = {
    0: 'MAV_TYPE_GENERIC',
    1: 'MAV_TYPE_FIXED_WING',
    2: 'MAV_TYPE_QUADROTOR',
    3: 'MAV_TYPE_COAXIAL',
    4: 'MAV_TYPE_HELICOPTER',
    5: 'MAV_TYPE_ANTENNA_TRACKER',
    6: 'MAV_TYPE_GCS',
    7: 'MAV_TYPE_AIRSHIP',
    8: 'MAV_TYPE_FREE_BALLOON',
    9: 'MAV_TYPE_ROCKET',
    10: 'MAV_TYPE_GROUND_ROVER',
    11: 'MAV_TYPE_SURFACE_BOAT',
    12: 'MAV_TYPE_SUBMARINE',
    13: 'MAV_TYPE_HEXAROTOR',
    14: 'MAV_TYPE_OCTOROTOR',
    15: 'MAV_TYPE_TRICOPTER',
    16: 'MAV_TYPE_FLAPPING_WING',
    17: 'MAV_TYPE_KITE',
    18: 'MAV_TYPE_ONBOARD_CONTROLLER',
    19: 'MAV_TYPE_VTOL_DUOROTOR',
    20: 'MAV_TYPE_VTOL_QUADROTOR',
    21: 'MAV_TYPE_VTOL_TILTROTOR',
    22: 'MAV_TYPE_VTOL_RESERVED2',
    23: 'MAV_TYPE_VTOL_RESERVED3',
    24: 'MAV_TYPE_VTOL_RESERVED4',
    25: 'MAV_TYPE_VTOL_RESERVED5',
    26: 'MAV_TYPE_GIMBAL',
    27: 'MAV_TYPE_ADSB',
    28: 'MAV_TYPE_PARAFOIL',
    29: 'MAV_TYPE_DODECAROTOR',
    30: 'MAV_TYPE_CAMERA',
    31: 'MAV_TYPE_CHARGING_STATION',
    32: 'MAV_TYPE_FLARM',
}

MAV_VTOL_STATE = {
    0: 'MAV_VTOL_STATE_UNDEFINED',
    1: 'MAV_VTOL_STATE_TRANSITION_TO_FW',
    2: 'MAV_VTOL_STATE_TRANSITION_TO_MC',
    3: 'MAV_VTOL_STATE_MC',
    4: 'MAV_VTOL_STATE_FW',
}

MAVLINK_DATA_STREAM_TYPE = set([
    'MAVLINK_DATA_STREAM_IMG_BMP',
    'MAVLINK_DATA_STREAM_IMG_JPEG',
    'MAVLINK_DATA_STREAM_IMG_PGM',
    'MAVLINK_DATA_STREAM_IMG_PNG',
    'MAVLINK_DATA_STREAM_IMG_RAW32U',
    'MAVLINK_DATA_STREAM_IMG_RAW8U',
])

MOTOR_TEST_ORDER = {
    0: 'MOTOR_TEST_ORDER_DEFAULT',
    1: 'MOTOR_TEST_ORDER_SEQUENCE',
    2: 'MOTOR_TEST_ORDER_BOARD',
}

MOTOR_TEST_THROTTLE_TYPE = {
    0: 'MOTOR_TEST_THROTTLE_PERCENT',
    1: 'MOTOR_TEST_THROTTLE_PWM',
    2: 'MOTOR_TEST_THROTTLE_PILOT',
    3: 'MOTOR_TEST_COMPASS_CAL',
}

PARAM_ACK = {
    0: 'PARAM_ACK_ACCEPTED',
    1: 'PARAM_ACK_VALUE_UNSUPPORTED',
    2: 'PARAM_ACK_FAILED',
    3: 'PARAM_ACK_IN_PROGRESS',
}

RC_TYPE = {
    0: 'RC_TYPE_SPEKTRUM_DSM2',
    1: 'RC_TYPE_SPEKTRUM_DSMX',
}

RTK_BASELINE_COORDINATE_SYSTEM = {
    0: 'RTK_BASELINE_COORDINATE_SYSTEM_ECEF',
    1: 'RTK_BASELINE_COORDINATE_SYSTEM_NED',
}

SERIAL_CONTROL_DEV = {
    0: 'SERIAL_CONTROL_DEV_TELEM1',
    1: 'SERIAL_CONTROL_DEV_TELEM2',
    2: 'SERIAL_CONTROL_DEV_GPS1',
    3: 'SERIAL_CONTROL_DEV_GPS2',
    10: 'SERIAL_CONTROL_DEV_SHELL',
}

SERIAL_CONTROL_FLAG = {
    1: 'SERIAL_CONTROL_FLAG_REPLY',
    2: 'SERIAL_CONTROL_FLAG_RESPOND',
    4: 'SERIAL_CONTROL_FLAG_EXCLUSIVE',
    8: 'SERIAL_CONTROL_FLAG_BLOCKING',
    16: 'SERIAL_CONTROL_FLAG_MULTI',
}

UAVCAN_NODE_HEALTH = {
    0: 'UAVCAN_NODE_HEALTH_OK',
    1: 'UAVCAN_NODE_HEALTH_WARNING',
    2: 'UAVCAN_NODE_HEALTH_ERROR',
    3: 'UAVCAN_NODE_HEALTH_CRITICAL',
}

UAVCAN_NODE_MODE = {
    0: 'UAVCAN_NODE_MODE_OPERATIONAL',
    1: 'UAVCAN_NODE_MODE_INITIALIZATION',
    2: 'UAVCAN_NODE_MODE_MAINTENANCE',
    3: 'UAVCAN_NODE_MODE_SOFTWARE_UPDATE',
    7: 'UAVCAN_NODE_MODE_OFFLINE',
}

VTOL_TRANSITION_HEADING = {
    0: 'VTOL_TRANSITION_HEADING_VEHICLE_DEFAULT',
    1: 'VTOL_TRANSITION_HEADING_NEXT_WAYPOINT',
    2: 'VTOL_TRANSITION_HEADING_TAKEOFF',
    3: 'VTOL_TRANSITION_HEADING_SPECIFIED',
    4: 'VTOL_TRANSITION_HEADING_ANY',
}

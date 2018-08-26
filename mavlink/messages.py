#!/usr/bin/env python3
'''MAVLink message definitions'''

from scapy.packet import Packet
from scapy.fields import *
from .enums import *

class Heartbeat(Packet):

    name = 'HEARTBEAT'
    fields_desc = [
        ByteEnumField('type', None, MAV_TYPE),
        ByteEnumField('autopilot', None, MAV_AUTOPILOT),
        XByteField('base_mode', None),
        LEIntField('custom_mode', None),
        ByteEnumField('system_status', None, MAV_STATE),
        ByteField('mavlink_version', None)
    ]

MESSAGES = {
    0: Heartbeat,
}
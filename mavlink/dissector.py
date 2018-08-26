#!/usr/bin/env python3
'''MAVLink packet dissector'''

import struct
from scapy.all import bind_layers, TCP, UDP, Packet, Raw, Padding, conf
from scapy.fields import *
from .enums import *

conf.min_pkt_size = 270
conf.padding = 1
conf.interactive = True

class MAVLinkMessageID(Field):
    def __init__(self, name, default):
        Field.__init__(self, name, default, '<I')
    
    def addfield(self, pkt, s, val):
        if pkt.magic == 0xfd:
            v = []
            v[0] = int(val & 0xff)
            v[1] = int((val & 0xff00) / 0x0100)
            v[2] = int((val & 0xff0000) / 0x010000)
            return s + struct.pack('BBB', v[0], v[1], v[2])
        else:
            return s + struct.pack('B', int(val & 0xff))
    
    def getfield(self, pkt, s):
        if pkt.magic == 0xfd:
            return s[3:], self.m2i(pkt, struct.unpack(self.fmt, s[:3] + b'\x00')[0])            # MAVLink 2.0 has 3-byte message ID
        else:
            return s[1:], self.m2i(pkt, struct.unpack(self.fmt, s[:1] + b'\x00\x00\x00')[0])    # 1-byte message ID

class MAVLink(Raw):
    name = 'MAVLink'
    fields_desc = [
        XByteField('magic', None),
        ByteField('length', None),
        ConditionalField(XByteField('incompat_flags', None), lambda pkt: pkt.magic == 0xfd),
        ConditionalField(XByteField('compat_flags', None), lambda pkt: pkt.magic == 0xfd),
        ByteField('sequence', None),
        XByteField('sysid', None),
        XByteField('compid', None),
        MAVLinkMessageID('msgid', None),
        XStrLenField('payload', None, length_from=lambda pkt: pkt.length),
        LEShortField('crc', None),
    ]

    def __init__(self, _pkt=b'', index=0, **kwargs):
        Packet.__init__(self, _pkt, index, kwargs)
        print(len(_pkt))

bind_layers(TCP, MAVLink, sport=5760)
bind_layers(TCP, MAVLink, dport=5760)

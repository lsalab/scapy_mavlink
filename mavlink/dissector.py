#!/usr/bin/env python3
'''MAVLink packet dissector'''

import struct
from scapy.all import bind_layers, TCP, UDP, Packet, Raw, Padding, conf
from scapy.fields import *
from .enums import *
from .messages import *

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

class MAVLinkMessage(StrField):
    __slots__ = ['msgid', 'length_from']
    holds_packets = 1

    def __init__(self, name, default, remain=0, msgid=None, length_from=None, **kwargs):
        StrField.__init__(self, name, default, remain=remain)
        self.msgid = msgid
        self.length_from = length_from
    
    def i2m(self, pkt, i):
        if i is None:
            return b""
        return raw(i)

    def m2i(self, pkt, m):
        return MESSAGES[self.msgid(pkt)](m)

    def getfield(self, pkt, s):
        l = self.length_from(pkt)
        try:
            i = self.m2i(pkt, s[:l])
        except Exception:
            if conf.debug_dissector:
                raise
            i = conf.raw_layer(load=s[:l])
        return s[l:], i

class MAVLink(Raw):
    name = 'MAVLink'
    fields_desc = [
        ByteEnumField('magic', None, {0x55: '(0x55) MAVLink v0.9', 0xfe: '(0xfe) MAVLink v1.0', 0xfd: '(0xfd) MAVLink v2.0'}),
        ByteField('length', None),
        ConditionalField(XByteField('incompat_flags', None), lambda pkt: pkt.magic == 0xfd),
        ConditionalField(XByteField('compat_flags', None), lambda pkt: pkt.magic == 0xfd),
        ByteField('sequence', None),
        XByteField('sysid', None),
        XByteField('compid', None),
        MAVLinkMessageID('msgid', None),
        ConditionalField(XStrLenField('raw_data', None, length_from=lambda pkt: pkt.length), lambda pkt: pkt.msgid not in MESSAGES.keys()),
        ConditionalField(MAVLinkMessage('message', None, msgid=lambda pkt: pkt.msgid, length_from=lambda pkt: pkt.length), lambda pkt: pkt.msgid in MESSAGES.keys()),
        LEShortField('crc', None),
        ConditionalField(StrFixedLenField('signature', None, length=13), lambda pkt: pkt.magic == 0xfd and (pkt.incompat_flags & 0x01) > 0x00),
    ]

    def __init__(self, _pkt=b'', index=0, **kwargs):
        Packet.__init__(self, _pkt, index, kwargs)

    def extract_padding(self, s):
        return None, s

    def pre_dissect(self, s):
        offset = 0
        while offset < len(s) and s[offset] not in [0x55, 0xfe, 0xfd]:
            offset += 1
        if offset > 0 and self.underlayer is not None:
            self.underlayer.add_payload(Padding(s[:offset]))
        return s[offset:]

    def dissect(self, s):
        s = self.pre_dissect(s)
        s = self.do_dissect(s)
        s = self.post_dissect(s)
        payl, pad = self.extract_padding(s)
        self.do_dissect_payload(payl)
        if pad and conf.padding:
            if pad[0] in [0x55, 0xfe, 0xfd]:
                self.add_payload(MAVLink(pad))
            else:
                self.add_payload(Padding(pad))

bind_layers(TCP, MAVLink, sport=5760)
bind_layers(TCP, MAVLink, dport=5760)

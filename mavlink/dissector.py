#!/usr/bin/env python3
'''MAVLink packet dissector'''

import struct
from scapy.packet import Raw, Padding, conf, bind_layers
from scapy.layers.inet import TCP
from scapy.fields import Field, StrField, ByteEnumField, ByteField, ConditionalField, LEShortField, StrFixedLenField, XByteField, XStrLenField
from scapy.compat import raw
from .enums import * # pylint: disable=W0401,W0614
from .messages import MESSAGES

class MAVLinkMessageID(Field):
    '''
    MAVLink message ID field.
    The field can be defined as a 1-byte field for MAVLink v0.9 and v1.0,
    and as a 3-byte field for MAVLink v2.0.
    '''
    def __init__(self, name, default):
        Field.__init__(self, name, default, '<I')

    def addfield(self, pkt, s, val):
        if pkt.magic == 0xfd:
            value = []
            value[0] = int(val & 0xff)
            value[1] = int((val & 0xff00) / 0x0100)
            value[2] = int((val & 0xff0000) / 0x010000)
            return s + struct.pack('BBB', value[0], value[1], value[2])
        else:
            return s + struct.pack('B', int(val & 0xff))

    def getfield(self, pkt, s):
        if pkt.magic == 0xfd:
            return s[3:], self.m2i(pkt, struct.unpack(self.fmt, s[:3] + b'\x00')[0])            # MAVLink 2.0 has 3-byte message ID
        else:
            return s[1:], self.m2i(pkt, struct.unpack(self.fmt, s[:1] + b'\x00\x00\x00')[0])    # 1-byte message ID

class MAVLinkMessage(StrField):
    '''
    MAVLink message field.

    Dynamically selects the appropriate Packet class to store the variable-length data of the payload,
    based upon the message ID. The parameter 'msgid' is meant to be a lambda function that extracts the
    message ID from a MAVLink Packet.
    '''

    __slots__ = ['msgid', 'length_from']
    holds_packets = 1

    def __init__(self, name, default, remain=0, msgid=None, length_from=None):
        StrField.__init__(self, name, default, remain=remain)
        self.msgid = msgid
        self.length_from = length_from

    def i2m(self, pkt, x):
        if x is None:
            return b""
        return raw(x)

    def m2i(self, pkt, x):
        return MESSAGES[self.msgid(pkt)](x)

    def getfield(self, pkt, s):
        length = self.length_from(pkt)
        try:
            i = self.m2i(pkt, s[:length])
        except Exception: # pylint: disable=W0703
            i = conf.raw_layer(load=s[:length])
        return s[length:], i

class MAVLink(Raw):
    '''
    MAVLink message scapy layer.
    '''
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
        Raw.__init__(self, _pkt, index, kwargs)

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
                if self.underlayer is not None:
                    self.underlayer.add_payload(MAVLink(pad))
                else:
                    self.add_payload(MAVLink(pad))
            else:
                if self.underlayer is not None:
                    self.underlayer.add_payload(Padding(pad))
                else:
                    self.add_payload(Padding(pad))

bind_layers(TCP, MAVLink, sport=5760)
bind_layers(TCP, MAVLink, dport=5760)

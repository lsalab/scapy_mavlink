#!/usr/bin/env python3
'''Custom fields'''

from struct import pack, unpack
from scapy.compat import raw
from scapy.packet import conf
from scapy.fields import Field, StrField, LELongField, LEShortField, XByteField

class LESignedShortField(Field):
    '''
    little-endian signed short integer (2-byte)
    '''
    def __init__(self, name, default):
        Field.__init__(self, name, default, '<h')

class XLEShortField(LEShortField, XByteField):
    '''
    little-endian hex represented unsinged short
    '''
    def i2repr(self, pkt, x):
        return XByteField.i2repr(self, pkt, x)

class XLELongField(LELongField, XByteField):
    '''
    little-endian hex represented unsigned long
    '''
    def i2repr(self, pkt, x):
        return XByteField.i2repr(self, pkt, x)

class LEFloatField(Field):
    '''
    little-endian float
    '''
    def __init__(self, name, default):
        Field.__init__(self, name, default, '<f')

class WGS84(Field):
    '''
    WGS84 encoded GPS coordinate.
    '''

    def m2i(self, pkt, x):
        val = [x[3], x[0], x[1], x[2]]
        val = unpack('<i', bytes(val))[0]
        return val

    def i2m(self, pkt, x):
        val = pack('<i', x)
        val = [val[1], val[2], val[3], val[0]]
        return bytes(val)

    def addfield(self, pkt, s, val):
        return s + self.i2m(pkt, val)

    def getfield(self, pkt, s):
        return self.m2i(pkt, s[:4])

class TagField(Field):
    '''
    Fixed-length tag field
    '''

    __slots__ = ['count']

    def __init__(self, name, default, count=None):
        Field.__init__(self, name, default, '<' + 'B'*count)
        self.count = count
    
    def addfield(self, pkt, s, val):
        return s + val.encode('utf-8')
    
    def getfield(self, pkt, s):
        return s[self.count:], s[:self.count]

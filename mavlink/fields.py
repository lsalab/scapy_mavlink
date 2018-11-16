#!/usr/bin/env python3
'''Custom fields'''

from scapy.fields import Field, LELongField, LEShortField, XByteField

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

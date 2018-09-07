#!/usr/bin/env python3
'''Custom fields'''

from scapy.fields import Field

class LESignedShortField(Field):
    '''
    little-endian signed short integer (2-byte)
    '''
    def __init__(self, name, default):
        Field.__init__(self, name, default, '<h')

class LEFloatField(Field):
    '''
    little-endian float
    '''
    def __init__(self, name, default):
        Field.__init__(self, name, default, '<f')
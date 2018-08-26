#!/usr/bin/env python3
'''Custom fields'''

from scapy.fields import *

class LESignedShortField(Field):
    def __init__(self, name, default):
        Field.__init__(self, name, default, '<h')
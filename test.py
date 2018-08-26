#!/usr/bin/env python3
from binascii import unhexlify
from scapy.all import Ether
from mavlink.dissector import *
if __name__ == '__main__':
    testmsg = unhexlify(
        '80e65010b6b8a0c589096d7508004502005c43dc40004006e1b70a2a00010a2a' +
        '00b21680d8a9a64321f4cc468107801800e36e0f00000101080afffea12d1f54' +
        '44da'
    )
    mvmsg = unhexlify(
        '0304030305fd0900001301010000000001ff301a340004039802fd1c00001201011e00' +
        '00bb690300aad60b3cf2a8943b8210ab3f10589e3baca91d3ab48f9639ec5bfd' +
        '2000004001011f000051a80100ca494d3fdaa18e3aea38c03b72f0183fde141c' +
        'bbded4f1bac6879b3a804afe09aa000001010203040506070809bbdc'
    )
    packet = Ether(testmsg)/MAVLink(mvmsg)
    packet.show()
    
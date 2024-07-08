# mpremote mip install unittest

import unittest
from blaster import *

class TestPacket(unittest.TestCase):
    def test_empty_packet(self):
        p = Packet(0,False)
        self.assertEqual(list(p.bits()), [0]*32)
        
    def test_raw_packet_nocrc(self):
        p = Packet(123, False)
        self.assertEqual(list(p.bits()), [1,1,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
        
    def test_raw_crc(self):
        p = Packet(0, True)
        self.assertEqual(p.crc,0)
        
        p = Packet(123, True)
        self.assertEqual(p.crc,102)
            
        p = Packet(16777215, True)
        self.assertEqual(p.crc,15)
        
    def test_crc_overwrites_8_upperbits(self):
        p = Packet(16777216, True)
        self.assertEqual(p.crc,0)
        self.assertEqual(p._value, 0)
        
        p = Packet(16777216+123, True)
        self.assertEqual(p.crc,102)
        self.assertEqual(p._value & 0xFFFFFF, 123)
        
    def test_get_bits(self):
        p = Packet(0x0F0F, False)
        self.assertEqual(p.get_bits(0,4),15)
        self.assertEqual(p.get_bits(2,4),3)
        self.assertEqual(p.get_bits(4,4),0)
        self.assertEqual(p.get_bits(8,1),1)
        self.assertEqual(p.get_bits(0,8),0x0F)
        self.assertEqual(p.get_bits(4,8),0xF0)
        self.assertEqual(p.get_bits(0,16),0x0F0F)
        
    def test_set_bits(self):
        p = Packet(0, False)
        self.assertEqual(p.get_bits(10,8),0)
        p.set_bits(10,8,123)
        self.assertEqual(p.get_bits(10,8),123)
        
    def test_caluclate_crc(self):
        test_pairs = (
            (0x000000, 0),
            (0x00000F, 45),
            (0x0000F0, 222),
            (0x000F00, 195),
            (0x00F000, 20),
            (0x0F0000, 71),
            (0xF00000, 108),
            (0xF000000, 0),
            (0xF0000000, 0),
            )
        for a,b in test_pairs:
            p = Packet(a, True)
            self.assertEqual(p.crc, b)

class TestBlaster(unittest.TestCase):
    def test_dummyu(self):
        ...

if __name__ in ('__main__'):
    unittest.main()


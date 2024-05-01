class RawIrPacket():
    def __init__(self, value = None):
        self._raw = bytearray(32) #sacrificing memory for ferformance.
                    
    @classmethod
    def from_int(cls, value):
        o = cls()
        if isinstance(value, int):
            # 0 - 16777215
            for i in range(24):
                o._raw[i] = (value >> i) % 2
            o.update_crc()
        return o
        
    @classmethod
    def from_bitarray(cls, bits):
        o = cls()
        for i in range(32):
            o._raw[i] = bits[i]
        return o
        
                           
    def int_value(self):
        number = 0
        for i in range(24):
            number = number << 1
            number |= self._raw[23-i]
        return number
            
            
    def calculate_crc(self):
        """
        Kind of ugly way to calculate a crc but it's the most performant version I can think of.
        Storing the polynomial in a array and looping over it is slower. 
        """
        crc = bytearray(8)
        crc[0] = (self._raw[0]  ^ self._raw[2]  ^ self._raw[4]  ^ self._raw[6]  ^
                  self._raw[8]  ^ self._raw[10] ^ self._raw[12] ^ self._raw[14] ^
                  self._raw[16] ^ self._raw[18] ^ self._raw[20] ^ self._raw[22])
        crc[1] = (self._raw[1]  ^ self._raw[3]  ^ self._raw[5]  ^ self._raw[7]  ^
                  self._raw[9]  ^ self._raw[11] ^ self._raw[13] ^ self._raw[15] ^
                  self._raw[17] ^ self._raw[19] ^ self._raw[21] ^ self._raw[23])
        crc[2] = (self._raw[0]  ^ self._raw[1]  ^ self._raw[4]  ^ self._raw[5]  ^
                  self._raw[8]  ^ self._raw[9]  ^ self._raw[12] ^ self._raw[13] ^
                  self._raw[16] ^ self._raw[17] ^ self._raw[20] ^ self._raw[21])
        crc[3] = (self._raw[2]  ^ self._raw[3]  ^ self._raw[6]  ^ self._raw[7]  ^
                  self._raw[10] ^ self._raw[11] ^ self._raw[14] ^ self._raw[15] ^
                  self._raw[18] ^ self._raw[19] ^ self._raw[22] ^ self._raw[23])
        crc[4] = (self._raw[0]  ^ self._raw[1]  ^ self._raw[2]  ^ self._raw[3]  ^
                  self._raw[8]  ^ self._raw[9]  ^ self._raw[10] ^ self._raw[11] ^
                  self._raw[16] ^ self._raw[17] ^ self._raw[18] ^ self._raw[19])
        crc[5] = (self._raw[4]  ^ self._raw[5]  ^ self._raw[6]  ^ self._raw[7]  ^
                  self._raw[12] ^ self._raw[13] ^ self._raw[14] ^ self._raw[15] ^
                  self._raw[20] ^ self._raw[21] ^ self._raw[22] ^ self._raw[23])
        crc[6] = (self._raw[0]  ^ self._raw[1]  ^ self._raw[2]  ^ self._raw[3]  ^
                  self._raw[4]  ^ self._raw[5]  ^ self._raw[6]  ^ self._raw[7]  ^
                  self._raw[16] ^ self._raw[17] ^ self._raw[18] ^ self._raw[19] ^
                  self._raw[20] ^ self._raw[21] ^ self._raw[22] ^ self._raw[23])
        crc[7] = (self._raw[8]  ^ self._raw[9]  ^ self._raw[10] ^ self._raw[11] ^
                  self._raw[12] ^ self._raw[13] ^ self._raw[14] ^ self._raw[15])
        return crc
            
    def update_crc(self):
        crc = self.calculate_crc()
        for i in range(8):
            self._raw[24+i] = crc[i]
            
    def validate_crc(self):
        crc = self.calculate_crc()
        for i in range(8):
            if self._raw[24+i] != crc[i]:
                return False
        return True
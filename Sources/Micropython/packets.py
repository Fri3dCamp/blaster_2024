import micropython

CHANNEL_STR = {
    0: "Standalone",
    1: "Managed"
}

TEAM_STR = {
    0:"None",
    1:"Red",
    2:"Green",
    3:"Yellow",
    4:"Blue",
    5:"Purple",
    6:"Cyan",
    7:"White"
    }

ACTION_STR = {
    0:"None",
    1:"Damage",
    2:"Heal",
    3:"Animation"
    }

COMMAND_STR = {
    0: "ACK",             #Blaster-Badge (sent when a packet was received)
    1: "MODE",            #Badge Sets mode+Team+Action+ID+HP+ReadyToShoot
    2: "SHOT",            #Badge or Blaster when IR shot received (same channel and different team)
    3: "TRIGGER",         #Blaster when trigger was pressed
    4: "TEAM",            #Blaster when team changed
    5: "Animation"        #Badge to trigger animation on blaster
    }

class Packet():
    def __init__(self, raw_value, update_crc):
        self._value = raw_value
        if update_crc:
            self.update_crc()
            
    @micropython.native   
    def bits(self) -> "List[int]":
        for i in range(32):
            yield (self._value >> i) & 0b1 
        
    @micropython.native
    def get_bits(self, pos: int, length: int) -> int:
        return (self._value >> pos) & 2**length-1
    
    @micropython.native
    def set_bits(self, pos: int, length: int, value: int) -> None:
        if value >= 2**length: return
        if value < 0: return
        # reset bits
        self._value &= ~(2**length-1<<pos)
        # set bits
        self._value |= value << pos
        
    @micropython.native
    def calculate_crc(self) -> int:
        """
        Source: Maarten Baert
        """
        checksum = (
            ((self._value <<  2) & 0b10000000111111110111111100) ^
            ((self._value <<  1) & 0b01111111100000000111111110) ^
            ((self._value <<  0) & 0b00000000111111111111111111) ^
            ((self._value >>  1) & 0b00000000100000000000000000) ^
            ((self._value >>  2) & 0b00000000011111110000000000) ^
            ((self._value >>  3) & 0b00000000111111111000000000) ^
            ((self._value >>  4) & 0b00000011100000001111111100) ^
            ((self._value >>  5) & 0b00000000111111111000000010)
        )
        checksum = checksum ^ (checksum >> 8) ^ (checksum >> 16) ^ (checksum >> 24)
        return checksum & 0xFF
    
    @micropython.native
    def update_crc(self):
        self.crc = self.calculate_crc()
    
    @micropython.native
    def validate_crc(self) -> bool:
        checksum = self.calculate_crc()
        return self.crc == checksum
    
    @property
    def crc(self): return self.get_bits(24, 8)
    
    @crc.setter
    def crc(self, value): self.set_bits(24, 8, value)

class IrPacket(Packet):
    """
    Packets sent over IR
    """
    
    def __init__(self, value=0, update_crc=False):
        super().__init__(value, update_crc)

    """
    IR packets can have 0 channels 00-11, default channel = 00
    The blaster and badge ignore IR packets on a different channel
    """

    def __str__(self):
        channel = self.channel
        team = TEAM_STR[self.team]
        action = ACTION_STR[self.action]
        action_parameter = self.action_parameter
        player_id = self.player_id
        crc = self.crc
        crc_test = "OK" if self.validate_crc() else "FAIL"
        return f"IR {channel=} {team=} {action=} {action_parameter=} {player_id=} {crc=} [{crc_test}]"

    """
    0 when Standalone
    1 when Managed
    """
    @property
    def channel(self): return self.get_bits(0, 1)
    
    @channel.setter
    def channel(self, value): self.set_bits(0, 1, value)
    
    
    """
    one bit per color channel (RGB)
    Combinations are allowed.
    000/Black/off means no channel set
    """
    @property
    def team(self): return self.get_bits(1, 3)
    
    @team.setter
    def team(self, value): self.set_bits(1, 3, value)


    """
    None, Shoot, Heal, Animation, ..
    """
    @property
    def action(self): return self.get_bits(4, 2)
    
    @action.setter
    def action(self, value): self.set_bits(4, 2, value)


    """
    Meaning of this propery is depending on the action.
    """
    @property
    def action_parameter(self): return self.get_bits(6, 4)
    
    @action_parameter.setter
    def action_parameter(self, value): self.set_bits(6, 4, value)


    """
    4096 possible player id numbers
    0*  Blaster_2024 Xor 96 UID bits into a 11 bit value
    10* Random value (used by the 2022 blaster)
    11* Set by badge Using the SetPlayerId command
    
    """
    @property
    def player_id(self): return self.get_bits(10, 12)
    
    @player_id.setter
    def player_id(self, value): self.set_bits(10, 12, value)            


class LinkPacket(Packet):
    """
    Packets sent over the badge/blaster link
    """
    def __init__(self, value=0, update_crc=False):
        super().__init__(value, update_crc)

    def __str__(self):
        command = COMMAND_STR[self.command]
        param = self.parameter
        crc = self.crc
        crc_test = "OK" if self.validate_crc() else "FAIL"
        return f"LINK: {command=} {param=} {crc=} [{crc_test}]"

    @property
    def command(self): return self.get_bits(0, 3)
    
    @command.setter
    def command(self, value): self.set_bits(0, 3, value)
    
    
    """
    See command for the meaning of this parameter
    """
    @property
    def parameter(self): return self.get_bits(3, 21)
    
    @parameter.setter
    def parameter(self, value): self.set_bits(3, 21, value)
    
    
class ModeLinkPacket(LinkPacket):
    """
    Packets sent over the badge/blaster link
    """
    def __init__(self, value=0, update_crc=False):
        super().__init__(value, update_crc)
        
    @property
    def mode(self): return self.get_bits(3, 1)
    
    @mode.setter
    def mode(self, value): self.set_bits(3, 1, value)
    
    @property
    def team(self): return self.get_bits(4, 3)
    
    @mode.setter
    def team(self, value): self.set_bits(4, 3, value)
    
    @property
    def action(self): return self.get_bits(7, 2)
    
    @mode.setter
    def action(self, value): self.set_bits(7, 2, value)
    
    @property
    def player_id(self): return self.get_bits(9, 11)
    
    @mode.setter
    def player_id(self, value): self.set_bits(9, 11, value)
    
    @property
    def hp(self): return self.get_bits(20, 3)
    
    @mode.setter
    def hp(self, value): self.set_bits(20, 3, value)
    
    @property
    def ready(self): return self.get_bits(23, 1)
    
    @mode.setter
    def ready(self, value): self.set_bits(23, 1, value)
    
    
    
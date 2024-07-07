import esp32
import asyncio
from time import ticks_us, ticks_diff, sleep
from blaster.packets import Packet, IrPacket, LinkPacket
from machine import Pin
import micropython
micropython.alloc_emergency_exception_buf(100)

#TODO: BUG: LINK communication is to unreliable for some reason.
#           Mostly when both IR and LINK data is comming in.

class Pulse:
    """
    simpe class to calculate if a measured pulse length matches a predifined value + tolerance.
    The tolerance needs to be quite high in micropython due to the way ISR routines are scheduled.
    this scheduling can affect the time between the hardware interrupt happening and the ISR executing.
    """
    def __init__(self, high, low, tolerance):
        self.high = high
        self.low = low
        self.pulse = (high, low)
        self.tolerance = tolerance
        self.length = high + low
        self.length_min = round(self.length * (1 - tolerance))
        self.length_max = round(self.length * (1 + tolerance))
        
    @micropython.native
    def is_match(self, pulse_length):
        return pulse_length > self.length_min and pulse_length < self.length_max
  
class rx_state:
    disabled = 0
    enabled = 1
    ready = 2
    
class rx_machine:
    def __init__(self, pin, packet_type):
        self.state = rx_state.disabled
        self._rx_pin = pin
        self._buffer = 0
        self._pointer = 0
        self._last_pin_change = 0
        self._pin_change_time = 0
        self._pin_change_delta = 0
        self._packet_type = packet_type
        
    """
    The method below is scheduled by µP when a pin change is detected.
    Due to this scheduling there can be a varialble amount of time between the hardware change
    and this function being executed.
    When the CPU is very busy it may happen that this method is delayed so much that the IR signal is not recognized anymore
    """
    @micropython.native
    def _ISR(self, p):
        self._pin_change_time = ticks_us()
        if self.state != rx_state.enabled: return
        self._pin_change_delta = ticks_diff(self._pin_change_time, self._last_pin_change)
        
        if Blaster.start_pulse.is_match(self._pin_change_delta):
            self._pointer = 0
            self._buffer = 0
        elif self._pointer >= 0:
            if Blaster.one_pulse.is_match(self._pin_change_delta):
                self._buffer = self._buffer >> 1
                self._buffer |= 0x80000000
                self._pointer += 1
            elif Blaster.zero_pulse.is_match(self._pin_change_delta):
                self._buffer = self._buffer >> 1
                self._pointer += 1
            else:
                self._pointer = -1
                print("F", self._pin_change_delta)
            if self._pointer == 32:
                self.state = rx_state.ready
                return
            elif self._pointer > 32:
                self._pointer = -1
        self._last_pin_change = self._pin_change_time
        
    def enable(self):
        self._rx_pin.init(Pin.IN)
        self.state = rx_state.enabled
        self._buffer = 0
        self._pointer = -1        
        self._rx_pin.irq(
            trigger=self._rx_pin.IRQ_FALLING, handler=self._ISR
        )
        
    def disable(self):
        self._rx_pin.irq(handler=None)
        self.state = rx_state.disabled
        
    def packet_ready(self):
        return self.state == rx_state.ready
    
    def get_packet(self, wait=False):
        if self.state == rx_state.disabled:
            return None        
        if wait:
            while self.state != rx_state.ready:
                sleep(0.01)
        elif self.state != rx_state.ready:
            return None
        packet = self._packet_type(self._buffer, False)
        self._pointer = -1
        self.state = rx_state.enabled
        if packet.validate_crc():
            return packet
        else:
            return None
    

class Blaster:
    """
    By using 20% (0.2) tolerance we increase the likelyhood that a packet is read correctly.
    This is required because of how µP handles pin change interrupts
    """
    start_pulse = Pulse(9_000, 4_500, 0.2)
    one_pulse = Pulse(560, 560 * 3, 0.2)
    zero_pulse = Pulse(560, 560, 0.2)
    stop_pulse = Pulse(560, 9_000, 0.2)
    IR_CARRIER_FREQ = 38_000
    ir_rmt = None

    def __init__(self, link_pin=None, ir_rx_pin=None, ir_tx_pin=None):
        self._link_pin = link_pin
        self._ir_rx_pin = ir_rx_pin
        self._ir_tx_pin = ir_tx_pin
        
        if link_pin:
            link_pin.init(Pin.IN)
            self.link_rx = rx_machine(link_pin, LinkPacket)
        if ir_rx_pin:
            ir_rx_pin.init(Pin.IN)
            self.ir_rx = rx_machine(ir_rx_pin, IrPacket)
        if ir_tx_pin:
            ir_tx_pin.init(Pin.OUT)
            # resolution (µs) = 1/(80_000_000/240)*1_000_000 = 3.0 µs
            if Blaster.ir_rmt:
                Blaster.ir_rmt.deinit()
            Blaster.ir_rmt = esp32.RMT(
                0,
                pin=ir_tx_pin,
                clock_div=240,
                tx_carrier=(Blaster.IR_CARRIER_FREQ, 50, 1),
            )
        

    @staticmethod
    def blaster_2020(SaoBlaster=False):
        return Blaster(Pin(4), Pin(25), Pin(13) if SaoBlaster else None)
        
    @staticmethod
    def blaster_2024(SaoBlaster=False):
        return Blaster(Pin(10), Pin(11), Pin(13) if SaoBlaster else None)

    def send_linkpacket(self, packet: Packet):
        packet.update_crc() 
        self._send_linkpacket(packet)
        retry = 2
        while retry > 0:
            sleep(.1)
            p = self.link_rx.get_packet(wait=False)
            if p and p.command == 0: #ACK
                break
            else:
                print("retry")
                retry -= 1
                self._send_linkpacket(packet)

    @micropython.native
    def _send_linkpacket(self, packet: Packet):
        print(bin(packet._value))
        if not self._link_pin:
            return       
        rmt = esp32.RMT(1, pin=self._link_pin, clock_div=240, idle_level=1)
        pulse_train = []
        pulse_train.extend(Blaster.start_pulse.pulse)
        for bit in packet.bits():
            if bit:
                pulse_train.extend(Blaster.one_pulse.pulse)
            else:
                pulse_train.extend(Blaster.zero_pulse.pulse)
        pulse_train.extend(Blaster.stop_pulse.pulse)
        # update the pulse lengths because the resolution is 3µs not 1
        pulse_train = [x // 3 for x in pulse_train]
        rmt.write_pulses(pulse_train, 0)
        while not rmt.wait_done(timeout=10): ...                     
        
        # note: the ESP is not done with sending when the write_pulses returns        
        rmt.deinit()
        self._link_pin.init(Pin.IN)

    @micropython.native
    def send_irpacket(self, packet: Packet):
        if not self._ir_tx_pin:
            return
        # there is no async version of this method.
        Blaster.ir_rmt.wait_done(timeout=100)
        pulse_train = []
        pulse_train.extend(Blaster.start_pulse.pulse)
        for bit in packet.bits():
            if bit:
                pulse_train.extend(Blaster.one_pulse.pulse)
            else:
                pulse_train.extend(Blaster.zero_pulse.pulse)
        pulse_train.extend(Blaster.stop_pulse.pulse)
        # update the pulse lengths because the resolution is 3µs not 1
        pulse_train = [x // 3 for x in pulse_train]
        Blaster.ir_rmt.write_pulses(pulse_train, 1)
        # note: the ESP is not done with sending when the write_pulses returns








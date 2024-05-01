import esp32
from time import ticks_us, ticks_diff, sleep
from blaster.raw_ir_packet import RawIrPacket

class Blaster():
    START_PULSE  = (9000, 4500)
    ONE_PULSE  = (560, 1680)
    ZERO_PULSE = (560, 560)
    STOP_PULSE   = (560, 9000)
    IR_CARRIER_FREQ = 38000
    
    def __init__(self, link_pin, ir_rx_pin, ir_tx_pin=None):
        """
        link_pin: the IO pin used to connect the badge to the blaster
                  2020 Badge : 4
                  2024 Badge : 10
        ir_rx_pin: the IO pin connected to the IR reciever
                   2020 Badge : 25
                   2024 Badge : 11
        ir_tx_pin: the IO pin connected to the optional IR transmitter
                   When using SAO : 13 (both 2020 and 2024 Badge)
                   
        """
        self._link_pin = link_pin
        self._ir_rx_pin = ir_rx_pin
        self._ir_tx_pin = ir_tx_pin
        if self._link_pin: self._link_pin.init(self._link_pin.IN)
        if self._ir_rx_pin: self._ir_rx_pin.init(self._ir_rx_pin.IN)
        if self._ir_tx_pin:
            self._ir_tx_pin.init(self._ir_tx_pin.OUT)
            # resolution (µs) = 1/(80_000_000/240)*1_000_000 = 3.0 µs
            self._ir_rmt = esp32.RMT(0, pin=self._ir_tx_pin, clock_div=240, tx_carrier=(Blaster.IR_CARRIER_FREQ, 50, 1))
        self._ir_rx_last_timestamp = 0
    
    def ir_transmit(self, packet: RawPacket):
        if not self._ir_tx_pin:
            print("Can't transmit IR if no IR tx pin is configured")
            return
        
        pulse_train = []
        pulse_train.extend(Blaster.START_PULSE)
        for bit in packet:
            if bit: pulse_train.extend(Blaster.ONE_PULSE)
            else: pulse_train.extend(Blaster.ZERO_PULSE)
        pulse_train.extend(Blaster.STOP_PULSE)
        pulse_train = [x//3 for x in pulse_train]
        self._ir_rmt.write_pulses(pulse_train, 1)

    def enable_ir_receiver(self):
        self._ir_rx_buffer = bytearray(32)
        self._ir_rx_pointer = -1
        self._ir_rx_pin.irq(trigger=self._ir_rx_pin.IRQ_FALLING, handler=self._ir_rx_callback)
        
    def disable_ir_receiver(self):
        self._ir_rx_pin.irq(handler=None)
        
    def _ir_rx_callback(self, p):
        now = ticks_us()
        delta = ticks_diff(now, self._ir_rx_last_timestamp)
        
        if self._ir_rx_pointer >= 32:
            return
        
        if sum(Blaster.START_PULSE)*0.85 <= delta <= sum(Blaster.START_PULSE)*1.15:
            self._ir_rx_pointer = 0
        elif self._ir_rx_pointer >= 0:
            if sum(Blaster.ONE_PULSE)*0.85 <= delta <= sum(Blaster.ONE_PULSE)*1.15:
                self._ir_rx_buffer[self._ir_rx_pointer] = 1
                self._ir_rx_pointer += 1
            elif sum(Blaster.ZERO_PULSE)*0.85 <= delta <= sum(Blaster.ZERO_PULSE)*1.15:
                self._ir_rx_buffer[self._ir_rx_pointer] = 0
                self._ir_rx_pointer += 1
            else:
                self._ir_rx_pointer = -1  #don't start new buffer when this happens
        
        self._ir_rx_last_timestamp = now
        
    def ir_get_packet(self, wait=False):
        #self._received_ir_packet = 
        if wait:
            while self._ir_rx_pointer < 32:
                sleep(.01)
        packet = RawIrPacket.from_bitarray(self._ir_rx_buffer)
        self._ir_rx_pointer = -1
        return packet

                
        
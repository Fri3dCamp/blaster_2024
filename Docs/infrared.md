# Communication over the wire and IR

## Infrared
Infrared data is transmitted on 940 nm with a 38khz carrier wave.

## Protocol
The infrared protocol used is based on the [NEC](https://www.sbprojects.net/knowledge/ir/nec.php) IR Protocol. Note that the blaster used in 2022 used the JVC protocol. These are similar but differ slightly in the length of the pulses and the amount of data that is send in one packet. (32 bit vs 16 bit)

The main motivator to change the protocol to NEC is because of the number of bits that can be send.

# Timings
Before sending bit a start pulse is transmitted. This pulse is 9ms on (38khz) followed by 4.5ms off

# Data 

## data packet

The 32 bits of the data packet are divided in 2 parts.   
3 bytes of data and 1 bytes with a CRC for data validation

## CRC
__TODO__


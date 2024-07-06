# Infrared

This document gives some information on the infrared protocol used by the blaster and badge.

Infrared is light that has a wavelength between 740 nm (nanometer) and 1 mm.
The blaster uses infrared light with a wavelength of 940 nm.  
The IR receives are sensitive for light with this wavelength.  
To help the blaster/badge distinguishing between our IR data and sunlight we modulate the infrared using a 38khz carrier wave. This means we turn on and off the IR led 38.000 times per second.


## Protocol
The infrared protocol used is based on the [NEC](https://www.sbprojects.net/knowledge/ir/nec.php) IR Protocol. Note that the blaster used in 2022 used the JVC protocol. These are similar but differ slightly in the length of the pulses and the amount of data that is send in one packet. (32 bit vs 16 bit)

The main motivator to change the protocol to NEC is because of the number of bits that can be send.

### Timings
The blaster sends out a start pulse before sending out data bits. The start pulse is 9ms on followed by 4.5ms off.

After this start pulse we send 32 bits. a 0 bit is 560ms on follwed by 560ms off. A 1 bit is 560ms on followed by 1.680ms off.

At the end of the 32 bits there is a short 560ms on pulse to indicate the end of the data stream.

# Data 

The 32 bits of the data packet are divided in 2 parts.   
3 bytes of data and 1 bytes with a CRC for data validation

## Data

These are the data fields in the Infrared Datapacket.

* Channel (2 bits) : Only packets that have the same channel as the blaster/badge will be read. This allows for up to 4 games to happen without interfering.   
*Default=0*
* Team (3 bits) : This is the team that has send out the packet. Each bit represents one color (RGB). In normal operation exactly one of these 3 bits will be enabled.  
But for custom games you can combine them to get other colors (Yellow, Magenta, Cyan and White)
* Action (3 bits) : Indicatates what this IR packet will do upon reception. Currently only 2 Actions are used: 0: Damage, 1: Heal. In standalone operation the blaster will only use Damage.
* Action Parameter (4 bits) : This is an optional field that can be used to contain a bit of extra information for the action. For example it could indicate how many hitpoints of damage will be dealth.
* Player ID (12 bits) : This is the Unique ID for the Badge.
For the 2022 blaster this is a random number, for the 2024 this is a number based on the CHIP ID. The ID can be overwritten by the badge.
* CRC (8 bits) : This is a calculation based on the other bits. The receiver can use this calculation to verify if the packet was damaged or not.



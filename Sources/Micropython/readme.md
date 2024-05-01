This is the micropython library for badge to blaster communication.  

# installation

## option 1: mip

If your badge has a working internet connection you can use mip to install this library.
You only need to do this once.

```Python
import mip
mip.install("github:Fri3dCamp/blaster_2024/Sources/Micropython")
```

## option 2: mpremote

You can install the library using mpremote with a badge that is connected to your computer.

```Bash
$ mpremote mip install github:Fri3dCamp/blaster_2024/Sources/Micropython
```

## option 3: Include in the micropython firmware.

To conserve memory you can include the .mpy version of this library in the micropython firmware.  
https://docs.micropython.org/en/latest/reference/packages.html

# usage

## transmitting

The badge does not have a IR LED to transmit data, you can easily add one with a IR LED a transistor and some resistors or use the SAO board created by Wim.

```Python
from blaster import Blaster, RawPacket
from machine import Pin

#The correct pins depend on your badge version. Below are the pins for the 2022 version of the badge
b = Blaster(Pin(4), Pin(25), Pin(13))

#create a simple IR packet based on a 24-bit unsigned integer.
p = RawPacket(i)

#transmit the packet
b.ir_transmit(p)
```

## receiving

The following code enables IR receving on the badge and waits for a packet.

```Python
from blaster.blaster import Blaster
from machine import Pin

b = Blaster(Pin(4), Pin(25), None) #The IR TX led is not defined bacause we will only listen for data

#Start listening for IR data
b.enable_ir_receiver()

#Wait for a IR packet and return it as a RawIrPacket, the badge will start listening for the next packket after returning from this function
p = b.ir_get_packet(True) 

#Get the int representation 
v = p.int_value()

print("received", v)

#disable the IR receiver
b.disable_ir_receiver()
```
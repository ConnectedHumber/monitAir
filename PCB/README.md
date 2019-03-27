# PCB #

A repository for any KiCad PCB designs

## Node1-Prototype ##

This node is based on the WeMOS, a BME280 and a serial dust sensor.

The board receives 5V DC from a wall wart conveniently located. It is recommended that the wall wart be 12V minimum driving a buck convertor reducing to 5V for the unit.

BME280 and WeMOS modules are afixed to the board using 2.54mm header sockets. NOTE the prototype board design only supports the 4 pin module variant of the BME280 sensor but the 6 pin variant can be used if the last two pins are omitted and the firmware uses the I2C address 0x77. The production unit will support the 6 pin module. 

The dust sensor is connected using a JST 2.54mm 1x4 male connector. Power (5V) to the dust sensor is via a high side switch (enabled via GPIO14 and off by default) to support sensors which do not have software sleep configurable modes.

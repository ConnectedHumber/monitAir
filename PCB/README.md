# PCB #

A repository for any KiCad PCB designs

## Node1-Prototype ##

This node is based on the WeMOS, a BME280 and a serial dust sensor.

The board receives 5V DC from a wall wart conveniently located.

To minimise soldering the BME280 and WeMOS modules are soldered to the board using 2.54mm header pins. NOTE the board design only supports the 4 pin module variant of the BME280 sensor. The production unit will support the 6 pin module. 

The dust sensor is connected via a 2.54mm 1x4 pin header on the prototype which will be replaced by a more robust JST 2.54mm 1x4. Power (5V) to the dust sensor is via a high side switch to support sensors which do not have software sleep configurable modes.

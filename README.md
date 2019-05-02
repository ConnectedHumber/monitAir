# monitAir
Files relating to the MonitAir project

## DEEP_SLEEP.ino ##
Robin and I have experienced Aliexpress WeMOS devices which don't come out of deep sleep hence this sketch.

If you experience problems with wakeup using GPIO16 try this sketch to check if yours has that problem.

The fix: Just a judicious use of a soldering iron. I added solder paste to the right hand row , viewed with USB at bottom, of connectors on the ESP8266 module and it started working.

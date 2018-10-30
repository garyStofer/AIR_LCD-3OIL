# Air_LCD_All
This is a collection of applications for the AIR_LCDuino board documented in[ AIR_LCDuino](http://https://github.com/garyStofer/AIR_LCDuino)
The file '`build_opts.h`' has a bunch of #defines that can be used to enable/disable individual features to measure and display. 
Also added a rudimentary data logging capability for the NTC based temperature measurements that could be expanded for other measure/display items as well.
 
- // #define `V_BUS` -- measures input voltage
- // #define `WITH_BARO_HYG_TEMP` - Uses TMP100 or BMP085 or Si7021 to measure atmospheric items
- // #define `WetBulbTemp` - displays Humidity in terms of Wet Bulb temp
- // #define `WITH_RPM` - A simple RPM measurement from a digital pulse signal. 
- // #define `WITH_WIND` - The RPM reading expressed in windspeed using Windsensor hardware from [Wunder Weather Station](http://wws.us.to)
- // #define `WITH_AOA` - A potentiometer based angle readout
- // #define `WITH_NTC` - NTC based temp readings from 3 NTC resistors
- // #define `WITH_SERVO` - A pulse signal output to drive a RC servo
- // #define `WITH_SD_CARD` - A rudimentary datalog function for the NTC temp readings
- // #define `ALARMS_A` - An Alarm mechanism to light up the red LED and lock the display to the alarming reading. 
- // #define `MENUWRAP`  - allows the menu to wrap around from last to first entry or stop at the beginning and end

Not all combinations are valid or fit into memory at the same time. For example the SD card uses the signal attached to the red LED for the chip select, likewise the RC servo signal is shared by the blue LED.   

Last compiled on Arduino IDE 1.6.6, written in C. 
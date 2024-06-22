
# Arduino controlled Milk Heater And Stirrer

This is the controller firmware for the project that is extensively documented at :

https://www.skynext.tech/index.php/2022/12/27/design-and-fast-prototyping-of-a-milk-curd-stirrer-and-heater-using-arduino/

This project is intended for the dairy industry and small farms.
It can be used to ease the processes of :

- Pasteurization.
- Cheese making (some steps are manual, like curd cutting, for now)
- Milk jam making.

It can be used for fruit jams, although be sure to use properly
passivated stainless steel for the hardware part of the implementation, and adapt thermodynamic constants.


## Features :

The firmware performs the following functions :

- Temperature management through PID control, rise and temperature
hold ramp management. That is, you can program it to get from temperature T0 to temperature T1 in a specified amount of time, provided the hotplate it drives is powerful enough. It uses a SSR relay to control the hotplate duty cycle.

- Stirring on/off by controlling a high torque AC motor through a simple relay.

- Measuring liquid quantity through a VL160x laser range finder. Making valid measurements requires careful module & cover placement and proper calibration ! Refer to the main project page mentioned at the beginning of the article for more info.

- Measuring contents temperature through a probe sheat with a Dallas DS1820 sensor.


## TODO


### Software :

- Enable audible power-off warning through Vcc voltage reading.
- Make the project Arduino UNO R4 compatible and add wifi telemetry & remote control & remote program definition/inspection.
- Show program caption on the Lcd keypad when selecting the program.
- Streamline code-bundled program definitions (into FLASH).
- Add the possibility of inputing custom programs through the LCD keypad and store them in the EEPROM.
- Add other kind of liquids support (and their thermodynamic constants).
- Additional temperature measurement code (ambient & motor).
- Rotor lock detection code.
- Motor CW/CCW rotation control.

- Code cleanup and factorization, reducing RAM footprint with string tables and use of F() as much as possible.



### Hardware :

- Larger capacitor for loss of power detection.
- Rotor lock detection (the range finder could be used, or a hall sensor / magnet assembly or variable reluctance sensors).
- For complete motor protection, current sensing could be added.

- Two additional temperature sensors : one for ambient temperature and another for AC motor temperature measurement for overheat protection.
- Add cylindrical heatsink on top of AC motor.
- Add additional wire from motor relay to motor for switching between CW/CCW motor rotation.
- Add curd cutting grates to the stainless steel paddle assembly.

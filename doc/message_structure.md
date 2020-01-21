# Message Structure

This file defines the serial messaging structure used to communicate between host computer and PID controller. The
messaging structure should be simple, fast to implement, and easy to parse on underpowered hardware.

## Required Messages

- Enable/Disable controller

- Tune PID constant values
  - Realtime tuning should be allowed
  - Tuned values should be saved in EEPROM on microcontroller

- EEPROM Save message; save the current PID constants to EEPROM. EEPROM writes take a long time to complete (~3.3 ms) so
  they sould only be done if absolutely necessary.

- Host receive motor position from controller

- Get PID constant values from controller

- Get target value from controller

- Set new target angle

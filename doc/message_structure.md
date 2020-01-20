# Message Structure

This file defines the serial messaging structure used to communicate between host computer and PID controller. The
messaging structure should be simple, fast to implement, and easy to parse on underpowered hardware.

## Required Messages

- Enable/Disable controller

- Tune PID constant values
  - Realtime tuning should be allowed
  - Tuned values should be saved in EEPROM on microcontroller

- Host receive motor position from controller

- Get PID constant values from controller

- Get target value from controller

- Set new target angle

# Message Structure

This file defines the serial messaging structure used to communicate between host computer and PID controller. The
messaging structure should be simple, fast to implement, and easy to parse on underpowered hardware.

## Header Structure

The 3-byte message header is used to distinguish between messages. All messages will be prefixed by the header.

| Start Bytes | Data Length |
| :---------: | :---------: |
| 0x55 0xAA   | mn          |

Where `mn` is the number of data bytes following the header. Data Length will not exceed one byte.

## Messages

All ASCII letters in single-quotes should be interpreted as their [ASCII](http://www.asciitable.com/) values.

Example:

```plaintext
'P' = 0x50
'a' = 0x61
```

### Enable/Disable controller

These commands are responsible for enabling the Arduino module. To protect the sensor, the motor subsystem will be
disabled when power is initially applied to the device.

- `'P' - 0x01` : Enable the device.
- `'P' - 0x00` : Disable the device.

Enable example with header:

```plaintext
0x55, 0xAA, 0x02, 0x50, 0x01
```

### Tune PID constant values (UNDER CONSTRUCTION)

The PID control loop needs to be tuned to preform properly. This command is used to send new constants to the
controller.

### Save PID constant values to EEPROM

Save the current PID constants to EEPROM. EEPROM writes take a long time to complete (~3.3 ms), so a write should only
be done if absolutely necessary. PID constants are automatically loaded from EEPROM on device power.

- `'S'` : Save current PID constants to EEPROM.

Save example with header:

```plaintext
0x55 0xAA 0x01 0x53
```

### Set target position

Set the new target position for the control loop. This value is mechanically limited to 0 - 270 degrees (0 - 4.7124
radians) . The Arduino controller will include these limits in software to protect the sensor.

- `'T' - mn` : Send new position in **degrees**

Send to 100 degrees example:

```plaintext
0x55 0xAA 0x02 0x54 0x64
```

### Request PID constants from controller (UNDER CONSTRUCTION)

### Request target value from controller

Get the current target value from the controller.

- Request: `'t'`
- Response: `'T' - mn`: same as [above](#set-target-position)

### Request current position of the motor

This request gets the current position of the motor. Remember, this may be different from the requested position of the
motor!

- Request: `'s'`
- Response: `'S' - mn`: where `mn` is the current position in **degrees**

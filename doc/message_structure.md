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

### Tune PID constant values

The PID control loop needs to be tuned to preform properly. This command is used to send new constants to the
controller. New constants are immediately applied to the control loop, but for them to persist they must be saved to
EEPROM with the [save](#save-pid-constant-values-to-eeprom) command.

- `'C' - mn - op - qr - st - uv - wx`

| Constant | short     | power |
| :------: | :-------: | :---: |
| kp       | `mn - op` | 10^-3 |
| ki       | `qr - st` | 10^-3 |
| kd       | `uv - wx` | 10^-3 |

Each short value is signed. The value to send can be calculated by taking the desired constant and multiplying it by
10^3. For example, to set the constant 0.13: `0.13 * 10^3 = 130`. The calculated value is then sent in the signed short
value. The full example below should make this more clear.

Example: `kp = 0.53; ki = 0.05; kd = 0.13`

```plaintext
kp = 530 (0x02 0x12)
ki = 50 (0x00 0x32)
kd = 130 (0x00 0x82)

Send: 0x55 0xAA 0x07 0x43 0x02 0x12 0x00 0x32 0x00 0x82
      | - header - |  C |   kp    |   ki    |   kd    |
```

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

- `'T' - mn - op` : Send new position in **degrees**

Send to 100 degrees example:

```plaintext
0x55 0xAA 0x03 0x54 0x00 0x64
```

### Request PID constants from controller

Get the PID constants from the controller

- Request: `'c'`
- Response: `'C' - mn - op - qr - st - uv - wx` : same structure as [send PID constants](#tune-pid-constant-values)

To retrieve the decimal value from the returned signed shorts multiply the value by 10^-3. For example, to get kp from a
message where `mn = 0x44` and `op = 0x22`: `0x4422 (17442) * 10^-3 = 17.442`.

### Request target value from controller

Get the current target value from the controller.

- Request: `'t'`
- Response: `'T' - mn - op`: same as [above](#set-target-position)

### Request current position of the motor

This request gets the current position of the motor. Remember, this may be different from the requested position of the
motor!

- Request: `'s'`
- Response: `'S' - mn - op`: where `mn - op` is the current position in **degrees**

### Request voltage value

Get the voltage value from the current sense resistor. This can be used to calculate the current through the motor. Each
step of the ADC reading corresponds to 4.9 mV of voltage.

Equation:

```plaintext
(V * 4.9mV) = IR where R = 1 ohm
(V * 4.9mV)/1 = I
(V * 4.9mV) = I mA
```

- Request: `'v'`
- Response: `'V' - mn - op` : where `mn - op` is the 10-bit ADC reading from the voltage sense resistor. `mn` is the
  most significant byte.

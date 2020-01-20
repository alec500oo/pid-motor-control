/** pid_firmware.ino
 *  This firmware is meant to be loaded on to an Arduino compatable board with
 *  a motor shield. A PID control loop is used to position a DC motor in the
 *  correct angular position, and stabilize the motor in that target position.
 *  Basic computer control will be available over a serial link to the Arduino
 *  device.
 *
 * @author Alec Matthews <alec500oo\@gmail.com>
 */

#include "pid.h"

#define TARGET (440+959)/2
#define FORWARD_SPEED 10

/* -- Pin Definitions -- */

void setup() {

  Serial.begin(9600);
}

void loop() {


  delay(50);
}

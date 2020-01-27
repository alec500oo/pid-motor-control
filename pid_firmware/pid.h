/** pid.h
 * PID control loop class that contains the math and constants necessary to
 * create a PID loop.
 * @author Alec Matthews <alec500oo\@gmail.com>
 */

#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID {
  unsigned char target_deg = 0;

  unsigned short target_volts = 0;

  int integral = 0;
  int derivative = 0;
  short prev_error = 0;

public:
  short kp = 0;
  short ki = 0;
  short kd = 0;

  /**
   * Initialize the PID class with initial constant values for each control
   * term.
   * @param kp P term constant
   * @param ki I term constant
   * @param kd D term constant
   */
  PID(short kp, short ki, short kd) : kp(kp), ki(ki), kd(kd) {}

  /**
   * Process the next iteration of the PID control loop
   * @param sensorReading The current sensor reading from the feedback sensor in
   * the control loop.
   * @returns The compensation value required to move the actuator error result
   * to move the actuator (DC motor) by.
   */
  long ProcessLoop(short sensorReading);

  /** Set the target in degrees. */
  void SetTarget(unsigned char target);

  /** Get target in degrees. */
  unsigned char GetTarget() { return target_deg; }
};

#endif /* PID_H */

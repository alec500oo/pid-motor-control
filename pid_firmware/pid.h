/** pid.h
 * PID control loop class that contains the math and constants necessary to
 * create a PID loop.
 * @author Alec Matthews <alec500oo\@gmail.com>
 */

#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID {
  double integral = 0;
  double derivative = 0;

  double prev_error = 0;

public:
  short kp = 0;
  short ki = 0;
  short kd = 0;

  unsigned char target = 0;

  /**
   * Initialize the PID class with initial constant values for each control
   * term.
   * @param kp P term constant
   * @param ki I term constant
   * @param kd D term constant
   */
  PID(double kp, double ki, double kd);

  /**
   * Process the next iteration of the PID control loop
   * @param sensorReading The current sensor reading from the feedback sensor in
   * the control loop.
   * @returns The compensation value required to move the actuator error result
   * to move the actuator (DC motor) by.
   */
  double ProcessLoop(double sensorReading);
};

#endif /* PID_H */

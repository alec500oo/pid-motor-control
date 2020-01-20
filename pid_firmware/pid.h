/** pid.h
 * PID control loop class that contains the math and constants necessary to
 * create a PID loop.
 * @author Alec Matthews <alec500oo\@gmail.com>
 */

#ifndef PID_H
#define PID_H

#include "Arduino.h"

class PID {
  double kp = 0;
  double ki = 0;
  double kd = 0;

  double target = 0;

  double integral = 0;
  double derivative = 0;

  double prev_error = 0;

public:
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

  /**
   * Set a new target value for the PID control loop.
   * @param target New target for the algorithm to seek.
   */
  void SetTarget(double target) { this->target = target; }

  /**
   * Set the P term constant for the PID control loop.
   * @param kp P term constant
   */
  void SetPTerm(double kp) { this->kp = kp; }

  /**
   * Set the I term constant for the PID contrl loop.
   * @param ki I term constant
   */
  void SetITerm(double ki) { this->ki = ki;}

  /**
   * Set the D term constant for the PID control loop.
   * @param kd D term constant.
   */
  void SetDTerm(double kd) { this->kd = kd; }
};

#endif /* PID_H */

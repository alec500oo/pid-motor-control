/** pid.h
 * PID control loop class that contains the math and constants necessary to
 * create a PID loop.
 * @author Alec Matthews <alec500oo\@gmail.com>
 */

#ifndef PID_H
#define PID_H

#include "Arduion.h"

class PID {

public:

  /**
   * Initialize the PID class with initial constant values for each control
   * term.
   * @param kp P term constant
   * @param ki I term constant
   * @param kd D term constant
   */
  PID(int kp, int ki, int kd);

  /**
   * Process the next iteration of the PID control loop
   * @param sensorReading The current sensor reading from the feedback sensor in
   * the control loop.
   * @returns TODO: return the error result to move the actuator (DC motor) by.
   */
  void ProcessLoop(int sensorReading);

  /**
   * Set the P term constant for the PID control loop.
   * @param kp P term constant
   */
  void SetPTerm(int kp);

  /**
   * Set the I term constant for the PID contrl loop.
   * @param ki I term constant
   */
  void SetITerm(int ki);

  /**
   * Set the D term constant for the PID control loop.
   * @param kd D term constant.
   */
  void SetDTerm(int kd);
};

#endif /* PID_H */

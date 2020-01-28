#include "pid.h"

#include <math.h>

long PID::ProcessLoop(short sensorReading) {
  short error = target_volts - sensorReading;

  // Calculate the integral term and check for runaway error.
  // TODO: a dT term may need to be added if the loop takes a different amount
  // of time to run each loop.
  integral += error;
  if (error == 0) integral = 0;
  // If we are greater than a quarter of the maximum away there is not point in
  // integrating.
  if (abs(error) > 256) integral = 0;

  // Calculate the derivative term.
  // TODO: May need to add a dT term depending on loop time
  derivative = error - prev_error;

  prev_error = error;
  return (kp * (long)error) + (ki * (long)integral) + (kd * (long)derivative);
}

void PID::SetTarget(unsigned char target) {
  this->target_deg = target;
  this->target_volts = round((double)(1024/270) * (target - 15));
}

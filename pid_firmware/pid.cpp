#include "pid.h"

PID::PID(double kp, double ki, double kd) : kp(kp), ki(ki), kd(kd) { }

double PID::ProcessLoop(double sensorReading) {
  double error = target - sensorReading;

  // Calculate the integral term and check for runaway error.
  // TODO: a dT term may need to be added if the loop takes a different ammount
  // of time to run each loop.
  integral += error;
  if (error == 0) integral = 0;
  if (abs(error) > 40) integral = 0;

  // Calculate the derivative term.
  // TODO: May need to add a dT term depending on loop time
  derivative -= prev_error;

  prev_error = error;
  return (kp * error) + (ki * integral) + (kd * derivative);
}

/** MotorDrive.h
 * Motor driver code for the ArduMoto motor driver board. This board contains an
 * ST L298 to drive a high current brushed DC motor. This driver class uses PWM
 * to vary the speed of the motor and a high/low control voltage to change
 * direction.
 *
 * @author Alec Matthews <alec500oo\@gmail.com>
 */

#include <Arduino.h>

/** -- Pin Definitions -- */
#define DIR_A 2
#define PWM_A 3
#define DIR_B 4
#define PWM_B 11

/** -- Motor Constants -- */

enum class Motor {
  A,
  B
}

typedef unsigned char DIRECTION;

#define FORWARD 0;
#define REVERSE 1;

class MotorDrive {

  // Default to motor a
  Motor motor = Motor::A;

  /** Setup motor A pins */
  SetupA();

  /** Setup motor B pins */
  SetupB();

public:
  /**
   * Constructor. Create a motor driver with for a defined motor.
   * @param motor Motor to drive with this class.
   */
  MotorDrive(Motor motor) : motor(motor) {}

  /** Setup motor pins */
  void Setup();

  /**
   * Set PWM motor speed.
   * @param value PWM value to send to the motor (0-255).
   */
  void SetSpeed(unsigned char value);

  /**
   * Set motor direction
   * @param dir New motor direction.
   */
  void SetDirection(DIRECTION dir);

  /** Disable the motor. */
  void Disable();
};

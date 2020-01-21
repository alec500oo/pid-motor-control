#include "MotorDrive.h"

void MotorDrive::SetupA() {
  pinMode(DIR_A, OUTPUT);
  pinMode(PWM_A, OUTPUT);
  digitalWrite(DIR_A, LOW);
  digitalWrite(PWM_A, LOW);
}

void MotorDrive::setupB() {
  pinMode(DIR_B, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  digitalWrite(DIR_B, LOW);
  digitalWrite(PWM_B, LOW);
}

void MotorDrive::Setup() {
  if (motor == Motor::A)
    SetupA();
  else
    SetupB();
}

void MotorDrive::SetSpeed(unsigned char value) {
  if (motor == Motor::A)
    analogWrite(PWM_A, value);
  else
    analogWrite(PWM_B, value); 
}

void MotorDrive::SetDirection(DIRECTION dir) {
  if (motor == Motor::A)
    digitalWrite(DIR_A, dir);
  else
    digitalWrite(DIR_B, dir);
}

void MotorDrive::Disable() {
  if (motor == Motor::A) {
    digitalWrite(DIR_A, LOW);
    digitalWrite(PWM_A, LOW);
  } else {
    digitalWrite(DIR_B, LOW);
    digitalWrite(PWM_B, LOW);
  }
}

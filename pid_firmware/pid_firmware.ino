/** pid_firmware.ino
 *  This firmware is meant to be loaded on to an Arduino compatable board with
 *  a motor shield. A PID control loop is used to position a DC motor in the
 *  correct angular position, and stabilize the motor in that target position.
 *  Basic computer control will be available over a serial link to the Arduino
 *  device.
 *
 * @author Alec Matthews <alec500oo\@gmail.com>
 */

#include <EEPROM.h>
#include <math.h>

#include "message.h"
#include "motor_drive.h"
#include "pid.h"

/* -- Pin Definitions -- */
#define POT_PIN A0
#define SENSE_PIN A1

/* -- Structure Definitions -- */
struct pidConstants {
  short kp;
  short ki;
  short kd;
};

/* -- Function forward declerations -- */
void ProcessSerialMessage();
void ParseMessage();

void EnableDevice(char* msg_ptr);
void SetConstants(char* msg_ptr, PID* pid);
void SaveConstants(PID* pid);
void SetTarget(char* msg_ptr, PID* pid);

void SendConstants(PID* pid);
void SendTarget(PID* pid);
void SendCurrentPos();
void SendVoltage();

/* -- Global Variables -- */
PID controlLoop = PID(0, 0, 0);
MotorDrive motor = MotorDrive(Motor::A);
Message msg = Message(&Serial);

bool enabled = false;

void setup() {
  // Setup analog pins
  pinMode(POT_PIN, INPUT);
  pinMode(SENSE_PIN, INPUT);

  // Get constant values from EEPROM
  pidConstants c;
  EEPROM.get(0, c);

  // Create control loop object and set default target.
  controlLoop = PID(c.kp, c.ki, c.kd);
  controlLoop.SetTarget(100);

  // Setup motor.
  motor.Setup();

  // Setup serial communication
  // Baudrate: 115200
  // 8 data bits, 1 stop bit, even parity
  Serial.begin(115200, SERIAL_8E1);

  // Wait for serial connection to start
  while (!Serial);
}

void loop() {

  // Read sensor position
  short sensor_pos = analogRead(POT_PIN);
  long speed = controlLoop.ProcessLoop(sensor_pos);
  double outspeed = (double)speed/1000;

  // Update motor position

  // Limit the motor from moving outside the bounds of 15 - 285 deg.
  if (!enabled || sensor_pos <= 51 || sensor_pos >= 973) {
    motor.Disable();
  } else {
    motor.SetDirection(!(outspeed < 0));
    motor.SetSpeed(abs(outspeed));
  }

  ProcessSerialMessage();
}

void ProcessSerialMessage() {
  char* msg_ptr = msg.GetNextMessage();
  if (msg_ptr) ParseMessage(msg_ptr);
}

void ParseMessage(char* msg_ptr) {

  switch (msg_ptr[3])
  {
  case 'P':
    EnableDevice(msg_ptr);
    break;
  case 'C':
    SetConstants(msg_ptr, &controlLoop);
    break;
  case 'S':
    SaveConstants(&controlLoop);
    break;
  case 'T':
    SetTarget(msg_ptr, &controlLoop);
    break;
  case 'c':
    SendConstants(&controlLoop);
    break;
  case 't':
    SendTarget(&controlLoop);
    break;
  case 's':
    SendCurrentPos();
    break;
  case 'v':
    SendVoltage();
    break;  
  default:
    break;
  }

  msg.Clear();
}

void EnableDevice(char* msg_ptr) {
  enabled = msg_ptr[4];
}

void SaveConstants(PID* pid) {
  pidConstants c = {pid->kp, pid->ki, pid->kd};
  EEPROM.put(0, c);
}

void SetConstants(char* msg_ptr, PID* pid) {
  pid->kp = ((short)msg_ptr[4] << 8) + msg_ptr[5];
  pid->ki = ((short)msg_ptr[6] << 8) + msg_ptr[7];
  pid->kd = ((short)msg_ptr[8] << 8) + msg_ptr[9];
}

void SetTarget(char* msg_ptr, PID* pid) {
  unsigned short target = ((short)msg_ptr[4] << 8) + msg_ptr[5];
  pid->SetTarget(target);
}

void SendConstants(PID* pid) {
  unsigned char buf[10] = {0x55, 0xAA, 0x7, 'C', 0};
  buf[4] = pid->kp >> 8;
  buf[5] = pid->kp & 0xFF;
  buf[6] = pid->ki >> 8;
  buf[7] = pid->ki & 0xFF;
  buf[8] = pid->kd >> 8;
  buf[9] = pid->kd & 0xFF;

  Serial.write(buf, 10);
}

void SendTarget(PID* pid) {
  unsigned char buf[6] = {0x55, 0xAA, 0x02, 'T', 0};
  short tmp = pid->GetTarget();
  buf[4] = tmp >> 8;
  buf[5] = tmp & 0xFF;

  Serial.write(buf, 6);
}

void SendCurrentPos() {
  unsigned char buf[6] = {0x55, 0xAA, 0x02, 'S', 0};
  short val = analogRead(POT_PIN);
  short tmp = 15 + round((270/1024) * val);
  buf[4] = tmp >> 8;
  buf[5] = tmp & 0xFF;

  Serial.write(buf, 6);
}

void SendVoltage() {
  unsigned char buf[6] = {0x55, 0xAA, 0x03, 'V', 0};
  short volts = analogRead(SENSE_PIN);
  buf[4] = volts >> 8;
  buf[5] = volts & 0xFF;

  Serial.write(buf, 6);
}

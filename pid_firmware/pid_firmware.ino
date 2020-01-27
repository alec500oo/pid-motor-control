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
#include "MotorDrive.h"
#include "pid.h"

/* -- Pin Definitions -- */
#define POT_PIN A0

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
  pinMode(POT_PIN, INPUT);
  // Get constant values from EEPROM
  pidConstants c;
  EEPROM.get(0, c);

  // Create control loop object.
  // controlLoop = PID(c.kp, c.ki, c.kd);
  // TODO: initialize PID with saved values.
  controlLoop = PID(130, 0, 0);
  controlLoop.SetTarget(100);

  // Setup motor.
  motor.Setup();

  // Setup serial communication
  // Baudrate: 115200
  // 8 data bits, 1 stop bit, even parity
  Serial.begin(115200, SERIAL_8E1);

  // Wait for serial connection to start
  // TODO: Maybe time this out if we don't care about serial
  while (!Serial); 
}

void loop() {

  // Read sensor position
  short sensor_pos = analogRead(POT_PIN);
  long speed = controlLoop.ProcessLoop(sensor_pos);

  Serial.print(" Out: ");
  Serial.println(speed/1000);

  // Update motor position
  // TODO: start motor disabled.
  if (!enabled);

  if (sensor_pos <= 150 || sensor_pos >= 900) {
    motor.Disable();
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
  pid->kp = (msg_ptr[4] << 8) + msg_ptr[5];
  pid->ki = (msg_ptr[6] << 8) + msg_ptr[7];
  pid->kd = (msg_ptr[8] << 8) + msg_ptr[9];
}

void SetTarget(char* msg_ptr, PID* pid) {
  pid->SetTarget(msg_ptr[4]);
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
  unsigned char buf[5] = {0x55, 0xAA, 0x02, 'T', 0};
  buf[4] = pid->GetTarget();

  Serial.write(buf, 5);
}

void SendCurrentPos() {
  unsigned char buf[5] = {0x55, 0xAA, 0x02, 'S', 0};
  short val = analogRead(POT_PIN);
  buf[4] = 15 + round((270/1024) * val);

  Serial.write(buf, 5);
}

void SendVoltage() {
  unsigned char buf[5] = {0x55, 0xAA, 0x03, 'V', 0};
  // TODO: Figure out what analog pin the current sense resistor will be on.
}

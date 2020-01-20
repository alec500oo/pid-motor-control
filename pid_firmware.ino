/** pid_firmware.ino
 *  This firmware is meant to be loaded on to an Arduino compatable board with
 *  a motor shield. A PID control loop is used to position a DC motor in the
 *  correct angular position and stabilize it in that target position. Basic
 *  computer control will be available over a serial link to the Arduino device.
 *
 * @author Alec Matthews <alec500oo\@gmail.com>
 */


#define TARGET (440+959)/2
#define FORWARD_SPEED 10

/* -- Pin Definitions -- */

int target = 0;
double Kp = 0.032;
int Ki = 0.000001;
int Kd = 0.000008;
int error = 0;
int integral = 0;
int derivative = 0;
int sensorVal = 0;
int previous_error = 0;
double mSpeed = 0;

void setup() {
  // put your setup code here, to run once:
  target = (440 + 959) / 2;

  tone(BUZZER_PIN, 500, 500);
  delay(500);
  tone(BUZZER_PIN, 1500, 500);
  delay(500);
  tone(BUZZER_PIN, 1000, 500);
  delay(500);
  tone(BUZZER_PIN, 2000, 500);
  delay(1000);
  pinMode(IR_REC_PIN, INPUT);
  pinMode(IR_LED_PIN, OUTPUT);

  leftServo.attach(LEFTSERVO);
  rightServo.attach(RIGHTSERVO);
  leftServo.write(90);
  rightServo.write(90);

  Serial.begin(9600);
  pinMode(QTI_PIN, INPUT);
  pinMode(WIFI_PIN, INPUT);
}

void loop() {
  //-- Pulse IR LED
  tone(IR_LED_PIN, 38000, 8);

  //-- Wait and read IR sensor
  delay(1);
  byte irSensor = digitalRead(IR_REC_PIN);

  //-- Charge cap in sensor
  pinMode(QTI_PIN, OUTPUT);
  digitalWrite(QTI_PIN, HIGH);
  delay(1);

  //-- Let cap discharge
  pinMode(QTI_PIN, INPUT);
  delay(1);

  sensorVal = analogRead(QTI_PIN); //save the value from the analog read into sensorVal
  error = target - sensorVal;
  integral = integral + error;
  if (error == 0) {
    integral = 0;
  }
  if (abs(error) > 40) {
    integral = 0;
  }

  derivative = error - previous_error;
  mSpeed = Kp * error + Ki * integral + Kd * derivative;
  previous_error = error;
  if (irSensor == HIGH && digitalRead(WIFI_PIN) == HIGH) {
    speedToServo(mSpeed, FORWARD_SPEED);
  } else {
    halt();
  }

  Serial.print(sensorVal); //display sensorVal
  Serial.print(" ");
  Serial.print(error);
  Serial.print(" ");
  Serial.print(integral);
  Serial.print(" ");
  Serial.print(derivative);
  Serial.print(" ");
  Serial.println(mSpeed);

  delay(50);
}

void speedToServo(double mSpeed, int forwardSpeed) {
  leftServo.write(90 + mSpeed - forwardSpeed);
  rightServo.write(90 + mSpeed + forwardSpeed);
}

void halt() {
  leftServo.write(90);
  rightServo.write(90);
}

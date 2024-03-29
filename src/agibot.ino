#include <SharpIR.h>
#include <QTRSensors.h>
#include <ezButton.h>

SharpIR sensor(SharpIR:: GP2Y0A41SK0F, A7);
QTRSensors qtr;

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];
const int startButton = 10;

// RIGHT MOTOR PINS
#define PWMA 10 // speed control
#define AIN1 5
#define AIN2 6

// LEFT MOTOR PINS
#define PWMB 9 // speed control
#define BIN1 2
#define BIN2 3

// PID TUNING
#define Kp 1.9 // 0.041
#define Kd 40 // 0
#define maxSpeed 210
#define baseSpeed 170
#define turnSpeed 110

/* Tuning Presets
Kp = 2      Kp = 1      Kp = 1
Kd = 40     Kd = 62     Kd = 62
mS = 210    mS = 188    mS = 218
bs = 170    bS = 148    bS = 168
tS = 110    tS = 112    tS = 112
*/

int lastError = 0;
long now = millis();
long lastMeasure = 0;

void setup() {
  pinMode(startButton, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  Serial.begin(9600);
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);
  delay(250);

  // calibrate sensors
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint8_t i = 0; i < 100; i++) {
    if (i < 25 || i > 75) {
      move(0, 80, 0);
      move(1, 80, 1);
    } 
    else {
      move(0, 80, 1);
      move(1, 80, 0);
    }
    qtr.calibrate();
    delay(10);
  }
  delay(500);
  move(0, 0, 0);
  move(1, 0, 0);
  digitalWrite(LED_BUILTIN, LOW);
  delay(5000);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  uint16_t position = qtr.readLineBlack(sensorValues);

  if (position > 4700) {
    move(1, turnSpeed, 0);
    move(0, turnSpeed, 1);
    return;
  }
  if (position < 300) {
    move(1, turnSpeed, 1);
    move(0, turnSpeed, 0);
    return;
  }
  // PID Algorithm
  int error = position - 3500;
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;
  int rightMotorSpeed = baseSpeed + motorSpeed;
  int leftMotorSpeed = baseSpeed - motorSpeed;

  if (rightMotorSpeed > maxSpeed) {
    rightMotorSpeed = maxSpeed;
  }
  if (leftMotorSpeed > maxSpeed) {
    leftMotorSpeed = maxSpeed;
  }
  if (rightMotorSpeed < 0) {
    rightMotorSpeed = 0;
  }
  if (leftMotorSpeed < 0) {
    leftMotorSpeed = 0;
  }
  move(1, rightMotorSpeed, 0);
  move(0, leftMotorSpeed, 0);
}

void move (int motor, int speed, int direction) {
  boolean inPin1 = HIGH, inPin2 = LOW;
  if (direction == 1) {
    inPin1 = HIGH;
    inPin2 = LOW;
  }
  if (direction == 0) {
    inPin1 = LOW;
    inPin2 = HIGH;
  }
  if (motor == 0) {
    digitalWrite(AIN1, inPin1);
    digitalWrite(AIN2, inPin2);
    analogWrite(PWMA, speed);
  }
  if (motor == 1) {
    digitalWrite(BIN1, inPin1);
    digitalWrite(BIN2, inPin2);
    analogWrite(PWMB, speed);
  }
}

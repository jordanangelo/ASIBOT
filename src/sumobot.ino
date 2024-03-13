#include <QTRSensors.h>
#include <SharpIR.h>
#include <ezButton.h>

QTRSensors qtr;
SharpIR sensor(SharpIR::GP2Y0A21YK0F, A7);
ezButton agiSwitch(4); // toggle switch for agibot
ezButton sumoSwitch(7); // toggle switch for sumobot
ezButton intelSwitch(8); // toggle switch for intellibot

const uint8_t SensorCount = 4;
uint16_t sensorValues[SensorCount];
#define startButton 11;

// Arena Settings
#define QTR_THRESHOLD 300  // microseconds (need tuning per each environment)

// Speed Settings
#define speedTurn 100     // Default - 80
#define speedForward 55   // Default - 255
#define speedBackward 100  // Default - 255
#define speedCharge 120    // Default - 255

// Left Motor Pins
#define PWMA 10   // speedControl
#define AIN1 5  // pinDirection
#define AIN2 6  // pinDirection

// Right Motor Pins
#define PWMB 9  // speedControl
#define BIN1 2  // pinDirection
#define BIN2 3  // pinDirection

bool agiDelay = true;
int lastError = 0;

void setup() {
  Serial.begin(9600);
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){
                      A0, A3, A4, A5},
                    SensorCount);
  qtr.setEmitterPin(8);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  delay(5000);
  startRoutine();

}

void loop() {
  int distance = sensor.getDistance();
  //Serial.println(distance);
  qtr.read(sensorValues);
  //Serial.println(sensorValues);

  if (sensorValues[0] < QTR_THRESHOLD || sensorValues[1] < QTR_THRESHOLD) {
    // Leftmost Sensor Detected the Border
    move(1, speedBackward, 1);
    move(0, speedBackward, 1);
    delay(500); 
    move(0, speedTurn, 1);
    move(1, 0, 1);
    if (distance < 20) {
      move(1, speedCharge, 0);
      move(0, speedCharge, 0);
    }
    delay(600);/*
    move(1, speedForward, 1);
    move(0, speedForward, 1);*/
  } 
  else if (sensorValues[3] < QTR_THRESHOLD || sensorValues[2] < QTR_THRESHOLD) {
    // Rightmost Sensor Detected The Border
    move(1, speedBackward, 1);
    move(0, speedBackward, 1);
    delay(500);
    move(1, speedTurn, 0);
    move(0, 0, 1);
    if (distance < 20) {
      move(1, speedCharge, 0);
      move(0, speedCharge, 0);
    }
    delay(600);/*
    move(1, speedForward, 1);
    move(0, speedForward, 1);*/
  } 
  else {
    if (distance <= 20) {      
      move(1, speedCharge, 0);
      move(0, speedCharge, 0);
      /*
      delay(1000);
      move(1, speedTurn, 0);
      move(0, speedCharge, 1);
      delay(1000);
      if (distance < 15) {
      move(1, speedCharge, 1);
      move(0, speedCharge, 1);
      return;
      }*/
    }
    else if (distance > 20 && distance <= 30) {      
      move(0, 0, 0);
      move(1, speedCharge, 0);
      delay(100);
      move(0, speedCharge, 0);
      move(1, speedCharge, 0);
      delay(300);
      move(0, speedCharge, 0);
      move(1, 0, 0);
      delay(100);/*
      move(0, speedCharge, 0);
      move(1, speedCharge, 0);*/
    }
    else {
      // search
      move(1, speedForward, 0);
      move(0, speedTurn, 0);
    }
  }
}

void move(int motor, int speed, int direction) {
  // motor 0 = left, 1 = right || direction 0 = forward, 1 = backward
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

void startRoutine() {
  for(int i=0; i<2; i++) {
    move(1, 255, 1);
    move(0, 255, 0);
    delay(900);
    move(1, 0, 0);
    move(0, 0, 0);
    delay(10);
  }
}

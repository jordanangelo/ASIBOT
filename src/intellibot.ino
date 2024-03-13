#include <ezButton.h>
#include <QTRSensors.h>
#include <util/delay.h>

#define intButton 12

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
uint16_t position = 0;
int speed_lt = 90;

// Line threshold
#define WT 300
#define BT 550

// Left Motor Pins
#define PWMA 10   // speedControl
#define AIN1 5  // pinDirection
#define AIN2 6  // pinDirection

// Right Motor Pins
#define PWMB 9  // speedControl
#define BIN1 2  // pinDirection
#define BIN2 3  // pinDirection

// Speed tunings
#define calibSpeed 116   // calibration - 165
#define turnSpeed 84  // turn - 120
#define turnSpeedSlow 70  // slow turn - 100
#define drivePastDelay 180 // align wheels turn - 180

// PID initializations
float lastError = 0;
float error = 0;
float PV = 0;
float kp = 0;
float kd = 0;
int Lspeed = 0;
int Rspeed = 0;
int motorSpeed = 0;

// Path variable storage - 'L' for left, 'R' for right, 'S' for straight, and 'B' for back (U-turn)
char path[100] = "";
unsigned char pathLength = 0;

void calibrate() {
  delay(250);
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint8_t i = 0; i < 100; i++) {
    if (i < 25 || i > 75) {
      move(0, 80, 1);
      move(1, 80, 0);
    } 
    else {
      move(0, 80, 0);
      move(1, 80, 1);
    }
    qtr.calibrate();
    delay(10);
  }
  delay(500);
  move(0, 0, 1);
  move(1, 0, 1);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
}

void setup() {
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(intButton, INPUT_PULLUP);
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){
                      A0, A1, A2, A3, A4, A5, A6, 13},
                    SensorCount);
  Serial.begin(9600);
  calibrate();
  delay(5000);
}

void PID() {
  lastError = 0;
  while(1) {
    position = qtr.readLineBlack(sensorValues);
    switch(position) {
      //Line has moved off the left edge of sensor
      case 0:
        move(0, 150, 1);
        move(1, 90, 0);
      break;
      //Line has moved off the right edge of sensor
      case 7000:
        move(0, 90, 0);
        move(1, 150, 1);
      break;
      default:
        error = (float)position - 3500;
        kp = 1.9;
        kd = 40;
        PV = kp * error + kd * (error - lastError);
        lastError = error;
        if (PV > 55) {      
          PV = 55;
        }   
        if (PV < -55) {
          PV = -55;
        }

        Rspeed = 200 + PV - speed_lt;
        Lspeed = 200 - PV - speed_lt;
        move(0, Lspeed, 0);
        move(1, Rspeed, 0);
        break;
    }

    if (sensorValues[0] < WT && sensorValues[1] < WT && sensorValues[2] < WT && sensorValues[3] < WT && sensorValues[4] < WT && sensorValues[5] < WT && sensorValues[6] < WT && sensorValues[7] < WT) {
      // There is no line visible ahead - DEAD END
      //Serial.print(" dead end");
      return;
    }
    else if (sensorValues[0] > BT && sensorValues[1] > BT && sensorValues[2] > BT && sensorValues[3] > BT && sensorValues[4] > BT && sensorValues[5] > BT && sensorValues[6] > BT && sensorValues[7] > BT) {
      // Found intersection
      return;
    }
  }
}

// function to determine which way to turn during maze learaning phase
char select_turn(unsigned char foundLeft, unsigned char foundStraight, unsigned char foundRight) {
  // Left-hand rule strategy
  if (foundLeft) {
    return 'L';
  }
  else if (foundStraight) {
    return 'S';
  }
  else if (foundRight) {
    return 'R';
  }
  else {
    return 'B';
  }
}

void turn(char dir) {
  switch(dir) {
    //Turn left 90deg
    case 'L':
      move(0, turnSpeedSlow, 1);
      move(1, turnSpeedSlow, 0);
      //find center position
      while (position < 1200 && sensorValues[1] < BT) {
        position = qtr.readLineBlack(sensorValues);
      }
      // stop
      move(1, 0, 0);
      move(0, 0, 0);
      break;

    //Turn right 90deg
    case 'R':
      move(0, turnSpeedSlow, 0);
      move(1, turnSpeedSlow, 1);
      //find center position
      while (position < 6700 && sensorValues[6] < BT) {
        position = qtr.readLineBlack(sensorValues);
      }
      // stop
      move(0, 0, 0);
      move(1, 0, 0);
      break;
    
    //Turn right 180deg - Go back (U-Turn)
    case 'B':
      move(0, turnSpeedSlow, 0);
      move(1, turnSpeedSlow, 1);
      //find center position
      while (position < 6700 && sensorValues[6] < BT) {
        position = qtr.readLineBlack(sensorValues);
      }
      // stop
      move(0, 0, 0);
      move(1, 0, 0);
      break;

    //Straight ahead
    case 'S':
      break;
  }
}

void simplifyPath() {
  // only simplify the path if the second-to-last turn was a 'B'
  if (pathLength < 3 || path[pathLength - 2] != 'B') {
    return;
  }

  int totalAngle = 0;
  for (int i = 1; i <= 3; i++) {
    switch(path[pathLength - i]) {
      case 'R':
        totalAngle += 90;
        break;
      case 'L':
        totalAngle += 270;
        break;
      case 'B':
        totalAngle += 180;
        break;
    }
  }

  // get the angle as a number between 0 and 360deg
  totalAngle = totalAngle % 360;
  // replace all turns with a single turn
  switch(totalAngle) {
    case 0:
      path[pathLength - 3] = 'S';
      break;
    case 90:
      path[pathLength - 3] = 'R';
      break;
    case 180:
      path[pathLength - 3] = 'B';
      break;
    case 270:
      path[pathLength - 3] = 'L';
      break;
  }
  // the path is now 2 steps shorter
  pathLength -= 2;
}

void solveMaze() {
  while(1) {
    PID();
    unsigned char foundLeft = 0;
    unsigned char foundStraight = 0;
    unsigned char foundRight = 0;
    // check intersection type
    position = qtr.readLineBlack(sensorValues);
    // check for if left and right exists
    if (sensorValues[7] > WT) {
      foundRight = 1;
    }
    if (sensorValues[0] > WT) {
      foundLeft = 1;
    }
    move(0, 130, 0);
    move(1, 130, 0);
    delay(drivePastDelay);
    move(0, 0, 0);
    move(1, 0, 0);
    delay(400);

    position = qtr.readLineBlack(sensorValues);
    if(sensorValues[2] > BT || sensorValues[3] > BT || sensorValues[4] > BT || sensorValues[5] > BT) {
      foundStraight = 1;
    }
    // if all sensors in black = maze solved
    if(sensorValues[0] > 600 && sensorValues[1] > 600 && sensorValues[2] && 600 && sensorValues[3] > 600 && sensorValues[4] > 600 && sensorValues[5] > 600 && sensorValues[6] && 600 && sensorValues[7] > 600) {
      break;
    }

    // follow existing path
    unsigned char dir = select_turn(foundLeft, foundStraight, foundRight);
    turn(dir);
    // store intersection in array
    path[pathLength] = dir;
    pathLength ++;
    // simplify path
    simplifyPath();
  }
  // stop
  move(0, 0, 0);
  move(1, 0, 0);
  // indicator
  for(int i = 0 ; i < 5; i++) {
      digitalWrite(LED_BUILTIN, HIGH);
      _delay_ms(250);
  }
  // rerun the maze
  while(1) {
    move(0, 0, 0);
    move(1, 0, 0);
    delay(5000);
    
    for (int i = 0; i < pathLength; i++) {
      PID();
      // move slightly slower on intersections
      move(0, 200, 0);
      move(1, 200, 0);
      delay(drivePastDelay); // tune this to align wheels on the line
      turn(path[i]);
    }
    PID();
    // move slightly slower on intersections
    move(0, 200, 0);
    move(1, 200, 0);
    delay(drivePastDelay); // tune this to align wheels on the line
  }
}

void loop() {
  position = qtr.readLineBlack(sensorValues);
  // start maze solve
  solveMaze();
  delay(250);
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

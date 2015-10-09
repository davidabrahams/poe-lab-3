

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotorL = AFMS.getMotor(3);
Adafruit_DCMotor *myMotorR = AFMS.getMotor(4);

int baseSpeed = 30;
int correction = 5;
int breakValue = 25;

byte motorSpeedL;
byte motorSpeedR;

int threshold = 2700;
int overLine = 3800;

int sensorValueL;
int sensorValueR;

// variable to store the current time
unsigned long currTime;
byte sleepTime = 25; // ms

// These constants won't change.  They're used to give names
// to the pins used:
const int analogInPin1 = A0;  // Analog input pin that the potentiometer is attached to
const int analogInPin2 = A1;  // Analog input pin that the

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  AFMS.begin();
  motorSpeedL = baseSpeed;
  motorSpeedR = baseSpeed;
  myMotorL->setSpeed(motorSpeedL);
  myMotorL->run(FORWARD);
  myMotorR->setSpeed(motorSpeedR);
  myMotorR->run(FORWARD);
}

void update_speeds(int sensorL, int sensorR)
{

  int coercedL = min(max(sensorL, threshold), overLine);
  int coercedR = min(max(sensorR, threshold), overLine);

  int diffRange = overLine - threshold;

  int diff = coercedR - coercedL;
  int speedCorrection = map(diff, -diffRange, diffRange, -correction,
                            correction);

  // If sensorR is greater, the right sensor is over the tape. The left motor
  // must go faster.
  int adjBase = baseSpeed - breakValue * abs(speedCorrection) / correction;

  motorSpeedL = adjBase + speedCorrection;
  motorSpeedR = adjBase - speedCorrection;

}

void loop()
{
  currTime = millis();
  
  sensorValueL = analogRead(analogInPin1);
  sensorValueR = analogRead(analogInPin2);
  int outputValueL = map(sensorValueL, 0, 1023, 0, 5000);
  int outputValueR = map(sensorValueR, 0, 1023, 0, 5000);

  update_speeds(outputValueL, outputValueR);
  myMotorL->setSpeed(motorSpeedL);
  myMotorR->setSpeed(motorSpeedR);
  
//  Serial.print("Left: ");
//  Serial.print(motorSpeedL);
//  Serial.print("Right: ");
//  Serial.println(motorSpeedR);

  delay(sleepTime - (millis() - currTime()));
}
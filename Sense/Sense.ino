

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotorL = AFMS.getMotor(1);
Adafruit_DCMotor *myMotorR = AFMS.getMotor(2);

byte baseSpeed = 50;
byte correction = 40;

byte motorSpeedL;
byte motorSpeedR;

int threshold = 2700;
int overLine = 3800;

// These constants won't change.  They're used to give names
// to the pins used:
const int analogInPin1 = A0;  // Analog input pin that the potentiometer is attached to
const int analogInPin2 = A1;  // Analog input pin that the potentiometer is attached to

int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)

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

void update_speeds(sensorL, sensorR)
{

  coercedL = min(max(sensorL, threshold), overLine);
  coercedR = min(max(sensorR, threshold), overLine);

  diffRange = overLine - threshold;

  diff = coercedR - coercedL;
  speedCorrection = map(diff, -diffRange, diffRange, -correction, correction);

  // If sensorR is greater, the right sensor is over the tape. The left motor
  // must go faster.
  motorSpeedL = baseSpeed + speedCorrection / 2;
  motorSpeedR = baseSpeed - speedCorrection / 2;

}

void loop()
{
  sensorValue1 = analogRead(analogInPin1);
  sensorValue2 = analogRead(analogInPin2);
  outputValue1 = map(sensorValue1, 0, 1023, 0, 5000);
  outputValue2 = map(sensorValue2, 0, 1023, 0, 5000);

  update_speeds(outputValue1, outputValue2);
  myMotorL->setSpeed(motorSpeedL);
  myMotorR->setSpeed(motorSpeedR);

  delay(100);
}

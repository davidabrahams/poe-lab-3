
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  AFMS.begin();
  myMotor->setSpeed(100);
  myMotor->run(FORWARD);
}

void loop() {
  delay(100);
}

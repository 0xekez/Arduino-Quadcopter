#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 150
#define SERVOMAX 600

uint8_t servonum = 0;

void setup() {
  Serial.begin(9600);

  pwm.begin();

  pwm.setPWMFreq(60); // these boys run @ 60hz
  delay(1000)
}

void loop() {
  for(uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++){
    pwm.setPWM(servonum, 0, pulselen);
    delay(30);
  }
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(servonum, 0, pulselen);
    delay(30);
  }
  delay(100);
}

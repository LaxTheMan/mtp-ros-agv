#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

class Motor {
  int pwm_pin;
  int dir_pin;
  int enca_pin;
  int encb_pin;

  int prevPos;
  float v;
  float vFilt;
  float vPrev;

  long prevTime;

  float velocityCPS;
  float velocityRPM;

  float filterVelocity(float velocityRaw);
public:
  volatile int pos_i;
  Motor(int PWM_PIN, int DIR_PIN, int ENCA_PIN, int ENCB_PIN);
  void setPWM(int pwm, int dir);

  void init();
  void update();

  float getVelocityCPS();
  float getVelocityRPM();

  int getCounts();
};

#endif
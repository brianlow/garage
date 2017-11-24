#include "maxsonar.h"
#include "application.h"

Maxsonar::Maxsonar(pin_t trigger, pin_t echo) :
 triggerPin(trigger), echoPin(echo)
{
}

void Maxsonar::init()
{
  pinMode(triggerPin, OUTPUT);
  digitalWriteFast(triggerPin, LOW);
  pinMode(echoPin, INPUT);
}

float Maxsonar::ping()
{
  uint32_t duration;
  float inches, cm;

  /* Trigger the sensor by sending a HIGH pulse of 20 or more microseconds */
  digitalWriteFast(triggerPin, HIGH);
  delayMicroseconds(22);
  digitalWriteFast(triggerPin, LOW);

  duration = pulseIn(echoPin, HIGH);

  // 147 ms per inch
  inches = duration / 147.0;
  cm = inches * 2.54;

  return cm;
}

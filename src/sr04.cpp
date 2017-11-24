#include "sr04.h"
#include "application.h"

SR04::SR04(pin_t trigger, pin_t echo) :
 triggerPin(trigger), echoPin(echo)
{
}

void SR04::init()
{
  pinMode(triggerPin, OUTPUT);
  digitalWriteFast(triggerPin, LOW);
  pinMode(echoPin, INPUT);
}

uint32_t SR04::ping()
{
  uint32_t duration, inches, cm;

  /* Trigger the sensor by sending a HIGH pulse of 10 or more microseconds */
  digitalWriteFast(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWriteFast(triggerPin, LOW);

  duration = pulseIn(echoPin, HIGH);

  /* Convert the time into a distance */
  // Sound travels at 1130 ft/s (73.746 us/inch)
  // or 340 m/s (29 us/cm), out and back so divide by 2
  // Ref: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  inches = duration / 74 / 2;
  cm = duration / 29 / 2;

  return cm;
}

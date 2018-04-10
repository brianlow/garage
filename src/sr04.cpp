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

  duration = pulseInWithTimeout(echoPin, HIGH, 40000);

  /* Convert the time into a distance */
  // Sound travels at 1130 ft/s (73.746 us/inch)
  // or 340 m/s (29 us/cm), out and back so divide by 2
  // Ref: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  inches = duration / 74 / 2;
  cm = duration / 29 / 2;

  return cm;
}

// Version of the built-in pulseIn() that adds a configurable timeout (in microseconds)
unsigned long SR04::pulseInWithTimeout(int pin, int value, int timeout) {

  unsigned long now = micros();
  while(pinReadFast(pin) == value) { // wait if pin is already HIGH when the function is called, but timeout if it never goes LOW
    if (micros() - now > timeout) {
      return 0;
    }
  }

  now = micros(); // could delete this line if you want only one timeout period from the start until the actual pulse width timing starts
  while (pinReadFast(pin) != value) { // pin is LOW, wait for it to go HIGH befor we start timing, but timeout if it never goes HIGH within the timeout period
    if (micros() - now > timeout) {
      return 0;
    }
  }

  now = micros();
  while (pinReadFast(pin) == value) { // start timing the HIGH pulse width, but time out if over timeout milliseconds
    if (micros() - now > timeout) {
      return 0;
    }
  }
  return micros() - now;
}

#include "sensor.h"

Sensor::Sensor(pin_t trigger, pin_t echo, int minCm, int maxCm) :
 sr04(SR04(trigger, echo)), minCm(minCm), maxCm(maxCm)
{
}

void Sensor::init()
{
  sr04.init();
}

void Sensor::measure()
{
  float rawCm = sr04.ping();
  int newCm = filter.in(rawCm);

  // do nothing until the filter is primed
  if (readings < filterSize) {
    readings++;
    return;
  }

  if (newCm != cm && (abs(cm - newCm) >= 2)) {
    cm = newCm;
    cmChanged = true;
    changed = true;
  }

  int newState;
  newState = minCm <= cm && cm <= maxCm ? ON : OFF;
  if (newState != state) {
    state = newState;
    stateChanged = true;
    changed = true;
  }
}

void Sensor::clearChanged() {
  cmChanged = false;
  stateChanged = false;
  changed = false;
}

void Sensor::setChanged() {
  cmChanged = true;
  stateChanged = true;
  changed = true;
}

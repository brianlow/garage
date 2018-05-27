#ifndef __SENSOR__
#define __SENSOR__

#include "application.h"
#include "sr04.h"
#include "median_filter.h"

class Sensor {

  public:
    Sensor(pin_t trigger, pin_t echo, int minCm, int maxCm);
    void init();
    void measure();
    void clearChanged();

    const static int OFF = -1;
    const static int UNKNOWN = 0;
    const static int ON = 1;

    int cm = -1;
    int state = UNKNOWN;
    bool cmChanged = false;
    bool stateChanged = false;

  private:
    SR04 sr04;

    int minCm;
    int maxCm;

    const int filterSize = 100;
    int readings = 0; // number of readings we've taken (only counts up to filterSize)
    MedianFilter filter = MedianFilter(filterSize, 0);
};

#endif // __SENSOR__

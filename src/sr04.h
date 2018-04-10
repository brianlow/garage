#ifndef __SR04__
#define __SR04__

#include "application.h"

class SR04 {

  public:
    SR04(pin_t trigger, pin_t echo);
    void init();
    uint32_t ping();

  private:
    pin_t triggerPin;
    pin_t echoPin;
    unsigned long pulseInWithTimeout(int pin, int value, int timeout);
};

#endif // __SR04__

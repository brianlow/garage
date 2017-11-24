#ifndef __MAXSONAR__
#define __MAXSONAR__

#include "application.h"

class Maxsonar {

  public:
    Maxsonar(pin_t trigger, pin_t echo);
    void init();
    float ping();

  private:
    pin_t triggerPin;
    pin_t echoPin;
};

#endif // __MAXSONAR__

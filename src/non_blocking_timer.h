#ifndef __NON_BLOCKING_TIMER__
#define __NON_BLOCKING_TIMER__

#include "application.h"

class NonBlockingTimer {

  typedef std::function<void(void)> timer_callback_fn;

  public:
    NonBlockingTimer(unsigned int period, timer_callback_fn callback);
    void start();
    void loop();

  private:
    void trigger();
    timer_callback_fn callback;
    Timer* timer;
    bool triggered;
};

#endif // __NON_BLOCKING_TIMER__

#include "non_blocking_timer.h"
#include "application.h"

NonBlockingTimer::NonBlockingTimer(unsigned int period, timer_callback_fn callback)
  : callback(callback), timer(new Timer(period, &NonBlockingTimer::trigger, *this)) 
{
}

void NonBlockingTimer::start()
{
  timer->start();
}

void NonBlockingTimer::loop()
{
  if (triggered) {
    triggered = false;
    callback();
  }
}

void NonBlockingTimer::trigger()
{
  triggered = true;
}

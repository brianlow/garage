/*
 * Based on https://github.com/dxinteractive/ResponsiveAnalogRead
 */

#ifndef __RESPONSIVE_FILTER_H
#define __RESPONSIVE_FILTER_H

#include <Arduino.h>

class ResponsiveFilter
{
  public:

    // sleepEnable - enabling sleep will cause values to take less time to stop changing and potentially stop changing more abruptly,
    //   where as disabling sleep will cause values to ease into their correct position smoothly
    // snapMultiplier - a value from 0 to 1 that controls the amount of easing
    //   increase this to lessen the amount of easing (such as 0.1) and make the responsive values more responsive
    //   but doing so may cause more noise to seep through if sleep is not enabled

    ResponsiveFilter(bool sleepEnable, int maxValue = 1024, int minValue = 0, float snapMultiplier = 0.01);

    inline int getValue() { return responsiveValue; } // get the responsive value from last update
    inline int getRawValue() { return rawValue; } // get the raw analogRead() value from last update
    inline bool hasChanged() { return responsiveValueHasChanged; } // returns true if the responsive value has changed during the last update
    inline bool isSleeping() { return sleeping; } // returns true if the algorithm is currently in sleeping mode
    void update(); // updates the value by performing an analogRead() and calculating a responsive value based off it
    void update(int rawValue); // updates the value accepting a value and calculating a responsive value based off it

    void setSnapMultiplier(float newMultiplier);
    inline void enableSleep() { sleepEnable = true; }
    inline void disableSleep() { sleepEnable = false; }
    inline void enableEdgeSnap() { edgeSnapEnable = true; }
    // edge snap ensures that values at the edges of the spectrum (0 and 1023) can be easily reached when sleep is enabled
    inline void disableEdgeSnap() { edgeSnapEnable = false; }
    inline void setActivityThreshold(float newThreshold) { activityThreshold = newThreshold; }
    // the amount of movement that must take place to register as activity and start moving the output value. Defaults to 4.0

  private:
    int minValue;
    int maxValue;
    float snapMultiplier;
    bool sleepEnable;
    float activityThreshold = 4.0;
    bool edgeSnapEnable = true;

    float smoothValue;
    unsigned long lastActivityMS;
    float errorEMA = 0.0;
    bool sleeping = false;

    int rawValue;
    int responsiveValue;
    int prevResponsiveValue;
    bool responsiveValueHasChanged;

    int getResponsiveValue(int newValue);
    float snapCurve(float x);
};

#endif

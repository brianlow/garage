#ifndef __COLOR__
#define __COLOR__

#include "application.h"

void hslToRgb(double h, double s, double l, byte rgb[]);
double hue2rgb(double p, double q, double t);

#endif // __COLOR__

/* HC-SR04 Ping / Range finder wiring:
 * -----------------------------------
 * Particle - HC-SR04
 *      GND - GND
 *      VIN - VCC
 *       D2 - TRIG
 *       D6 - ECHO
 *
 * Sparkfun Level Shifter
 * ----------------------
 * Particle - Sparkfun Level Shifter
 *      GND - HI GND
 *      GND - LO GND
 *      VIN - HI
 *      3v3 - LO

 * Sparkfun APA102 Led Strip
 * -------------------------
 * Particle - Sparkfun APA102 Led Strip
 *      VIN - VCC
 *      GND - GND
 *       A5 - Level Shifter LO1 - HI1 - 400 ohm resistor - Data
 *       A3 - Level Shifter LO2 - HI2 - Clock
 *
 * Maxbotics LV-MaxSonar-EZ1
 * -------------------------
 * Particle - MaxBotix Sensor
 *      VIN - BW (Pin 1)
 *       D1 - Level Shifter LO4 - HI4 - PW (Pin 2)
 *       D0 - Level Shifter L03 - HI3 - RX (Pin 4)
 *      VIN - +5V
 *      GND - GND
 *
 */

#include "application.h"
#include "sr04.h"
#include "maxsonar.h"
#include "color.h"
#include "responsive_filter.h"
#include <dotstar.h>


// Led Strip
int NUMPIXELS = 10;
int DATAPIN = A5;
int CLOCKPIN = A3;
float CM_PER_PIXEL = 2.54;
byte rgb[3];
Adafruit_DotStar strip = Adafruit_DotStar(NUMPIXELS, DATAPIN, CLOCKPIN);

// SR04 Sensor
pin_t SR04_TRIGGERPIN = D2;
pin_t SR04_ECHOPIN = D6;
SR04 sr04 = SR04(SR04_TRIGGERPIN, SR04_ECHOPIN);

// LV-MaxSonar-EZ1
pin_t MAXSONAR_TRIGGERPIN = D0;
pin_t MAXSONAR_ECHOPIN = D1;
Maxsonar maxsonar = Maxsonar(MAXSONAR_TRIGGERPIN, MAXSONAR_ECHOPIN);

ResponsiveFilter filter = ResponsiveFilter(true);
int cm = 0;

bool useMaxsonar = false;
int toggleSensor(String x) {
    useMaxsonar = !useMaxsonar;
    return 0;
}

void setup() {
  strip.begin();
  strip.show();

  sr04.init();
  maxsonar.init();

  Particle.variable("useMaxsonar", useMaxsonar);
  Particle.variable("cm", cm);
  Particle.function("toggleSensor", toggleSensor);

  delay(50);
}

void loop() {
  float rawCm = 0;
  if (useMaxsonar) {
      rawCm = maxsonar.ping();
  } else {
      rawCm = sr04.ping();
  }

  filter.update(rawCm);

  if (filter.hasChanged()) {
    cm = filter.getValue();

    char cm_string[10];
    itoa(cm, cm_string, 10);
    Particle.publish("distance-changed", cm_string);

    for (int i = 0; i < NUMPIXELS; i++) {
      distanceToRgb(cm + (i * CM_PER_PIXEL), rgb);
      strip.setPixelColor(i, rgb[0], rgb[2], rgb[1]);
    }
    strip.show();
  }

  delay(50);
}

void distanceToRgb(float cm, byte rgb[]) {
  float hue = 0.0;
  float saturation = 1.0;
  float lightness = 0.01;
  if (cm < 30) {
      hue = 0.0/360;
      lightness = 0.01;
  }
  else if (cm < 60) hue = 40.0/360.0;
  else if (cm < 90) hue = 120.0/360.0;
  else if (cm < 120) hue = 180.0/360.0;
  else {
      saturation = 0;
      lightness = 0;
  }
  hslToRgb(hue, saturation, lightness, rgb);
}

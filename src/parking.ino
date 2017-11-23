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
#include <dotstar.h>

float CM_PER_PIXEL = 2.54;

// Leds
int NUMPIXELS = 10;
int DATAPIN = A5;
int CLOCKPIN = A3;

// SR04 Sensor
pin_t TRIGGERPIN = D2;
pin_t ECHOPIN = D6;

// LV-MaxSonar-EZ1
pin_t MS_TRIGGERPIN = D0;
pin_t MS_ECHOPIN = D1;

byte rgb[3];

Adafruit_DotStar strip = Adafruit_DotStar(NUMPIXELS, DATAPIN, CLOCKPIN);

bool useMaxbotix = true;
int toggleSensor(String x) {
    useMaxbotix = !useMaxbotix;
    return 0;
}

void setup() {
  pinMode(TRIGGERPIN, OUTPUT);
  digitalWriteFast(TRIGGERPIN, LOW);
  pinMode(ECHOPIN, INPUT);

  pinMode(MS_TRIGGERPIN, OUTPUT);
  digitalWriteFast(MS_TRIGGERPIN, LOW);
  pinMode(MS_ECHOPIN, INPUT);

  strip.begin();
  strip.show();

  Particle.function("toggleSensor", toggleSensor);

  delay(50);
}

void loop() {
  float cm = 0;
  if (useMaxbotix) {
      cm = ms_ping();
  } else {
      cm = ping();
  }

  for (int i = 0; i < NUMPIXELS; i++) {
    distanceToRgb(cm + (i * CM_PER_PIXEL), rgb);
    strip.setPixelColor(i, rgb[0], rgb[2], rgb[1]);
  }
  strip.show();
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

uint32_t ping()
{
    uint32_t duration, inches, cm;

    /* Trigger the sensor by sending a HIGH pulse of 10 or more microseconds */
    digitalWriteFast(TRIGGERPIN, HIGH);
    delayMicroseconds(10);
    digitalWriteFast(TRIGGERPIN, LOW);

    duration = pulseIn(ECHOPIN, HIGH);

    /* Convert the time into a distance */
    // Sound travels at 1130 ft/s (73.746 us/inch)
    // or 340 m/s (29 us/cm), out and back so divide by 2
    // Ref: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
    inches = duration / 74 / 2;
    cm = duration / 29 / 2;

    return cm;
}

float ms_ping()
{
    uint32_t duration;
    float inches, cm;

    /* Trigger the sensor by sending a HIGH pulse of 20 or more microseconds */
    digitalWriteFast(MS_TRIGGERPIN, HIGH);
    delayMicroseconds(22);
    digitalWriteFast(MS_TRIGGERPIN, LOW);

    duration = pulseIn(MS_ECHOPIN, HIGH);

    // 147 ms per inch
    inches = duration / 147.0;
    cm = inches * 2.54;

    return cm;
}


/**
 * Converts an HSL color value to RGB. Conversion formula
 * adapted from http://en.wikipedia.org/wiki/HSL_color_space.
 * Assumes h, s, and l are contained in the set [0, 1] and
 * returns r, g, and b in the set [0, 255].
 *
 * @param   Number  h       The hue
 * @param   Number  s       The saturation
 * @param   Number  l       The lightness
 * @return  Array           The RGB representation
 */
void hslToRgb(double h, double s, double l, byte rgb[]) {
    double r, g, b;

    if (s == 0) {
        r = g = b = l; // achromatic
    } else {
        double q = l < 0.5 ? l * (1 + s) : l + s - l * s;
        double p = 2 * l - q;
        r = hue2rgb(p, q, h + 1/3.0);
        g = hue2rgb(p, q, h);
        b = hue2rgb(p, q, h - 1/3.0);
    }

    rgb[0] = r * 255;
    rgb[1] = g * 255;
    rgb[2] = b * 255;
}

double rotateHue(double x) {
    x = x + 0.01;
    if (x > 1.0) {
        x = 0.0;
    }
    return x;
}

double hue2rgb(double p, double q, double t) {
    if(t < 0) t += 1;
    if(t > 1) t -= 1;
    if(t < 1/6.0) return p + (q - p) * 6 * t;
    if(t < 1/2.0) return q;
    if(t < 2/3.0) return p + (q - p) * (2/3.0 - t) * 6;
    return p;
}

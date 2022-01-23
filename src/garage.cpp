/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "/Users/brian/dev/particle/garage/src/garage.ino"
/* HC-SR04 Ping / Range finder wiring:
 * -----------------------------------
 * Particle - HC-SR04
 *      GND - GND
 *      VIN - VCC
 *       D0 - TRIG
 *       D1 - ECHO
 *
 * HC-SR04 Ping / Range finder wiring:
 * -----------------------------------
 * Particle - HC-SR04
 *      GND - GND
 *      VIN - VCC
 *       D2 - TRIG
 *       D3 - ECHO
 *
 *
 * HC-SR04 Ping / Range finder wiring:
 * -----------------------------------
 * Particle - HC-SR04
 *      GND - GND
 *      VIN - VCC
 *       D4 - TRIG
 *       D5 - ECHO
 * *
 */

#include "application.h"
#include "sr04.h"
#include "sensor.h"
#include "median_filter.h"
#include "secrets.h"
#include "non_blocking_timer.h"
#include "Adafruit_IO_Client.h"

void measure();
void report();
void reReport();
int reportToParticleCloud(String unused);
void reportToAdafruit();
void setup();
void loop();
#line 36 "/Users/brian/dev/particle/garage/src/garage.ino"
pin_t DOOR_TRIGGERPIN = D0;
pin_t DOOR_ECHOPIN = D1;
Sensor door = Sensor(DOOR_TRIGGERPIN, DOOR_ECHOPIN, 0, 45);

pin_t STALL1_TRIGGERPIN = D2;
pin_t STALL1_ECHOPIN = D3;
Sensor stall1 = Sensor(STALL1_TRIGGERPIN, STALL1_ECHOPIN, 45, 200);

pin_t STALL2_TRIGGERPIN = D4;
pin_t STALL2_ECHOPIN = D5;
Sensor stall2 = Sensor(STALL2_TRIGGERPIN, STALL2_ECHOPIN, 45, 200);

TCPClient client;
Adafruit_IO_Client adafruit = Adafruit_IO_Client(client, IO_KEY);
Adafruit_IO_Feed doorFeed = adafruit.getFeed("garage-door");
Adafruit_IO_Feed stall1Feed = adafruit.getFeed("garage-stall-1");
Adafruit_IO_Feed stall2Feed = adafruit.getFeed("garage-stall-2");

char stringBuffer[200] = "";

void measure() {
  // door.measure();
  stall1.measure();
  stall2.measure();
}

void report() {
  if (door.changed || stall1.changed || stall2.changed) {

    char doorState[25] = "";
    if (door.state == Sensor::ON) {
      strcpy(doorState, "open");
    } else if (door.state == Sensor::OFF) {
      strcpy(doorState, "closed");
    } else {
      strcpy(doorState, "unknown");
    }

    char stall1State[25] = "";
    if (stall1.state == Sensor::ON) {
      strcpy(stall1State, "present");
    } else if (stall1.state == Sensor::OFF) {
      strcpy(stall1State, "not present");
    } else {
      strcpy(stall1State, "unknown");
    }

    char stall2State[25] = "";
    if (stall2.state == Sensor::ON) {
      strcpy(stall2State, "present");
    } else if (stall2.state == Sensor::OFF) {
      strcpy(stall2State, "not present");
    } else {
      strcpy(stall2State, "unknown");
    }

    JSONBufferWriter writer(stringBuffer, sizeof(stringBuffer));
    writer.beginObject();
        writer.name("door").value(doorState);
        writer.name("doorCm").value(door.cm);
        writer.name("stall1").value(stall1State);
        writer.name("stall1Cm").value(stall1.cm);
        writer.name("stall2").value(stall2State);
        writer.name("stall2Cm").value(stall2.cm);
    writer.endObject();
    writer.buffer()[std::min(writer.bufferSize(), writer.dataSize())] = 0; // null terminate
    Particle.publish("state-update", stringBuffer);

    door.clearChanged();
    stall1.clearChanged();
    stall2.clearChanged();
  }
}

void reReport() {
  door.setChanged();
  stall1.setChanged();
  stall2.setChanged();
}

int reportToParticleCloud(String unused) {
  reReport();
  return 0;
}

void reportToAdafruit() {
  doorFeed.send(door.cm);
  stall1Feed.send(stall1.cm);
  stall2Feed.send(stall2.cm);
}

NonBlockingTimer measureTimer(50, measure);
NonBlockingTimer reportTimer(500L, report);
NonBlockingTimer reReportTimer(1000 * 60 * 60, reReport);
NonBlockingTimer reportToAdafruitTimer(1000 * 15, reportToAdafruit);

void setup() {
  // WiFi.selectAntenna(ANT_EXTERNAL);

  door.init();
  stall1.init();
  stall2.init();
  measureTimer.start();
  reportTimer.start();
  reReportTimer.start();
  reportToAdafruitTimer.start();

  Particle.function("report", reportToParticleCloud);

  adafruit.begin();
}

void loop() {
  measureTimer.loop();
  reportTimer.loop();
  reReportTimer.loop();
  reportToAdafruitTimer.loop();
}

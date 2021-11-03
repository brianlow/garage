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

#include "ArduinoJson.h"
#include "application.h"
#include "sr04.h"
#include "sensor.h"
#include "median_filter.h"
#include "secrets.h"
#include "non_blocking_timer.h"

pin_t DOOR_TRIGGERPIN = D0;
pin_t DOOR_ECHOPIN = D1;
Sensor door = Sensor(DOOR_TRIGGERPIN, DOOR_ECHOPIN, 0, 45);

pin_t STALL1_TRIGGERPIN = D2;
pin_t STALL1_ECHOPIN = D3;
Sensor stall1 = Sensor(STALL1_TRIGGERPIN, STALL1_ECHOPIN, 45, 200);

pin_t STALL2_TRIGGERPIN = D4;
pin_t STALL2_ECHOPIN = D5;
Sensor stall2 = Sensor(STALL2_TRIGGERPIN, STALL2_ECHOPIN, 45, 200);

char stringBuffer[200] = "";
StaticJsonDocument<200> doc;


void measure() {
  door.measure();
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

    doc["door"] = doorState;
    doc["doorCm"] = door.cm;
    doc["stall1"] = stall1State;
    doc["stall1Cm"] = stall1.cm;
    doc["stall2"] = stall2State;
    doc["stall2Cm"] = stall2.cm;
    serializeJson(doc, stringBuffer);

    //sprintf(stringBuffer, "{\"door\":\"%s\",\"stall1\":\"%s\",\"stall2\":\"%s\"}", doorState, stall1State, stall2State);
    Particle.publish("state-update", stringBuffer, 60, PRIVATE);

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

NonBlockingTimer measureTimer(50, measure);
NonBlockingTimer reportTimer(500L, report);
NonBlockingTimer reReportTimer(1000 * 60 * 60, reReport);

void setup() {
  // WiFi.selectAntenna(ANT_EXTERNAL);

  door.init();
  stall1.init();
  stall2.init();
  measureTimer.start();
  reportTimer.start();
  reReportTimer.start();

  Particle.function("report", reportToParticleCloud);
}

void loop() {
  measureTimer.loop();
  reportTimer.loop();
  reReportTimer.loop();
}

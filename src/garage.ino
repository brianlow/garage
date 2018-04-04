// event when open for 30s, 5min, 30min, 1h, every 2 hours
// Blynk - colorize door status
// Blynk - table with recently opened times (and durations)

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
#include "median_filter.h"
#include <blynk.h>
#include <MQTT.h>

// Blynk
char auth[] = "82e05781d4ad49c9babe4eebf5565640";
WidgetTerminal terminal(V2);

// Onboard Led
int ONBOARD_LED = D7;

// SR04 Sensor
pin_t SR04_TRIGGERPIN = D2;
pin_t SR04_ECHOPIN = D6;
SR04 sr04 = SR04(SR04_TRIGGERPIN, SR04_ECHOPIN);

// LV-MaxSonar-EZ1
pin_t MAXSONAR_TRIGGERPIN = D0;
pin_t MAXSONAR_ECHOPIN = D1;
Maxsonar maxsonar = Maxsonar(MAXSONAR_TRIGGERPIN, MAXSONAR_ECHOPIN);

const int filterSize = 100;
MedianFilter filter = MedianFilter(filterSize, 0);
int cm = 0;
int openDoorThreshold = 20;  // distances below this indicate open door
int readings = 0; // number of readings we've taken (only counts up to filterSize)

const int CLOSED = -1;
const int UNKNOWN = 0;
const int OPEN = 1;
int state = UNKNOWN;

bool useMaxsonar = false;
int toggleSensor(String x) {
    useMaxsonar = !useMaxsonar;
    return 0;
}

BlynkTimer timers;
int lastReportedCm = 0;
char stateStr[50] = "";
char lastReportedStateStr[50] = "";
unsigned long openSince;
int notificationTimer = -1;  // timer id to be used with BlynkTimer
const int NOTIFICATIONS_LENGTH = 4;
unsigned long notifications[NOTIFICATIONS_LENGTH] = {5, 15, 30, 60};
int notificationPriority[NOTIFICATIONS_LENGTH] = {0, 0, 1, 1};
int notificationIndex = 0; // index into above array indicating current timeout

void mqtt_receive(char* topic, byte* payload, unsigned int length);
void mqtt_receive(char* topic, byte* payload, unsigned int length) {};
// byte mqttServer[] = {192,168,1,95};
// MQTT mqtt(mqttServer, 1883, mqtt_receive);
MQTT mqtt("nas", 1883, mqtt_receive);

void setup() {
  sr04.init();
  maxsonar.init();

  pinMode(ONBOARD_LED, OUTPUT);
  digitalWrite(ONBOARD_LED, LOW);

  Particle.variable("useMaxsonar", useMaxsonar);
  Particle.variable("cm", cm);
  Particle.variable("state", state);
  Particle.function("toggleSensor", toggleSensor);

  delay(250);

  Blynk.begin(auth);

  mqtt.connect("photon1_" + String(Time.now()));
  if (mqtt.isConnected()) {
    mqtt.publish("/photon1","hello world 3");
  }

  timers.setInterval(500L, report);
}

void loop() {
  float rawCm = useMaxsonar ? maxsonar.ping() : sr04.ping();

  cm = filter.in(rawCm);

  int new_state;
  if (readings < filterSize) {
    readings++;
    new_state = UNKNOWN;
  } else {
    new_state = cm < openDoorThreshold ? OPEN : CLOSED;
  }

  if (new_state == OPEN) {
    if (state == CLOSED) {
      pushoverPush("Garage door opened", 0);
    }
    if (state != OPEN) {
      openSince = millis();
      notificationIndex = 0;
      notificationTimer = timers.setTimeout(notifications[0] * 1000, notify);
    }
    digitalWrite(ONBOARD_LED, HIGH);
    state = OPEN;
  } else if (new_state == CLOSED) {
    if (state == OPEN) {
      pushoverPush("Garage door closed", -1);
    }
    digitalWrite(ONBOARD_LED, LOW);
    state = CLOSED;
    if (notificationTimer >= 0) timers.deleteTimer(notificationTimer);
  }

  timers.run();
  Blynk.run();

  delay(50);
}

void report() {
  if (cm != lastReportedCm) {
    Blynk.virtualWrite(V0, cm);
    lastReportedCm = cm;
  }

  calcStateStr();
  if (strcmp(stateStr, lastReportedStateStr) != 0) {
    Blynk.virtualWrite(V1, stateStr);
    strcpy(lastReportedStateStr, stateStr);
  }
}

void calcStateStr() {
  if (state == CLOSED) {
    strcpy(stateStr, "Closed");
  } else if (state == OPEN) {
    unsigned long seconds = (millis() - openSince) / 1000;
    unsigned long minutes = seconds / 60;
    unsigned long hours = minutes / 60;
    if (seconds < 60) {
      sprintf(stateStr, "Open for %d second%s", seconds, (seconds==1 ? "" : "s"));
    } else if (minutes < 60) {
      sprintf(stateStr, "Open for %d minute%s", minutes, (minutes==1 ? "" : "s"));
    } else {
      sprintf(stateStr, "Open for %d hour%s", hours, (hours==1 ? "" : "s"));
    }
  } else {
    strcpy(stateStr, "Not sure");
  }
}

void notify() {
  calcStateStr();
  pushoverPush(stateStr, notificationPriority[notificationIndex]);

  unsigned long duration;
  if (notificationIndex + 1 == NOTIFICATIONS_LENGTH) {
    duration = notifications[NOTIFICATIONS_LENGTH-1];
  } else {
    notificationIndex++;
    duration = notifications[notificationIndex] - notifications[notificationIndex-1];
  }
  notificationTimer = timers.setTimeout(duration * 1000, notify);
}

void pushoverPush(char* message, int priority) {
  char data[100];
  sprintf(data, "{\"message\":\"%s\", \"priority\": %d}", message, priority);
  Particle.publish("pushover-push", data);
}

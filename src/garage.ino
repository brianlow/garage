// HA with SSL
// add 2nd parking sensor


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
#include "secrets.h"
#include <blynk.h>
#include <MQTT.h>

// Blynk
char auth[] = BLYNK_AUTH;
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

const int filterSize = 25;
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

int lastReportedCm = 0;
char stringBuffer[25] = "";
char stateStr[50] = "";
char lastReportedStateStr[50] = "";
unsigned long openSince;

bool measureFlag;
void setMeasureFlag() {
  measureFlag = true;
}
Timer measureTimer(50, setMeasureFlag);

bool reportFlag;
void setReportFlag() {
  reportFlag = true;
}
Timer reportTimer(500L, setReportFlag);

void mqtt_receive(char* topic, byte* payload, unsigned int length);
void mqtt_receive(char* topic, byte* payload, unsigned int length) {};
MQTT mqtt(MQTT_SERVER, 1883, mqtt_receive);

void mqtt_connect(String name, String willTopic, String willMessage) {
  // unique_name = name + "_" + String(Time.now())
  mqtt.connect(name, NULL, NULL, willTopic, MQTT::QOS0, true, willMessage, true);
}

void mqtt_pub(char* topic, char* message) {
  bool retain = true;
  if (mqtt.isConnected()) {
    mqtt.publish(topic, (uint8_t*)message, strlen(message), retain);
  }
}
void mqtt_pub(char* topic, int) {
  sprintf(stringBuffer, "%d", cm);
  mqtt_pub(topic, stringBuffer);
}

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

  mqtt_connect("photon1", "home/device/photon_garage/available", "offline");

  mqtt_pub("home/cover/garage_door/config", "{\"name\": \"garage_door\", \"availability_topic\": \"home/device/photon_garage/available\"}");
  mqtt_pub("home/sensor/parking1_distance/config", "{\"name\": \"parking1_distance\", \"unit_of_measurement\": \"cm\", \"availability_topic\": \"home/device/photon_garage/available\"}");
  mqtt_pub("home/binary_sensor/parking1/config", "{\"name\": \"parking1\", \"device_class\": \"occupancy\", \"availability_topic\": \"home/device/photon_garage/available\"}");

  mqtt_pub("home/device/photon_garage/available", "online");

  measureTimer.start();
  reportTimer.start();
}

// home/device/photon_garage/available

// home/cover/garage_door/config {"name": "garage_door", "availability_topic": "home/device/photon_garage/available"}
// home/cover/garage_door/state open|closed

// home/sensor/parking1_distance/config {"name": "Vincci's Parking Spot Distance", "unit_of_measurement": "cm", "availability_topic": "home/device/photon_garage/available"}
// home/sensor/parking1_distance/state 55

// home/binary_sensor/parking1/config {"name": "Vincci's Parking Spot", "device_class":"occupancy", "icon": "mdi:car", "availability_topic": "home/device/photon_garage/available"}
// home/binary_sensor/parking1/state on|off


void loop() {

  Blynk.run();
  mqtt.loop();

  if (measureFlag) {
    measureFlag = false;
    measure();
  }

  if (reportFlag) {
    reportFlag = false;
    report();
  }
}

void measure() {
  float rawCm = useMaxsonar ? maxsonar.ping() : sr04.ping();

  cm = filter.in(rawCm);

  int new_state;
  if (readings < filterSize) {
    readings++;
    new_state = UNKNOWN;
  } else {
    new_state = cm < openDoorThreshold ? OPEN : CLOSED;
  }

  if (new_state == OPEN && state != OPEN) {
    mqtt_pub("home/cover/garage_door/state", "open");
    openSince = millis();
    digitalWrite(ONBOARD_LED, HIGH);
    state = OPEN;
  } else if (new_state == CLOSED && state != CLOSED) {
    mqtt_pub("home/cover/garage_door/state", "closed");
    digitalWrite(ONBOARD_LED, LOW);
    state = CLOSED;
  }
}

void report() {
  if (cm != lastReportedCm) {
    Blynk.virtualWrite(V0, cm);
    mqtt_pub("home/sensor/parking1_distance/state", cm);
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

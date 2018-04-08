// HA with SSL
// add 2nd parking sensor


/* HC-SR04 Ping / Range finder wiring:
 * -----------------------------------
 * Particle - HC-SR04
 *      GND - GND
 *      VIN - VCC
 *       D2 - TRIG
 *       D6 - ECHO
 * *
 */

#include "application.h"
#include "sr04.h"
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

const int filterSize = 25;
MedianFilter filter = MedianFilter(filterSize, 0);
int cm = 0;
int openDoorThreshold = 20;  // distances below this indicate open door
int parkingOccupiedThreshold = 260; // distance below this indicates car is present
int readings = 0; // number of readings we've taken (only counts up to filterSize)

const int CLOSED = -1;
const int UNKNOWN = 0;
const int OPEN = 1;
int state = UNKNOWN;

const int EMPTY = -1;
const int OCCUPIED = 1;
int parking_state = UNKNOWN;

int lastReportedCm = 0;
char stringBuffer[25] = "";

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

  pinMode(ONBOARD_LED, OUTPUT);
  digitalWrite(ONBOARD_LED, LOW);

  Particle.variable("cm", cm);
  Particle.variable("state", state);

  delay(250);

  Blynk.begin(auth);

  mqtt_connect("photon1", "home/device/photon_garage/available", "offline");

  mqtt_pub("home/cover/garage_door/config", "{\"name\": \"garage_door\", \"availability_topic\": \"home/device/photon_garage/available\"}");
  mqtt_pub("home/sensor/stall1/config", "{\"name\": \"stall1\", \"unit_of_measurement\": \"cm\", \"availability_topic\": \"home/device/photon_garage/available\"}");
  mqtt_pub("home/binary_sensor/stall1/config", "{\"name\": \"stall1\", \"device_class\": \"occupancy\", \"availability_topic\": \"home/device/photon_garage/available\"}");

  mqtt_pub("home/device/photon_garage/available", "online");

  measureTimer.start();
  reportTimer.start();
}

void loop() {

  Blynk.run();
  mqtt.loop();

  if (measureFlag) {
    measureFlag = false;
    measure();
    calculate_door();
    calculate_parking();
  }

  if (reportFlag) {
    reportFlag = false;
    report();
  }
}

void measure() {
  float rawCm = sr04.ping();

  cm = filter.in(rawCm);
}

void calculate_door() {
  int new_state;
  if (readings < filterSize) {
    readings++;
    new_state = UNKNOWN;
  } else {
    new_state = cm < openDoorThreshold ? OPEN : CLOSED;
  }

  if (new_state == OPEN && state != OPEN) {
    mqtt_pub("home/cover/garage_door/state", "open");
    digitalWrite(ONBOARD_LED, HIGH);
    state = OPEN;
  } else if (new_state == CLOSED && state != CLOSED) {
    mqtt_pub("home/cover/garage_door/state", "closed");
    digitalWrite(ONBOARD_LED, LOW);
    state = CLOSED;
  }
}

void calculate_parking() {
  int new_state;
  if (readings < filterSize || state == OPEN) {
    readings++;
    new_state = UNKNOWN;
  } else {
    new_state = cm < parkingOccupiedThreshold ? OCCUPIED : EMPTY;
  }

  if (new_state == OCCUPIED && parking_state != OCCUPIED) {
    mqtt_pub("home/binary_sensor/stall1/state", "ON");
    parking_state = OPEN;
  } else if (new_state == EMPTY && parking_state != EMPTY) {
    mqtt_pub("home/binary_sensor/stall1/state", "OFF");
    parking_state = CLOSED;
  }
}

void report() {
  if (cm != lastReportedCm) {
    Blynk.virtualWrite(V0, cm);
    mqtt_pub("home/sensor/stall1/state", cm);
    lastReportedCm = cm;
  }
}

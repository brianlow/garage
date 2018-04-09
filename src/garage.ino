// HA with SSL
// add 3rd parking sensor

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
 * *
 */

#include "application.h"
#include "sr04.h"
#include "sensor.h"
#include "median_filter.h"
#include "secrets.h"
#include <blynk.h>
#include <MQTT.h>

// Blynk
char auth[] = BLYNK_AUTH;
WidgetTerminal terminal(V2);

// Onboard Led
int ONBOARD_LED = D7;

pin_t DOOR_TRIGGERPIN = D0;
pin_t DOOR_ECHOPIN = D1;
Sensor door = Sensor(DOOR_TRIGGERPIN, DOOR_ECHOPIN, 0, 20);

pin_t STALL1_TRIGGERPIN = D2;
pin_t STALL1_ECHOPIN = D3;
Sensor stall1 = Sensor(STALL1_TRIGGERPIN, STALL1_ECHOPIN, 20, 220);

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

void mqtt_pub(char* topic, const char* message) {
  bool retain = true;
  if (mqtt.isConnected()) {
    mqtt.publish(topic, (uint8_t*)message, strlen(message), retain);
  }
}
void mqtt_pub(char* topic, int cm) {
  sprintf(stringBuffer, "%d", cm);
  mqtt_pub(topic, stringBuffer);
}

void setup() {
  door.init();
  stall1.init();

  pinMode(ONBOARD_LED, OUTPUT);
  digitalWrite(ONBOARD_LED, LOW);

  delay(250);

  Blynk.begin(auth);

  mqtt_connect("photon1", "home/device/photon_garage/available", "offline");

  mqtt_pub("home/cover/garage_door/config", "{\"name\": \"garage_door\", \"availability_topic\": \"home/device/photon_garage/available\"}");
  mqtt_pub("home/sensor/garage_door/config", "{\"name\": \"garage_door\", \"unit_of_measurement\": \"cm\", \"availability_topic\": \"home/device/photon_garage/available\"}");
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
    door.measure();
    stall1.measure();
  }

  if (reportFlag) {
    reportFlag = false;
    reportDoor();
    reportStall1();
  }
}

void reportDoor() {
  if (door.cmChanged) {
    Blynk.virtualWrite(V0, door.cm);
    mqtt_pub("home/sensor/garage_door/state", door.cm);
  }

  if (door.stateChanged && door.state != Sensor::UNKNOWN) {
    mqtt_pub("home/cover/garage_door/state", (door.state == Sensor::ON ? "open" : "closed"));
  }

  door.clearChanged();
}

void reportStall1() {
  if (stall1.cmChanged) {
    Blynk.virtualWrite(V0, stall1.cm);
    mqtt_pub("home/sensor/stall1/state", stall1.cm);
  }

  if (stall1.stateChanged && stall1.state != Sensor::UNKNOWN) {
    mqtt_pub("home/binary_sensor/stall1/state", (stall1.state == Sensor::ON ? "ON" : "OFF"));
  }

  stall1.clearChanged();
}

// HA with SSL

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

pin_t DOOR_TRIGGERPIN = D0;
pin_t DOOR_ECHOPIN = D1;
Sensor door = Sensor(DOOR_TRIGGERPIN, DOOR_ECHOPIN, 0, 45);

pin_t STALL1_TRIGGERPIN = D2;
pin_t STALL1_ECHOPIN = D3;
Sensor stall1 = Sensor(STALL1_TRIGGERPIN, STALL1_ECHOPIN, 45, 200);

pin_t STALL2_TRIGGERPIN = D4;
pin_t STALL2_ECHOPIN = D5;
Sensor stall2 = Sensor(STALL2_TRIGGERPIN, STALL2_ECHOPIN, 45, 200);

char stringBuffer[25] = "";

bool reconnectFlag;
void setReconnectFlag() {
  reconnectFlag = true;
}
Timer reconnectTimer(5000L, setReconnectFlag);

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

bool reReportFlag;
void setReReportFlag() {
  reReportFlag = true;
}
Timer reReportTimer(1000 * 60 * 60, setReReportFlag);

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
  stall2.init();
  Blynk.begin(auth);
  mqtt_reconnect();
  reconnectTimer.start();
  measureTimer.start();
  reportTimer.start();
}

void loop() {

  Blynk.run();
  mqtt.loop();

  if (reconnectFlag) {
    reconnectFlag = false;
    mqtt_reconnect();
  }

  if (measureFlag) {
    measureFlag = false;
    door.measure();
    stall1.measure();
    stall2.measure();
  }

  if (reportFlag) {
    reportFlag = false;
    reportDoor();
    reportStall1();
    reportStall2();
  }

  if (reReportFlag) {
    reReportFlag = false;
    door.setChanged();
    stall1.setChanged();
    stall2.setChanged();
  }
}

void mqtt_reconnect() {
  if (!mqtt.isConnected()) {
    Particle.publish("mqtt-reconnect");

    mqtt_connect("photon1", "home/device/photon_garage/available", "offline");

    mqtt_pub("home/cover/garage_door/config", "{\"name\": \"garage_door\", \"availability_topic\": \"home/device/photon_garage/available\"}");
    mqtt_pub("home/sensor/garage_door/config", "{\"name\": \"garage_door\", \"unit_of_measurement\": \"cm\", \"availability_topic\": \"home/device/photon_garage/available\"}");
    mqtt_pub("home/sensor/stall1/config", "{\"name\": \"stall1\", \"unit_of_measurement\": \"cm\", \"availability_topic\": \"home/device/photon_garage/available\"}");
    mqtt_pub("home/binary_sensor/stall1/config", "{\"name\": \"stall1\", \"device_class\": \"occupancy\", \"availability_topic\": \"home/device/photon_garage/available\"}");
    mqtt_pub("home/sensor/stall2/config", "{\"name\": \"stall2\", \"unit_of_measurement\": \"cm\", \"availability_topic\": \"home/device/photon_garage/available\"}");
    mqtt_pub("home/binary_sensor/stall2/config", "{\"name\": \"stall2\", \"device_class\": \"occupancy\", \"availability_topic\": \"home/device/photon_garage/available\"}");

    mqtt_pub("home/device/photon_garage/available", "online");
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
    Blynk.virtualWrite(V1, stall1.cm);
    mqtt_pub("home/sensor/stall1/state", stall1.cm);
  }

  if (stall1.stateChanged && stall1.state != Sensor::UNKNOWN) {
    mqtt_pub("home/binary_sensor/stall1/state", (stall1.state == Sensor::ON ? "ON" : "OFF"));
  }

  stall1.clearChanged();
}

void reportStall2() {
  if (stall2.cmChanged) {
    Blynk.virtualWrite(V2, stall2.cm);
    mqtt_pub("home/sensor/stall2/state", stall2.cm);
  }

  if (stall2.stateChanged && stall2.state != Sensor::UNKNOWN) {
    mqtt_pub("home/binary_sensor/stall2/state", (stall2.state == Sensor::ON ? "ON" : "OFF"));
  }

  stall2.clearChanged();
}

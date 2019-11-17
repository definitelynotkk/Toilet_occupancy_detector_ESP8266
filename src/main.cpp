#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <VL53L0X.h>
#include "string.h"
#include <SimpleTimer.h>
#define BUTTON 0         //D3
#define LED_Indicator 15 //D8
#define XSHUT1_PIN 12
#define XSHUT2_PIN 14
#define LONG_RANGE

// #define HIGH_SPEED
#define HIGH_ACCURACY

const char *ssid = "Your ssid";
const char *password = "password";
const char *mqtt_server = "10.0.0.20";
const char *mqtt_user = "mqtt";
const char *mqtt_password = "mqtt";
const int mqtt_port = 1883;
const char *client_id = "peoplecounter";
const char *TOPIC = "VL53L0X/toilet";
const char *SUB_TOPIC = "VL53L0X/toilet/command";
const char *STATUS_TOPIC = "VL53L0X/toilet/status";
bool requestRestart = false;
unsigned long run_time;

unsigned long buttonTime;
unsigned long longpressTime = 2000;
boolean buttonPress = false;
boolean longPress = false;

VL53L0X sensor1;
VL53L0X sensor2;
int last_dist_1;
int last_dist_2;
int sensor1dist = 0;
int sensor2dist = 0;
long sensor1time;
long sensor2time;
int peopleNum = 0;
int delay_time = 0;
int peopleNumold = 0;

int change_threshold = 350;
unsigned long count = 0;

WiFiClient wifiClient;
PubSubClient client(wifiClient);
SimpleTimer timer;



void blinkLED(int pin, int duration, int cycle)
{
  Serial.println("blink led");
  Serial.print(pin);
  Serial.print(duration);
  Serial.print(cycle);
  for (int i = 0; i < cycle; i++)
  {
    digitalWrite(pin, HIGH);
    delay(duration);
    digitalWrite(pin, LOW);
    delay(duration);
    Serial.print(i);
  }
}

void button()
{
  if (digitalRead(BUTTON) == LOW)
  {
    delay(50);
    if (digitalRead(BUTTON) == LOW)
    {
      if (buttonPress == false)
      {
        buttonPress = true;
        buttonTime = millis();
      }
      buttonPress = true;
    }
    else
    {
      buttonPress = false;
    }
    
  }
  //long press to reboot
  if ((buttonPress == true) && (millis() - longpressTime > buttonTime) && (longPress == false) && (digitalRead(BUTTON) == LOW))
  {
    Serial.println("long pressed");
    longPress = true;
    blinkLED(LED_Indicator, 100, 10);
    delay(200);
    client.publish(STATUS_TOPIC, "manual reboot");
    peopleNum = 0;
    client.publish(TOPIC, "0");
    requestRestart = true;
  }
  //single press to reset people number
  if ((buttonPress == true) && (digitalRead(BUTTON) == HIGH) && (longPress == false))
  {
    blinkLED(LED_Indicator, 200, 3);
    delay(200);
    digitalWrite(LED_Indicator, LOW);
    client.publish(STATUS_TOPIC, "manual reset peoplecounter");
    client.publish(TOPIC, "0");
    peopleNum = 0;
    buttonPress = false;
    longPress = false;
    sensor1dist = 2001;
    sensor2dist = 2001;
    delay(2000);
  }
  
}

/*
only support one person crossing at a time.
scenarios:
1 enter  //working 
sensor1:-----111111--------
sensor2:---------111111----
2 leave  //working 
sensor1:--------1111111----
sensor2:----1111111--------

3 half way turn back  // NOT IMPLEMENTED YET.
sensor1:----1111111-------
sensor2:------1111--------

4 if sensors is far from each other  //should not work with current implementation
sensor1:----111111--------------     //but this sensor has a FOV of 25 degree, it will work in some cases.
sensor2:-------------1111111----
*/

void onchange()
{
  //when people number changes
  //turn on led if number of people > 0
  //send mqtt message to trigger other home automation actions.
  if (peopleNumold != peopleNum)
  {
    if (peopleNum == 0)
    {
      digitalWrite(LED_Indicator, LOW);
    }
    if (peopleNum > 0)
    {

      digitalWrite(LED_Indicator, HIGH);
    }

    String payload = String(peopleNum);
    // Serial.println(payload.c_str());
    client.publish(TOPIC, payload.c_str());
  }
}
void peopleCounter()
{
  yield();
  sensor1dist = sensor1.readRangeContinuousMillimeters();
  sensor2dist = sensor2.readRangeContinuousMillimeters();

  if (sensor1dist > 2001)
  // eliminate sensor jitter
  // this sensor is only capable of measuring 0-2M
  // any data beyond that value is considered garbage.
  {
    sensor1dist = 2001;
  }
  if (sensor2dist > 2001)
  {
    sensor2dist = 2001;
  }

  peopleNumold = peopleNum;
  if ((sensor1dist > last_dist_1 + change_threshold && sensor1dist > 0) || (sensor1dist < last_dist_1 - change_threshold && sensor1dist > 0))
  {
    sensor1time = millis();
    last_dist_1 = sensor1dist;
    if (labs(sensor2time - sensor1time) < 1000)
    {
      if (sensor1time > sensor2time && sensor1dist < 2001 && sensor2dist < 2001)
      {

        peopleNum--;
        if (peopleNum < 0)
        {
          peopleNum = 0;
        }
        delay(1000);
        sensor1dist = 2001;
        sensor2dist = 2001;
      }
    }
  }
  if ((sensor2dist > last_dist_2 + change_threshold && sensor2dist > 0) || (sensor2dist < last_dist_2 - change_threshold && sensor2dist > 0))
  {
    sensor2time = millis();
    last_dist_2 = sensor2dist;
    if (labs(sensor2time - sensor1time) < 1000)
    {
      if (sensor2time > sensor1time && sensor1dist < 2001 && sensor2dist < 2001)
      {
        peopleNum++;
        delay(1000);
        sensor1dist = 2001;
        sensor2dist = 2001;
      }
    }
    onchange();
  }
}



void setup_wifi()
{
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  int connect = 0;
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(++connect);
    Serial.print(" ");
  }

  Serial.println("\n");
  Serial.println("Connection established");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
void getMaxdist()
{
  for (size_t i = 0; i < 20; i++)
  {
  }
}

void mqtt_reconnect()
{
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(client_id, mqtt_user, mqtt_password))
    {
      Serial.println("connected");
      client.subscribe(SUB_TOPIC);
      client.publish(STATUS_TOPIC, "connected");
    }
    else
    {
      Serial.print("failed connecting mqtt");
      Serial.print(client.state());
      Serial.println("retry");
      // Wait 2 seconds before retrying
      delay(2000);
    }
  }
}

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.println("callback ...");
  String mqtt_message;
  String mqttTopic = topic;
  for (int i = 0; i < (int)length; i++)
  {
    Serial.println((char)payload[i]);
    mqtt_message += (char)payload[i];
  }
  Serial.print("mqtt_message:");
  Serial.println(mqtt_message);
  // if (mqtt_message.equals(String("reset")))
  // {
  //   Serial.println("Reset ...");
  //   requestRestart = true;
  // }
  if (mqttTopic == "VL53L0X/toilet/command")
  {
    if (mqtt_message.equals(String("reset")))
    {
      peopleNum = 0;
      onchange();
    }
    if (mqtt_message.equals(String("reboot")))
    {
      requestRestart = true;
      peopleNum = 0;
      client.publish(TOPIC, "0");
    }
  }
}



void sensor_setup()
{
  pinMode(XSHUT2_PIN, OUTPUT);
  pinMode(XSHUT1_PIN, OUTPUT);
  digitalWrite(XSHUT1_PIN, LOW);
  digitalWrite(XSHUT2_PIN, LOW);
  delay(500);
  Wire.begin();
  pinMode(XSHUT1_PIN, INPUT);
  delay(150);
  Serial.print("sensor1 initing.....");
  sensor1.init(true);
  delay(100);
  sensor1.setAddress((uint8_t)22);
  Serial.println("done.");
  pinMode(XSHUT2_PIN, INPUT);
  delay(150);
  sensor2.init(true);
  Serial.print("sensor2 initing.....");
  delay(100);
  sensor2.setAddress((uint8_t)25);
  Serial.println("done.");
  sensor1.setTimeout(500);
  sensor2.setTimeout(500);

#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor1.setSignalRateLimit(0.2);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor2.setSignalRateLimit(0.2);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  sensor1.setMeasurementTimingBudget(20000);
  sensor2.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 35 ms
  sensor1.setMeasurementTimingBudget(35000);
  sensor2.setMeasurementTimingBudget(35000);
#endif
  sensor1.startContinuous();
  sensor2.startContinuous();
}

void setup()
{
  pinMode(LED_Indicator, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  digitalWrite(LED_Indicator, LOW);
  Serial.begin(115200);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.mode(WIFI_STA);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  ArduinoOTA.setHostname("human counter");
  ArduinoOTA.begin();
  delay(500);
  sensor_setup();
  timer.setInterval(100, button);
}

void loop()
{
  if (!client.connected())
  {
    mqtt_reconnect();
  }
  ArduinoOTA.handle();
  client.loop();
  peopleCounter();
  if (requestRestart)
  {
    Serial.println("reboot");
    // client.publish(STATUS_TOPIC, "rebooting");
    ESP.reset();
  }
  delay(delay_time);
  timer.run();
}

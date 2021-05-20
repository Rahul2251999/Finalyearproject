//#include <SoftwareSerial.h>
//SoftwareSerial mySerial(14, 12);

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN D2

#define DHTTYPE    DHT11

DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;

#include <Arduino.h>
#include "Ubidots.h"

const char* UBIDOTS_TOKEN = "BBFF-g6IKYI9b6TvUqp2prrsNyQrTUggMG0";  // Put here your Ubidots TOKEN
const char* WIFI_SSID = "Oneplus 7";      // Put here your Wi-Fi SSID
const char* WIFI_PASS = "12345678";      // Put here your Wi-Fi password
Ubidots ubidots(UBIDOTS_TOKEN, UBI_HTTP);

void setup() {
  Serial.begin(9600);
  ubidots.wifiConnect(WIFI_SSID, WIFI_PASS);
  dht.begin();
  Serial.println(F("DHTxx Unified Sensor Example"));
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);
  delayMS = sensor.min_delay / 1000;
  while (!Serial);
  pinMode(D0,INPUT);
  pinMode(A0,INPUT);
  pinMode(D1,INPUT);
  pinMode(D4,OUTPUT); 
    digitalWrite(D4, LOW);
}

void loop() {
  int acceleration = analogRead(A0);
 Serial.print("Movements: ");
 Serial.println(acceleration);
 ubidots.add("Movements", acceleration);
  if (acceleration >10){
  digitalWrite(D4, HIGH);
  }
  else
  {
  digitalWrite(D4, LOW);
  }
  int pulse = digitalRead(D1);
  if (pulse =1) ubidots.add("Pulse");
  int resp = digitalRead(D0);
  ubidots.add("Respiratory", resp);
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
    int far = (event.temperature * 1.8)+32;
    Serial.print(far);
    Serial.println("°F");
    ubidots.add("Temperature", far);
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    ubidots.add("Humidity", event.relative_humidity);
    Serial.println(F("%"));
  }
        bool bufferSent = false;
        bufferSent = ubidots.send();  // Will send data to a device label that matches the device Id
        if (bufferSent) {
        Serial.println("Values sent by the device");
      }
        delay(2000);
   }

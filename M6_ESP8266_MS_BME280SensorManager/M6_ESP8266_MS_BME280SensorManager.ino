/*
   dataneo @2018 - M6_MS_BME280SensorManager
   MySensors BME280 Sensor Manager 1.0
   see https://sites.google.com/site/dataneosoftware/arduino/mysensors-bme280-sensor-manager
*/

// Enable debug prints to serial monitor
#define MY_DEBUG

// Enable serial gateway
#define MY_GATEWAY_SERIAL


#include <ESP8266WiFi.h>
#include "I:\7.Projekty\5.Arduino\M_Library\BME280Manager\BME280Manager.h"

BME280Manager myBME280Manager = BME280Manager(5); // set scan interval in seconds
/*  End of M6_MS_BME280SensorManager */

void before()
{
  WiFi.mode(WIFI_OFF); 
  /* M6_MS_BME280SensorManager */
  myBME280Manager.addSensor(0x76, 0x70, 7, "Kuchnia"); // M6_MS_BME280SensorManager
}

void setup() 
{
  
}

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("BME280 Sensor Manager", "1.0");

  myBME280Manager.presentAllToControler(); //M6_MS_BME280SensorManager
}

void loop()
{
  myBME280Manager.sensorsCheck(); //M6_MS_BME280SensorManager
}

void receive(const MyMessage &message) {}

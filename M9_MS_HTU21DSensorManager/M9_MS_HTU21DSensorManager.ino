/*
   dataneo @2018 - M9_MS_HTU21DSensorManager
   MySensors HTU21D Sensor Manager 1.0
   see https://sites.google.com/site/dataneosoftware/arduino/mysensors-htu21d-sensor-manager
*/

// Enable serial gateway
#define MY_GATEWAY_SERIAL
#include "I:\7.Projekty\5.Arduino\M_Library\HTU21DManager\HTU21DManager.h"

HTU21DManager myHTU21DManager = HTU21DManager(30); // set scan interval in seconds
/*  End of M9_MS_HTU21DSensorManager */

void before()
{
  /* M9_MS_HTU21DSensorManager */
  myHTU21DManager.addSensor(0, 0, "Kuchnia");  // M9_MS_HTU21DSensorManager
}

void setup() { }

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("HTU21D Sensor Manager", "1.0");

  myHTU21DManager.presentAllToControler(); //M9_MS_HTU21DSensorManager
}

void loop()
{
  myHTU21DManager.sensorsCheck(); //M9_MS_HTU21DSensorManager
}

void receive(const MyMessage &message) { }

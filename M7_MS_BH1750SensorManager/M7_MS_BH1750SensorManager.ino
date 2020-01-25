/*
   dataneo @2019 - M7_MS_BH1750SensorManager
   MySensors BH1750 Sensor Manager 1.1
   see https://sites.google.com/site/dataneosoftware/arduino/mysensors-bh1750-sensor-manager
*/

#define MY_GATEWAY_SERIAL // Enable serial gateway
#include "I:\7.Projekty\5.Arduino\M_Library\BH1750Manager\BH1750Manager.h"

BH1750Manager myBH1750Manager(30); // set scan interval in seconds
/*  End of M7_MS_BH1750SensorManager */

void before()
{
  /* M7_MS_BH1750SensorManager */
  myBH1750Manager.addSensor(0x23, 0, 0, "Czujnik poziomu oświetlenia");  // M7_MS_BH1750SensorManager
}

void setup() { }

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("BH1750 Sensor Manager", "1.1");

  myBH1750Manager.presentAllToControler(); //M7_MS_BH1750SensorManager
}

void loop()
{
  myBH1750Manager.sensorsCheck(); //M7_MS_BH1750SensorManager
}

void receive(const MyMessage &message) { }

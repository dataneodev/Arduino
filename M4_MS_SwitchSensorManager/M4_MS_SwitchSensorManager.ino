/*
   dataneo @2018 - M4_MS_SwitchSensorManager
   MySensors Switch Sensor Manager 1.1
   Mechanical switch manager with debouncer
   see https://sites.google.com/site/dataneosoftware/arduino/switch-sensor-manager
*/

#define MY_GATEWAY_SERIAL // Enable serial gateway
#include "I:\7.Projekty\5.Arduino\M_Library\SwitchManager\SwitchManager.h"

SwitchManager mySwitchManager = SwitchManager();
/*
   End of M4_MS_SwitchSensorManager
*/

void before()
{
  /* M4_MS_SwitchSensorManager */
  mySwitchManager.addSwitch(A0, NORMAL_CLOSE, "drzwi kuchnia");  // M4_MS_SwitchSensorManager
}

void setup() { }

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Switch Sensor Manager", "1.0");

  mySwitchManager.presentAllToControler(); //M4_MS_SwitchSensorManager
}

void loop()
{
  mySwitchManager.switchCheckState(); //M4_MS_SwitchSensorManager
}

void receive(const MyMessage &message) { }

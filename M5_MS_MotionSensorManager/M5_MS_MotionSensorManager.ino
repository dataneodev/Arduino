/*
   dataneo @2018 - M5_MS_MotionSensorManager
   MySensors Motion Sensor Manager 1.2
   see https://sites.google.com/site/dataneosoftware/arduino/mysensors-motion-sensor-manager
*/

#define MY_GATEWAY_SERIAL // Enable serial gateway

#include "I:\7.Projekty\5.Arduino\M_Library\MotionManager\MotionManager.h"

MotionManager myMotionManager = MotionManager(DOMOTICZ);

void before()
{
  /* M5_MS_MotionSensorManager */
  myMotionManager.addMotion(7, SENSOR_ON_HIGH, "Czujnik ruchu salon");  // M5_MS_MotionSensorManager
  myMotionManager.addMotion(8, SENSOR_ON_HIGH, "Czujnik zalania kuchnia", S_WATER_LEAK);  // M5_MS_MotionSensorManager use only sensors type with V_TRIPPED : S_DOOR, S_MOTION, S_SMOKE, S_SPRINKLER, S_WATER_LEAK, S_SOUND, S_VIBRATION, S_MOISTURE
}

void setup() { }

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Motion Sensor Manager", "1.2");

  myMotionManager.presentAllToControler(); //M5_MS_MotionSensorManager
}

void loop()
{
  myMotionManager.motionCheckState(); //M5_MS_MotionSensorManager
}

void receive(const MyMessage &message)
{
  myMotionManager.setStatetFromControler(message); //M5_MS_MotionSensorManager
}

/*
   dataneo @2018 - M2_MS_Heartbeat
   MySensors Heartbeat Manager 1.0
   see https://sites.google.com/site/dataneosoftware/arduino/mysensors-heartbeat
*/

#define MY_GATEWAY_SERIAL
#include "I:\7.Projekty\5.Arduino\M_Library\HeartBeatManager\HeartBeatManager.h"

HeartBeatManager myHeartBeatManager = HeartBeatManager(13);

void receiveTime(uint32_t ts) {
  myHeartBeatManager.ControllerReciveMsg();
}

void before() { }

void setup() { }

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("M2_MS_Heartbeat", "1.0");
}

void loop()
{
  myHeartBeatManager.HeartBeat(); // M2_MS_Heartbeat
  wait(1, C_SET, V_STATUS);
}

void receive(const MyMessage &message)
{
  myHeartBeatManager.ControllerReciveMsg();// M2_MS_Heartbeat
}

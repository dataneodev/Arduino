/*
   dataneo @2019
   MySensors Heater 1.0
*/

#define MY_GATEWAY_SERIAL // Enable serial gateway

#include "I:/7.Projekty/5.Arduino/M_Library/DS18B20Manager/DS18B20Manager.h"
#include "I:/7.Projekty/5.Arduino/M_Library/PZEM016Manager/PZEM016Manager.h"
#include "I:/7.Projekty/5.Arduino/M_Library/BuzzerManager/BuzzerManager.h"

PZEM016Manager PZEM016ManagerObj = PZEM016Manager(Serial1, 10, 2, 10);

void before()
{
}

void setup() {}

void presentation()
{
    sendSketchInfo("MySensors House Heater", "1.0");
}

void loop()
{
}

void receive(const MyMessage &message) {}
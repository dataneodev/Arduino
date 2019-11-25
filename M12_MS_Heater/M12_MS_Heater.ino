/*
   dataneo @2019
   MySensors Heater 1.0
*/

#define MY_GATEWAY_SERIAL // Enable serial gateway

#include <DS18B20Manager.h>
#include <PZEM016Manager.h>

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
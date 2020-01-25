#include <Arduino.h>
#line 1 "i:\\7.Projekty\\5.Arduino\\M12_MS_Heater\\M12_MS_Heater.ino"
/*
   dataneo @2019
   MySensors Heater 1.0
*/

#define MY_GATEWAY_SERIAL // Enable serial gateway

#include "I:/7.Projekty/5.Arduino/M_Library/DS18B20Manager/DS18B20Manager.h"
#include "I:/7.Projekty/5.Arduino/M_Library/PZEM016Manager/PZEM016Manager.h"
#include "I:/7.Projekty/5.Arduino/M_Library/BuzzerManager/BuzzerManager.h"

PZEM016Manager PZEM016ManagerObj = PZEM016Manager(Serial1, 10, 2, 10);

#line 14 "i:\\7.Projekty\\5.Arduino\\M12_MS_Heater\\M12_MS_Heater.ino"
void before();
#line 18 "i:\\7.Projekty\\5.Arduino\\M12_MS_Heater\\M12_MS_Heater.ino"
void setup();
#line 20 "i:\\7.Projekty\\5.Arduino\\M12_MS_Heater\\M12_MS_Heater.ino"
void presentation();
#line 25 "i:\\7.Projekty\\5.Arduino\\M12_MS_Heater\\M12_MS_Heater.ino"
void loop();
#line 29 "i:\\7.Projekty\\5.Arduino\\M12_MS_Heater\\M12_MS_Heater.ino"
void receive(const MyMessage &message);
#line 14 "i:\\7.Projekty\\5.Arduino\\M12_MS_Heater\\M12_MS_Heater.ino"
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

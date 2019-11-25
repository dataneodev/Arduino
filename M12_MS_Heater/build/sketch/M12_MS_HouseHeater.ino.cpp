#include <Arduino.h>
#line 1 "i:\\7.Projekty\\5.Arduino\\M12_MS_HouseHeater\\M12_MS_HouseHeater.ino"
/*
   dataneo @2019
   MySensors House Heater 1.0
*/

#define MY_GATEWAY_SERIAL // Enable serial gateway

#include <DS18B20Manager.h>
#include <PZEM016Manager.h>

#line 11 "i:\\7.Projekty\\5.Arduino\\M12_MS_HouseHeater\\M12_MS_HouseHeater.ino"
void before();
#line 15 "i:\\7.Projekty\\5.Arduino\\M12_MS_HouseHeater\\M12_MS_HouseHeater.ino"
void setup();
#line 17 "i:\\7.Projekty\\5.Arduino\\M12_MS_HouseHeater\\M12_MS_HouseHeater.ino"
void presentation();
#line 22 "i:\\7.Projekty\\5.Arduino\\M12_MS_HouseHeater\\M12_MS_HouseHeater.ino"
void loop();
#line 26 "i:\\7.Projekty\\5.Arduino\\M12_MS_HouseHeater\\M12_MS_HouseHeater.ino"
void receive(const MyMessage &message);
#line 11 "i:\\7.Projekty\\5.Arduino\\M12_MS_HouseHeater\\M12_MS_HouseHeater.ino"
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

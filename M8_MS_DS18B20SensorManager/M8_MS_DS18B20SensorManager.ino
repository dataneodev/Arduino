/*
   dataneo @2019 - M8_MS_DS18B20SensorManager
   MySensors DS18B20 Sensor Manager 1.2
   see https://sites.google.com/site/dataneosoftware/arduino/mysensors-ds18b20-sensor-manager
*/

#define MY_GATEWAY_SERIAL // Enable serial gateway
#include "DS18B20Manager.h"

DS18B20Manager myDS18B20Manager = DS18B20Manager(7, 10, 750, true);

void before() //MySensors
{
    /* 
    M8_MS_DS18B20SensorManager 
    myDS18B20Manager.addSensor(
    - adres czujnika DS18B20 należy ustalić przed dodaniem,
    - opis,
    - czy prezentować czujnik kontrolerowi - domyślnie true, 
    - funkcja wywoływana po odczycie, np. void testTemperatureRead(uint8_t DS18B20Adress[8], float temp){}, w przypadku braku dać nullptr
    - funkcja wywoływana po błędzie odczytu np. void testTemperatureReadError(uint8_t DS18B20Adress[8]){}, w przypadku braku dać nullptr
    */

    // myDS18B20Manager.addSensor(0x28, 0xEE, 0xAF, 0x47, 0x1A, 0x16, 0x01, 0x23, "Kibel"); // M8_MS_DS18B20SensorManager
    // myDS18B20Manager.addSensor(0x28, 0xEE, 0xAF, 0x47, 0x1A, 0x16, 0x01, 0x24, "Salon", false, nullptr, nullptr); // M8_MS_DS18B20SensorManager

      myDS18B20Manager.addSensor(0x28, 0xFF, 0x04, 0x23, 0x6E, 0x18, 0x01, 0x3A,
                             "TempSolarRelay",
                             true,
                             nullptr,
                             nullptr); // M8_MS_DS18B20SensorManager
}

void setup() {}

void presentation() //MySensors
{
    sendSketchInfo("DS18B20 Sensor Manager", "1.2"); //M8_MS_DS18B20SensorManager
    myDS18B20Manager.presentAllToControler();        //M8_MS_DS18B20SensorManager
}

void loop() //Main loop
{
    myDS18B20Manager.sensorsCheckLoop(); //M8_MS_DS18B20SensorManager
}

void receive(const MyMessage &message) {} //MySensors

// test function
void testTemperatureRead(uint8_t DS18B20Adress[8], float temp)
{
    Serial.print("Odczytano temperature: ");
    Serial.println(temp);
}

// test function
void testTemperatureReadError(uint8_t DS18B20Adress[8])
{
    Serial.println("Error read test ");
}

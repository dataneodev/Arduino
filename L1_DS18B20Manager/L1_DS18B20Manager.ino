
#define MY_GATEWAY_SERIAL

#include "DS18B20Manager.h"

uint8_t *pAddr(uint8_t a1, uint8_t a2, uint8_t a3, uint8_t a4, uint8_t a5, uint8_t a6, uint8_t a7, uint8_t a8)
{
    uint8_t *c = new uint8_t[8];
    c[0] = a1;
    c[1] = a2;
    c[2] = a3;
    c[3] = a4;
    c[4] = a5;
    c[5] = a6;
    c[6] = a7;
    c[7] = a8;
    return c;
}

//DS18B20Manager myDS18B20Manager = DS18B20Manager(4, 6, 1500, true);
/*  End of M8_MS_DS18B20SensorManager */

void before()
{
    /* M8_MS_DS18B20SensorManager */
   // myDS18B20Manager.addSensor(pAddr(0x28, 0xEE, 0xAF, 0x47, 0x1A, 0x16, 0x01, 0x26), "Kuchnia"); // M8_MS_DS18B20SensorManager
}

void setup() {}

void presentation()
{
    // Send the sketch version information to the gateway and Controller
   /// sendSketchInfo("DS18B20 Sensor Manager", "1.1");

   // myDS18B20Manager.presentAllToControler(); //M8_MS_DS18B20SensorManager
}

void loop()
{
   // myDS18B20Manager.sensorsCheck(); //M8_MS_DS18B20SensorManager
}

//void receive(const MyMessage &message) {}

/* Instalacja SoftwareSerial (EspSoftwareSerial) dla bibloteki MySensors
Podmienić w pliku MySensors\MySensors.h :

<-----
#elif defined(MY_RS485)
#if !defined(MY_RS485_HWSERIAL)
#if defined(__linux__)
#error You must specify MY_RS485_HWSERIAL for RS485 transport
#endif
#include "drivers/AltSoftSerial/AltSoftSerial.cpp"
#endif
#include "hal/transport/RS485/MyTransportRS485.cpp"
#elif defined(MY_RADIO_RFM69) 
----->

na :

<-----
#elif defined(MY_RS485)
#if !defined(MY_RS485_HWSERIAL)
#if defined(__linux__)
#error You must specify MY_RS485_HWSERIAL for RS485 transport
#endif
#if defined(MY_RS485_ESP)
#include <SoftwareSerial.h>
#else
#include "drivers/AltSoftSerial/AltSoftSerial.cpp"
#endif
#endif
#include "hal/transport/RS485/MyTransportRS485.cpp"
#elif defined(MY_RADIO_RFM69)
----->

Podmienić w pliku MySensors\hal\transport\RS485\MyTransportRS485.cpp:
<-----
#if defined(__linux__)
SerialPort _dev = SerialPort(MY_RS485_HWSERIAL);
#elif defined(MY_RS485_HWSERIAL)
HardwareSerial& _dev = MY_RS485_HWSERIAL;
#else
AltSoftSerial _dev;
#endif
----->

na : 

<-----
#if defined(__linux__)
SerialPort _dev = SerialPort(MY_RS485_HWSERIAL);
#elif defined(MY_RS485_HWSERIAL)
HardwareSerial& _dev = MY_RS485_HWSERIAL;
#elif defined(MY_RS485_ESP)
SoftwareSerial& _dev = MY_RS485_ESP;
#else
AltSoftSerial _dev;
#endif
----->
*/
/* #endregion */

/* #region  configuration */
//#define MY_DEBUG
#define TCAADDR 0x70

#define MY_NODE_ID 37
#define MY_RS485


/* #endregion */

/* #region  rs485 */
#include <SoftwareSerial.h>
SoftwareSerial swESP(D5, D7); //RX - RO, TX - DI

#define MY_RS485_DE_PIN D6 // DE
#define MY_RS485_BAUD_RATE 9600
#define MY_RS485_ESP swESP
/* #endregion */

/* #region  i2c */
#define SDA_PIN D1
#define SCL_PIN D2
/* #endregion */

/* #region  oneWire */
#define ONEWIRE_1 D3
#define ONEWIRE_1 D4
/* #endregion */


#define MY_GATEWAY_SERIAL
#include <MySensors.h>
#include <ESP8266WiFi.h>

#include "I:\7.Projekty\5.Arduino\M_Library\BME280Manager\BME280Manager.h"
BME280Manager myBME280Manager = BME280Manager(30); // set scan interval in seconds

#include "I:\7.Projekty\5.Arduino\M_Library\BH1750Manager\BH1750Manager.h"
BH1750Manager myBH1750Manager(30); // set scan interval in seconds 

void before() //MySensors
{
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin(); //15mA
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  /* M6_MS_BME280SensorManager */
  myBME280Manager.addSensor(0x76, TCAADDR, 4, "Nazewnątrz"); // M6_MS_BME280SensorManager

   /* M7_MS_BH1750SensorManager */
  myBH1750Manager.addSensor(0x23, TCAADDR, 4, "Naswietlenie");  // M7_MS_BH1750SensorManager
}

void receive(const MyMessage &message)
{
}

void setup(){}

void presentation()
{
  sendSketchInfo("BME280 Sensor Manager", "1.0");

  myBME280Manager.presentAllToControler(); //M6_MS_BME280SensorManager
  myBH1750Manager.presentAllToControler(); //M7_MS_BH1750SensorManager
}

void loop()
{
  myBME280Manager.sensorsCheck(); //M6_MS_BME280SensorManager
  myBH1750Manager.sensorsCheck(); //M7_MS_BH1750SensorManager
}

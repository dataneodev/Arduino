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


// Enable RS485 transport layer

//#define MY_DEBUG
//#define MY_NODE_ID 24
// Define this to enables DE-pin management on defined pin
#define MY_RS485
#define MY_RS485_DE_PIN D3 // DE
// Set RS485 baud rate to use
#define MY_RS485_BAUD_RATE 9600

#include <SoftwareSerial.h>

SoftwareSerial swESP(D4, D2); //RX - RO, TX - DI
#define MY_RS485_ESP swESP

#define MY_GATEWAY_SERIAL

#include <MySensors.h>
#include <ESP8266WiFi.h>


void before() //MySensors
{
  WiFi.mode(WIFI_OFF); 
  WiFi.forceSleepBegin(); //15mA
  Serial.begin(115200);
}

void receive(const MyMessage &message)
{
 
}

void setup() 
{

  
}

void presentation()
{
    // Present locally attached sensors
}

void loop() {



}

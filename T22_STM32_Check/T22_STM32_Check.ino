#define ARDUINO_ARCH_STM32F1

//                      RX    TX
HardwareSerial Serial2(PA10, PA9);

#define MY_DISABLED_SERIAL         // manual configure Serial1
#define MY_RS485                   // Enable RS485 transport layer
#define MY_RS485_DE_PIN PA1         // Define this to enables DE-pin management on defined pin
#define MY_RS485_BAUD_RATE 9600    // Set RS485 baud rate to use
#define MY_RS485_HWSERIAL Serial2  //
#define MY_RS485_SOH_COUNT 6
#define MY_TRANSPORT_WAIT_READY_MS 1

#include <MySensors.h>
#include "24C32.h"
#include <Wire.h>





TwoWire Wire2(PB11, PB10);
#define Wire Wire2

EE EEPROM24C32;


void setup() {
  Wire2.begin(); 
  EEPROM24C32.begin(0x50, false);

  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, EEPROM24C32.checkPresence());


  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

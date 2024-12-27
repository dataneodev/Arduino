// Board: https://github.com/MCUdude/MicroCore
// ATtiny13
// Lib: https://github.com/MCUdude/MicroCore/tree/master/avr/libraries/ADCTouch
// Clock (Internal) 128 kHz

#include "ADCTouch.h"
#include <EEPROM.h>

// Touch threshold for turning on or off the LEDs
// Lower is more sensitive
//60 at 1.2MHz
//25 at 128kHz
const uint8_t threshold = 100;

// Sample each touch pin 128 times
const uint16_t ADCTouch::samples = 128;

#define SENSOR_PIN A2
#define OUT_PIN PB0
#define RECALIBRATE 3600000  //1h

//ustawić odpowiednio clock i bound rate dla Serial
/*
(External) 20 MHz 	115200
(External) 16 MHz 	115200
(External) 12 MHz 	115200
(External) 8 MHz 	115200
(External) 1 MHz 	19200
(Internal) 9.6 MHz 	115200
(Internal) 4.8 MHz 	57600
(Internal) 1.2 MHz 	19200
(Internal) 600 kHz 	9600
(Internal) 128 kHz 	Not supported
*/
//#define DEBUG

uint16_t reference;
uint32_t lastTouch;

void setReference() {
  reference = Touch.read(SENSOR_PIN);
}

void checkReference(uint32_t now, bool isTouch) {
  if (isTouch) {
    lastTouch = now;
    return;
  }

  if (lastTouch != 0) {

    if (lastTouch > now) {  //over
      lastTouch = now;
      return;
    }

    uint32_t compare = lastTouch + RECALIBRATE;  //1h from last touch

    if (compare < lastTouch) {  //over
      lastTouch = now;
      return;
    }

    if (compare < now) {
      lastTouch = now;
      setReference();  //recalibrate
      return;
    }
  } else {
    if (now > RECALIBRATE) {
      lastTouch = now;
      setReference();
      return;
    }
  }
}

bool getTouchResult() {
  int16_t value = Touch.read(SENSOR_PIN);
  int16_t r = value > reference ? (value - reference) : (reference - value);
  bool isTouch = r > threshold;

#ifdef DEBUG
  Serial.print("T: ");
  Serial.print(value);
  Serial.write('\n');

  Serial.print("R: ");
  Serial.print(reference);
  Serial.write('\n');

  Serial.print("TS: ");
  Serial.print(threshold);
  Serial.write('\n');

  Serial.print("B: ");
  Serial.print(isTouch);  // Send (boolean) pressed or not pressed
  Serial.write('\n');

  delay(1000);
#endif

  if (r < 4) {
    reference = reference + r * (value > reference ? 1 : -1)/2;
  }

  return isTouch;
}

void setup() {
  uint8_t cal = EEPROM.read(0);
  if (cal < 0x80)
    OSCCAL = cal;

#ifdef DEBUG
  Serial.begin();
#endif

  pinMode(OUT_PIN, OUTPUT);
  digitalWrite(OUT_PIN, LOW);

  setReference();
}

void loop() {
  bool isTouch = getTouchResult();

  digitalWrite(OUT_PIN, isTouch ? HIGH : LOW);

  checkReference(millis(), isTouch);
}

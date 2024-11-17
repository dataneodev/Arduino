// Board: https://github.com/MCUdude/MicroCore
// ATtiny13
// Lib: https://github.com/MCUdude/MicroCore/tree/master/avr/libraries/ADCTouch

#include <ADCTouch.h>
#include <EEPROM.h>

// Touch threshold for turning on or off the LEDs
// Lower is more sensitive
const uint8_t threshold = 50;

// Sample each touch pin 32 times
const uint16_t ADCTouch::samples = 32;

#define SENSOR_PIN A3
#define OUT_PIN PB4
#define RECALIBRATE 3600000  //1h
#define DEBUG

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
      setReference();      //recalibrate
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
  bool isTouch = value - reference > threshold;

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

  return isTouch;
}

void setup() {
#ifdef DEBUG
  Serial.begin();
#endif

  uint8_t cal = EEPROM.read(0);
  if (cal < 0x80)
    OSCCAL = cal;

  pinMode(OUT_PIN, OUTPUT);
  digitalWrite(OUT_PIN, LOW);

  setReference();
}

void loop() {
  bool isTouch = getTouchResult();

  digitalWrite(OUT_PIN, isTouch ? HIGH : LOW);

  checkReference(millis(), isTouch);
}

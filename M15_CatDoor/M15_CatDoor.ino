#define SOFTWARE_VERION "1.0"
#define SKETCH_NAME "M15_Cat_Door"

#pragma region INSTALATION
#include "esp_random.h" //brakuje w plikach mysensors dla esp32, sprawdzicz y mozna usunąć w nowych wersjach
/*

dla ESP32-C6 zakomentować w pliku:
C:\Users\Smith\Documents\Arduino\libraries\MySensors\hal\architecture\ESP32\MyHwESP32.h

static __inline__ void __psRestore(const uint32_t *__s)
{
	//XTOS_RESTORE_INTLEVEL(*__s);
}

*/
#pragma endregion INSTALATION

#pragma region CONFIGURATION
/*
przykład

#define MOTION_1_DELAY 5000
#define MOTION_1_DELAY_WAIT 4000

*/
#define ALARM_ENABLED // w przypadku błędow uruchamiać alarm dzwiękowy

#define MOTION_1_DELAY 5 * 1000       // czas pomiędzy pierwszym wykryciem ruchu a kolejnym wykryciem uruchamiajacym otwarcie drzwi dla sensoru 1,
#define MOTION_1_DELAY_WAIT 4 * 1000  // czas oczekiwania na 2 wykrycie ruchu dla sensoru 1,

#define MOTION_2_DELAY 5 * 1000       // czas pomiędzy pierwszym wykryciem ruchu a wykryciem uruchamiajacym otwarcie dla sensoru 2,
#define MOTION_2_DELAY_WAIT 5 * 1000  // czas oczekiwania na 2 wykrycie ruchu dla sensoru 2,

#define OPENING_DOOR_TIME 11 * 1000                // czas otwierania drzwi
#define OPEN_DOOR_TIME 8 * 1000                    // czas oczekiwania na zamknięcie drzwi od ostatnieo wykrycia ruchu
#define TO_LONG_OPEN_DOOR_TIME 60 * 1000           // czas zbyt długiego otwarcia drzwi aby włączyc alarm
#define TIME_SLEEP_AFTER_LAST_DETECTION 90 * 1000  // czas przejscia w deep sleep od ostatniego wykrycia ruchu

#define MY_NODE_ID 90 //id wezła dla my sensors

#define MY_DEBUG //for tests
#pragma endregion CONFIGURATION

#pragma region BOARD_PIN_CONFIGURATION
#define MOTION_SENSOR_1_PIN 20
#define MOTION_SENSOR_2_PIN 19
#define MOTION_SENSOR_3_PIN 18

#define OUPUT_1_PIN 5
#define OUPUT_2_PIN 0
#define OUPUT_3_PIN 1

#define POWER_PIN 2

#define OPEN_DOOR_PIN 10   // pin otwarcia
#define CLOSE_DOOR_PIN 11  // pin zamkniecia

#define SDA_PIN 6
#define SCL_PIN 7

#define RX_PIN 21
#define TX_PIN 23
#pragma endregion BOARD_PIN_CONFIGURATION

//RS485
#define MY_DISABLED_SERIAL //manualu configure Serial1
#define MY_RS485                // Enable RS485 transport layer
#define MY_RS485_DE_PIN 22      // Define this to enables DE-pin management on defined pin
#define MY_RS485_BAUD_RATE 9600 // Set RS485 baud rate to use
#define MY_RS485_HWSERIAL Serial1 //

//#define MY_TRANSPORT_DONT_CARE_MODE //requires setting MY_PARENT_NODE_ID
#define MY_TRANSPORT_UPLINK_CHECK_DISABLED
#define MY_TRANSPORT_WAIT_READY_MS 1
#define MY_PARENT_NODE_ID 0
#define MY_PARENT_NODE_IS_STATIC

#pragma region TYPES
class DoorManager {
private:
  byte _doorOpenPin;
  byte _doorClosePin;
public:
  DoorManager(byte doorOpenPin, byte doorClosePin) {
    _doorOpenPin = doorOpenPin;
    _doorClosePin = doorClosePin;
  }

  void open() {
    digitalWrite(_doorClosePin, LOW);
    digitalWrite(_doorOpenPin, HIGH);
  }

  void close() {
    digitalWrite(_doorOpenPin, LOW);
    digitalWrite(_doorClosePin, HIGH);
  }

  void end() {
    digitalWrite(_doorClosePin, LOW);
    digitalWrite(_doorOpenPin, LOW);
  }
};

class StateTime {
private:
  unsigned long startState;

public:
  void stateStart() {
    startState = millis();
  }

  bool isElapsed(unsigned long elapsed) {
    unsigned long current = millis();

    if (startState > current) {
      startState = current;
      return false;
    }

    if (startState + elapsed < current) {
      return true;
    }


    return false;
  }
};

enum MotionDetectState {
  NO_MOTION = 0,
  ONE_MOTION_DETECTED = 1,
  MOTIONS_DETECTED = 2,
};

class MotionDetect {
private:
  unsigned long _startAt;
  unsigned long _lastMotionAt;

  byte _pin;
  unsigned long _motionDelay;
  unsigned long _motionDelayTotal;

  bool _lastState;
  bool _secondMotionDetected;

public:
  MotionDetect(byte pin, unsigned long motionDelay, unsigned long motionDelayWait) {
    _pin = pin;
    _motionDelay = motionDelay;
    _motionDelayTotal = motionDelay + motionDelayWait;
  }

  void start() {
    _secondMotionDetected = false;
    _lastState = getPinState();

    if (_lastState) {
      _lastMotionAt = millis();
    }
  }

  void weakUp() {
    _lastState = getPinState();

    if (_lastState) {
      _startAt = millis();
      _lastMotionAt = _startAt;
    }
  }

  MotionDetectState ping() {
    unsigned long current = millis();
    bool state = getPinState();

    if (state) {
      _lastMotionAt = current;
    }

    bool invoke = !_lastState && state;
    _lastState = state;

    bool isTotalPassed = current > _startAt + _motionDelayTotal;
    bool isDelayPassed = isTotalPassed || current > _startAt + _motionDelay;

    if (_startAt > current) {
      _startAt = current;
      _secondMotionDetected = false;

      return invoke ? ONE_MOTION_DETECTED : NO_MOTION;
    }

    if (invoke && isDelayPassed && !isTotalPassed) {
      _secondMotionDetected = true;
      return MOTIONS_DETECTED;
    }

    if (_secondMotionDetected && !isTotalPassed) {
      return MOTIONS_DETECTED;
    }

    if (!isTotalPassed) {
      return ONE_MOTION_DETECTED;
    }

    _secondMotionDetected = false;

    if (invoke) {
      _startAt = current;
      return ONE_MOTION_DETECTED;
    }

    return NO_MOTION;
  }

  bool isElapsedFromLastMotionDetection(unsigned long elapsed) {
    unsigned long current = millis();

    return _lastMotionAt + elapsed < current;
  }

private:
  bool getPinState() {
    return digitalRead(_pin);
  }
};
#pragma endregion TYPES

#pragma region GLOBAL_VARIABLE
#include "driver/gpio.h"
#include <Wire.h>
#include <MySensors.h>
#include <StateMachine.h>
#include "24C32.h"
#include <Blinkenlight.h>

EE EEPROM24C32;
MyMessage mMessage;

DoorManager Door(OPEN_DOOR_PIN, CLOSE_DOOR_PIN);
StateTime T;
MotionDetect M1(MOTION_SENSOR_1_PIN, MOTION_1_DELAY, MOTION_1_DELAY_WAIT);
MotionDetect M2(MOTION_SENSOR_2_PIN, MOTION_2_DELAY, MOTION_2_DELAY_WAIT);
MotionDetect M3(MOTION_SENSOR_3_PIN, 5 * 1000, 1);

Blinkenlight Out1(OUPUT_1_PIN);
Blinkenlight Out2(OUPUT_2_PIN);
Blinkenlight Out3(OUPUT_3_PIN);

#pragma endregion GLOBAL_VARIABLE

#pragma region STATES

StateMachine SM = StateMachine();

State* S_MOTION_DETECTION = SM.addState(&s_MOTION_DETECTION);
void s_MOTION_DETECTION() {
  if (SM.executeOnce) {
    Serial.println("S_MOTION_DETECTION");

    T.stateStart();

    M1.start();
    M2.start();
    M3.start();

    Door.end();
    Out1.off();
    Out2.off();
    Out3.off();
    return;
  }

  M3.ping();

  if (M1.ping() == ONE_MOTION_DETECTED || M2.ping() == ONE_MOTION_DETECTED) {
    Out1.on();
    Out2.off();
    Out3.off();
  } else {
    Out1.off();
    Out2.off();
    Out3.off();
  }
}

State* S_WAKE_UP = SM.addState(&s_WAKE_UP);
void s_WAKE_UP() {
  if (SM.executeOnce) {
    Serial.println("S_WAKE_UP");

    M1.weakUp();
    M2.weakUp();
    M3.weakUp();
  }
}


State* S_SLEEP = SM.addState(&s_SLEEP);
void s_SLEEP() {
  if (SM.executeOnce) {
    Serial.println("S_SLEEP");
    Serial.flush();

    T.stateStart();

    Door.end();

    Out1.off();
    Out2.off();
    Out3.off();    

    digitalWrite(POWER_PIN, LOW);

    delay(100);
    
    esp_light_sleep_start();

    ///sleep

    digitalWrite(POWER_PIN, HIGH);
  }
}

State* S_FATAL_ERROR = SM.addState(&s_FATAL_ERROR);
void s_FATAL_ERROR() {
  if (SM.executeOnce) {
    Serial.println("S_FATAL_ERROR");
    T.stateStart();

    Door.end();

    Out1.blink();
    Out2.blink();
    Out3.blink();
  }
}

State* S_MOTION_DETECTED = SM.addState(&s_MOTION_DETECTED);
void s_MOTION_DETECTED() {
  if (SM.executeOnce) {
    Serial.println("S_MOTION_DETECTED");
    T.stateStart();

    Out1.on();
    Out2.off();
    Out3.off();
  }
}

State* S_MOTION_DETECTED_NO_AUTH = SM.addState(&s_MOTION_DETECTED_NO_AUTH);
void s_MOTION_DETECTED_NO_AUTH() {
  if (SM.executeOnce) {
    Serial.println("S_MOTION_DETECTED_NO_AUTH");
    T.stateStart();

    Out1.off();
    Out2.blink();
    Out3.blink();
  }
}

State* S_OPENING_DOOR = SM.addState(&s_OPENING_DOOR);
void s_OPENING_DOOR() {
  if (SM.executeOnce) {
    Serial.println("S_OPENING_DOOR");
    T.stateStart();

    Door.open();

    Out1.blink();
    Out2.off();
    Out3.off();

    eeSetDoorOpen(true);
  }
}

State* S_DOOR_OPEN = SM.addState(&s_DOOR_OPEN);
void s_DOOR_OPEN() {
  if (SM.executeOnce) {
    Serial.println("S_DOOR_OPEN");
    T.stateStart();

    Door.end();

    Out1.on();
    Out2.off();
    Out3.off();
  }
}

State* S_DOOR_TO_LONG_OPEN = SM.addState(&s_DOOR_TO_LONG_OPEN);
void s_DOOR_TO_LONG_OPEN() {
  if (SM.executeOnce) {
    Serial.println("S_DOOR_TO_LONG_OPEN");
    T.stateStart();

    Out1.blink();
    Out2.blink();
    Out3.blink();
  }
}

State* S_CLOSING_DOOR = SM.addState(&s_CLOSING_DOOR);
void s_CLOSING_DOOR() {
  if (SM.executeOnce) {
    Serial.println("S_CLOSING_DOOR");
    T.stateStart();

    Door.close();

    Out1.blink();
    Out2.off();
    Out3.off();
  }
}

State* S_DOOR_CLOSED = SM.addState(&s_DOOR_CLOSED);
void s_DOOR_CLOSED() {
  if (SM.executeOnce) {
    Serial.println("S_DOOR_CLOSED");
    T.stateStart();

    Door.end();
    Out1.off();
    Out2.off();
    Out3.off();

    eeSetDoorOpen(false);
  }
}

State* S_CLOSING_DOOR_INTERRUPTED = SM.addState(&s_CLOSING_DOOR_INTERRUPTED);
void s_CLOSING_DOOR_INTERRUPTED() {
  if (SM.executeOnce) {
    Serial.println("S_CLOSING_DOOR_INTERRUPTED");
    T.stateStart();

    Door.end();

    Out1.blink();
    Out2.off();
    Out3.blink();
  }
}

bool T_S_SLEEP_S_WAKE_UP() {
  return true;
}

bool T_S_WAKE_UP_S_MOTION_DETECTION() {
  return true;
}

bool T_S_MOTION_DETECTION_S_MOTION_DETECTED() {
  bool isMotionDetect = M1.ping() == MOTIONS_DETECTED || M2.ping() == MOTIONS_DETECTED;

  //add ble auth


  return isMotionDetect;
}

bool T_S_MOTION_DETECTION_S_SLEEP() {
  if (!T.isElapsed(TIME_SLEEP_AFTER_LAST_DETECTION)) {
    return false;
  }

  bool noMotionDetect = M1.ping() == NO_MOTION && M2.ping() == NO_MOTION;

  if (!noMotionDetect) {
    return false;
  }

  bool isTooLongActive = (M1.isElapsedFromLastMotionDetection(TIME_SLEEP_AFTER_LAST_DETECTION) || M2.isElapsedFromLastMotionDetection(TIME_SLEEP_AFTER_LAST_DETECTION));

  return isTooLongActive;
}

bool T_S_MOTION_DETECTED_S_OPENING_DOOR() {
  return T.isElapsed(100);
}

bool T_S_OPENING_DOOR_S_DOOR_OPEN() {
  return T.isElapsed(OPENING_DOOR_TIME);
}

bool T_S_DOOR_OPEN_S_CLOSING_DOOR() {
  bool m1IsElapsed = M1.isElapsedFromLastMotionDetection(OPEN_DOOR_TIME);
  bool m2IsElapsed = M2.isElapsedFromLastMotionDetection(OPEN_DOOR_TIME);
  bool m3IsElapsed = M3.isElapsedFromLastMotionDetection(OPEN_DOOR_TIME);

  return T.isElapsed(OPEN_DOOR_TIME) && m1IsElapsed && m2IsElapsed && m3IsElapsed;
}

bool T_S_DOOR_OPEN_S_DOOR_TO_LONG_OPEN() {
  return T.isElapsed(TO_LONG_OPEN_DOOR_TIME);
}

bool T_S_DOOR_TO_LONG_OPEN_S_CLOSING_DOOR() {
  bool m1IsElapsed = M1.isElapsedFromLastMotionDetection(OPEN_DOOR_TIME);
  bool m2IsElapsed = M2.isElapsedFromLastMotionDetection(OPEN_DOOR_TIME);
  bool m3IsElapsed = M3.isElapsedFromLastMotionDetection(OPEN_DOOR_TIME);

  return T.isElapsed(OPEN_DOOR_TIME) && m1IsElapsed && m2IsElapsed && m3IsElapsed;
}

bool T_S_CLOSING_DOOR_S_DOOR_CLOSED() {
  return T.isElapsed(OPENING_DOOR_TIME);
}

bool T_S_S_DOOR_CLOSED_S_MOTION_DETECTION() {
  return T.isElapsed(100);
}

bool T_S_CLOSING_DOOR_S_CLOSING_DOOR_INTERRUPTED() {
  bool m1 = M1.ping() != NO_MOTION;
  bool m2 = M2.ping() != NO_MOTION;
  bool m3 = M3.ping() != NO_MOTION;

  return m1 || m2 || m3;
}

bool T_S_CLOSING_DOOR_INTERRUPTED_S_OPENING_DOOR() {
  return T.isElapsed(4000);
}

void defineTransition() {
  S_SLEEP->addTransition(&T_S_SLEEP_S_WAKE_UP, S_WAKE_UP);

  S_WAKE_UP->addTransition(&T_S_WAKE_UP_S_MOTION_DETECTION, S_MOTION_DETECTION);

  S_MOTION_DETECTION->addTransition(&T_S_MOTION_DETECTION_S_MOTION_DETECTED, S_MOTION_DETECTED);
  S_MOTION_DETECTION->addTransition(&T_S_MOTION_DETECTION_S_SLEEP, S_SLEEP);

  S_MOTION_DETECTED->addTransition(&T_S_MOTION_DETECTED_S_OPENING_DOOR, S_OPENING_DOOR);

  S_OPENING_DOOR->addTransition(&T_S_OPENING_DOOR_S_DOOR_OPEN, S_DOOR_OPEN);

  S_DOOR_OPEN->addTransition(&T_S_DOOR_OPEN_S_CLOSING_DOOR, S_CLOSING_DOOR);
  S_DOOR_OPEN->addTransition(&T_S_DOOR_OPEN_S_DOOR_TO_LONG_OPEN, S_DOOR_TO_LONG_OPEN);

  S_DOOR_TO_LONG_OPEN->addTransition(&T_S_DOOR_TO_LONG_OPEN_S_CLOSING_DOOR, S_CLOSING_DOOR);

  S_CLOSING_DOOR->addTransition(&T_S_CLOSING_DOOR_S_DOOR_CLOSED, S_DOOR_CLOSED);
  S_CLOSING_DOOR->addTransition(&T_S_CLOSING_DOOR_S_CLOSING_DOOR_INTERRUPTED, S_CLOSING_DOOR_INTERRUPTED);

  S_CLOSING_DOOR_INTERRUPTED->addTransition(&T_S_CLOSING_DOOR_INTERRUPTED_S_OPENING_DOOR, S_OPENING_DOOR);

  S_DOOR_CLOSED->addTransition(&T_S_S_DOOR_CLOSED_S_MOTION_DETECTION, S_MOTION_DETECTION);
}
#pragma endregion STATES

#pragma region EEPROM
#define CHECK_NUMBER 0x63
#define OPEN_DOOR_ID 0x66

bool eeIsDoorOpen() {

  if (EEPROM24C32.readByte(105) != CHECK_NUMBER) {

    EEPROM24C32.writeByte(105, CHECK_NUMBER, false, false);
    EEPROM24C32.writeByte(106, 0x101, false, false);
    EEPROM24C32.writeUInt32(110, 0, false, false);
  }

  return EEPROM24C32.readByte(106) == OPEN_DOOR_ID;
}

void eeSetDoorOpen(bool isOpen) {
  EEPROM24C32.writeByte(106, isOpen ? OPEN_DOOR_ID : 0x101, false, false);

  if (isOpen) {
    uint openCount = EEPROM24C32.readUInt32(110);
    EEPROM24C32.writeUInt32(110, openCount + 1, false, false);
  }
}

uint getDoorOpenCount() {
  return EEPROM24C32.readUInt32(110);
}
#pragma endregion EEPROM

#pragma region MAIN
void setDefaultState() {

  // if (!EEPROM24C32.checkPresence()) {
  //   Serial.println("EEPROM24C32 check presence faild");
  //   SM.transitionTo(S_FATAL_ERROR);
  //   return;
  // }

  // if (eeIsDoorOpen()) {
  //   SM.transitionTo(S_DOOR_OPEN);
  //   return;
  // }

  SM.transitionTo(S_MOTION_DETECTION);
}

void before() {
  Serial.begin(115200);  
  Serial1.begin(MY_RS485_BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.println(SKETCH_NAME);

  inicjalizePins();
}

void setup() {
  Serial.begin(115200);  
  Serial1.begin(MY_RS485_BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.println(SKETCH_NAME);
    
  WiFi.mode( WIFI_MODE_NULL );

  inicjalizePins();
  inicjalizeI2C();

  delay(10000);

  defineTransition();
  setDefaultState();
}

void presentation()  //MySensors
{
  // sendSketchInfo(SKETCH_NAME, SOFTWARE_VERION);
  // present(1, S_BINARY, "Door");
}

void loop() {
  SM.run();
  Out1.update();
  Out2.update();
  Out3.update();

  M1.ping();
  M2.ping();
  M3.ping();

  delay(1);
}
#pragma endregion MAIN

#pragma region INICJALIZE
void inicjalizeI2C() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  EEPROM24C32.begin(0x50, false);
}

void inicjalizePins() {

  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, HIGH);

  pinMode(OUPUT_1_PIN, OUTPUT);
  digitalWrite(OUPUT_1_PIN, LOW);

  pinMode(OUPUT_2_PIN, OUTPUT);
  digitalWrite(OUPUT_2_PIN, LOW);

  pinMode(OUPUT_3_PIN, OUTPUT);
  digitalWrite(OUPUT_3_PIN, LOW);

  pinMode(OPEN_DOOR_PIN, OUTPUT);
  digitalWrite(OPEN_DOOR_PIN, LOW);

  pinMode(CLOSE_DOOR_PIN, OUTPUT);
  digitalWrite(CLOSE_DOOR_PIN, LOW);

  pinMode(MOTION_SENSOR_1_PIN, INPUT);
  pinMode(MOTION_SENSOR_2_PIN, INPUT);
  pinMode(MOTION_SENSOR_3_PIN, INPUT);

  gpio_wakeup_enable(GPIO_NUM_20, GPIO_INTR_HIGH_LEVEL);
  gpio_wakeup_enable(GPIO_NUM_19, GPIO_INTR_HIGH_LEVEL);
  esp_sleep_enable_gpio_wakeup();
}
#pragma endregion INICJALIZE

#define SOFTWARE_VERION "1.0"
#define SKETCH_NAME "M16_Cat_Door"

#pragma region INSTALATION

//#include "esp_random.h"  //brakuje w plikach mysensors dla esp32, sprawdzic i usunąć w nowych wersjach
/*
Działa z wersja 3.0.1
Uwaga aby nie zainstalować Arduino ESP32 Boards - trzeba po odinstalowaniu przeinstalować esp32
Dodać link w ustawieniach additional boards manager urls: https://espressif.github.io/arduino-esp32/package_esp32_dev_index.json
Płytka: ESP32C6 Dev Module

dla ESP32-C6 zakomentować w pliku:
C:\Users\Smith\Documents\Arduino\libraries\MySensors\hal\architecture\ESP32\MyHwESP32.h

static __inline__ void __psRestore(const uint32_t *__s)
{
  //XTOS_RESTORE_INTLEVEL(*__s);
}

*/
#pragma endregion INSTALATION

#include "DeviceDef.h"

#pragma region CONFIGURATION
//#define DEBUG_GK
#define ALARM_ENABLED              // w przypadku błędow uruchamiać alarm dzwiękowy
#define OPEN_CLOSE_SOUND           // sygnał dzwiekowy przy otwarciu/zamknieciu drzwi
#define OUT_2_ENABLED              // czy są 2 diodu - OUT1 -zielona, OUT2 - czerwona
#define USE_M1_M2_ON_DOOR_CLOSING  // czy wykrycie ruchy przez m1 i m2 także przerywa zamykanie drzwi
#define USE_M3_ON_DOOR_CLOSING     // czy wykrycie ruchy przez czujnik na sterowniku przerywa zamykanie drzwi

#define MOTION_1_DELAY 5 * 1000       // czas pomiędzy pierwszym wykryciem ruchu a kolejnym wykryciem uruchamiajacym otwarcie drzwi dla sensoru 1,
#define MOTION_1_DELAY_WAIT 4 * 1000  // czas oczekiwania na 2 wykrycie ruchu dla sensoru 1,

#define MOTION_2_DELAY 5 * 1000       // czas pomiędzy pierwszym wykryciem ruchu a wykryciem uruchamiajacym otwarcie dla sensoru 2,
#define MOTION_2_DELAY_WAIT 4 * 1000  // czas oczekiwania na 2 wykrycie ruchu dla sensoru 2,

#define OPENING_DOOR_TIME 11 * 1000               // czas otwierania drzwi
#define OPEN_DOOR_TIME 10 * 1000                  // czas oczekiwania na zamknięcie drzwi od ostatnieo wykrycia ruchu
#define TO_LONG_OPEN_DOOR_TIME 100 * 1000         // czas zbyt długiego otwarcia drzwi aby włączyc alarm
#define TIME_SLEEP_AFTER_LAST_DETECTION 5 * 1000  // czas przejscia w light sleep od ostatniego wykrycia ruchu, nie moze byc mniejsze niż MOTION_1_DELAY + MOTION_1_DELAY_WAIT
#define TIME_SLEEP_AFTER_START 40 * 1000          // czas przejscia w light sleep od startu
#define DOOR_INTERRUPTED_WAITING 6 * 1000         // czas zatrzymania w przypadku wykrycia ruchy przy zamykaniu - po tym czasie następuje otwarcie

#define TIME_SLEEP_AFTER_LAST_DETECTION_M1 MOTION_1_DELAY + MOTION_1_DELAY_WAIT + TIME_SLEEP_AFTER_LAST_DETECTION
#define TIME_SLEEP_AFTER_LAST_DETECTION_M2 MOTION_2_DELAY + MOTION_2_DELAY_WAIT + TIME_SLEEP_AFTER_LAST_DETECTION

#define CPU_SPEED 160

#define CHECK_NUMBER 0x40  //zmienic aby zresetować ustawienia zapisane w pamięci
#define FADE 2
#define FADE_OFF 100000
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

#define RX_PIN 21     //RX - RO,
#define TX_PIN 23     //TX - DI
#define DE_RE_PIN 22  //RE - DE
#pragma endregion BOARD_PIN_CONFIGURATION

#pragma region MY_SENSORS_CONFIGURATION
#define MY_NODE_ID 95  // id wezła dla my sensors
#define MY_PARENT_NODE_ID 0
#define MY_PARENT_NODE_IS_STATIC
#define MY_PASSIVE_NODE
#define MY_TRANSPORT_WAIT_READY_MS 1

// RS485
#define MY_DISABLED_SERIAL         // manual configure Serial1
#define MY_RS485                   // Enable RS485 transport layer
#define MY_RS485_DE_PIN 22         // Define this to enables DE-pin management on defined pin
#define MY_RS485_BAUD_RATE 9600    // Set RS485 baud rate to use
#define MY_RS485_HWSERIAL Serial1  //
#define MY_RS485_SOH_COUNT 6

#define MS_DOOR_STATUS_ID 1
#define MS_OPEN_DOOR_COUNT_ID 20
#define MS_OPEN_DOOR_ID 21
#define MS_CLOSE_DOOR_ID 22
#define MS_AUTH_BLE_ID 23
#define MS_LIGHT_ID 24
#define MS_TEMP_ID 25
#define MS_MIN_RSSI_ID 26
#define MS_OPEN_LOCK_ID 27

#define MS_DOOR_EDIT_START_ID 38

#define MS_SEND_TIMEOUT 10 * 1000
#define WAIT_TIME_FOR_MS_MESSAGE_BEFORE_SLEEP 100
#pragma endregion MY_SENSORS_CONFIGURATION

#pragma region TYPES

class StateChangeManager {
private:
  bool _states[10];

public:
  bool isStateChanged(bool state, int index) {
    bool changed = _states[index] != state;
    _states[index] = state;

    return changed;
  }
};

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


// klasa wykrywa ruch na podstawie przejscia 0/1
// class MotionDetect {
// private:
//   unsigned long _startAt;
//   unsigned long _lastMotionAt;

//   byte _pin;
//   unsigned long _motionDelay;
//   unsigned long _motionDelayTotal;

//   bool _lastState;
//   bool _secondMotionDetected;

// public:
//   MotionDetect(byte pin, unsigned long motionDelay, unsigned long motionDelayWait) {
//     _pin = pin;
//     _motionDelay = motionDelay;
//     _motionDelayTotal = motionDelay + motionDelayWait;
//   }

//   bool getPinState() {
//     return digitalRead(_pin);
//   }

//   void start() {
//     _secondMotionDetected = false;
//     _lastState = getPinState();
//     _startAt = 0;

//     if (_lastState) {
//       _lastMotionAt = millis();
//     }
//   }

//   void weakUp() {
//     _lastState = getPinState();
//     _secondMotionDetected = false;
//     if (_lastState) {
//       _startAt = millis();
//       _lastMotionAt = _startAt;
//     }
//   }

//   MotionDetectState ping() {
//     unsigned long current = millis();
//     bool state = getPinState();

//     if (state) {
//       _lastMotionAt = current;
//     }

//     if (_startAt > current) {
//       _startAt = 0;
//       _secondMotionDetected = false;
//       _lastState = state;
//       return NO_MOTION;
//     }

//     bool invoke = !_lastState && state && ((current - _startAt) > 100);
//     _lastState = state;

//     bool init = _startAt == 0;
//     bool isTotalPassed = current > (_startAt + _motionDelayTotal);
//     bool isDelayPassed = isTotalPassed || (current > (_startAt + _motionDelay));

//     if (invoke && isDelayPassed && !isTotalPassed && !init) {
//       _secondMotionDetected = true;
//       return MOTIONS_DETECTED;
//     }

//     if (_secondMotionDetected && !isTotalPassed && !init) {
//       return MOTIONS_DETECTED;
//     }

//     if (!isTotalPassed && !init) {
//       return ONE_MOTION_DETECTED;
//     }

//     _secondMotionDetected = false;

//     if (invoke) {
//       _startAt = current;
//       return ONE_MOTION_DETECTED;
//     }

//     return NO_MOTION;
//   }

//   bool isElapsedFromLastMotionDetection(unsigned long elapsed) {
//     unsigned long current = millis();

//     return _lastMotionAt + elapsed < current;
//   }

// private:
// };

// klasa wykrywa ruch na podstawie ciągłego stanu
class MotionDetectContinue {
private:
  unsigned long _lastLow;
  unsigned long _lastMotionAt;

  byte _pin;
  unsigned long _motionDelay;

public:
  MotionDetectContinue(byte pin, unsigned long motionDelay) {
    _pin = pin;
    _motionDelay = motionDelay;
  }

  bool getPinState() {
    return digitalRead(_pin);
  }

  void start() {
    bool lastState = getPinState();
    unsigned long current = millis();

    _lastLow = current;
    _lastMotionAt = lastState ? current : 0;
  }

  void weakUp() {
    bool lastState = getPinState();
    unsigned long current = millis();

    _lastLow = current;
    _lastMotionAt = lastState ? current : 0;
  }

  MotionDetectState ping() {
    unsigned long current = millis();
    bool state = getPinState();

    if (state) {
      _lastMotionAt = current;
    }

    if (_lastLow > current) {  // overflow
      _lastLow = current;
    }

    if (!state) {
      _lastLow = current;
      return NO_MOTION;
    }

    if (_lastLow + _motionDelay < current) {
      return MOTIONS_DETECTED;
    }

    if (_lastLow + 400 < current) {
      return ONE_MOTION_DETECTED;
    }

    return NO_MOTION;
  }

  bool isElapsedFromLastMotionDetection(unsigned long elapsed) {
    unsigned long current = millis();

    return _lastMotionAt + elapsed < current;
  }

private:
};
#pragma endregion TYPES

#pragma region GLOBAL_VARIABLE
#include "driver/ledc.h"
#include "driver/temperature_sensor.h"
#include "esp_bt_main.h"
#include "esp_bt.h"
#include "driver/uart.h"
#include "esp_wifi.h"
#include "driver/gpio.h"
#include <Wire.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEAddress.h>

#include <MySensors.h>
#include <StateMachine.h>
#include <24C32.h>
#include "Storage.h"
#include "BLEScanner.h"

#include "Blinkenlight.h"
#include "Fadinglight.h"

MyMessage mMessage;
StateChangeManager SCM;
EE EE24C32;
Storage EEStorage(&EE24C32);
BLEScanner ScannerGK(&EEStorage);

DoorManager Door(OPEN_DOOR_PIN, CLOSE_DOOR_PIN);
StateTime T;
StateTime MessageReceiveTime;
StateTime MessageSentTime;
bool messageSent = true;

//trigger detection
//MotionDetect M1(MOTION_SENSOR_1_PIN, MOTION_1_DELAY, MOTION_1_DELAY_WAIT);
//MotionDetect M2(MOTION_SENSOR_2_PIN, MOTION_2_DELAY, MOTION_2_DELAY_WAIT);
//MotionDetect M3(MOTION_SENSOR_3_PIN, 5 * 1000, 1);

//continue detection
MotionDetectContinue M1(MOTION_SENSOR_1_PIN, MOTION_1_DELAY);
MotionDetectContinue M2(MOTION_SENSOR_2_PIN, MOTION_2_DELAY);
MotionDetectContinue M3(MOTION_SENSOR_3_PIN, 5 * 1000);

Fadinglight Out1(OUPUT_1_PIN, false, 2);
Blinkenlight Out2(OUPUT_2_PIN);
Blinkenlight Out3(OUPUT_3_PIN);
#pragma endregion GLOBAL_VARIABLE

#pragma region TIME
time_t _lastDoorClose = 0;

time_t getNow() {
  time_t now;
  time(&now);
  return now;
}

bool canOpenDoorFromLastClose() {
  if (_lastDoorClose == 0) {
    return true;
  }

  time_t now = getNow();

  if (now < _lastDoorClose) {
    _lastDoorClose = 0;
    return true;
  }

  return _lastDoorClose + EEStorage.getDoorLockTime() < now;
}

void doorCloseMark() {
  _lastDoorClose = getNow();
}
#pragma endregion TIME

#pragma region STATES
void allLedOff() {
  Out1.off();
  Out2.off();
  Out3.off();
}

void setLightOn() {
  if (EEStorage.useLight()) {
    digitalWrite(POWER_PIN, HIGH);
  }
}

void setLightOff() {
  digitalWrite(POWER_PIN, LOW);
}

const SpeedSetting openingBuzzerSpeedSetting = {
  .on_ms = 150,
  .off_ms = 150,
  .pause_ms = 200,
  .ending_ms = 400,
};
StateMachine SM = StateMachine();

State *S_START_UP = SM.addState(&s_START_UP);
void s_START_UP() {
  if (SM.executeOnce) {
#if defined(DEBUG_GK)
    Serial.println("S_START_UP");
#endif

    T.stateStart();

    M1.start();
    M2.start();
    M3.start();

    allLedOff();

    Out1.updateFadeSpeed(FADE_OFF);
    Out1.blink(SPEED_RAPID);
  }
}

State *S_WAKE_UP = SM.addState(&s_WAKE_UP);
void s_WAKE_UP() {
  if (SM.executeOnce) {
#if defined(DEBUG_GK)
    Serial.println("S_WAKE_UP");
#endif

    T.stateStart();

    M1.weakUp();
    M2.weakUp();
    M3.weakUp();

    allLedOff();
  }
}

State *S_START_MOTION_DETECTION = SM.addState(&s_START_MOTION_DETECTION);
void s_START_MOTION_DETECTION() {
  if (SM.executeOnce) {
#if defined(DEBUG_GK)
    Serial.println("S_START_MOTION_DETECTION");
#endif
    T.stateStart();

    Door.end();

    M1.start();
    M2.start();
    M3.start();

    allLedOff();
  }
}

State *S_MOTION_DETECTION = SM.addState(&s_MOTION_DETECTION);
void s_MOTION_DETECTION() {
  if (SM.executeOnce) {
#if defined(DEBUG_GK)
    Serial.println("S_MOTION_DETECTION");
#endif
    T.stateStart();
    Door.end();

    allLedOff();

    if (EEStorage.isAuth()) {
      deInitBle();
    }
  }
}

State *S_ONE_MOTION_DETECTION = SM.addState(&s_ONE_MOTION_DETECTION);
void s_ONE_MOTION_DETECTION() {
  if (SM.executeOnce) {
#if defined(DEBUG_GK)
    Serial.println("S_ONE_MOTION_DETECTION");
#endif

    SpeedSetting waitingFade = {
      .on_ms = 1500,
      .off_ms = 1500,
      .pause_ms = 3000,
      .ending_ms = 6000,
    };

    Out1.updateFadeSpeed(FADE);
    Out1.blink(waitingFade);
    Out2.off();
    Out3.off();

    if (EEStorage.isAuth()) {
      initBle();
    }
  }
}


State *S_MOTION_DETECTED = SM.addState(&s_MOTION_DETECTED);
void s_MOTION_DETECTED() {
  if (SM.executeOnce) {
#if defined(DEBUG_GK)
    Serial.println("S_MOTION_DETECTED");
#endif
    T.stateStart();

    allLedOff();
  }
}

State *S_SLEEP = SM.addState(&s_SLEEP);
void s_SLEEP() {
  if (SM.executeOnce) {
#if defined(DEBUG_GK)
    Serial.println("S_SLEEP");
#endif

    T.stateStart();

    Door.end();
    allLedOff();
    setLightOff();

    /// sleep
    esp_light_sleep_start();

#if defined(DEBUG_GK)
    Serial.println("AWAKE_FROM_SLEEP");
#endif
  }
}

State *S_FATAL_ERROR = SM.addState(&s_FATAL_ERROR);
void s_FATAL_ERROR() {
  if (SM.executeOnce) {
#if defined(DEBUG_GK)
    Serial.println("S_FATAL_ERROR");
#endif

    T.stateStart();

    Door.end();

#ifdef OUT_2_ENABLED
    Out1.off();
    Out2.blink(SPEED_RAPID);
#else
    Out1.updateFadeSpeed(FADE_OFF);
    Out1.blink(SPEED_RAPID);
    Out2.off();
#endif

#ifdef ALARM_ENABLED
    Out3.blink(SPEED_RAPID);  // error dla buzzera
#endif
  }
}

State *S_OPENING_DOOR = SM.addState(&s_OPENING_DOOR);
void s_OPENING_DOOR() {
  if (SM.executeOnce) {
#if defined(DEBUG_GK)
    Serial.println("S_OPENING_DOOR");
#endif
    T.stateStart();

    Door.open();

    const SpeedSetting openingSetting = {
      .on_ms = 400,
      .off_ms = 400,
      .pause_ms = 800,
      .ending_ms = 1600,
    };

    Out1.updateFadeSpeed(FADE_OFF);
    Out1.blink(openingSetting);

    Out2.off();
#ifdef OPEN_CLOSE_SOUND
    Out3.pattern(2, openingBuzzerSpeedSetting, false);
#endif

    setLightOn();

    EEStorage.setDoorOpen(true);
    sentDoorOpen();
  }
}

State *S_DOOR_OPEN = SM.addState(&s_DOOR_OPEN);
void s_DOOR_OPEN() {
  if (SM.executeOnce) {
#if defined(DEBUG_GK)
    Serial.println("S_DOOR_OPEN");
#endif

    T.stateStart();

    Door.end();

    if (EEStorage.isDoorAlwaysOpen()) {
      Out1.off();
    } else {
      Out1.updateFadeSpeed(FADE_OFF);
      Out1.on();
    }

    Out2.off();
    Out3.off();

    if (EEStorage.useLight() && !EEStorage.isDoorAlwaysOpen()) {
      setLightOn();
    } else {
      setLightOff();
    }
  }
}

State *S_DOOR_TO_LONG_OPEN = SM.addState(&s_DOOR_TO_LONG_OPEN);
void s_DOOR_TO_LONG_OPEN() {
  if (SM.executeOnce) {
#if defined(DEBUG_GK)
    Serial.println("S_DOOR_TO_LONG_OPEN");
#endif

    T.stateStart();

#ifdef OUT_2_ENABLED
    Out1.off();
    Out2.blink(SPEED_RAPID);
#else
    Out1.updateFadeSpeed(FADE_OFF);
    Out1.blink(SPEED_RAPID);
    Out2.off();
#endif

#ifdef ALARM_ENABLED
    Out3.blink(SPEED_RAPID);  // error dla buzzera
#endif
  }
}

State *S_CLOSING_DOOR = SM.addState(&s_CLOSING_DOOR);
void s_CLOSING_DOOR() {
  if (SM.executeOnce) {
#if defined(DEBUG_GK)
    Serial.println("S_CLOSING_DOOR");
#endif

    T.stateStart();

    Door.close();

    const SpeedSetting closingSetting = {
      .on_ms = 400,
      .off_ms = 400,
      .pause_ms = 800,
      .ending_ms = 1600,
    };

    Out1.updateFadeSpeed(FADE_OFF);
    Out1.blink(closingSetting);
    Out2.off();

#ifdef OPEN_CLOSE_SOUND
    Out3.pattern(3, openingBuzzerSpeedSetting, false);
#endif
  }
}

State *S_DOOR_CLOSED = SM.addState(&s_DOOR_CLOSED);
void s_DOOR_CLOSED() {
  if (SM.executeOnce) {
#if defined(DEBUG_GK)
    Serial.println("S_DOOR_CLOSED");
#endif

    T.stateStart();

    Door.end();

    allLedOff();
    setLightOff();

    M1.start();
    M2.start();
    M3.start();

    EEStorage.setDoorOpen(false);
    doorCloseMark();

    sentDoorClose();
  }
}

State *S_CLOSING_DOOR_INTERRUPTED = SM.addState(&s_CLOSING_DOOR_INTERRUPTED);
void s_CLOSING_DOOR_INTERRUPTED() {
  if (SM.executeOnce) {
#if defined(DEBUG_GK)
    Serial.println("S_CLOSING_DOOR_INTERRUPTED");
#endif

    T.stateStart();

    Door.end();

#ifdef OUT_2_ENABLED
    Out1.off();
    Out2.blink(SPEED_RAPID);
#else
    Out1.updateFadeSpeed(FADE_OFF);
    Out1.blink(SPEED_RAPID);
    Out2.off();
#endif

    Out3.off();
  }
}

State *S_AUTH = SM.addState(&s_AUTH);
void s_AUTH() {
  if (SM.executeOnce) {
#if defined(DEBUG_GK)
    Serial.println("S_AUTH");
#endif
    T.stateStart();

    Out1.off();
#ifdef OUT_2_ENABLED
    Out2.on();
#else
    Out1.updateFadeSpeed(FADE_OFF);
    Out1.on();
#endif
    Out3.off();

    ScannerGK.scan();
  }
}

State *S_AUTH_SUCCESS = SM.addState(&s_AUTH_SUCCESS);
void s_AUTH_SUCCESS() {
  if (SM.executeOnce) {
#if defined(DEBUG_GK)
    Serial.println("S_AUTH_SUCCESS");
#endif
    T.stateStart();

    Out1.updateFadeSpeed(FADE_OFF);
    Out1.blink(SPEED_RAPID);

#ifdef OUT_2_ENABLED
    Out2.off();
#endif
    Out3.off();
  }
}

State *S_AUTH_FAILED = SM.addState(&s_AUTH_FAILED);
void s_AUTH_FAILED() {
  if (SM.executeOnce) {
#if defined(DEBUG_GK)
    Serial.println("S_AUTH_FAILED");
#endif
    T.stateStart();

    Out1.off();
#ifdef OUT_2_ENABLED
    Out2.blink(SPEED_RAPID);
#endif
    Out3.off();
  }
}

bool T_S_MOTION_DETECTION_S_ONE_MOTION_DETECTION() {
  if (EEStorage.isDoorAlwaysOpen()) {
    return true;
  }

  MotionDetectState m1Ping = M1.ping();

  if (m1Ping == ONE_MOTION_DETECTED || m1Ping == MOTIONS_DETECTED) {
    return true;
  }

  MotionDetectState m2Ping = M2.ping();

  if (m2Ping == ONE_MOTION_DETECTED || m2Ping == MOTIONS_DETECTED) {
    return true;
  }

  return false;
}

bool T_S_START_UP_S_MOTION_DETECTION() {
  return T.isElapsed(1400);
}

bool T_S_SLEEP_S_WAKE_UP() {
  if (EEStorage.isDoorOpen()) {
    return false;
  }

  return T.isElapsed(25) && M1.getPinState() == HIGH || M2.getPinState() == HIGH;
}

bool T_S_SLEEP_S_START_MOTION_DETECTION() {
  if (EEStorage.isDoorOpen()) {
    return false;
  }

  return T.isElapsed(5) && M1.getPinState() == LOW && M2.getPinState() == LOW;
}

bool T_S_SLEEP_S_DOOR_OPEN() {
  return T.isElapsed(25) && EEStorage.isDoorOpen();
}

bool T_S_WAKE_UP_S_MOTION_DETECTION() {
  return T.isElapsed(25);
}

bool T_S_ONE_MOTION_DETECTION_S_MOTION_DETECTION() {
  if (EEStorage.isDoorAlwaysOpen()) {
    return false;
  }

  return T.isElapsed(25) && M1.ping() == NO_MOTION && M2.ping() == NO_MOTION;
}

bool S_ONE_MOTION_DETECTIONN_S_MOTION_DETECTED() {
  if (EEStorage.isDoorAlwaysOpen()) {
    return true;
  }

  if (EEStorage.isDoorAlwaysClose()) {
    return false;
  }

  if (!canOpenDoorFromLastClose()) {
    return false;
  }

  return T.isElapsed(25) && M1.ping() == MOTIONS_DETECTED || M2.ping() == MOTIONS_DETECTED;
}

bool T_S_MOTION_DETECTION_S_SLEEP() {
  if (!T.isElapsed(WAIT_TIME_FOR_MS_MESSAGE_BEFORE_SLEEP)) {
    return false;
  }

  if (M1.ping() != NO_MOTION) {
    return false;
  }

  if (M2.ping() != NO_MOTION) {
    return false;
  }

  if (!M1.isElapsedFromLastMotionDetection(TIME_SLEEP_AFTER_LAST_DETECTION_M1)) {
    return false;
  }

  if (!M2.isElapsedFromLastMotionDetection(TIME_SLEEP_AFTER_LAST_DETECTION_M2)) {
    return false;
  }

  if (millis() < TIME_SLEEP_AFTER_START) {
    return false;
  }

  if (!MessageReceiveTime.isElapsed(TIME_SLEEP_AFTER_LAST_DETECTION)) {
    return false;
  }

  if (!messageSent && !MessageSentTime.isElapsed(MS_SEND_TIMEOUT)) {
    return false;
  }

  return true;
}

bool T_S_MOTION_DETECTED_S_START_MOTION_DETECTION() {
  if (EEStorage.isDoorAlwaysClose()) {
    return true;
  }

  return false;
}

bool T_S_MOTION_DETECTED_S_OPENING_DOOR() {
  if (EEStorage.isDoorAlwaysOpen()) {
    return true;
  }

  if (EEStorage.isDoorAlwaysClose()) {
    return false;
  }

  if (EEStorage.isAuth()) {
    return false;
  }

  return T.isElapsed(25);
}

bool T_S_MOTION_DETECTED_S_AUTH() {
  if (EEStorage.isDoorAlwaysOpen()) {
    return false;
  }

  if (EEStorage.isDoorAlwaysClose()) {
    return false;
  }

  if (!EEStorage.isAuth()) {
    return false;
  }

  return T.isElapsed(25);
}

bool T_S_AUTH_S_AUTH_SUCCESS() {
  if (!ScannerGK.isScanComplete()) {
    return false;
  }

  return ScannerGK.isAuth();
}

bool T_S_AUTH_SUCCESS_S_OPENING_DOOR() {
  return T.isElapsed(1200);
}

bool T_S_AUTH_S_AUTH_FAILED() {
  if (!ScannerGK.isScanComplete()) {
    return false;
  }

  return !ScannerGK.isAuth();
}

bool T_S_AUTH_FAILED_S_START_MOTION_DETECTION() {
  return T.isElapsed(1200);
}

bool T_S_OPENING_DOOR_S_DOOR_OPEN() {
  return T.isElapsed(OPENING_DOOR_TIME);
}

bool T_S_DOOR_OPEN_S_CLOSING_DOOR() {
  if (EEStorage.isDoorAlwaysOpen()) {
    return false;
  }

#ifdef USE_M1_M2_ON_DOOR_CLOSING
  if (!M1.isElapsedFromLastMotionDetection(OPEN_DOOR_TIME)) {
    return false;
  }

  if (!M2.isElapsedFromLastMotionDetection(OPEN_DOOR_TIME)) {
    return false;
  }
#endif

#ifdef USE_M3_ON_DOOR_CLOSING
  if (!M3.isElapsedFromLastMotionDetection(OPEN_DOOR_TIME)) {
    return false;
  }
#endif

  return T.isElapsed(OPEN_DOOR_TIME);
}

bool T_S_DOOR_OPEN_S_DOOR_TO_LONG_OPEN() {
  if (EEStorage.isDoorAlwaysOpen()) {
    return false;
  }

  return T.isElapsed(TO_LONG_OPEN_DOOR_TIME);
}

bool T_S_DOOR_OPEN_S_SLEEP() {
  if (!EEStorage.isDoorAlwaysOpen()) {
    return false;
  }

  if (!T.isElapsed(WAIT_TIME_FOR_MS_MESSAGE_BEFORE_SLEEP)) {
    return false;
  }

  if (!messageSent && !MessageSentTime.isElapsed(MS_SEND_TIMEOUT)) {
    return false;
  }

  if (!MessageReceiveTime.isElapsed(TIME_SLEEP_AFTER_LAST_DETECTION)) {
    return false;
  }

  if (M1.ping() != NO_MOTION) {
    return false;
  }

  if (M2.ping() != NO_MOTION) {
    return false;
  }

  if (!M1.isElapsedFromLastMotionDetection(TIME_SLEEP_AFTER_LAST_DETECTION_M1)) {
    return false;
  }

  if (!M2.isElapsedFromLastMotionDetection(TIME_SLEEP_AFTER_LAST_DETECTION_M2)) {
    return false;
  }

  if (millis() < TIME_SLEEP_AFTER_START) {
    return false;
  }

  return true;
}

bool T_S_DOOR_TO_LONG_OPEN_S_CLOSING_DOOR() {
#ifdef USE_M1_M2_ON_DOOR_CLOSING
  if (!M1.isElapsedFromLastMotionDetection(OPEN_DOOR_TIME)) {
    return false;
  }

  if (!M2.isElapsedFromLastMotionDetection(OPEN_DOOR_TIME)) {
    return false;
  }
#endif

#ifdef USE_M3_ON_DOOR_CLOSING
  if (!M3.isElapsedFromLastMotionDetection(OPEN_DOOR_TIME)) {
    return false;
  }
#endif

  return T.isElapsed(OPEN_DOOR_TIME);
}

bool T_S_CLOSING_DOOR_S_DOOR_CLOSED() {
  return T.isElapsed(OPENING_DOOR_TIME);
}

bool T_S_S_DOOR_CLOSED_S_START_MOTION_DETECTION() {
  return T.isElapsed(25);
}

bool T_S_START_MOTION_DETECTION_S_MOTION_DETECTION() {
  return T.isElapsed(25);
}

bool T_S_CLOSING_DOOR_S_CLOSING_DOOR_INTERRUPTED() {
#ifdef USE_M1_M2_ON_DOOR_CLOSING
  if (M1.ping() != NO_MOTION)
    return true;

  if (M2.ping() != NO_MOTION)
    return true;
#endif

#ifdef USE_M3_ON_DOOR_CLOSING
  if (M3.ping() != NO_MOTION)
    return true;
#endif

  return false;
}

bool T_S_CLOSING_DOOR_INTERRUPTED_S_OPENING_DOOR() {
  return T.isElapsed(DOOR_INTERRUPTED_WAITING);
}

void defineTransition() {
  S_START_UP->addTransition(&T_S_START_UP_S_MOTION_DETECTION, S_MOTION_DETECTION);

  S_SLEEP->addTransition(&T_S_SLEEP_S_WAKE_UP, S_WAKE_UP);
  S_SLEEP->addTransition(&T_S_SLEEP_S_START_MOTION_DETECTION, S_START_MOTION_DETECTION);
  S_SLEEP->addTransition(&T_S_SLEEP_S_DOOR_OPEN, S_DOOR_OPEN);

  S_WAKE_UP->addTransition(&T_S_WAKE_UP_S_MOTION_DETECTION, S_MOTION_DETECTION);

  S_START_MOTION_DETECTION->addTransition(&T_S_START_MOTION_DETECTION_S_MOTION_DETECTION, S_MOTION_DETECTION);

  S_MOTION_DETECTION->addTransition(&T_S_MOTION_DETECTION_S_ONE_MOTION_DETECTION, S_ONE_MOTION_DETECTION);
  S_MOTION_DETECTION->addTransition(&T_S_MOTION_DETECTION_S_SLEEP, S_SLEEP);

  S_ONE_MOTION_DETECTION->addTransition(&S_ONE_MOTION_DETECTIONN_S_MOTION_DETECTED, S_MOTION_DETECTED);
  S_ONE_MOTION_DETECTION->addTransition(&T_S_ONE_MOTION_DETECTION_S_MOTION_DETECTION, S_MOTION_DETECTION);

  S_MOTION_DETECTED->addTransition(&T_S_MOTION_DETECTED_S_OPENING_DOOR, S_OPENING_DOOR);
  S_MOTION_DETECTED->addTransition(&T_S_MOTION_DETECTED_S_AUTH, S_AUTH);
  S_MOTION_DETECTED->addTransition(&T_S_MOTION_DETECTED_S_START_MOTION_DETECTION, S_START_MOTION_DETECTION);

  S_AUTH->addTransition(&T_S_AUTH_S_AUTH_SUCCESS, S_AUTH_SUCCESS);
  S_AUTH->addTransition(&T_S_AUTH_S_AUTH_FAILED, S_AUTH_FAILED);

  S_AUTH_SUCCESS->addTransition(&T_S_AUTH_SUCCESS_S_OPENING_DOOR, S_OPENING_DOOR);
  S_AUTH_FAILED->addTransition(&T_S_AUTH_FAILED_S_START_MOTION_DETECTION, S_START_MOTION_DETECTION);

  S_OPENING_DOOR->addTransition(&T_S_OPENING_DOOR_S_DOOR_OPEN, S_DOOR_OPEN);

  S_DOOR_OPEN->addTransition(&T_S_DOOR_OPEN_S_CLOSING_DOOR, S_CLOSING_DOOR);
  S_DOOR_OPEN->addTransition(&T_S_DOOR_OPEN_S_DOOR_TO_LONG_OPEN, S_DOOR_TO_LONG_OPEN);
  S_DOOR_OPEN->addTransition(&T_S_DOOR_OPEN_S_SLEEP, S_SLEEP);

  S_DOOR_TO_LONG_OPEN->addTransition(&T_S_DOOR_TO_LONG_OPEN_S_CLOSING_DOOR, S_CLOSING_DOOR);

  S_CLOSING_DOOR->addTransition(&T_S_CLOSING_DOOR_S_DOOR_CLOSED, S_DOOR_CLOSED);
  S_CLOSING_DOOR->addTransition(&T_S_CLOSING_DOOR_S_CLOSING_DOOR_INTERRUPTED, S_CLOSING_DOOR_INTERRUPTED);

  S_CLOSING_DOOR_INTERRUPTED->addTransition(&T_S_CLOSING_DOOR_INTERRUPTED_S_OPENING_DOOR, S_OPENING_DOOR);

  S_DOOR_CLOSED->addTransition(&T_S_S_DOOR_CLOSED_S_START_MOTION_DETECTION, S_START_MOTION_DETECTION);
}
#pragma endregion STATES

#pragma region MY_SENSORS
bool isPresentedToController = false;
void presentation()  // MySensors
{
  MessageReceiveTime.stateStart();

  sendSketchInfo(SKETCH_NAME, SOFTWARE_VERION);

  present(MS_DOOR_STATUS_ID, S_DOOR, "Status otwarcia dzwi");
  present(MS_OPEN_DOOR_COUNT_ID, S_INFO, "Liczba cykli otwarcia");
  present(MS_OPEN_DOOR_ID, S_BINARY, "Drzwi zawsze otwarte");
  present(MS_CLOSE_DOOR_ID, S_BINARY, "Drzwi zawsze zamknięte");
  present(MS_AUTH_BLE_ID, S_BINARY, "Autoryzacja BLE");
  present(MS_LIGHT_ID, S_BINARY, "Swiatło");
  present(MS_TEMP_ID, S_TEMP, "Temperatura");
  present(MS_MIN_RSSI_ID, S_INFO, "Min RSSI");
  present(MS_OPEN_LOCK_ID, S_INFO, "Czas blokady");

  present(MS_DOOR_ADD_ID, S_INFO, "Dodanie BLE Address");
  present(MS_DOOR_REMOVE_ID, S_INFO, "Usunięcie BLE Address");

  presentBleDevices();

  SCM.isStateChanged(false, 1);
  isPresentedToController = true;
}

void presentBleDevices() {
  present(1, S_DOOR, SKETCH_NAME);

  for (int i = 0; i < EEStorage.getBleDevicesCount(); i++) {
    present(EEStorage.getBleId(i), S_DOOR, EEStorage.getBleId(i));
  }

  for (int i = 0; i < EEStorage.getBleDevicesCount(); i++) {
    present(EEStorage.getEditNoId(i), V_TEXT, EEStorage.getBleId(i));
  }
}

void sentMyDoorAlwaysOpenStatus() {
  MessageSentTime.stateStart();
  messageSent = false;

#if defined(DEBUG_GK)
  Serial.print("sentMyDoorAlwaysOpenStatus");
  Serial.println(EEStorage.isDoorAlwaysOpen() ? "1" : "0");
#endif

  mMessage.setType(V_STATUS);
  mMessage.setSensor(MS_OPEN_DOOR_ID);
  send(mMessage.set(EEStorage.isDoorAlwaysOpen() ? "1" : "0"));

  messageSent = true;
}

void sentMyDoorAlwaysCloseStatus() {
  MessageSentTime.stateStart();
  messageSent = false;

#if defined(DEBUG_GK)
  Serial.print("sentMyDoorAlwaysCloseStatus");
  Serial.println(EEStorage.isDoorAlwaysClose() ? "1" : "0");
#endif

  mMessage.setType(V_STATUS);
  mMessage.setSensor(MS_CLOSE_DOOR_ID);
  send(mMessage.set(EEStorage.isDoorAlwaysClose() ? "1" : "0"));

  messageSent = true;
}

void sentMyBleAuthStatus() {
  MessageSentTime.stateStart();
  messageSent = false;

  bool useAuth = EEStorage.isAuth();

#if defined(DEBUG_GK)
  Serial.print("sentMyBleAuthStatus");
  Serial.println(useAuth ? "1" : "0");
#endif

  mMessage.setType(V_STATUS);
  mMessage.setSensor(MS_AUTH_BLE_ID);
  send(mMessage.set(useAuth ? "1" : "0"));

  messageSent = true;
}

void sentLightStatus() {
  MessageSentTime.stateStart();
  messageSent = false;

  bool useLight = EEStorage.useLight();

#if defined(DEBUG_GK)
  Serial.print("sentLightStatus");
  Serial.println(useLight ? "1" : "0");
#endif

  mMessage.setType(V_STATUS);
  mMessage.setSensor(MS_LIGHT_ID);
  send(mMessage.set(useLight ? "1" : "0"));

  messageSent = true;
}

// void sentTempStatus() {
//   MessageSentTime.stateStart();
//   messageSent = false;

//   temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 50);
//   temperature_sensor_handle_t temp_sensor = NULL;
//   temperature_sensor_install(&temp_sensor_config, &temp_sensor);
//   temperature_sensor_enable(temp_sensor);
//   float tvalue;
//   temperature_sensor_get_celsius(temp_sensor, &tvalue);
//   temperature_sensor_disable(temp_sensor);
//   temperature_sensor_uninstall(temp_sensor);

// #if defined(DEBUG_GK)
//   Serial.print("sentTempStatus");
//   Serial.println(tvalue);
// #endif

//   mMessage.setType(V_TEMP);
//   mMessage.setSensor(MS_TEMP_ID);
//   send(mMessage.set(tvalue, 1));

//   messageSent = true;
// }

void sentMyDoorOpenCount() {
  MessageSentTime.stateStart();
  messageSent = false;

#if defined(DEBUG_GK)
  Serial.print("sentMyDoorOpenCount");
  Serial.println(EEStorage.getDoorOpenCount());
#endif

  uint32_t openCount = EEStorage.getDoorOpenCount();

  mMessage.setType(V_TEXT);
  mMessage.setSensor(MS_OPEN_DOOR_COUNT_ID);
  send(mMessage.set(openCount));

  messageSent = true;
}

void sentMyAllClientOpenDoorDefaultStatus() {
#if defined(DEBUG_GK)
  Serial.println("sentMyAllClientOpenDoorDefaultStatus");
#endif
  int devCount = EEStorage.getBleDevicesCount();

  sentMyClientOpenDoorStatusMy(1, EEStorage.isDoorOpen());

  for (int i = 0; i < devCount; i++) {
    sentMyClientOpenDoorStatusMy(EEStorage.getBleId(i), false);
  }
}

void sentMyClientOpenDoorStatus(int clientId, bool status) {
  sentMyClientOpenDoorStatusMy(1, status);

  if (clientId > 1) {
    sentMyClientOpenDoorStatusMy(clientId, status);
  }
}

void sentMyClientOpenDoorStatusMy(int clientId, bool status) {
  MessageSentTime.stateStart();
  messageSent = false;

#if defined(DEBUG_GK)
  Serial.println("sentMyClientOpenDoorStatus");

  Serial.print("Client ID: ");
  Serial.println(clientId);

  Serial.print("Status: ");
  Serial.println(status ? "1" : "0");
#endif

  mMessage.setType(V_TRIPPED);
  mMessage.setSensor(clientId);
  send(mMessage.set(status ? "1" : "0"));

  messageSent = true;
}

void sentMyAllClientDoorAddress() {
#if defined(DEBUG_GK)
  Serial.println("sentMyAllClientDoorAddress");
#endif
  int devCount = EEStorage.getBleDevicesCount();

  for (int i = 0; i < devCount; i++) {
    sentMyDoorAddress(EEStorage.getEditNoId(i), EEStorage.getBleAddress(i));
  }
}

void sentMyDoorAddress(int clientId, BLEAddress *address) {
  MessageSentTime.stateStart();
  messageSent = false;

  esp_bd_addr_t *adr = address->getNative();
  uint8_t a[6];
  memcpy(&a, adr, 6);

  auto size = 18;
  char *namech = (char *)malloc(size);
  snprintf(namech, size, "%02X:%02X:%02X:%02X:%02X:%02X", a[0], a[1], a[2], a[3], a[4], a[5]);

  mMessage.setType(V_TEXT);
  mMessage.setSensor(clientId);
  send(mMessage.set(namech));

  messageSent = true;
  free(namech);
}

int lastOpenClientId = 0;

void sentDoorOpen() {
  lastOpenClientId = ScannerGK.getAuthDeviceId();

  sentMyClientOpenDoorStatus(lastOpenClientId, true);
  sentMyDoorOpenCount();
}

void sentDoorClose() {
  if (lastOpenClientId == 0) {
    lastOpenClientId = 1;
  }

  sentMyClientOpenDoorStatus(lastOpenClientId, false);
  //sentTempStatus();
}

void sentMinRssi() {
  MessageSentTime.stateStart();
  messageSent = false;

#if defined(DEBUG_GK)
  Serial.print("sentMinRssi");
  Serial.println(EEStorage.getMinRSSI());
#endif

  uint8_t minRSSI = EEStorage.getMinRSSI();

  mMessage.setType(V_TEXT);
  mMessage.setSensor(MS_MIN_RSSI_ID);
  send(mMessage.set(minRSSI));

  messageSent = true;
}

void sentDoorLockTime() {
  MessageSentTime.stateStart();
  messageSent = false;

#if defined(DEBUG_GK)
  Serial.print("setDoorLockTime");
  Serial.println(EEStorage.getDoorLockTime());
#endif

  uint32_t doorLockTime = EEStorage.getDoorLockTime();

  mMessage.setType(V_TEXT);
  mMessage.setSensor(MS_OPEN_LOCK_ID);
  send(mMessage.set(doorLockTime));

  messageSent = true;
}

void sentAddressManager() {
  MessageSentTime.stateStart();
  messageSent = false;

  mMessage.setType(V_TEXT);
  mMessage.setSensor(MS_DOOR_ADD_ID);
  send(mMessage.set("FF:FF:FF:FF:FF:FF"));

  MessageSentTime.stateStart();
  mMessage.setSensor(MS_DOOR_REMOVE_ID);
  send(mMessage.set("FF:FF:FF:FF:FF:FF"));

  messageSent = true;
}

void sendAllMySensorsStatus() {
  sentMyAllClientOpenDoorDefaultStatus();
  sentMyAllClientDoorAddress();
  sentMyDoorOpenCount();
  sentMyDoorAlwaysOpenStatus();
  sentMyDoorAlwaysCloseStatus();
  sentMyBleAuthStatus();
  sentLightStatus();
  //sentTempStatus();
  sentMinRssi();
  sentDoorLockTime();
  sentAddressManager();
}

#pragma endregion MY_SENSORS

#pragma region MAIN
void setDefaultState() {
  if (!EEStorage.IsInicjalized()) {
#if defined(DEBUG_GK)
    Serial.println("EEStorage check presence failed");
#endif

    SM.transitionTo(S_FATAL_ERROR);
    return;
  }

  if (EEStorage.isDoorOpen()) {
    SM.transitionTo(S_DOOR_OPEN);
    return;
  }

  SM.transitionTo(S_START_UP);
}

void preHwInit() {
  setCpuFrequencyMhz(CPU_SPEED);

#if defined(DEBUG_GK)
  Serial.begin(115200);
#endif
  Serial1.begin(MY_RS485_BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);

#if defined(DEBUG_GK)
  Serial.println(SKETCH_NAME);
#endif

  inicjalizePins();
  inicjalizeI2C();

  EEStorage.Inicjalize();

  setLightOff();

  ScannerGK.init();
  deInitBle();
}

void before() {}

void setup() {
  defineTransition();
  setDefaultState();

  MessageReceiveTime.stateStart();
}

void loop() {
  M1.ping();
  M2.ping();
  M3.ping();

  SM.run();

  Out1.update();
  Out2.update();
  Out3.update();

  if (SCM.isStateChanged(isPresentedToController, 1)) {
    sendAllMySensorsStatus();
  }
}

void receive(const MyMessage &message) {
  MessageReceiveTime.stateStart();

  if (message.isAck())
    return;

  if (MS_OPEN_DOOR_ID == message.sensor && message.getType() == V_STATUS) {
    if (message.getBool() && EEStorage.isDoorAlwaysClose()) {
      EEStorage.setDoorAlwaysClose(false);
      sentMyDoorAlwaysCloseStatus();
    }

    EEStorage.setDoorAlwaysOpen(message.getBool());
    sentMyDoorAlwaysOpenStatus();
  }

  if (MS_CLOSE_DOOR_ID == message.sensor && message.getType() == V_STATUS) {
    if (message.getBool() && EEStorage.isDoorAlwaysOpen()) {
      EEStorage.setDoorAlwaysOpen(false);
      sentMyDoorAlwaysOpenStatus();
    }

    EEStorage.setDoorAlwaysClose(message.getBool());
    sentMyDoorAlwaysCloseStatus();
  }


  if (MS_AUTH_BLE_ID == message.sensor && message.getType() == V_STATUS) {
    bool prevAuth = EEStorage.isAuth();
    bool auth = message.getBool();

    if (!EEStorage.isAnyDeviceDefined()) {
      auth = false;
    }

    if (prevAuth && !auth) {
      deInitBle();
    }

    if (!prevAuth && auth) {
      initBle();
    }

    EEStorage.setAthorizationBle(auth);
    sentMyBleAuthStatus();
  }

  if (MS_LIGHT_ID == message.sensor && message.getType() == V_STATUS) {
    EEStorage.setLight(message.getBool());
    sentLightStatus();
  }


  if (MS_MIN_RSSI_ID == message.sensor && message.getType() == V_TEXT) {
    EEStorage.setMinRSSI(message.getByte());
    sentMinRssi();
  }

  if (MS_OPEN_LOCK_ID == message.sensor && message.getType() == V_TEXT) {
    EEStorage.setDoorLockTime(message.getUInt());
    sentDoorLockTime();
  }


  if (message.sensor >= MS_DOOR_EDIT_START_ID + 2 && message.sensor <= MS_DOOR_EDIT_START_ID + 11 && message.getType() == V_TEXT) {
    const char *mess = message.getString();

    uint8_t data[6];
    sscanf(mess, "%x:%x:%x:%x:%x:%x", &data[0], &data[1], &data[2], &data[3], &data[4], &data[5]);

    BLEAddress bleAddress(data);

    EEStorage.editBleAddress(message.sensor, &bleAddress);

    sentMyAllClientDoorAddress();
  }
}
#pragma endregion MAIN

#pragma region INICJALIZE
void initBle() {
  esp_wifi_start();

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  esp_err_t ret = esp_bt_controller_init(&bt_cfg);
  if (ret) {
    return;
  }

  ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
  if (ret) {
    return;
  }

  ret = esp_bluedroid_init();
  if (ret) {
    return;
  }

  ret = esp_bluedroid_enable();
  if (ret) {
    return;
  }
}

void deInitBle() {
  esp_wifi_stop();
  esp_bluedroid_disable();
  esp_bluedroid_deinit();
  esp_bt_controller_disable();
  esp_bt_controller_deinit();
  esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
}

void inicjalizeI2C() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
}

void inicjalizePins() {
#ifdef RGB_BUILTIN
  digitalWrite(RGB_BUILTIN, LOW);  // Turn the RGB LED off
#endif

  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, LOW);

  pinMode(OUPUT_1_PIN, OUTPUT);
  digitalWrite(OUPUT_1_PIN, LOW);
  analogWriteFrequency(OUPUT_1_PIN, 1000);

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

  gpio_sleep_set_direction(GPIO_NUM_21, GPIO_MODE_INPUT);
  gpio_sleep_set_pull_mode(GPIO_NUM_21, GPIO_PULLUP_ONLY);
  uart_set_wakeup_threshold(UART_NUM_1, 3);
  esp_sleep_enable_uart_wakeup(UART_NUM_1);

  gpio_wakeup_enable(GPIO_NUM_20, GPIO_INTR_HIGH_LEVEL);
  gpio_wakeup_enable(GPIO_NUM_19, GPIO_INTR_HIGH_LEVEL);

  esp_sleep_enable_gpio_wakeup();
}
#pragma endregion INICJALIZE

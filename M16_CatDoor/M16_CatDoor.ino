#define SOFTWARE_VERION "1.0"
#define SKETCH_NAME "M15_Cat_Door"

#pragma region INSTALATION

#include "esp_random.h"  //brakuje w plikach mysensors dla esp32, sprawdzicz y mozna usunąć w nowych wersjach
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

#include "DeviceDef.h"

DeviceDef devices[] = {
  DeviceDef(1, 3, 6, 2, 6, "Test")
};

#define ALARM_ENABLED  // w przypadku błędow uruchamiać alarm dzwiękowy
#define OUT_2_ENABLED  // czy dioda 2 jest zainstalowana - czerwona błędu - inne zachowanie jak są 2 diody

#define BLE_AUTH  // autoryzacja ble wymagana aby otworzyć drzwi - sterowane przez mysensors, aby zmienic trzeba

#define USE_M1_M2_ON_DOOR_CLOSING  // czy wykrycie ruchy przez m1 i m2 także przerywa zamykanie drzwi

#define MOTION_1_DELAY 7 * 1000       // czas pomiędzy pierwszym wykryciem ruchu a kolejnym wykryciem uruchamiajacym otwarcie drzwi dla sensoru 1,
#define MOTION_1_DELAY_WAIT 4 * 1000  // czas oczekiwania na 2 wykrycie ruchu dla sensoru 1,

#define MOTION_2_DELAY 7 * 1000       // czas pomiędzy pierwszym wykryciem ruchu a wykryciem uruchamiajacym otwarcie dla sensoru 2,
#define MOTION_2_DELAY_WAIT 4 * 1000  // czas oczekiwania na 2 wykrycie ruchu dla sensoru 2,

#define OPENING_DOOR_TIME 11 * 1000                 // czas otwierania drzwi
#define OPEN_DOOR_TIME 10 * 1000                     // czas oczekiwania na zamknięcie drzwi od ostatnieo wykrycia ruchu
#define TO_LONG_OPEN_DOOR_TIME 60 * 1000            // czas zbyt długiego otwarcia drzwi aby włączyc alarm
#define TIME_SLEEP_AFTER_LAST_DETECTION 120 * 1000  // czas przejscia w deep sleep od ostatniego wykrycia ruchu
#define DOOR_INTERRUPTED_WAITING 4 * 1000           // czas zatrzymania w przypadku wykrycia ruchy przy zamykaniu - po tym czasie następuje otwarcie

#define MY_NODE_ID 90  // id wezła dla my sensors

#define CHECK_NUMBER 0x68  //zmienic aby zresetować ustawienia zapisane w pamięci
#define DEBUG_GK           // for tests
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

#define RX_PIN 21
#define TX_PIN 23
#pragma endregion BOARD_PIN_CONFIGURATION

#pragma region MY_SENSORS_CONFIGURATION
// RS485
#define MY_DISABLED_SERIAL         // manual configure Serial1
#define MY_RS485                   // Enable RS485 transport layer
#define MY_RS485_DE_PIN 22         // Define this to enables DE-pin management on defined pin
#define MY_RS485_BAUD_RATE 9600    // Set RS485 baud rate to use
#define MY_RS485_HWSERIAL Serial1  //
#define MY_TRANSPORT_WAIT_READY_MS 1

#define MS_OPEN_DOOR_COUNT_ID 20
#define MS_OPEN_DOOR_ID 21
#define MS_CLOSE_DOOR_ID 22
#define MS_AUTH_BLE_ID 23
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
    _startAt = 0;

    if (_lastState) {
      _lastMotionAt = millis();
    }
  }

  void weakUp() {
    _lastState = getPinState();
    _secondMotionDetected = false;
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

    if (_startAt > current) {
      _startAt = 0;
      _secondMotionDetected = false;
      _lastState = state;
      return NO_MOTION;
    }

    bool invoke = !_lastState && state && ((current - _startAt) > 500);
    _lastState = state;

    bool isTotalPassed = current > (_startAt + _motionDelayTotal);
    bool isDelayPassed = isTotalPassed || (current > (_startAt + _motionDelay));

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
#include "Storage.h"

#include "Blinkenlight.h"
#include "Fadinglight.h"

MyMessage mMessage;
StateChangeManager SCM;
EE EE24C32;
Storage EEStorage(&EE24C32);

DoorManager Door(OPEN_DOOR_PIN, CLOSE_DOOR_PIN);
StateTime T;
MotionDetect M1(MOTION_SENSOR_1_PIN, MOTION_1_DELAY, MOTION_1_DELAY_WAIT);
MotionDetect M2(MOTION_SENSOR_2_PIN, MOTION_2_DELAY, MOTION_2_DELAY_WAIT);
MotionDetect M3(MOTION_SENSOR_3_PIN, 5 * 1000, 1);

Fadinglight Out1(OUPUT_1_PIN, false, 2);
Blinkenlight Out2(OUPUT_2_PIN);
Blinkenlight Out3(OUPUT_3_PIN);

#pragma endregion GLOBAL_VARIABLE

#pragma region BLE
int getBLEDevicesCount() {
  return sizeof(devices) / sizeof(*devices);
}

int getClientId() {

  int devicesCount = sizeof(devices) / sizeof(*devices);
  if (devicesCount == 0) {
    return 1;
  }

  // get ble devices
  return 1;
}

bool canClientOpenDoor() {
  if (!EEStorage.useAthorizationBle()) {
    return true;
  }

  if (getBLEDevicesCount() == 0) {
    return true;
  }

  return true;
}
#pragma endregion BLE

#pragma region MY_SENSORS
void sentMyDoorAlwaysOpenStatus() {
#if defined(DEBUG_GK)
  Serial.print("sentMyDoorAlwaysOpenStatus");
  Serial.println(EEStorage.isDoorAlwaysOpen() ? "1" : "0");
#endif

  mMessage.setType(V_STATUS);
  mMessage.setSensor(MS_OPEN_DOOR_ID);
  send(mMessage.set(EEStorage.isDoorAlwaysOpen() ? "1" : "0"));
}

void sentMyDoorAlwaysCloseStatus() {
#if defined(DEBUG_GK)
  Serial.print("sentMyDoorAlwaysCloseStatus");
  Serial.println(EEStorage.isDoorAlwaysClose() ? "1" : "0");
#endif

  mMessage.setType(V_STATUS);
  mMessage.setSensor(MS_CLOSE_DOOR_ID);
  send(mMessage.set(EEStorage.isDoorAlwaysClose() ? "1" : "0"));
}

void sentMyBleAuthStatus() {
#if defined(DEBUG_GK)
  Serial.print("sentMyBleAuthStatus");
  Serial.println(EEStorage.useAthorizationBle() ? "1" : "0");
#endif

  mMessage.setType(V_STATUS);
  mMessage.setSensor(MS_AUTH_BLE_ID);
  send(mMessage.set(EEStorage.useAthorizationBle() ? "1" : "0"));
}

void sentMyDoorOpenCount() {
#if defined(DEBUG_GK)
  Serial.print("sentMyDoorOpenCount");
  Serial.println(EEStorage.getDoorOpenCount());
#endif

  uint32_t openCount = EEStorage.getDoorOpenCount();

  mMessage.setType(V_TEXT);
  mMessage.setSensor(MS_OPEN_DOOR_COUNT_ID);
  send(mMessage.set(openCount));
}

void sentMyAllClientOpenDoorDefaultStatus() {
#if defined(DEBUG_GK)
  Serial.println("sentMyAllClientOpenDoorDefaultStatus");
#endif

  int devCount = getBLEDevicesCount();

  if (devCount == 0) {
    sentMyClientOpenDoorStatus(1, false);
    return;
  }

  for (int i = 0; i < getBLEDevicesCount(); i++) {
    sentMyClientOpenDoorStatus(devices[i].Id, false);
  }
}

void sentMyClientOpenDoorStatus(int clientId, bool status) {
#if defined(DEBUG_GK)
  Serial.println("sentMyClientOpenDoorStatus");

  Serial.print("Client ID: ");
  Serial.println(clientId);

  Serial.print("Status: ");
  Serial.println(status ? "1" : "0");
#endif

  mMessage.setType(V_STATUS);
  mMessage.setSensor(clientId);
  send(mMessage.set(status ? "1" : "0"));
}

int lastOpenClientId = 0;

void sentDoorOpen() {
  lastOpenClientId = getClientId();

  sentMyClientOpenDoorStatus(lastOpenClientId, true);
  sentMyDoorOpenCount();
}

void sentDoorClose() {
  if (lastOpenClientId == 0) {
    lastOpenClientId = 1;
  }

  sentMyClientOpenDoorStatus(lastOpenClientId, false);
}

void sendAllMySensorsStatus() {
  sentMyAllClientOpenDoorDefaultStatus();
  sentMyDoorOpenCount();
  sentMyDoorAlwaysOpenStatus();
  sentMyDoorAlwaysCloseStatus();
  sentMyBleAuthStatus();
}

#pragma endregion MY_SENSORS

#pragma region STATES
void allLedOff() {
  Out1.off();
  Out2.off();
  Out3.off();
}
StateMachine SM = StateMachine();

State *S_START_UP = SM.addState(&s_START_UP);
void s_START_UP() {
  if (SM.executeOnce) {
#if defined(DEBUG_GK)
    Serial.println("S_START_UP");
#endif

    T.stateStart();

    allLedOff();

    Out1.updateFadeSpeed(FADE_OFF);
    Out1.blink(SPEED_RAPID);
  }

  M1.start();
  M2.start();
  M3.start();
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

State *S_MOTION_DETECTION = SM.addState(&s_MOTION_DETECTION);
void s_MOTION_DETECTION() {
  if (SM.executeOnce) {
#if defined(DEBUG_GK)
    Serial.println("S_MOTION_DETECTION");
#endif

    T.stateStart();

    Door.end();

    allLedOff();
    return;
  }

  bool m1Ping = M1.ping() == ONE_MOTION_DETECTED;
  bool m2Ping = M2.ping() == ONE_MOTION_DETECTED;
  bool oneMotionDetected = m1Ping || m2Ping;

  if (!SCM.isStateChanged(oneMotionDetected, 0)) {
    return;
  }

  if (oneMotionDetected) {
#if defined(DEBUG_GK)
    Serial.println("ONE_MOTION_DETECTED");
#endif

    SpeedSetting waitingFade = {
      .on_ms = 1500,
      .off_ms = 1500,
      .pause_ms = 3000,
      .ending_ms = 6000,
    };

    Out1.updateFadeSpeed(FADE);
    Out1.blink(waitingFade);

  } else {
#if defined(DEBUG_GK)
    Serial.println("END_ONE_MOTION_DETECTED");
#endif

    Out1.off();
  }
}


State *S_SLEEP = SM.addState(&s_SLEEP);
void s_SLEEP() {
  if (SM.executeOnce) {
#if defined(DEBUG_GK)
    Serial.println("S_SLEEP");
    ///  Serial.flush();
    // Serial.end();
#endif

    //  Serial1.flush();
    // Serial1.end();

    T.stateStart();

    Door.end();

    allLedOff();

    digitalWrite(POWER_PIN, LOW);

    /// sleep
    esp_light_sleep_start();

    //wakeup
    digitalWrite(POWER_PIN, HIGH);

#if defined(DEBUG_GK)
    //    Serial.begin(115200);
    //   Serial.flush();
    Serial.println("AWAKE_FROM_SLEEP");
#endif

    //   Serial1.begin(MY_RS485_BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);

    //  inicjalizeI2C();

    //  EEStorage.Inicjalize();
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
    Out3.blink();  // error dla buzzera
#endif
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

State *S_MOTION_DETECTED_NO_AUTH = SM.addState(&s_MOTION_DETECTED_NO_AUTH);
void s_MOTION_DETECTED_NO_AUTH() {
  if (SM.executeOnce) {
#if defined(DEBUG_GK)
    Serial.println("S_MOTION_DETECTED_NO_AUTH");
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

    Out3.off();
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
    Out3.off();

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

    Out1.updateFadeSpeed(FADE_OFF);
    Out1.on();
    Out2.off();
    Out3.off();
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
    Out3.blink();  // error dla buzzera
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
    Out3.off();

    sentDoorClose();
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

    EEStorage.setDoorOpen(false);
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

bool T_S_START_UP_S_MOTION_DETECTION() {
  return T.isElapsed(2000);
}

bool T_S_SLEEP_S_WAKE_UP() {
  return T.isElapsed(100);
}

bool T_S_WAKE_UP_S_MOTION_DETECTION() {
  return T.isElapsed(100);
}

bool T_S_MOTION_DETECTION_S_MOTION_DETECTED() {
  if (EEStorage.isDoorAlwaysClose()) {
    return false;
  }

  bool isMotionDetect = M1.ping() == MOTIONS_DETECTED || M2.ping() == MOTIONS_DETECTED;

  // add ble auth

  return isMotionDetect;
}

bool T_S_MOTION_DETECTION_S_SLEEP() {
  if (!T.isElapsed(TIME_SLEEP_AFTER_LAST_DETECTION)) {
    return false;
  }

  if (M1.ping() != NO_MOTION) {
    return false;
  }

  if (M2.ping() != NO_MOTION) {
    return false;
  }

  if (M3.ping() != NO_MOTION) {
    return false;
  }

  if (!M1.isElapsedFromLastMotionDetection(TIME_SLEEP_AFTER_LAST_DETECTION)) {
    return false;
  }

  if (!M2.isElapsedFromLastMotionDetection(TIME_SLEEP_AFTER_LAST_DETECTION)) {
    return false;
  }

  if (!M3.isElapsedFromLastMotionDetection(TIME_SLEEP_AFTER_LAST_DETECTION)) {
    return false;
  }

  return true;
}

bool T_S_MOTION_DETECTED_S_OPENING_DOOR() {
  return T.isElapsed(100);
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

  if (!M3.isElapsedFromLastMotionDetection(OPEN_DOOR_TIME)) {
    return false;
  }

  return T.isElapsed(OPEN_DOOR_TIME);
}

bool T_S_DOOR_OPEN_S_DOOR_TO_LONG_OPEN() {
  if (EEStorage.isDoorAlwaysOpen()) {
    return false;
  }

  return T.isElapsed(TO_LONG_OPEN_DOOR_TIME);
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

  if (!M3.isElapsedFromLastMotionDetection(OPEN_DOOR_TIME)) {
    return false;
  }

  return T.isElapsed(OPEN_DOOR_TIME);
}

bool T_S_CLOSING_DOOR_S_DOOR_CLOSED() {
  return T.isElapsed(OPENING_DOOR_TIME);
}

bool T_S_S_DOOR_CLOSED_S_MOTION_DETECTION() {
  return T.isElapsed(100);
}

bool T_S_CLOSING_DOOR_S_CLOSING_DOOR_INTERRUPTED() {
#ifdef USE_M1_M2_ON_DOOR_CLOSING
  if (M1.ping() != NO_MOTION)
    return true;

  if (M2.ping() != NO_MOTION)
    return true;
#endif

  if (M3.ping() != NO_MOTION)
    return true;

  return false;
}

bool T_S_CLOSING_DOOR_INTERRUPTED_S_OPENING_DOOR() {
  return T.isElapsed(DOOR_INTERRUPTED_WAITING);
}

void defineTransition() {
  S_START_UP->addTransition(&T_S_START_UP_S_MOTION_DETECTION, S_MOTION_DETECTION);

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

#pragma endregion EEPROM

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
#if defined(DEBUG_GK)
  Serial.begin(115200);
#endif
  Serial1.begin(MY_RS485_BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
}

void before() {}

bool isPresentedToController = false;
void presentation()  // MySensors
{
  sendSketchInfo(SKETCH_NAME, SOFTWARE_VERION);

  present(MS_OPEN_DOOR_COUNT_ID, S_INFO, "Ilość otwarcia drzwi");
  present(MS_OPEN_DOOR_ID, S_BINARY, "Drzwi zawsze zamknięte");
  present(MS_CLOSE_DOOR_ID, S_BINARY, "Drzwi zawsze otwarte");
  present(MS_AUTH_BLE_ID, S_BINARY, "Autoryzacja BLE");

  if (getBLEDevicesCount() > 0) {
    for (int i = 0; i < getBLEDevicesCount(); i++) {
      present(devices[i].Id, S_DOOR, devices[i].Name);
    }
  } else {
    present(1, S_DOOR, SKETCH_NAME);
  }

  isPresentedToController = true;
}

#include "driver/ledc.h"

void setup() {
#if defined(DEBUG_GK)
  Serial.println(SKETCH_NAME);
#endif

  // WiFi.mode(WIFI_MODE_NULL);



  inicjalizePins();
  inicjalizeI2C();

  EEStorage.Inicjalize();

  defineTransition();
  setDefaultState();
}

void loop() {
  SM.run();

  Out1.update();
  Out2.update();
  Out3.update();

  M1.ping();
  M2.ping();
  M3.ping();

  if (SCM.isStateChanged(isPresentedToController, 1)) {
    sendAllMySensorsStatus();
  }
}

void receive(const MyMessage &message) {
  if (message.isAck())
    return;

  if (MS_OPEN_DOOR_ID == message.sensor) {
    EEStorage.setDoorAlwaysOpen(message.getBool());
    sentMyDoorAlwaysOpenStatus();
  }

  if (MS_CLOSE_DOOR_ID == message.sensor) {
    EEStorage.setDoorAlwaysClose(message.getBool());
    sentMyDoorAlwaysCloseStatus();
  }

  if (MS_AUTH_BLE_ID == message.sensor) {
    EEStorage.setAthorizationBle(message.getBool());
    sentMyBleAuthStatus();
  }
}
#pragma endregion MAIN

#pragma region INICJALIZE
void inicjalizeI2C() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
}

void inicjalizePins() {
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, HIGH);

  pinMode(OUPUT_1_PIN, OUTPUT);
  digitalWrite(OUPUT_1_PIN, LOW);
  //analogWriteResolution(OUPUT_1_PIN, 8);
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

  gpio_wakeup_enable(GPIO_NUM_20, GPIO_INTR_HIGH_LEVEL);
  gpio_wakeup_enable(GPIO_NUM_19, GPIO_INTR_HIGH_LEVEL);
  esp_sleep_enable_gpio_wakeup();
}
#pragma endregion INICJALIZE

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
#include "DeviceDef.h"

DeviceDef devices[] = {
    DeviceDef(1, 3, 6, 2, 6, "Test")};

#define ALARM_ENABLED // w przypadku błędow uruchamiać alarm dzwiękowy
#define OUT_2_ENABLED // czy dioda 2 jest zainstalowana - czerwona błędu - inne zachowanie jak są 2 diody

#define BLE_AUTH // autoryzacja ble wymagana aby otworzyć drzwi

#define USE_M1_M2_ON_DOOR_CLOSING // czy wykrycie ruchy przez m1 i m2 także przerywa zamykanie drzwi

#define MOTION_1_DELAY 5 * 1000      // czas pomiędzy pierwszym wykryciem ruchu a kolejnym wykryciem uruchamiajacym otwarcie drzwi dla sensoru 1,
#define MOTION_1_DELAY_WAIT 4 * 1000 // czas oczekiwania na 2 wykrycie ruchu dla sensoru 1,

#define MOTION_2_DELAY 5 * 1000      // czas pomiędzy pierwszym wykryciem ruchu a wykryciem uruchamiajacym otwarcie dla sensoru 2,
#define MOTION_2_DELAY_WAIT 5 * 1000 // czas oczekiwania na 2 wykrycie ruchu dla sensoru 2,

#define OPENING_DOOR_TIME 11 * 1000               // czas otwierania drzwi
#define OPEN_DOOR_TIME 8 * 1000                   // czas oczekiwania na zamknięcie drzwi od ostatnieo wykrycia ruchu
#define TO_LONG_OPEN_DOOR_TIME 60 * 1000          // czas zbyt długiego otwarcia drzwi aby włączyc alarm
#define TIME_SLEEP_AFTER_LAST_DETECTION 90 * 1000 // czas przejscia w deep sleep od ostatniego wykrycia ruchu

#define MY_NODE_ID 90 // id wezła dla my sensors

#define DEBUG_GK // for tests
#pragma endregion CONFIGURATION

#pragma region BOARD_PIN_CONFIGURATION
#define MOTION_SENSOR_1_PIN 20
#define MOTION_SENSOR_2_PIN 19
#define MOTION_SENSOR_3_PIN 18

#define OUPUT_1_PIN 5
#define OUPUT_2_PIN 0
#define OUPUT_3_PIN 1

#define POWER_PIN 2

#define OPEN_DOOR_PIN 10  // pin otwarcia
#define CLOSE_DOOR_PIN 11 // pin zamkniecia

#define SDA_PIN 6
#define SCL_PIN 7

#define RX_PIN 21
#define TX_PIN 23
#pragma endregion BOARD_PIN_CONFIGURATION

#pragma region MY_SENSORS_CONFIGURATION
// RS485
#define MY_DISABLED_SERIAL        // manual configure Serial1
#define MY_RS485                  // Enable RS485 transport layer
#define MY_RS485_DE_PIN 22        // Define this to enables DE-pin management on defined pin
#define MY_RS485_BAUD_RATE 9600   // Set RS485 baud rate to use
#define MY_RS485_HWSERIAL Serial1 //
#define MY_TRANSPORT_WAIT_READY_MS 1
#pragma endregion MY_SENSORS_CONFIGURATION

#pragma region TYPES
int getDevicesCount()
{
  return sizeof(devices) / sizeof(*devices);
}

class StateChangeManager
{
private:
  bool _states[10];

public:
  bool isStateChanged(bool state, int index)
  {
    bool changed = _states[index] != state;
    _states[index] = state;

    return changed;
  }
};

class DoorManager
{
private:
  byte _doorOpenPin;
  byte _doorClosePin;

public:
  DoorManager(byte doorOpenPin, byte doorClosePin)
  {
    _doorOpenPin = doorOpenPin;
    _doorClosePin = doorClosePin;
  }

  void open()
  {
    digitalWrite(_doorClosePin, LOW);
    digitalWrite(_doorOpenPin, HIGH);
  }

  void close()
  {
    digitalWrite(_doorOpenPin, LOW);
    digitalWrite(_doorClosePin, HIGH);
  }

  void end()
  {
    digitalWrite(_doorClosePin, LOW);
    digitalWrite(_doorOpenPin, LOW);
  }
};

class StateTime
{
private:
  unsigned long startState;

public:
  void stateStart()
  {
    startState = millis();
  }

  bool isElapsed(unsigned long elapsed)
  {
    unsigned long current = millis();

    if (startState > current)
    {
      startState = current;
      return false;
    }

    if (startState + elapsed < current)
    {
      return true;
    }

    return false;
  }
};

enum MotionDetectState
{
  NO_MOTION = 0,
  ONE_MOTION_DETECTED = 1,
  MOTIONS_DETECTED = 2,
};

class MotionDetect
{
private:
  unsigned long _startAt;
  unsigned long _lastMotionAt;

  byte _pin;
  unsigned long _motionDelay;
  unsigned long _motionDelayTotal;

  bool _lastState;
  bool _secondMotionDetected;

public:
  MotionDetect(byte pin, unsigned long motionDelay, unsigned long motionDelayWait)
  {
    _pin = pin;
    _motionDelay = motionDelay;
    _motionDelayTotal = motionDelay + motionDelayWait;
  }

  void start()
  {
    _secondMotionDetected = false;
    _lastState = getPinState();

    if (_lastState)
    {
      _lastMotionAt = millis();
    }
  }

  void weakUp()
  {
    _lastState = getPinState();

    if (_lastState)
    {
      _startAt = millis();
      _lastMotionAt = _startAt;
    }
  }

  MotionDetectState ping()
  {
    unsigned long current = millis();
    bool state = getPinState();

    if (state)
    {
      _lastMotionAt = current;
    }

    bool invoke = !_lastState && state;
    _lastState = state;

    bool isTotalPassed = current > _startAt + _motionDelayTotal;
    bool isDelayPassed = isTotalPassed || current > _startAt + _motionDelay;

    if (_startAt > current)
    {
      _startAt = current;
      _secondMotionDetected = false;

      return invoke ? ONE_MOTION_DETECTED : NO_MOTION;
    }

    if (invoke && isDelayPassed && !isTotalPassed)
    {
      _secondMotionDetected = true;
      return MOTIONS_DETECTED;
    }

    if (_secondMotionDetected && !isTotalPassed)
    {
      return MOTIONS_DETECTED;
    }

    if (!isTotalPassed)
    {
      return ONE_MOTION_DETECTED;
    }

    _secondMotionDetected = false;

    if (invoke)
    {
      _startAt = current;
      return ONE_MOTION_DETECTED;
    }

    return NO_MOTION;
  }

  bool isElapsedFromLastMotionDetection(unsigned long elapsed)
  {
    unsigned long current = millis();

    return _lastMotionAt + elapsed < current;
  }

private:
  bool getPinState()
  {
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

StateChangeManager SCM;
EE EEPROM24C32;
MyMessage mMessage;

DoorManager Door(OPEN_DOOR_PIN, CLOSE_DOOR_PIN);
StateTime T;
MotionDetect M1(MOTION_SENSOR_1_PIN, MOTION_1_DELAY, MOTION_1_DELAY_WAIT);
MotionDetect M2(MOTION_SENSOR_2_PIN, MOTION_2_DELAY, MOTION_2_DELAY_WAIT);
MotionDetect M3(MOTION_SENSOR_3_PIN, 5 * 1000, 1);

Blinkenlight Out1(OUPUT_1_PIN);
Fadinglight Out1Fade(OUPUT_1_PIN, false, 2);

Blinkenlight Out2(OUPUT_2_PIN);
Blinkenlight Out3(OUPUT_3_PIN);

#pragma endregion GLOBAL_VARIABLE

#pragma region BLE
int getClientId()
{

  int devicesCount = sizeof(devices) / sizeof(*devices);
  if (devicesCount == 0)
  {
    return 1;
  }

  // get ble devices
  return 1;
}

bool canClientOpenDoor()
{
#ifndef BLE_AUTH
  return true;
#endif

  int devicesCount = sizeof(devices) / sizeof(*devices);
  if (devicesCount == 0)
  {
    return true;
  }

  return true;
}
#pragma endregion BLE

#pragma region MY_SENSORS
int lastOpenClientId = 0;

void sentDoorOpen()
{
  lastOpenClientId = getClientId();

  mMessage.setSensor(lastOpenClientId);
  send(mMessage.set("1"));
}

void sentDoorClose()
{
  if (lastOpenClientId == 0)
  {
    lastOpenClientId = 1;
  }

  mMessage.setSensor(lastOpenClientId);
  send(mMessage.set("0"));
}

#pragma endregion MY_SENSORS

#pragma region STATES

StateMachine SM = StateMachine();

State *S_MOTION_DETECTION = SM.addState(&s_MOTION_DETECTION);
void s_MOTION_DETECTION()
{
  if (SM.executeOnce)
  {
#if defined(DEBUG_GK)
    Serial.println("S_MOTION_DETECTION");
#endif

    T.stateStart();

    M1.start();
    M2.start();
    M3.start();

    Door.end();

    Out1.off();
    Out1Fade.off();
    Out2.off();
    Out3.off();

    return;
  }

  bool m1Ping = M1.ping() == ONE_MOTION_DETECTED;
  bool m2Ping = M2.ping() == ONE_MOTION_DETECTED;
  bool oneMotionDetected = m1Ping || m2Ping;

  if (!SCM.isStateChanged(oneMotionDetected, 0))
  {
    return;
  }

  if (oneMotionDetected)
  {
    SpeedSetting waitingFade = {
        .on_ms = 1500,
        .off_ms = 1500,
        .pause_ms = 3000,
        .ending_ms = 6000,
    };
    Out1Fade.blink(waitingFade);
  }
  else
  {
    Out1Fade.off();
  }
}

State *S_WAKE_UP = SM.addState(&s_WAKE_UP);
void s_WAKE_UP()
{
  if (SM.executeOnce)
  {
#if defined(DEBUG_GK)
    Serial.println("S_WAKE_UP");
#endif

    M1.weakUp();
    M2.weakUp();
    M3.weakUp();
  }
}

State *S_SLEEP = SM.addState(&s_SLEEP);
void s_SLEEP()
{
  if (SM.executeOnce)
  {
#if defined(DEBUG_GK)
    Serial.println("S_SLEEP");
#endif

    Serial.flush();

    T.stateStart();

    Door.end();

    Out1.off();
    Out1Fade.off();
    Out2.off();
    Out3.off();

    digitalWrite(POWER_PIN, LOW);

    delay(100);

    esp_light_sleep_start();

    /// sleep

    digitalWrite(POWER_PIN, HIGH);
  }
}

State *S_FATAL_ERROR = SM.addState(&s_FATAL_ERROR);
void s_FATAL_ERROR()
{
  if (SM.executeOnce)
  {
#if defined(DEBUG_GK)
    Serial.println("S_FATAL_ERROR");
#endif

    T.stateStart();

    Door.end();

    Out1Fade.off();

#ifdef OUT_2_ENABLED
    Out1.off();

    Out2.blink(SPEED_RAPID);
#else
    Out1.blink(SPEED_RAPID);
    Out2.off();
#endif

#ifdef ALARM_ENABLED
    Out3.blink(); // error dla buzzera
#endif
  }
}

State *S_MOTION_DETECTED = SM.addState(&s_MOTION_DETECTED);
void s_MOTION_DETECTED()
{
  if (SM.executeOnce)
  {
#if defined(DEBUG_GK)
    Serial.println("S_MOTION_DETECTED");
#endif

    T.stateStart();

    Out1Fade.off();

    const SpeedSetting openingSetting = {
        .on_ms = 400,
        .off_ms = 400,
        .pause_ms = 800,
        .ending_ms = 1600,
    };
    Out1.blink(openingSetting);

    Out2.off();
    Out3.off();
  }
}

State *S_MOTION_DETECTED_NO_AUTH = SM.addState(&s_MOTION_DETECTED_NO_AUTH);
void s_MOTION_DETECTED_NO_AUTH()
{
  if (SM.executeOnce)
  {
#if defined(DEBUG_GK)
    Serial.println("S_MOTION_DETECTED_NO_AUTH");
#endif

    T.stateStart();

    Out1Fade.off();
#ifdef OUT_2_ENABLED
    Out1.off();
    Out2.blink();

#else
    Out1.blink();
    Out2.off();

#endif

    Out3.off();
  }
}

State *S_OPENING_DOOR = SM.addState(&s_OPENING_DOOR);
void s_OPENING_DOOR()
{
  if (SM.executeOnce)
  {
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

    Out1.off();
    Out1Fade.blink(openingSetting);

    Out2.off();
    Out3.off();

    eeSetDoorOpen(true);
    sentDoorOpen();
  }
}

State *S_DOOR_OPEN = SM.addState(&s_DOOR_OPEN);
void s_DOOR_OPEN()
{
  if (SM.executeOnce)
  {
#if defined(DEBUG_GK)
    Serial.println("S_DOOR_OPEN");
#endif

    T.stateStart();

    Door.end();

    Out1Fade.off();
    Out1.on();
    Out2.off();
    Out3.off();
  }
}

State *S_DOOR_TO_LONG_OPEN = SM.addState(&s_DOOR_TO_LONG_OPEN);
void s_DOOR_TO_LONG_OPEN()
{
  if (SM.executeOnce)
  {
#if defined(DEBUG_GK)
    Serial.println("S_DOOR_TO_LONG_OPEN");
#endif

    T.stateStart();

    Out1Fade.off();
#ifdef OUT_2_ENABLED
    Out1.off();
    Out2.blink(SPEED_RAPID);
#else
    Out1.blink(SPEED_RAPID);
    Out2.off();
#endif

#ifdef ALARM_ENABLED
    Out3.blink(); // error dla buzzera
#endif
  }
}

State *S_CLOSING_DOOR = SM.addState(&s_CLOSING_DOOR);
void s_CLOSING_DOOR()
{
  if (SM.executeOnce)
  {
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

    Out1.off();
    Out1Fade.blink(closingSetting);

    Out2.off();
    Out3.off();

    sentDoorClose();
  }
}

State *S_DOOR_CLOSED = SM.addState(&s_DOOR_CLOSED);
void s_DOOR_CLOSED()
{
  if (SM.executeOnce)
  {
#if defined(DEBUG_GK)
    Serial.println("S_DOOR_CLOSED");
#endif

    T.stateStart();

    Door.end();

    Out1Fade.off();
    Out1.off();
    Out2.off();
    Out3.off();

    eeSetDoorOpen(false);
  }
}

State *S_CLOSING_DOOR_INTERRUPTED = SM.addState(&s_CLOSING_DOOR_INTERRUPTED);
void s_CLOSING_DOOR_INTERRUPTED()
{
  if (SM.executeOnce)
  {
#if defined(DEBUG_GK)
    Serial.println("S_CLOSING_DOOR_INTERRUPTED");
#endif

    T.stateStart();

    Door.end();

    Out1Fade.off();

#ifdef OUT_2_ENABLED
    Out1.off();
    Out2.blink(SPEED_RAPID);
#else
    Out1.blink(SPEED_RAPID);
    Out2.off();
#endif

    Out3.off();
  }
}

bool T_S_SLEEP_S_WAKE_UP()
{
  return true;
}

bool T_S_WAKE_UP_S_MOTION_DETECTION()
{
  return true;
}

bool T_S_MOTION_DETECTION_S_MOTION_DETECTED()
{
  bool isMotionDetect = M1.ping() == MOTIONS_DETECTED || M2.ping() == MOTIONS_DETECTED || M3.ping() == MOTIONS_DETECTED;

  // add ble auth

  return isMotionDetect;
}

bool T_S_MOTION_DETECTION_S_SLEEP()
{
  if (!T.isElapsed(TIME_SLEEP_AFTER_LAST_DETECTION))
  {
    return false;
  }

  if (M1.ping() != NO_MOTION)
  {
    return false;
  }

  if (M2.ping() != NO_MOTION)
  {
    return false;
  }

  if (M3.ping() != NO_MOTION)
  {
    return false;
  }

  if (!M1.isElapsedFromLastMotionDetection(TIME_SLEEP_AFTER_LAST_DETECTION))
  {
    return false;
  }

  if (!M2.isElapsedFromLastMotionDetection(TIME_SLEEP_AFTER_LAST_DETECTION))
  {
    return false;
  }

  if (!M3.isElapsedFromLastMotionDetection(TIME_SLEEP_AFTER_LAST_DETECTION))
  {
    return false;
  }

  return true;
}

bool T_S_MOTION_DETECTED_S_OPENING_DOOR()
{
  return T.isElapsed(100);
}

bool T_S_OPENING_DOOR_S_DOOR_OPEN()
{
  return T.isElapsed(OPENING_DOOR_TIME);
}

bool T_S_DOOR_OPEN_S_CLOSING_DOOR()
{
#ifdef USE_M1_M2_ON_DOOR_CLOSING
  if (!M1.isElapsedFromLastMotionDetection(OPEN_DOOR_TIME))
  {
    return false;
  }

  if (!M2.isElapsedFromLastMotionDetection(OPEN_DOOR_TIME))
  {
    return false;
  }
#endif

  if (!M3.isElapsedFromLastMotionDetection(OPEN_DOOR_TIME))
  {
    return false;
  }

  return T.isElapsed(OPEN_DOOR_TIME)
}

bool T_S_DOOR_OPEN_S_DOOR_TO_LONG_OPEN()
{
  return T.isElapsed(TO_LONG_OPEN_DOOR_TIME);
}

bool T_S_DOOR_TO_LONG_OPEN_S_CLOSING_DOOR()
{
#ifdef USE_M1_M2_ON_DOOR_CLOSING
  if (!M1.isElapsedFromLastMotionDetection(OPEN_DOOR_TIME))
  {
    return false;
  }

  if (!M2.isElapsedFromLastMotionDetection(OPEN_DOOR_TIME))
  {
    return false;
  }
#endif

  if (!M3.isElapsedFromLastMotionDetection(OPEN_DOOR_TIME))
  {
    return false;
  }

  return T.isElapsed(OPEN_DOOR_TIME);
}

bool T_S_CLOSING_DOOR_S_DOOR_CLOSED()
{
  return T.isElapsed(OPENING_DOOR_TIME);
}

bool T_S_S_DOOR_CLOSED_S_MOTION_DETECTION()
{
  return T.isElapsed(100);
}

bool T_S_CLOSING_DOOR_S_CLOSING_DOOR_INTERRUPTED()
{
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

bool T_S_CLOSING_DOOR_INTERRUPTED_S_OPENING_DOOR()
{
  return T.isElapsed(4000);
}

void defineTransition()
{
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

bool eeIsDoorOpen()
{

  if (EEPROM24C32.readByte(105) != CHECK_NUMBER)
  {

    EEPROM24C32.writeByte(105, CHECK_NUMBER, false, false);
    EEPROM24C32.writeByte(106, 0x101, false, false);
    EEPROM24C32.writeUInt32(110, 0, false, false);
  }

  return EEPROM24C32.readByte(106) == OPEN_DOOR_ID;
}

void eeSetDoorOpen(bool isOpen)
{
  EEPROM24C32.writeByte(106, isOpen ? OPEN_DOOR_ID : 0x101, false, false);

  if (isOpen)
  {
    uint openCount = EEPROM24C32.readUInt32(110);
    EEPROM24C32.writeUInt32(110, openCount + 1, false, false);
  }
}

uint getDoorOpenCount()
{
  return EEPROM24C32.readUInt32(110);
}
#pragma endregion EEPROM

#pragma region MAIN
void setDefaultState()
{

  if (!EEPROM24C32.checkPresence())
  {
#if defined(DEBUG_GK)
    Serial.println("EEPROM24C32 check presence failed");
#endif

    SM.transitionTo(S_FATAL_ERROR);
    return;
  }

  if (eeIsDoorOpen())
  {
    SM.transitionTo(S_DOOR_OPEN);
    return;
  }

  SM.transitionTo(S_MOTION_DETECTION);
}

void preHwInit()
{
#if defined(DEBUG_GK)
  Serial.begin(115200);
#endif
  Serial1.begin(MY_RS485_BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
}

void before() {}

void presentation() // MySensors
{
  sendSketchInfo(SKETCH_NAME, SOFTWARE_VERION);

  if (getDevicesCount() > 0)
  {
    for (int i = 0; i < getDevicesCount(); i++)
    {
      present(devices[i].Id, S_DOOR, devices[i].Name);
    }
  }
  else
  {
    present(1, S_DOOR, SKETCH_NAME);
  }
}

void setup()
{
#if defined(DEBUG_GK)
  Serial.println(SKETCH_NAME);
#endif

  // WiFi.mode(WIFI_MODE_NULL);
  analogWriteFreq(100);

  inicjalizePins();
  inicjalizeI2C();

  defineTransition();
  setDefaultState();
}

void loop()
{
  SM.run();
  Out1.update();
  Out2.update();
  Out3.update();

  M1.ping();
  M2.ping();
  M3.ping();
}
#pragma endregion MAIN

#pragma region INICJALIZE
void inicjalizeI2C()
{
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  EEPROM24C32.begin(0x50, false);
}

void inicjalizePins()
{
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

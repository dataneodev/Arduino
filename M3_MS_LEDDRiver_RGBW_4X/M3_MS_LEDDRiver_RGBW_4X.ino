

/* #region  Instalation */
/*
Konfiguracja podłączenia kabli
-RGBW
  - 1 kanał - biały
  - 2 kanał - red
  - 3 kanał - green
  - 4 kanał - blue

-RGB
  - 1 kanał - red
  - 2 kanał - green
  - 3 kanał - blue

-Single
  - 1 kanał - white
*/
/* Instalacja 
Zainstalować  Arduino SAM Boards

MySensors 
  działa z oryginalnym boardem: https://github.com/rogerclarkmelbourne/Arduino_STM32
  nie działa natomiast z: https://github.com/stm32duino/Arduino_Core_STM32  (https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json)

 Skopiować folder do c:\Users\Smith\AppData\Local\Arduino15\packages\Arduino_STM32\
*/
/* #endregion */
//#include <libmaple/iwdg.h>

/* #region  user configuration */
#define SOFTWARE_VERION "1.0"
#define MIN_LIGHT_LEVEL 0      // minimalna jasnosc
#define MAX_LIGHT_LEVEL 1000   // maksymalna jasnosc nie moze przekroczyc PWM_PERIOD!!! 4kHz - 1000
#define STARTUP_LIGHT_LEVEL 5  //0-100 początkowa jasnosc jak włączono sterownik a poziom jasnosci jest 0

//tylko 1 z poniższych opcji moze być wybrana :
#define RGBW_MODE
//#define RGB_MODE
//#define SINGLE_LED_MODE

#define MY_NODE_ID 60  // id węzła my sensors - każdy sterownik musi miec inny numer
#define DIMMER_ID 1
#define RGBW_ID 1

/* #endregion */

/* #region  const configuration */

// RS485
#define MY_DISABLED_SERIAL         // manual configure Serial1
#define MY_RS485                   // Enable RS485 transport layer
#define MY_RS485_DE_PIN PA1         // Define this to enables DE-pin management on defined pin
#define MY_RS485_BAUD_RATE 9600    // Set RS485 baud rate to use
#define MY_RS485_HWSERIAL Serial2  //
#define MY_RS485_SOH_COUNT 6
#define MY_TRANSPORT_WAIT_READY_MS 1

//24C32
#define SCL_PIN PB10
#define SDA_PIN PB11

//relay
#define RELAY_PIN PB7

//pwm
#define PWM_1 PA10
#define PWM_2 PB6
#define PWM_3 PA8
#define PWM_4 PA9

//INPUT 
#define IN_1 PB1
#define IN_2 PB0
#define IN_3 PA5
#define IN_4 PA4

#if defined(RGBW_MODE)
#define SKETCH_NAME "RGBW_LED"
#endif

#if defined(RGB_MODE)
#define SKETCH_NAME "RGB_LED"
#endif

#if defined(SINGLE_LED_MODE)
#define SKETCH_NAME "SingleLED"
#endif
/* #endregion */

#include <MySensors.h>
#include "24C32.h"


//#define PWM_USE_NMI 1
#define PWM_PERIOD 1000  //4kH
#define PWM_CHANNELS 4


/* #region  global variable */
EE EEPROM24C32;
MyMessage mMessage;

bool deviceEnabled = true;
uint8_t deviceLightLevel = 50;  // 0 -100

#if defined(RGBW_MODE)
#define DEFAULT_CH2 255
#define DEFAULT_CH3 255
#define DEFAULT_CH4 255
#endif

#if defined(RGB_MODE)
#define DEFAULT_CH2 255
#define DEFAULT_CH3 255
#define DEFAULT_CH4 0
#endif

#if defined(SINGLE_LED_MODE)
#define DEFAULT_CH2 0
#define DEFAULT_CH3 0
#define DEFAULT_CH4 0
#endif

uint8_t channel1Level = 255;  // 0-255
uint8_t channel2Level = DEFAULT_CH2;
uint8_t channel3Level = DEFAULT_CH3;
uint8_t channel4Level = DEFAULT_CH4;

//unsigned long lastMessageRevice;
bool flashMemory = false;
bool statusChanged = false;
/* #endregion */

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

/* #region  basic function */

void before() {
  Serial2.begin(9600);
  inicjalizePins();
  inicjalizeI2C();
  readSettingFromEprom();
}

void setup() {
  calculate();
}

void presentation()  //MySensors
{
  sendSketchInfo(SKETCH_NAME, SOFTWARE_VERION);
  presentToControler();
  presentGlobalVariableToControler(true);
}

void loop() {
}

void receive(const MyMessage &message)  //MySensors
{
  if (message.isAck())
    return;

  if (message.sensor == DIMMER_ID && message.type == V_STATUS) {
    setDeviceEnabledFromControler(message.getBool());
    return;
  }

  if (message.sensor == DIMMER_ID && message.type == V_PERCENTAGE) {
    int val = atoi(message.data);
    if (val >= 0 && val <= 100) {
      setLightLevelFromControler(val);
    }
    return;
  }

#if defined(RGBW_MODE)
  if (message.sensor == RGBW_ID && message.type == V_RGBW) {
    unsigned long number = (unsigned long)strtoul(message.data, NULL, 16);
    uint8_t red = unsigned(number >> 24 & 0xFF);
    uint8_t green = unsigned(number >> 16 & 0xFF);
    uint8_t blue = unsigned(number >> 8 & 0xFF);
    uint8_t white = unsigned(number & 0xFF);

#if defined(MY_DEBUG)
    Serial.println("Otrzymano RGBW.");
    Serial.print("Decimal: ");
    Serial.println(number);

    Serial.print("Red: ");
    Serial.println(red);

    Serial.print("Green: ");
    Serial.println(green);

    Serial.print("Blue: ");
    Serial.println(blue);

    Serial.print("White: ");
    Serial.println(white);
#endif

    setRGBWvalueFromControler(red, green, blue, white);
    return;
  }
#endif

#if defined(RGB_MODE)
  if (message.sensor == RGBW_ID && message.type == V_RGB) {
    unsigned long number = (unsigned long)strtoul(message.data, NULL, 16);
    uint8_t red = unsigned(number >> 16 & 0xFF);
    uint8_t green = unsigned(number >> 8 & 0xFF);
    uint8_t blue = unsigned(number & 0xFF);

    setRGBValueFromControler(red, green, blue);
    return;
  }
#endif
}
/* #endregion */

/* #region  inicjalize */
void inicjalizePins() {
  //PWM
  digitalWrite(PWM_1, LOW);
  pinMode(PWM_1, OUTPUT);
  digitalWrite(PWM_1, LOW);

  digitalWrite(PWM_2, LOW);
  pinMode(PWM_2, OUTPUT);
  digitalWrite(PWM_2, LOW);

  digitalWrite(PWM_3, LOW);
  pinMode(PWM_3, OUTPUT);
  digitalWrite(PWM_3, LOW);

  digitalWrite(PWM_4, LOW);
  pinMode(PWM_4, OUTPUT);
  digitalWrite(PWM_4, LOW);

  digitalWrite(RELAY_PIN, LOW);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  pinMode(MY_RS485_DE_PIN, OUTPUT);

  pinMode(IN_1, INPUT);
  pinMode(IN_2, INPUT);
  pinMode(IN_3, INPUT);
  pinMode(IN_4, INPUT);
}

void inicjalizeI2C() {
  EEPROM24C32.begin(0x50, false);
}
/* #endregion */

/* #region  data read / save */
#if defined(RGBW_MODE)
#define CHECK_NUMBER 0x64
#endif

#if defined(RGB_MODE)
#define CHECK_NUMBER 0x62
#endif

#if defined(SINGLE_LED_MODE)
#define CHECK_NUMBER 0x63
#endif

void readSettingFromEprom() {
//   flashMemory = EEPROM24C32.checkPresence();
//   if (!flashMemory) {
// #if defined(MY_DEBUG)
//     Serial.println("Błąd pamieci 24C32");
// #endif
//     return;
//   }

  // if (EEPROM24C32.readByte(105) != CHECK_NUMBER) {
  //   setDefaultSetting();
  //   return;
  // }

  // deviceEnabled = EEPROM24C32.readByte(106) == 0x05;
  // deviceLightLevel = EEPROM24C32.readByte(107);
  // channel1Level = EEPROM24C32.readByte(108);
  // channel2Level = EEPROM24C32.readByte(109);
  // channel3Level = EEPROM24C32.readByte(110);
  // channel4Level = EEPROM24C32.readByte(111);

#if defined(MY_DEBUG)
  Serial.println("Odczytano wartosci:");
  Serial.print("channel1Level:");
  Serial.println(channel1Level);
#endif
}

void setDefaultSetting() {
#if defined(MY_DEBUG)
  Serial.println("Zapisuje domyslne ustawienia");
#endif
//  EEPROM24C32.writeByte(105, CHECK_NUMBER, false, false);
  setDeviceEnableToEeprom(deviceEnabled);
  setLightLevelToEeprom(deviceLightLevel);
  setChannelValue(1, channel1Level);
  setChannelValue(2, channel2Level);
  setChannelValue(3, channel3Level);
  setChannelValue(4, channel4Level);
}

void setDeviceEnableToEeprom(bool deviceEabled) {
 // EEPROM24C32.writeByte(106, deviceEabled ? 0x05 : 0x06, false, false);
}

void setLightLevelToEeprom(uint8_t lightLevel) {
 // EEPROM24C32.writeByte(107, lightLevel, false, false);
}

void setChannelValue(uint8_t channelNo, uint8_t value) {
  uint8_t channelAdress = 107 + channelNo;
 // EEPROM24C32.writeByte(channelAdress, value, false, false);
}
/* #endregion */

/* #region  present to controler */
void presentToControler() {
#if defined(SINGLE_LED_MODE)
  present(DIMMER_ID, S_DIMMER, "LED dimmer");
#endif

#if defined(RGBW_MODE)
  present(RGBW_ID, S_RGBW_LIGHT, "RGBW controller");
#endif

#if defined(RGB_MODE)
  present(RGBW_ID, S_RGB_LIGHT, "RGB controller");
#endif
}

void presentGlobalVariableToControler(bool forceSend) {
  if (forceSend || statusChanged) {
    mMessage.setSensor(DIMMER_ID);
    mMessage.setType(V_STATUS);
    send(mMessage.set(deviceEnabled));
    statusChanged = false;
  }

  mMessage.setType(V_PERCENTAGE);
  send(mMessage.set(deviceLightLevel));

#if defined(RGBW_MODE)
#if defined(MY_DEBUG)
  Serial.println("Wysyłam RGBW.");
  Serial.print("Red: ");
  Serial.println(channel2Level);

  Serial.print("Green: ");
  Serial.println(channel3Level);

  Serial.print("Blue: ");
  Serial.println(channel4Level);

  Serial.print("White: ");
  Serial.println(channel1Level);
#endif

  mMessage.setSensor(RGBW_ID);
  mMessage.setType(V_RGBW);

  char str[8];
  sprintf(&str[0], "%02x", channel2Level);
  sprintf(&str[2], "%02x", channel3Level);
  sprintf(&str[4], "%02x", channel4Level);
  sprintf(&str[6], "%02x", channel1Level);
  send(mMessage.set(str));
#endif

#if defined(RGB_MODE)
  mMessage.setSensor(RGBW_ID);
  mMessage.setType(V_RGB);

  char str[6];
  sprintf(&str[0], "%02x", channel1Level);
  sprintf(&str[2], "%02x", channel2Level);
  sprintf(&str[4], "%02x", channel3Level);
  send(mMessage.set(str));
#endif
}
/* #endregion */

void setDeviceEnabledFromControler(bool deviceEnbledToSet) {
  statusChanged = statusChanged || deviceEnabled != deviceEnbledToSet;
  deviceEnabled = deviceEnbledToSet;

  if (flashMemory) {
    setDeviceEnableToEeprom(deviceEnabled);
  }

  if (deviceEnabled && deviceLightLevel == 0) {
    setLightLevelFromControler(STARTUP_LIGHT_LEVEL);
    return;
  }

  calculate();
  presentGlobalVariableToControler(false);
}

void setLightLevelFromControler(uint8_t lightLevel) {
  deviceLightLevel = lightLevel;  // 0 -100

  if (flashMemory) {
    setLightLevelToEeprom(deviceLightLevel);
  }

  if (deviceLightLevel == 0 && deviceEnabled) {
    setDeviceEnabledFromControler(false);
    return;
  }

  if (deviceLightLevel > 0 && !deviceEnabled) {
    setDeviceEnabledFromControler(true);
    return;
  }

  calculate();
  presentGlobalVariableToControler(false);
}

#if defined(RGBW_MODE)
void setRGBWvalueFromControler(uint8_t red, uint8_t green, uint8_t blue, uint8_t white) {
  channel1Level = white;
  channel2Level = red;
  channel3Level = green;
  channel4Level = blue;

  calculate();
  presentGlobalVariableToControler(false);

  if (flashMemory) {
    setChannelValue(1, channel1Level);
    setChannelValue(2, channel2Level);
    setChannelValue(3, channel3Level);
    setChannelValue(4, channel4Level);
  }
}
#endif

#if defined(RGB_MODE)
void setRGBValueFromControler(uint8_t red, uint8_t green, uint8_t blue) {
  channel1Level = red;
  channel2Level = green;
  channel3Level = blue;

  calculate();
  presentGlobalVariableToControler(false);

  if (flashMemory) {
    setChannelValue(1, channel1Level);
    setChannelValue(2, channel2Level);
    setChannelValue(3, channel3Level);
  }
}
#endif

void calculate() {
  if (deviceEnabled) {
    unsigned int duty_1 = getChannel1Duty();
    unsigned int duty_2 = getChannel2Duty();
    unsigned int duty_3 = getChannel3Duty();
    unsigned int duty_4 = getChannel4Duty();

  //analogWrite
    // pwm_set_duty(duty_1, 0);
    // pwm_set_duty(duty_2, 1);
    // pwm_set_duty(duty_3, 2);
    // pwm_set_duty(duty_4, 3);
    // pwm_start();

    setRelayStatus(duty_1 > 0 || duty_2 > 0 || duty_3 > 0 || duty_4 > 0);
  } else {
    setRelayStatus(false);
    
    // pwm_set_duty(0, 0);
    // pwm_set_duty(0, 1);
    // pwm_set_duty(0, 2);
    // pwm_set_duty(0, 3);
    // pwm_start();
  }
}

void setRelayStatus(bool enabled) {
  digitalWrite(RELAY_PIN, enabled ? HIGH : LOW);
}

unsigned int getChannel1Duty() {
  return map(channel1Level * deviceLightLevel / 100, 0, 255, MIN_LIGHT_LEVEL, MAX_LIGHT_LEVEL);
}

unsigned int getChannel2Duty() {
#if defined(RGBW_MODE) || defined(RGB_MODE)
  return map(channel2Level * deviceLightLevel / 100, 0, 255, MIN_LIGHT_LEVEL, MAX_LIGHT_LEVEL);
#endif

#if defined(SINGLE_LED_MODE)
  return 0;
#endif
}

unsigned int getChannel3Duty() {
#if defined(RGBW_MODE) || defined(RGB_MODE)
  return map(channel3Level * deviceLightLevel / 100, 0, 255, MIN_LIGHT_LEVEL, MAX_LIGHT_LEVEL);
#endif

#if defined(SINGLE_LED_MODE)
  return 0;
#endif
}

unsigned int getChannel4Duty() {
#if defined(RGBW_MODE)
  return map(channel4Level * deviceLightLevel / 100, 0, 255, MIN_LIGHT_LEVEL, MAX_LIGHT_LEVEL);
#endif

#if defined(RGB_MODE) || defined(SINGLE_LED_MODE)
  return 0;
#endif
}

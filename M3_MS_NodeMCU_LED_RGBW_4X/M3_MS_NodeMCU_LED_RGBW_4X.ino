

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

/* #region  user configuration */
#define SOFTWARE_VERION "1.0"
#define MIN_LIGHT_LEVEL 0     // minimalna jasnosc
#define MAX_LIGHT_LEVEL 1023  // maksymalna jasnosc tj. otwarce kanału 0-1023
#define STARTUP_LIGHT_LEVEL 5 //0-100 początkowa jasnosc jak włączono sterownik a poziom jasnosci jest 0

//tylko 1 z poniższych opcji moze być wybrana :
#define RGBW_MODE
//#define RGB_MODE
//#define SINGLE_LED_MODE

#define MY_NODE_ID 30 // id węzła my sensors - każdy sterownik musi miec inny numer
#define DIMMER_ID 1
#define RGBW_ID 1

/* #endregion */

/* #region  const configuration */
#define MY_DISABLED_SERIAL
#define MY_PARENT_NODE_ID 0
#define MY_PARENT_NODE_IS_STATIC
//#define MY_TRANSPORT_WAIT_READY_MS 10000
#define MY_TRANSPORT_SANITY_CHECK
#define MY_TRANSPORT_SANITY_CHECK_INTERVAL_MS 600000
//#define MY_GATEWAY_SERIAL
//#define MY_DEBUG
//RS485
#define MY_RS485                // Enable RS485 transport layer
#define MY_RS485_DE_PIN D3      // Define this to enables DE-pin management on defined pin
#define MY_RS485_BAUD_RATE 9600 // Set RS485 baud rate to use
#include <SoftwareSerial.h>     //EspSoftwareSerial - dla płytki esp8266
SoftwareSerial swESP(D4, D2);   //RX - RO, TX - DI
#define MY_RS485_ESP swESP

//realy
#define RELAY_PIN 3

//24C32
#define SCL_PIN D5
#define SDA_PIN 1

//pwm
#define PWM_1 D1
#define PWM_2 D6
#define PWM_3 D7
#define PWM_4 D8

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

#include <Wire.h>
#include <MySensors.h>
#include <ESP8266WiFi.h>
#include <24C32.h>

/* #region  global variable */
EE EEPROM24C32;
MyMessage mMessage;

bool deviceEabled = true;
uint8_t deviceLightLevel = 50; // 0 -100

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

#define REVER_MESSAGE_TIME 1000

uint8_t channel1Level = 255; // 0-255
uint8_t channel2Level = DEFAULT_CH2;
uint8_t channel3Level = DEFAULT_CH3;
uint8_t channel4Level = DEFAULT_CH4;

//unsigned long lastMessageRevice;
bool flashMemory = false;
bool statusChanged = false;
bool levelChanged = false;
bool channelChanged = false;
/* #endregion */

/* #region  basic function */
void before()
{
  inicjalizePins();
  inicjalizeWifi();
  inicjalizeI2C();
  readSettingFromEprom();
  calculate();
}

void setup()
{
  // put your setup code here, to run once:
}

void presentation() //MySensors
{
  sendSketchInfo(SKETCH_NAME, SOFTWARE_VERION);
  presentToControler();
  presentGlobalVariableToControler(true);
}

void loop() {}

void receive(const MyMessage &message) //MySensors
{
  if (message.isAck())
    return;

  if (message.sensor == DIMMER_ID && message.type == V_STATUS)
  {
    setDeviceEnabledFromControler(message.getBool());
    return;
  }

  if (message.sensor == DIMMER_ID && message.type == V_PERCENTAGE)
  {
    int val = atoi(message.data);
    if (val >= 0 && val <= 100)
    {
      setLightLevelFromControler(val);
    }
    return;
  }

#if defined(RGBW_MODE)
  if (message.sensor == RGBW_ID && message.type == V_RGBW)
  {
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
  if (message.sensor == RGBW_ID && message.type == V_RGB)
  {
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
void inicjalizePins()
{
  //PWM
  analogWrite(PWM_1, 0);
  pinMode(PWM_1, OUTPUT);
  analogWrite(PWM_1, 0);

  analogWrite(PWM_2, 0);
  pinMode(PWM_2, OUTPUT);
  analogWrite(PWM_2, 0);

  analogWrite(PWM_3, 0);
  pinMode(PWM_3, OUTPUT);
  analogWrite(PWM_3, 0);

  analogWrite(PWM_4, 0);
  pinMode(PWM_4, OUTPUT);
  analogWrite(PWM_4, 0);

  analogWriteFreq(10000);

  digitalWrite(RELAY_PIN, LOW);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
}

void inicjalizeWifi()
{
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin(); //15mA
}

void inicjalizeI2C()
{
  Wire.begin(SDA_PIN, SCL_PIN);
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

void readSettingFromEprom()
{
  flashMemory = EEPROM24C32.checkPresence();
  if (!flashMemory)
  {
#if defined(MY_DEBUG)
    Serial.println("Błąd pamieci 24C32");
#endif
    return;
  }

  if (EEPROM24C32.readByte(105) != CHECK_NUMBER)
  {
    setDefaultSetting();
    return;
  }

  deviceEabled = EEPROM24C32.readByte(106) == 0x05;
  deviceLightLevel = EEPROM24C32.readByte(107);
  channel1Level = EEPROM24C32.readByte(108);
  channel2Level = EEPROM24C32.readByte(109);
  channel3Level = EEPROM24C32.readByte(110);
  channel4Level = EEPROM24C32.readByte(111);

#if defined(MY_DEBUG)
  Serial.println("Odczytano wartosci:");
  Serial.print("channel1Level:");
  Serial.println(channel1Level);
#endif
}

void setDefaultSetting()
{
#if defined(MY_DEBUG)
  Serial.println("Zapisuje domyslne ustawienia");
#endif
  EEPROM24C32.writeByte(105, CHECK_NUMBER, false, false);
  setDeviceEnableToEeprom(deviceEabled);
  setLightLevelToEeprom(deviceLightLevel);
  setChannelValue(1, channel1Level);
  setChannelValue(2, channel2Level);
  setChannelValue(3, channel3Level);
  setChannelValue(4, channel4Level);
}

void setDeviceEnableToEeprom(bool deviceEabled)
{
  EEPROM24C32.writeByte(106, deviceEabled ? 0x05 : 0x06, false, false);
}

void setLightLevelToEeprom(uint8_t lightLevel)
{
  EEPROM24C32.writeByte(107, lightLevel, false, false);
}

void setChannelValue(uint8_t channelNo, uint8_t value)
{
  uint8_t channelAdress = 107 + channelNo;
  EEPROM24C32.writeByte(channelAdress, value, false, false);
}
/* #endregion */

/* #region  present to controler */
void presentToControler()
{
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

void presentGlobalVariableToControler(bool forceSend)
{
  // if (forceSend || statusChanged)
  {
    mMessage.setSensor(DIMMER_ID);
    mMessage.setType(V_STATUS);
    send(mMessage.set(deviceEabled));
    statusChanged = false;
  }

  //  if (forceSend || levelChanged)
  {
    mMessage.setType(V_PERCENTAGE);
    send(mMessage.set(deviceLightLevel));
    levelChanged = false;
  }

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

  // if (forceSend || channelChanged)
  {
    mMessage.setSensor(RGBW_ID);
    mMessage.setType(V_RGBW);

    char str[8];
    sprintf(&str[0], "%02x", channel2Level);
    sprintf(&str[2], "%02x", channel3Level);
    sprintf(&str[4], "%02x", channel4Level);
    sprintf(&str[6], "%02x", channel1Level);
    send(mMessage.set(str));
    channelChanged = false;
  }
#endif

#if defined(RGB_MODE)
  // if (forceSend || channelChanged)
  {
    mMessage.setSensor(RGBW_ID);
    mMessage.setType(V_RGB);

    char str[6];
    sprintf(&str[0], "%02x", channel1Level);
    sprintf(&str[2], "%02x", channel2Level);
    sprintf(&str[4], "%02x", channel3Level);
    send(mMessage.set(str));
    channelChanged = false;
  }
#endif
}
/* #endregion */

void setDeviceEnabledFromControler(bool deviceEnbledToSet)
{
  statusChanged = deviceEabled != deviceEnbledToSet;
  deviceEabled = deviceEnbledToSet;

  if (flashMemory)
  {
    setDeviceEnableToEeprom(deviceEabled);
  }

  if (deviceEabled && deviceLightLevel == 0)
  {
    setLightLevelFromControler(STARTUP_LIGHT_LEVEL);
    return;
  }

  calculate();
  presentGlobalVariableToControler(false);
}

void setLightLevelFromControler(uint8_t lightLevel)
{
  levelChanged = deviceLightLevel != lightLevel;
  deviceLightLevel = lightLevel; // 0 -100

  if (flashMemory)
  {
    setLightLevelToEeprom(deviceLightLevel);
  }

  if (deviceLightLevel == 0 && deviceEabled)
  {
    setDeviceEnabledFromControler(false);
    return;
  }

  if (deviceLightLevel > 0 && !deviceEabled)
  {
    setDeviceEnabledFromControler(true);
    return;
  }

  calculate();
  presentGlobalVariableToControler(false);
}

#if defined(RGBW_MODE)
void setRGBWvalueFromControler(uint8_t red, uint8_t green, uint8_t blue, uint8_t white)
{
  channelChanged = channel1Level != white ||
                   channel2Level != red ||
                   channel3Level != green ||
                   channel4Level != blue;

  channel1Level = white;
  channel2Level = red;
  channel3Level = green;
  channel4Level = blue;

  calculate();
  presentGlobalVariableToControler(false);

  if (flashMemory)
  {
    setChannelValue(1, channel1Level);
    setChannelValue(2, channel2Level);
    setChannelValue(3, channel3Level);
    setChannelValue(4, channel4Level);
  }
}
#endif

#if defined(RGB_MODE)
void setRGBValueFromControler(uint8_t red, uint8_t green, uint8_t blue)
{
  channelChanged = channel1Level != red ||
                   channel2Level != green ||
                   channel3Level != blue;

  channel1Level = red;
  channel2Level = green;
  channel3Level = blue;

  calculate();
  presentGlobalVariableToControler(false);

  if (flashMemory)
  {
    setChannelValue(1, channel1Level);
    setChannelValue(2, channel2Level);
    setChannelValue(3, channel3Level);
  }
}
#endif

void calculate()
{
  if (deviceEabled)
  {
    analogWrite(PWM_1, getChannel1Duty());
    analogWrite(PWM_2, getChannel2Duty());
    analogWrite(PWM_3, getChannel3Duty());
    analogWrite(PWM_4, getChannel4Duty());
    digitalWrite(RELAY_PIN, HIGH);
  }
  else
  {
    analogWrite(PWM_1, 0);
    analogWrite(PWM_2, 0);
    analogWrite(PWM_3, 0);
    analogWrite(PWM_4, 0);
    digitalWrite(RELAY_PIN, LOW);
  }
}

unsigned int getChannel1Duty()
{
  return map(channel1Level * deviceLightLevel / 100, 0, 255, MIN_LIGHT_LEVEL, MAX_LIGHT_LEVEL);
}

unsigned int getChannel2Duty()
{
#if defined(RGBW_MODE) || defined(RGB_MODE)
  return map(channel2Level * deviceLightLevel / 100, 0, 255, MIN_LIGHT_LEVEL, MAX_LIGHT_LEVEL);
#endif

#if defined(SINGLE_LED_MODE)
  return 0;
#endif
}

unsigned int getChannel3Duty()
{
#if defined(RGBW_MODE) || defined(RGB_MODE)
  return map(channel3Level * deviceLightLevel / 100, 0, 255, MIN_LIGHT_LEVEL, MAX_LIGHT_LEVEL);
#endif

#if defined(SINGLE_LED_MODE)
  return 0;
#endif
}

unsigned int getChannel4Duty()
{
#if defined(RGBW_MODE)
  return map(channel4Level * deviceLightLevel / 100, 0, 255, MIN_LIGHT_LEVEL, MAX_LIGHT_LEVEL);
#endif

#if defined(RGB_MODE) || defined(SINGLE_LED_MODE)
  return 0;
#endif
}

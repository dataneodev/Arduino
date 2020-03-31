/*
   dataneo @2020 - M13_MS_SolarCWUHeater 1.0
*/
/* #region LCDConfiguration*/

// IMPORTANT: Adafruit_TFTLCD LIBRARY MUST BE SPECIFICALLY
// CONFIGURED FOR EITHER THE TFT SHIELD OR THE BREAKOUT BOARD.
// SEE RELEVANT COMMENTS IN Adafruit_TFTLCD.h FOR SETUP.

// The control pins for the LCD can be assigned to any digital or
// analog pins...but we'll use the analog pins as this allows us to
// double up the pins with the touch screen (see the TFT paint example).

// For the Arduino Mega, use digital pins 22 through 29
// (on the 2-row header at the end of the board).
//   D0 connects to digital pin 22
//   D1 connects to digital pin 23
//   D2 connects to digital pin 24
//   D3 connects to digital pin 25
//   D4 connects to digital pin 26
//   D5 connects to digital pin 27
//   D6 connects to digital pin 28
//   D7 connects to digital pin 29

#define LCD_CS 30    // Chip Select goes to Analog 3
#define LCD_CD 31    // Command/Data goes to Analog 2
#define LCD_WR 32    // LCD Write goes to Analog 1
#define LCD_RD 33    // LCD Read goes to Analog 0
#define LCD_RESET 36 // Can alternately just connect to Arduino's reset pin
#define LCD_ON_OFF A4

#define textScale 2
/* #endregion LCDConfiguration*/

/* #region  PINConfiguration */
#define PWM 5
#define BUZZER 6
#define CURRENT A1
#define VOLTAGE A7
#define RELAY 3
#define ONEWIRE 7
#define ENCODER_LEFT 37
#define ENCODER_RIGHT 34
#define ENCODER_DOWN 35
#define RESET_PIN 66
/* #endregion */

/* #region  MySensorsConfiguration */
#define MY_NODE_ID 50
#define MY_DEBUG
#define MY_GATEWAY_SERIAL // Enable serial gateway
#define MY_RS485          // Enable RS485 transport layer
#define MY_RS485_DE_PIN 4 // Define this to enables DE-pin management on defined pin
#define MY_RS485_BAUD_RATE 9600
#define MY_RS485_HWSERIAL Serial3
/* #endregion */

/* #region  OtherConfiguration */
#define EP24C32_ADDRESS 0x50
#define RELAY_PV_ID 1
#define RELAY_230V_ID 2
/* #endregion */

/* #region  GlobalVariable */
uint8_t ERROR_CODE = 0; // 0 no error

/* #endregion */

/* #region  objectInstances */
#include <Wire.h>

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library
Adafruit_TFTLCD gfx = Adafruit_TFTLCD(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

#include "I:/7.Projekty/5.Arduino/M_Library/DS18B20Manager/DS18B20Manager.h"
DS18B20Manager myDS18B20Manager = DS18B20Manager(ONEWIRE, 30, 900, false);

#include "I:\7.Projekty\5.Arduino\M_Library\BuzzerManager\BuzzerManager.h"
BuzzerManager buzzer = BuzzerManager(BUZZER);

#include "I:\7.Projekty\5.Arduino\M_Library\HeartBeatManager\HeartBeatManager.h"
HeartBeatManager gateway = HeartBeatManager(0);

#include <24C32.h>
EE EEPROM24C32;

#include "DataLayer.h"
DataLayer DL(&EEPROM24C32);

#include <DS3232RTC.h>
DS3232RTC myRTC;

#include <ClickEncoder.h>
ClickEncoder clickEncoder(ENCODER_LEFT, ENCODER_RIGHT, ENCODER_DOWN, 2, LOW);

#include "UserAction.h"
UserAction userAction(&clickEncoder);

#include "RelayManager.h"
RelayManager relayManager(DOMOTICZ, &DL);

#include "MenuCWU.h"

#include "MainScreen.h"
MainScreen mainScreen(&gfx, &DL);

#include "Display.h"
Display DI(&gfx, &navMenu, &userAction, &mainScreen, &DL);

#include "Core.h"
Core CO(&DL, &userAction);
/* #endregion objectInstances */

/* #region  Inicjalization */
void pinInicjalize()
{
  pinMode(CURRENT, INPUT);
  pinMode(VOLTAGE, INPUT);

  pinMode(PWM, OUTPUT);
  digitalWrite(PWM, LOW);

  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);

  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, LOW);

  pinMode(ONEWIRE, OUTPUT);
  digitalWrite(ONEWIRE, LOW);

  pinMode(LCD_ON_OFF, OUTPUT);
  digitalWrite(LCD_ON_OFF, LOW);

  pinMode(MY_RS485_DE_PIN, OUTPUT);
  digitalWrite(MY_RS485_DE_PIN, LOW);
}

void inicjalizeSystem()
{
  clickEncoder.setDoubleClickEnabled(false);
  clickEncoder.setButtonHeldEnabled(false);
  clickEncoder.setAccelerationEnabled(false);
  clickEncoder.setButtonOnPinZeroEnabled(false);

  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr);
}

void timerIsr()
{
  clickEncoder.service();
}

void mySensorsInicjalize()
{
  myDS18B20Manager.addSensor(0x28, 0xFF, 0x04, 0x23, 0x6E, 0x18, 0x01, 0x3A,
                             "TempSolarRelay",
                             true,
                             nullptr,
                             nullptr); // M8_MS_DS18B20SensorManager

  myDS18B20Manager.addSensor(0x28, 0xFF, 0x95, 0x1E, 0x6E, 0x18, 0x01, 0x95,
                             "Temp230VRelay",
                             true,
                             nullptr,
                             nullptr); // M8_MS_DS18B20SensorManager

  myDS18B20Manager.addSensor(0x28, 0xFF, 0x84, 0x1C, 0x6E, 0x18, 0x01, 0x23,
                             "TempWather",
                             true,
                             nullptr,
                             nullptr); // M8_MS_DS18B20SensorManager

  relayManager.addRelay(RELAY_PV_ID, "RelayPVEnable");
  relayManager.addRelay(RELAY_230V_ID, "Relay230VEnable");
}
/* #endregion Inicjalization */

void receiveTime(uint32_t ts)
{
  gateway.ControllerReciveMsg();
}

void receive(const MyMessage &message)
{
  gateway.ControllerReciveMsg();                          // M2_MS_Heartbeat
  relayManager.setStateOnRelayListFromControler(message); //M1_MS_RelayManager
}

void presentation() //MySensors
{
  sendSketchInfo("M13_MS_SolarCWUHeater", "1.0");
  myDS18B20Manager.presentAllToControler();
  relayManager.presentAllToControler(); //M1_MS_RelayManager
}

void before() //MySensors
{
  pinInicjalize();
  inicjalizeSystem();
  DL.Inicjalize(EP24C32_ADDRESS);
  mySensorsInicjalize();

  CO.Inicjalize();
  DI.Inicjalize();
}

void setup(void) {}

void loop(void)
{
  myDS18B20Manager.sensorsCheckLoop(); //M8_MS_DS18B20SensorManager
  gateway.HeartBeat();                 //M2_MS_Heartbeat

  userAction.Pool();
  CO.Pool();
  DI.Pool();
  navMenu.doInput();
}

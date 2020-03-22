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
/* #endregion */

/* #region  MySensorsConfiguration */
#define MY_DEBUG
#define MY_GATEWAY_SERIAL // Enable serial gateway
#define MY_RS485          // Enable RS485 transport layer
#define MY_RS485_DE_PIN 4 // Define this to enables DE-pin management on defined pin
#define MY_RS485_BAUD_RATE 9600
#define MY_RS485_HWSERIAL Serial3
/* #endregion */

/* #region  objectInstances */
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library
Adafruit_TFTLCD gfx = Adafruit_TFTLCD(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

#include "I:/7.Projekty/5.Arduino/M_Library/DS18B20Manager/DS18B20Manager.h"
DS18B20Manager myDS18B20Manager = DS18B20Manager(ONEWIRE, 30, 900, false);

#include "I:\7.Projekty\5.Arduino\M_Library\BuzzerManager\BuzzerManager.h"
BuzzerManager buzzer = BuzzerManager(BUZZER);

#include "I:\7.Projekty\5.Arduino\M_Library\HeartBeatManager\HeartBeatManager.h"
HeartBeatManager myHeartBeatManager = HeartBeatManager(0);

#include "I:\7.Projekty\5.Arduino\M_Library\RelayManager\RelayManager.h"
RelayManager myRelayController = RelayManager(HOMEASSISTANT, true);

#include <EE24C32.h>

#include <DS3232RTC.h>

#include <ClickEncoder.h>
ClickEncoder clickEncoder(ENCODER_LEFT, ENCODER_RIGHT, ENCODER_DOWN, 2, LOW);

/* #endregion objectInstances */

/* #region  Include */
#include "MenuCWU.h"
#include "Display.h"
#include "DataLayer.h"
#include "Core.h"

/* #endregion  Include */

/* #region  Inicjalization */
void pinInicjalize()
{
  pinMode(CURRENT, INPUT);
  pinMode(VOLTAGE, INPUT);

  pinMode(PWM, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(RELAY, OUTPUT);
  pinMode(ONEWIRE, OUTPUT);
  pinMode(LCD_ON_OFF, OUTPUT);
  digitalWrite(LCD_ON_OFF, HIGH);
}

void inicjalizeSystem()
{
  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr);
}

void timerIsr()
{
  clickEncoder.service();
}
/* #endregion Inicjalization */

void receiveTime(uint32_t ts)
{
  myHeartBeatManager.ControllerReciveMsg();
}

void receive(const MyMessage &message)
{
  myHeartBeatManager.ControllerReciveMsg();                    // M2_MS_Heartbeat
  myRelayController.setStateOnRelayListFromControler(message); //M1_MS_RelayManager
}

void presentation() //MySensors
{
}

void before() //MySensors
{
}

void setup(void)
{
  pinInicjalize();
  inicjalizeSystem();
  dataLayerInicjalization();
  coreInicjalization();
  lcdStartup();
  menuInicjalization();
  displayInicjalization();

  delay(1000);
  sendSketchInfo("M13_MS_SolarCWUHeater", "1.0");

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
  myDS18B20Manager.presentAllToControler();
  myRelayController.presentAllToControler(); //M1_MS_RelayManager
}

void loop(void)
{
  myDS18B20Manager.sensorsCheckLoop(); //M8_MS_DS18B20SensorManager
  myHeartBeatManager.HeartBeat();      //M2_MS_Heartbeat
  corePoll();
  delay(100); //simulate a delay when other tasks are done
}

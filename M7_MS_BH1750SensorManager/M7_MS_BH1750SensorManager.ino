/**
   The MySensors Arduino library handles the wireless radio link and protocol
   between your home built sensors/actuators and HA controller of choice.
   The sensors forms a self healing radio network with optional repeaters. Each
   repeater and gateway builds a routing tables in EEPROM which keeps track of the
   network topology allowing messages to be routed to nodes.

   Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
   Copyright (C) 2013-2015 Sensnology AB
   Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors

   Documentation: http://www.mysensors.org
   Support Forum: http://forum.mysensors.org

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License
   version 2 as published by the Free Software Foundation.
*/

// Enable debug prints to serial monitor
//#define MY_DEBUG

// Enable and select radio type attached
//#define MY_RADIO_NRF24
//#define MY_RADIO_NRF5_ESB
//#define MY_RADIO_RFM69
//#define MY_RADIO_RFM95

// Enable repeater functionality for this node
//#define MY_REPEATER_FEATURE

// Enable serial gateway
#define MY_GATEWAY_SERIAL

// Define a lower baud rate for Arduinos running on 8 MHz (Arduino Pro Mini 3.3V & SenseBender)
#if F_CPU == 8000000L
#define MY_BAUD_RATE 38400
#endif

// Enable inclusion mode
//#define MY_INCLUSION_MODE_FEATURE
// Enable Inclusion mode button on gateway
//#define MY_INCLUSION_BUTTON_FEATURE

// Inverses behavior of inclusion button (if using external pullup)
//#define MY_INCLUSION_BUTTON_EXTERNAL_PULLUP

// Set inclusion mode duration (in seconds)
//#define MY_INCLUSION_MODE_DURATION 60
// Digital pin used for inclusion mode button
//#define MY_INCLUSION_MODE_BUTTON_PIN  3

// Set blinking period
//#define MY_DEFAULT_LED_BLINK_PERIOD 300

// Inverses the behavior of leds
//#define MY_WITH_LEDS_BLINKING_INVERSE

// Flash leds on rx/tx/err
// Uncomment to override default HW configurations
//#define MY_DEFAULT_ERR_LED_PIN 4  // Error led pin
//#define MY_DEFAULT_RX_LED_PIN  6  // Receive led pin
//#define MY_DEFAULT_TX_LED_PIN  5  // the PCB, on board LED

#include <MySensors.h>

/*
   dataneo @2019 - M7_MS_BH1750SensorManager
   MySensors BH1750 Sensor Manager 1.1
   see https://sites.google.com/site/dataneosoftware/arduino/mysensors-bh1750-sensor-manager
*/
#include <Wire.h>
#include <ErriezBH1750.h>
#include <QList.h>

class BH1750Simple {
  public:
    BH1750Simple() {
      BH1750DeviceAdress = 0x23;
      TCA9548ADeviceAdress = 0;
      TCA9548APortNo = 0;
      _childID = 0;
      initCorrect = false;
    };
    BH1750Simple(uint8_t childID, uint8_t BH1750Adress, uint8_t TCA9548AAdress, uint8_t TCA9548APort, const char* bh1750_name = "") {
      BH1750DeviceAdress = BH1750Adress;
      TCA9548ADeviceAdress = TCA9548AAdress;
      TCA9548APortNo = TCA9548APort;
      _bh1750_name = bh1750_name;
      _childID = childID;
      initCorrect = false;
    };
    void readSensor(bool forceSendToController = false) {
      if(!initCorrect)
        return;
      float tempVar;
      if (TCA9548ADeviceAdress > 0)
        tcaSelect(TCA9548ADeviceAdress, TCA9548APortNo);
      tempVar = initCorrect ? bh1750Obj.read() * 0.5 : 0;
      if (tempVar != _lux || forceSendToController) {
        _lux = tempVar;
        sendStateToController();
      }
    }
    void initSensor() {
      if (TCA9548ADeviceAdress > 0)
        tcaSelect(TCA9548ADeviceAdress, TCA9548APortNo);
      if (BH1750DeviceAdress == 0x23)
        bh1750Obj = BH1750(LOW);
      else
        bh1750Obj = BH1750(HIGH);
      bh1750Obj.begin(ModeContinuous, ResolutionHigh);
      bh1750Obj.startConversion();
      initCorrect = true;
      readSensor(true);
    }
    void presentToControler() {
      present(getChildID(), S_LIGHT_LEVEL, _bh1750_name);
    }
    uint8_t getTCA9548ADeviceAdress() {
      return TCA9548ADeviceAdress;
    }
    uint8_t getTCA9548APortNo() {
      return TCA9548APortNo;
    }
    uint8_t getbh1750DeviceAdress() {
      return BH1750DeviceAdress;
    }
    uint8_t getChildID() {
      return _childID;
    }
    const char* getName() {
      return _bh1750_name;
    }
  private:
    uint8_t _childID;
    uint8_t TCA9548ADeviceAdress; // from 0x70 to 0x77 - use 0 for disable TCA9548A
    uint8_t TCA9548APortNo; // from 0 to 7
    uint8_t BH1750DeviceAdress; // 0x23 or 0x5C
    BH1750 bh1750Obj;
    float _lux;
    const char* _bh1750_name;
    bool initCorrect;
    static MyMessage msgLighLevel;
    
    void sendStateToController() {
      msgLighLevel.setSensor(_childID);
      send(msgLighLevel.set(_lux, 1));
    };
   
    void static tcaSelect(uint8_t TCA9548A, uint8_t i) {
      if (i > 7) return;
      Wire.beginTransmission(TCA9548A);
      Wire.write(1 << i);
      Wire.endTransmission();
    }
};

MyMessage BH1750Simple::msgLighLevel = MyMessage(1, V_LEVEL);

class BH1750Manager {
  public:
    BH1750Manager(uint8_t scanIntervalInSeconds) {
      if_init = false;
      scanInterval = scanIntervalInSeconds;
      lastScan = 0;
      lastID = START_ID;
    }
    void presentAllToControler() {
      if (BH1750List.length() > 0)
        for (byte i = 0; i < BH1750List.length(); i++)
          BH1750List[i].presentToControler();
      initAllSensors();
    }
    void sensorsCheck() {
      unsigned long timeNow = millis();
      if (lastScan > timeNow) { // overload
        lastScan = timeNow;
        return;
      }
      if (timeNow < lastScan + scanInterval * 1000)
        return;
      lastScan = timeNow;
      if (BH1750List.length() > 0 && if_init)
        for (byte i = 0; i < BH1750List.length(); i++)
          BH1750List[i].readSensor();
    }
    void addSensor() {
      addSensor(0x23, 0, 0, '\0');
    }
    void addEmpty() {
      lastID++;
    }
    void addSensor(uint8_t BH1750Adress, uint8_t TCA9548AAdress, uint8_t TCA9548APort, const char* bh1750_name) {
      if (BH1750Adress != 0x23 && BH1750Adress != 0x5C) return;
      if (TCA9548AAdress > 0 && (TCA9548AAdress < 0x70 || TCA9548AAdress > 0x77)) return;
      if (TCA9548APort < 0 || TCA9548APort > 7) return;
      //check if sensor exists
      bool exist = false;
      if (BH1750List.length() > 0)
        for (byte i = 0; i < BH1750List.length(); i++)
          if (BH1750List[i].getbh1750DeviceAdress() == BH1750Adress &&
              BH1750List[i].getTCA9548ADeviceAdress() == TCA9548AAdress &&
              BH1750List[i].getTCA9548APortNo() == TCA9548APort)
            exist = true;
      if (!exist){
        BH1750List.push_back(BH1750Simple(lastID, BH1750Adress, TCA9548AAdress, TCA9548APort, bh1750_name));
        lastID++;
      }
    }
  private:
    bool if_init;
    uint8_t scanInterval; // in seconds
    unsigned long lastScan;
    const uint8_t START_ID = 134;
    uint8_t lastID;
    QList<BH1750Simple> BH1750List;
    void initAllSensors() {
      Wire.begin();
      if (BH1750List.length() > 0 && !if_init)
        for (byte i = 0; i < BH1750List.length(); i++)
          BH1750List[i].initSensor();
      if_init = true;
    }
};

BH1750Manager myBH1750Manager(30); // set scan interval in seconds
/*  End of M7_MS_BH1750SensorManager */

void before()
{
  /* M7_MS_BH1750SensorManager */
  myBH1750Manager.addSensor(0x23, 0, 0, "Czujnik poziomu oświetlenia");  // M7_MS_BH1750SensorManager
}

void setup() { }

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("BH1750 Sensor Manager", "1.1");

  myBH1750Manager.presentAllToControler(); //M7_MS_BH1750SensorManager
}

void loop()
{
  myBH1750Manager.sensorsCheck(); //M7_MS_BH1750SensorManager
}

void receive(const MyMessage &message) { }

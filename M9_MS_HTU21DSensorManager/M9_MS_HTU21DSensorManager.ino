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
   dataneo @2018 - M9_MS_HTU21DSensorManager
   MySensors HTU21D Sensor Manager 1.0
   see https://sites.google.com/site/dataneosoftware/arduino/mysensors-htu21d-sensor-manager
*/
#include <Wire.h>
#include <HTU21D.h>
#include <QList.h>

class HTU21DSimple {
  public:
    HTU21DSimple() {
      TCA9548ADeviceAdress = 0;
      TCA9548APortNo = 0;
      child_id = 0;
      initCorrect = false;
    };
    HTU21DSimple(uint8_t id, uint8_t TCA9548AAdress, uint8_t TCA9548APort, const char* HTU21D_name) {
      TCA9548ADeviceAdress = TCA9548AAdress;
      TCA9548APortNo = TCA9548APort;
      _HTU21D_name = HTU21D_name;
      child_id = id;
      initCorrect = false;
    };
    void readSensor(bool forceSendToController = false) {
      float tempVar;
      if (TCA9548ADeviceAdress > 0)
        tcaSelect(TCA9548ADeviceAdress, TCA9548APortNo);
      //temp
      tempVar = initCorrect ? HTU21DObj.readTemperature() : 0;
      if (tempVar != _temperature || forceSendToController) {
        _temperature = tempVar;
        sendStateToController(S_TEMP);
      }
      //humi
      tempVar = initCorrect ? HTU21DObj.readHumidity() : 0;
      if (tempVar != _humidity || forceSendToController) {
        _humidity = tempVar;
        sendStateToController(S_HUM);
      }
    }
    void initSensor() {
      if (TCA9548ADeviceAdress > 0)
        tcaSelect(TCA9548ADeviceAdress, TCA9548APortNo);
      if (!HTU21DObj.begin())
        initCorrect = false;
      else{
        HTU21DObj.softReset();
        HTU21DObj.setResolution(HTU21D_RES_RH12_TEMP14);
        initCorrect = true;
      }  
      readSensor(true);
    }
    void presentToControler() {
      present(getChildID(S_TEMP), S_TEMP, _HTU21D_name);
      present(getChildID(S_HUM), S_HUM, _HTU21D_name);
    }
    uint8_t getTCA9548ADeviceAdress() {
      return TCA9548ADeviceAdress;
    }
    uint8_t getTCA9548APortNo() {
      return TCA9548APortNo;
    }
    uint8_t getHTU21DChildID() {
      return child_id;
    }
  private:
    HTU21D HTU21DObj;
    uint8_t TCA9548ADeviceAdress; // from 0x70 to 0x77 - use 0 for disable TCA9548A
    uint8_t TCA9548APortNo; // from 0 to 7
    float _temperature;
    float _humidity;
    const char* _HTU21D_name;
    uint8_t child_id;
    bool initCorrect;
    void sendStateToController(mysensors_sensor_t sensor) {
      MyMessage msgHumidity(getChildID(S_HUM), V_HUM);
      MyMessage msgTemperature(getChildID(S_TEMP), V_TEMP);;
      if (sensor == S_HUM)
        send(msgHumidity.set(_humidity, 1));
      if (sensor == S_TEMP)
        send(msgTemperature.set(_temperature, 1));
    };

    uint8_t getChildID(mysensors_sensor_t sensor) {
      uint8_t id = child_id;
      if (sensor == S_HUM) 
        id++;
      return id;
    };
    void tcaSelect(uint8_t TCA9548A, uint8_t i) {
      if (i > 7) return;
      Wire.beginTransmission(TCA9548A);
      Wire.write(1 << i);
      Wire.endTransmission();
    }
};

class HTU21DManager {
  public:
    HTU21DManager(uint8_t scanIntervalInSeconds) {
      if_init = false;
      scanInterval = scanIntervalInSeconds;
      lastScan = 0;
    }
    void presentAllToControler() {
      if (HTU21DList.length() > 0)
        for (byte i = 0; i < HTU21DList.length(); i++)
          HTU21DList[i].presentToControler();
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
      if (HTU21DList.length() > 0 && if_init)
        for (byte i = 0; i < HTU21DList.length(); i++)
          HTU21DList[i].readSensor();
    }
    void addSensor() {
      addSensor(0, 0, '\0');
    }
    void addSensor(uint8_t TCA9548AAdress, uint8_t TCA9548APort, const char* HTU21D_name) {
      if (HTU21DList.length() >= MAX_SENSORS) return;
      if (TCA9548AAdress > 0 && (TCA9548AAdress < 0x70 || TCA9548AAdress > 0x77)) return;
      if (TCA9548APort < 0 || TCA9548APort > 7) return;
      //check if sensor exists
      bool exist = false;
      if (HTU21DList.length() > 0)
        for (byte i = 0; i < HTU21DList.length(); i++)
          if (HTU21DList[i].getTCA9548ADeviceAdress() == TCA9548AAdress &&
              HTU21DList[i].getTCA9548APortNo() == TCA9548APort)
            exist = true;
      if (!exist){
        HTU21DList.push_back(HTU21DSimple(lastChildID, TCA9548AAdress, TCA9548APort, HTU21D_name));
        lastChildID = lastChildID + 2;  
      }
        
    }
  private:
    bool if_init;
    uint8_t scanInterval; // in seconds
    unsigned long lastScan;
    uint8_t lastChildID = 164;
    const byte MAX_SENSORS = 32;
    QList<HTU21DSimple> HTU21DList;
    void initAllSensors() {
      Wire.begin();
      if (HTU21DList.length() > 0 && !if_init)
        for (byte i = 0; i < HTU21DList.length(); i++)
          HTU21DList[i].initSensor();
      if_init = true;
    }
};

HTU21DManager myHTU21DManager = HTU21DManager(30); // set scan interval in seconds
/*  End of M9_MS_HTU21DSensorManager */

void before()
{
  /* M9_MS_HTU21DSensorManager */
  myHTU21DManager.addSensor(0, 0, "Kuchnia");  // M9_MS_HTU21DSensorManager
}

void setup() { }

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("HTU21D Sensor Manager", "1.0");

  myHTU21DManager.presentAllToControler(); //M9_MS_HTU21DSensorManager
}

void loop()
{
  myHTU21DManager.sensorsCheck(); //M9_MS_HTU21DSensorManager
}

void receive(const MyMessage &message) { }

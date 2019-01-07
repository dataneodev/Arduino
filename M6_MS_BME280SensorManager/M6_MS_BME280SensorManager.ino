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
   dataneo @2018 - M6_MS_BME280SensorManager
   MySensors BME280 Sensor Manager 1.0
   see https://sites.google.com/site/dataneosoftware/arduino/mysensors-bme280-sensor-manager
*/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <QList.h>

class BME280Simple {
  public:
    BME280Simple() {
      bme280DeviceAdress = 0x76;
      TCA9548ADeviceAdress = 0;
      TCA9548APortNo = 0;
      initCorrect = false;
    };
    BME280Simple(uint8_t bme280Adress, uint8_t TCA9548AAdress, uint8_t TCA9548APort, const char* bmp280_name) {
      bme280DeviceAdress = bme280Adress;
      TCA9548ADeviceAdress = TCA9548AAdress;
      TCA9548APortNo = TCA9548APort;
      _bmp280_name = bmp280_name;
      initCorrect = false;
    };
    void readSensor(bool forceSendToController = false) {
      float tempVar;
      if (TCA9548ADeviceAdress > 0)
        tcaSelect(TCA9548ADeviceAdress, TCA9548APortNo);

      bme280Obj.takeForcedMeasurement();
      //temp
      tempVar = initCorrect ? bme280Obj.readTemperature() : 0;
      if (tempVar != _temperature || forceSendToController) {
        _temperature = tempVar;
        sendStateToController(S_TEMP);
      }
      //humi
      tempVar = initCorrect ? bme280Obj.readHumidity() : 0;
      if (tempVar != _humidity || forceSendToController) {
        _humidity = tempVar;
        sendStateToController(S_HUM);
      }
      //press
      tempVar = initCorrect ? bme280Obj.readPressure() / 100.0F : 0;
      if (tempVar != _pressure || forceSendToController) {
        _pressure = tempVar;
        sendStateToController(S_BARO);
      }
    }
    void initSensor() {
      if (TCA9548ADeviceAdress > 0)
        tcaSelect(TCA9548ADeviceAdress, TCA9548APortNo);
      if (!bme280Obj.begin(bme280DeviceAdress))
        initCorrect = false;
      else
        initCorrect = true;
      bme280Obj.setSampling(Adafruit_BME280::MODE_FORCED,
                            Adafruit_BME280::SAMPLING_X1, // temperature
                            Adafruit_BME280::SAMPLING_X1, // pressure
                            Adafruit_BME280::SAMPLING_X1, // humidity
                            Adafruit_BME280::FILTER_OFF   );
      readSensor(true);
    }
    void presentToControler() {
      present(getChildID(S_TEMP), S_TEMP, _bmp280_name);
      present(getChildID(S_HUM), S_HUM, _bmp280_name);
      present(getChildID(S_BARO), S_BARO, _bmp280_name);
    }
    uint8_t getTCA9548ADeviceAdress() {
      return TCA9548ADeviceAdress;
    }
    uint8_t getTCA9548APortNo() {
      return TCA9548APortNo;
    }
    uint8_t getBme280DeviceAdress() {
      return bme280DeviceAdress;
    }
  private:
    Adafruit_BME280 bme280Obj;
    uint8_t TCA9548ADeviceAdress; // from 0x70 to 0x77 - use 0 for disable TCA9548A
    uint8_t TCA9548APortNo; // from 0 to 7
    uint8_t bme280DeviceAdress; // 0x76 or 0x77
    float _temperature;
    float _humidity;
    float _pressure;
    const char* _bmp280_name;
    bool initCorrect;
    void sendStateToController(mysensors_sensor_t sensor) {
      MyMessage msgHumidity(getChildID(S_HUM), V_HUM);
      MyMessage msgTemperature(getChildID(S_TEMP), V_TEMP);
      MyMessage msgPressure(getChildID(S_BARO), V_PRESSURE);
      MyMessage msgForecast(getChildID(S_BARO), V_FORECAST);
      if (sensor == S_HUM)
        send(msgHumidity.set(_humidity, 1));
      if (sensor == S_TEMP)
        send(msgTemperature.set(_temperature, 1));
      if (sensor == S_BARO) {
        send(msgPressure.set(_pressure, 1));
        send(msgForecast.set("unknown"));
      }
    };

    uint8_t getChildID(mysensors_sensor_t sensor) {
      uint8_t id = 70 + (bme280DeviceAdress - 0x76) * 3; // start from 70 to 134
      if (TCA9548ADeviceAdress > 0) {
        id = 76;
        id = id + (TCA9548ADeviceAdress - 0x70) * 24 + TCA9548APortNo * 3;
      }
      if (sensor == S_HUM) id = id + 1;
      if (sensor == S_BARO) id = id + 2;
      return id;
    };
    void tcaSelect(uint8_t TCA9548A, uint8_t i) {
      if (i > 7) return;
      Wire.beginTransmission(TCA9548A);
      Wire.write(1 << i);
      Wire.endTransmission();
    }
};

class BME280Manager {
  public:
    BME280Manager(uint8_t scanIntervalInSeconds) {
      if_init = false;
      scanInterval = scanIntervalInSeconds;
      lastScan = 0;
    }
    void presentAllToControler() {
      if (bmp280List.length() > 0)
        for (byte i = 0; i < bmp280List.length(); i++)
          bmp280List[i].presentToControler();
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
      if (bmp280List.length() > 0 && if_init)
        for (byte i = 0; i < bmp280List.length(); i++)
          bmp280List[i].readSensor();
    }
    void addSensor() {
      addSensor(0x76, 0, 0, '\0');
    }
    void addSensor(uint8_t bme280Adress, uint8_t TCA9548AAdress, uint8_t TCA9548APort, const char* bmp280_name) {
      if (bmp280List.length() >= MAX_SENSORS) return;
      if (bme280Adress != 0x76 && bme280Adress != 0x77) return;
      if (TCA9548AAdress > 0 && (TCA9548AAdress < 0x70 || TCA9548AAdress > 0x77)) return;
      if (TCA9548APort < 0 || TCA9548APort > 7) return;
      //check if sensor exists
      bool exist = false;
      if (bmp280List.length() > 0)
        for (byte i = 0; i < bmp280List.length(); i++)
          if (bmp280List[i].getBme280DeviceAdress() == bme280Adress &&
              bmp280List[i].getTCA9548ADeviceAdress() == TCA9548AAdress &&
              bmp280List[i].getTCA9548APortNo() == TCA9548APort)
            exist = true;
      if (!exist)
        bmp280List.push_back(BME280Simple(bme280Adress, TCA9548AAdress, TCA9548APort, bmp280_name));
    }
  private:
    bool if_init;
    uint8_t scanInterval; // in seconds
    unsigned long lastScan;
    const byte MAX_SENSORS = 32;
    QList<BME280Simple> bmp280List;
    void initAllSensors() {
      Wire.begin();
      if (bmp280List.length() > 0 && !if_init)
        for (byte i = 0; i < bmp280List.length(); i++)
          bmp280List[i].initSensor();
      if_init = true;
    }
};
BME280Manager myBME280Manager = BME280Manager(30); // set scan interval in seconds
/*  End of M6_MS_BME280SensorManager */

void before()
{
  /* M6_MS_BME280SensorManager */
  myBME280Manager.addSensor(0x76, 0, 0, "Kuchnia");  // M6_MS_BME280SensorManager
}

void setup() { }

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("BME280 Sensor Manager", "1.0");

  myBME280Manager.presentAllToControler(); //M6_MS_BME280SensorManager
}

void loop()
{
  myBME280Manager.sensorsCheck(); //M6_MS_BME280SensorManager
}

void receive(const MyMessage &message) { }

/*
   dataneo @2018 - M9_MS_HTU21DSensorManager
   MySensors HTU21D Sensor Manager 1.0
   see https://sites.google.com/site/dataneosoftware/arduino/mysensors-htu21d-sensor-manager
*/

#include <MySensors.h>
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
      addSensor(0, 0, "\0");
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

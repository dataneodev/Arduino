/*
   dataneo @2019 - M7_MS_BH1750SensorManager
   MySensors BH1750 Sensor Manager 1.1
   see https://sites.google.com/site/dataneosoftware/arduino/mysensors-bh1750-sensor-manager
*/

#include <MySensors.h>
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
      addSensor(0x23, 0, 0, "\0");
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

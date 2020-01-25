/*
   dataneo @2018 - M6_MS_BME280SensorManager
   MySensors BME280 Sensor Manager 1.0
   see https://sites.google.com/site/dataneosoftware/arduino/mysensors-bme280-sensor-manager
*/

#include <MySensors.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <QList.h>

class BME280Simple
{
public:
  BME280Simple()
  {
    bme280DeviceAdress = 0x76;
    TCA9548ADeviceAdress = 0;
    TCA9548APortNo = 0;
    initCorrect = false;
  };
  BME280Simple(uint8_t _childID,
               uint8_t bme280Adress,
               uint8_t TCA9548AAdress,
               uint8_t TCA9548APort,
               const char *bmp280_name)
  {
    childID = _childID;
    bme280DeviceAdress = bme280Adress;
    TCA9548ADeviceAdress = TCA9548AAdress;
    TCA9548APortNo = TCA9548APort;
    _bmp280_name = bmp280_name;
    initCorrect = false;
  };
  void readSensor(bool forceSendToController = false)
  {
    if (!initCorrect)
      return;
    float tempVar;
    if (TCA9548ADeviceAdress > 0)
      tcaSelect(TCA9548ADeviceAdress, TCA9548APortNo);

    bme280Obj.takeForcedMeasurement();
    //temp
    tempVar = initCorrect ? bme280Obj.readTemperature() : 0;
    if (tempVar != _temperature || forceSendToController)
    {
      _temperature = tempVar;
      sendStateToController(S_TEMP);
    }
    //humi
    tempVar = initCorrect ? bme280Obj.readHumidity() : 0;
    if (tempVar != _humidity || forceSendToController)
    {
      _humidity = tempVar;
      sendStateToController(S_HUM);
    }
    //press
    tempVar = initCorrect ? bme280Obj.readPressure() / 100.0F : 0;
    if (tempVar != _pressure || forceSendToController)
    {
      _pressure = tempVar;
      sendStateToController(S_BARO);
    }
  }
  void initSensor()
  {
    if (TCA9548ADeviceAdress > 0)
      tcaSelect(TCA9548ADeviceAdress, TCA9548APortNo);
    if (!bme280Obj.begin(bme280DeviceAdress))
    {
      initCorrect = false;
      return;
    }
    initCorrect = true;
    bme280Obj.setSampling(Adafruit_BME280::MODE_FORCED,
                          Adafruit_BME280::SAMPLING_X1, // temperature
                          Adafruit_BME280::SAMPLING_X1, // pressure
                          Adafruit_BME280::SAMPLING_X1, // humidity
                          Adafruit_BME280::FILTER_OFF);
    readSensor(true);
  }
  void presentToControler()
  {
    present(getChildID(S_TEMP), S_TEMP, _bmp280_name);
    present(getChildID(S_HUM), S_HUM, _bmp280_name);
    present(getChildID(S_BARO), S_BARO, _bmp280_name);
  }
  uint8_t getTCA9548ADeviceAdress()
  {
    return TCA9548ADeviceAdress;
  }
  uint8_t getTCA9548APortNo()
  {
    return TCA9548APortNo;
  }
  uint8_t getBme280DeviceAdress()
  {
    return bme280DeviceAdress;
  }

private:
  Adafruit_BME280 bme280Obj;
  uint8_t TCA9548ADeviceAdress; // from 0x70 to 0x77 - use 0 for disable TCA9548A
  uint8_t TCA9548APortNo;       // from 0 to 7
  uint8_t bme280DeviceAdress;   // 0x76 or 0x77
  float _temperature;
  float _humidity;
  float _pressure;
  const char *_bmp280_name;
  uint8_t childID;
  bool initCorrect;
  static MyMessage msgHumidity;
  static MyMessage msgTemperature;
  static MyMessage msgPressure;
  static MyMessage msgForecast;

  void sendStateToController(mysensors_sensor_t sensor)
  {
    msgHumidity.setSensor(getChildID(S_HUM));
    msgTemperature.setSensor(getChildID(S_TEMP));
    msgPressure.setSensor(getChildID(S_BARO));
    msgForecast.setSensor(getChildID(S_BARO));
    if (sensor == S_HUM)
      send(msgHumidity.set(_humidity, 1));
    if (sensor == S_TEMP)
      send(msgTemperature.set(_temperature, 1));
    if (sensor == S_BARO)
    {
      send(msgPressure.set(_pressure, 1));
      send(msgForecast.set("unknown"));
    }
  };

  uint8_t getChildID(mysensors_sensor_t sensor)
  {
    uint8_t id = childID;
    if (sensor == S_HUM)
      id = id + 1;
    if (sensor == S_BARO)
      id = id + 2;
    return id;
  };
  static void tcaSelect(uint8_t TCA9548A, uint8_t i)
  {
    if (i > 7)
      return;
    Wire.beginTransmission(TCA9548A);
    Wire.write(1 << i);
    Wire.endTransmission();
  }
};

MyMessage BME280Simple::msgHumidity = MyMessage(1, V_HUM);
MyMessage BME280Simple::msgTemperature = MyMessage(1, V_TEMP);
MyMessage BME280Simple::msgPressure = MyMessage(1, V_PRESSURE);
MyMessage BME280Simple::msgForecast = MyMessage(1, V_FORECAST);

class BME280Manager
{
public:
  BME280Manager(uint8_t scanIntervalInSeconds)
  {
    if_init = false;
    scanInterval = scanIntervalInSeconds;
    lastScan = 0;
  }
  void presentAllToControler()
  {
    if (bmp280List.length() > 0)
      for (byte i = 0; i < bmp280List.length(); i++)
        bmp280List[i].presentToControler();
    initAllSensors();
  }
  void sensorsCheck()
  {
    unsigned long timeNow = millis();
    if (lastScan > timeNow)
    { // overload
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
  void addSensor()
  {
    addSensor(0x76, 0, 0, "\0");
  }
  void addEmpty()
  {
    lastId = lastId + 3;
  }
  void addSensor(uint8_t bme280Adress, uint8_t TCA9548AAdress, uint8_t TCA9548APort, const char *bmp280_name)
  {
    if (bme280Adress != 0x76 && bme280Adress != 0x77)
      return;
    if (TCA9548AAdress > 0 && (TCA9548AAdress < 0x70 || TCA9548AAdress > 0x77))
      return;
    if (TCA9548APort < 0 || TCA9548APort > 7)
      return;
    //check if sensor exists
    bool exist = false;
    if (bmp280List.length() > 0)
      for (byte i = 0; i < bmp280List.length(); i++)
        if (bmp280List[i].getBme280DeviceAdress() == bme280Adress &&
            bmp280List[i].getTCA9548ADeviceAdress() == TCA9548AAdress &&
            bmp280List[i].getTCA9548APortNo() == TCA9548APort)
          exist = true;
    if (!exist)
    {
      bmp280List.push_back(BME280Simple(lastId, bme280Adress, TCA9548AAdress, TCA9548APort, bmp280_name));
      lastId = lastId + 3;
    }
  }

private:
  bool if_init;
  uint8_t scanInterval; // in seconds
  unsigned long lastScan;
  const uint8_t START_ID = 112;
  uint8_t lastId = START_ID;
  QList<BME280Simple> bmp280List;
  void initAllSensors()
  {
    Wire.begin();
    if (bmp280List.length() > 0 && !if_init)
      for (byte i = 0; i < bmp280List.length(); i++)
        bmp280List[i].initSensor();
    if_init = true;
  }
};

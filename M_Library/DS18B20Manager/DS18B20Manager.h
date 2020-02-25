/*
   dataneo @2019 - DS18B20Manager
   MySensors DS18B20 Sensor Manager 1.2
   see https://sites.google.com/site/dataneosoftware/arduino/mysensors-ds18b20-sensor-manager
*/

/* #region  DS18B20Manager */
#include <MySensors.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <QList.h>

class DS18B20Manager
{
public:
  DS18B20Manager(uint8_t pin,
                 unsigned short scanIntervalInSeconds,
                 uint16_t _conversionWait,
                 bool _sendSensorIdMessage = false)
  {
    if_init = false;
    scanInterval = min(scanIntervalInSeconds, 300);
    conversionWait = max(_conversionWait, 750);
    onewirePin = pin;
    oneWire = new OneWire(onewirePin);
    dallas = new DallasTemperature(oneWire);
    lastScanInit = 0;
    lastTempRequest = 0;
    requestTemp = false;
    sensorIdMessage = _sendSensorIdMessage;
  }

  void presentAllToControler()
  {
    if (DS18B20List.length() > 0)
      for (byte i = 0; i < DS18B20List.length(); i++)
        if (DS18B20List[i].presentToControler)
        {
          presentToControler(i);
        }

    initAllSensors();
  }

  void addSensor(uint8_t a1, uint8_t a2, uint8_t a3, uint8_t a4,
                 uint8_t a5, uint8_t a6, uint8_t a7, uint8_t a8,
                 const char *DS18B20Name)
  {
    addSensorToManager(pAddr(a1, a2, a3, a4, a5, a6, a7, a8),
                       DS18B20Name,
                       true,
                       nullptr,
                       nullptr);
  }

  void addSensor(uint8_t a1, uint8_t a2, uint8_t a3, uint8_t a4,
                 uint8_t a5, uint8_t a6, uint8_t a7, uint8_t a8,
                 const char *DS18B20Name,
                 bool presentSensorToControler,
                 void (*temperatureReadPtr)(uint8_t DS18B20Adress[8], float),
                 void (*temperatureReadErrorPtr)(uint8_t DS18B20Adress[8]))

  {
    addSensorToManager(pAddr(a1, a2, a3, a4, a5, a6, a7, a8),
                       DS18B20Name,
                       presentSensorToControler,
                       temperatureReadPtr,
                       temperatureReadErrorPtr);
  }

  void addSensor(uint8_t *DS18B20Adress,
                 const char *DS18B20Name)
  {
    addSensorToManager(DS18B20Adress,
                       DS18B20Name,
                       true,
                       nullptr,
                       nullptr);
  }

  void sensorsCheckLoop()
  {
    sensorsCheck(false);
  }

  bool getLastTemperatureRead(uint8_t *DS18B20Adress, float *temperature)
  {
    if (!if_init)
      return false;
    if (DS18B20List.length() == 0)
      return false;

    for (byte i = 0; i < DS18B20List.length(); i++)
      if (compareAdress(DS18B20List[i].DS18B20Adress, DS18B20Adress))
      {
        if (!DS18B20List[i].lastRead || !DS18B20List[i].init)
          return false;
        temperature = &DS18B20List[i].temperature;
        return true;
      }
    return false;
  }

private:
  const bool REPORT_ONLY_ON_CHANGE = false; //event fire only if temperature is change
  const float CHANGE_ON_DIFFERENCE = 0.1f;
  const uint8_t START_ID = 134; //start controler id

  bool if_init;
  uint8_t scanInterval;    // in seconds
  uint16_t conversionWait; //in miliseconds
  uint8_t onewirePin;
  unsigned long lastScanInit;
  unsigned long lastTempRequest;
  uint8_t lastIdTempRequest;
  bool requestTemp;
  uint8_t lastID = START_ID;
  bool sensorIdMessage;
  OneWire *oneWire;
  DallasTemperature *dallas;
  MyMessage msgTemperature = MyMessage(1, V_TEMP);
  MyMessage msgId = MyMessage(1, V_ID);

  struct dS18B20Single
  {
    uint8_t DS18B20Adress[8];
    float temperature;
    const char *DS18B20name;
    uint8_t ControlerID;
    bool init;
    bool lastRead;
    bool requestTemp;
    bool presentToControler;
    void (*temperatureReadPtr)(uint8_t DS18B20Adress[8], float);
    void (*temperatureReadErrorPtr)(uint8_t DS18B20Adress[8]);
  };

  typedef struct dS18B20Single DS18B20Single;
  QList<DS18B20Single> DS18B20List;

  void presentToControler(uint8_t id)
  {
    present(DS18B20List[id].ControlerID, S_TEMP, DS18B20List[id].DS18B20name);
  }

  void initAllSensors()
  {
    if (DS18B20List.length() == 0)
    {
      if_init = true;
      return;
    }
    dallas->begin();
    dallas->setResolution(12);
    dallas->setWaitForConversion(false);
    for (byte i = 0; i < DS18B20List.length(); i++)
      if (dallas->isConnected(DS18B20List[i].DS18B20Adress))
        DS18B20List[i].init = true;
      else
      {
        DS18B20List[i].init = false;
        if (DS18B20List[lastIdTempRequest].temperatureReadErrorPtr != nullptr)
        {
          DS18B20List[lastIdTempRequest].temperatureReadErrorPtr(DS18B20List[i].DS18B20Adress);
        }

        char hex[17];
        getAdress(DS18B20List[i].DS18B20Adress, hex);
        logMsg("The sensor with the given address was not found on the bus: ", hex);
      }
    if_init = true;
    sensorsCheck(true);
  }

  void sendStateToController(uint8_t id)
  {
    msgTemperature.setSensor(DS18B20List[id].ControlerID);
    send(msgTemperature.set(DS18B20List[id].temperature, 1));

    if (sensorIdMessage)
    {
      msgId.setSensor(DS18B20List[id].ControlerID);
      char hex[17];
      getAdress(DS18B20List[id].DS18B20Adress, hex);
      send(msgId.set(hex));
    }
  }

  void addSensorToManager(uint8_t *DS18B20Adress,
                          const char *DS18B20_name,
                          bool presentSensorToControler,
                          void (*_temperatureReadPtr)(uint8_t DS18B20Adress[8], float),
                          void (*_temperatureReadErrorPtr)(uint8_t DS18B20Adress[8]))
  {
    bool exist = false;
    if (DS18B20List.length() > 0)
      for (byte i = 0; i < DS18B20List.length(); i++)
        if (compareAdress(DS18B20List[i].DS18B20Adress, DS18B20Adress))
        {
          exist = true;

          if (_temperatureReadErrorPtr != nullptr)
          {
            _temperatureReadErrorPtr(DS18B20List[i].DS18B20Adress);
          }

          char hex[17];
          getAdress(DS18B20Adress, hex);
          logMsg("The sensor with the given address already exists: ", hex);
        }

    if (!exist)
    {
      DS18B20Single DS18B20;
      for (uint8_t i = 0; i < 8; i++)
      {
        DS18B20.DS18B20Adress[i] = *(DS18B20Adress + i);
      }

      DS18B20.DS18B20name = DS18B20_name;
      lastID++;
      DS18B20.ControlerID = lastID;
      DS18B20.init = false;
      DS18B20.lastRead = false;
      DS18B20.requestTemp = false;
      DS18B20.temperatureReadPtr = _temperatureReadPtr;
      DS18B20.temperatureReadErrorPtr = _temperatureReadErrorPtr;
      DS18B20.presentToControler = presentSensorToControler;
      DS18B20List.push_back(DS18B20);
    };
  }

  void sensorsCheck(bool firstRead)
  {
    if (!if_init)
      return;

    unsigned long timeNow = millis();
    if (lastScanInit > timeNow || lastTempRequest > timeNow)
    { // time overload
      lastScanInit = timeNow;
      lastTempRequest = timeNow;
      return;
    }

    // read temp
    if (requestTemp && (timeNow > lastTempRequest + conversionWait))
    {
      if (DS18B20List[lastIdTempRequest].init &&
          DS18B20List[lastIdTempRequest].requestTemp)
      {
        DS18B20List[lastIdTempRequest].requestTemp = false;
        float tempVal = dallas->getTempC(DS18B20List[lastIdTempRequest].DS18B20Adress);
        // error read
        if (tempVal == DEVICE_DISCONNECTED_C ||
            tempVal == -127.00 ||
            isnan(tempVal))
        {
          DS18B20List[lastIdTempRequest].lastRead = false;
          if (DS18B20List[lastIdTempRequest].temperatureReadErrorPtr != nullptr)
          {
            DS18B20List[lastIdTempRequest].temperatureReadErrorPtr(DS18B20List[lastIdTempRequest].DS18B20Adress);
          }
          char hex[17];
          getAdress(DS18B20List[lastIdTempRequest].DS18B20Adress, hex);
          logMsg("Sensor reading error: ", hex);
        }
        else
        {
          //read ok
          DS18B20List[lastIdTempRequest].lastRead = true;
          if ((REPORT_ONLY_ON_CHANGE && abs(tempVal - DS18B20List[lastIdTempRequest].temperature) > CHANGE_ON_DIFFERENCE) ||
              !REPORT_ONLY_ON_CHANGE)
          {
            DS18B20List[lastIdTempRequest].temperature = tempVal;
            if (DS18B20List[lastIdTempRequest].presentToControler)
            {
              sendStateToController(lastIdTempRequest);
            }

            if (DS18B20List[lastIdTempRequest].temperatureReadPtr != nullptr)
            {
              DS18B20List[lastIdTempRequest].temperatureReadPtr(DS18B20List[lastIdTempRequest].DS18B20Adress, tempVal);
            }
          }
          else
          {
            DS18B20List[lastIdTempRequest].temperature = tempVal;
          }
        }
      }

      // next temp request
      if (lastIdTempRequest < DS18B20List.length() - 1)
      {
        uint8_t nextID = getNextID(lastIdTempRequest);
        if (nextID == 0)
          return;
        lastIdTempRequest = nextID;
        lastTempRequest = timeNow;
        DS18B20List[lastIdTempRequest].requestTemp = true;
        dallas->requestTemperaturesByAddress(DS18B20List[lastIdTempRequest].DS18B20Adress);
      }
      else
      {
        requestTemp = false;
      }
    }

    //request temp from begining;
    unsigned long timeToCheck = lastScanInit + ((unsigned long)scanInterval) * 1000;
    if (!requestTemp && ((timeNow > timeToCheck) || firstRead))
    {
      lastScanInit = timeNow;
      lastTempRequest = timeNow;
      lastIdTempRequest = getNextID(-1);
      requestTemp = true;
      DS18B20List[lastIdTempRequest].requestTemp = true;
      dallas->requestTemperaturesByAddress(DS18B20List[lastIdTempRequest].DS18B20Adress);
    }
  }

  bool static compareAdress(uint8_t ad1[8], uint8_t ad2[8])
  {
    bool result = true;
    for (uint8_t i = 0; i < 8; i++)
      if (ad1[i] != ad2[i])
        result = false;
    return result;
  }
  uint8_t getNextID(int8_t id)
  {
    if (id < DS18B20List.length() - 1)
      for (int i = id + 1; i < DS18B20List.length(); i++)
        if (DS18B20List[i].init)
          return i;
    return 0;
  }

  void static getAdress(uint8_t adress[8], char *stringadress)
  {
    uint8_t temp[8];
    for (uint8_t i = 0; i < 8; i++)
    {
      temp[i] = *(adress + i);
    }

    for (int i = 0; i != 8; i++)
      sprintf(&stringadress[2 * i], "%02X", temp[i]);
    stringadress[16] = '\0';
  }

  void logMsg(const char *msg)
  {
    Serial.println(msg);
  }

  void logMsg(const char *msg, const char *msg2)
  {
    Serial.print(msg);
    Serial.println(msg2);
  }

  uint8_t *pAddr(uint8_t a1, uint8_t a2, uint8_t a3, uint8_t a4, uint8_t a5, uint8_t a6, uint8_t a7, uint8_t a8)
  {
    uint8_t *c = new uint8_t[8];
    c[0] = a1;
    c[1] = a2;
    c[2] = a3;
    c[3] = a4;
    c[4] = a5;
    c[5] = a6;
    c[6] = a7;
    c[7] = a8;
    return c;
  }
};
/* #endregion */

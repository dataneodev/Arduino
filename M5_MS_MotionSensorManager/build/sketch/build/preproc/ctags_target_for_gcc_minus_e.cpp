# 1 "i:\\7.Projekty\\5.Arduino\\M8_MS_DS18B20SensorManager\\M8_MS_DS18B20SensorManager.ino"
//
//    The MySensors Arduino library handles the wireless radio link and protocol
//    between your home built sensors/actuators and HA controller of choice.
//    The sensors forms a self healing radio network with optional repeaters. Each
//    repeater and gateway builds a routing tables in EEPROM which keeps track of the
//    network topology allowing messages to be routed to nodes.

//    Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
//    Copyright (C) 2013-2015 Sensnology AB
//    Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors

//    Documentation: http://www.mysensors.org
//    Support Forum: http://forum.mysensors.org

//    This program is free software; you can redistribute it and/or
//    modify it under the terms of the GNU General Public License
//    version 2 as published by the Free Software Foundation.

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


// Define a lower baud rate for Arduinos running on 8 MHz (Arduino Pro Mini 3.3V & SenseBender)




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

# 65 "i:\\7.Projekty\\5.Arduino\\M8_MS_DS18B20SensorManager\\M8_MS_DS18B20SensorManager.ino" 2

/*

   dataneo @2019 - M8_MS_DS18B20SensorManager

   MySensors DS18B20 Sensor Manager 1.2

   see https://sites.google.com/site/dataneosoftware/arduino/mysensors-ds18b20-sensor-manager

*/
# 71 "i:\\7.Projekty\\5.Arduino\\M8_MS_DS18B20SensorManager\\M8_MS_DS18B20SensorManager.ino"
# 72 "i:\\7.Projekty\\5.Arduino\\M8_MS_DS18B20SensorManager\\M8_MS_DS18B20SensorManager.ino" 2
# 73 "i:\\7.Projekty\\5.Arduino\\M8_MS_DS18B20SensorManager\\M8_MS_DS18B20SensorManager.ino" 2
# 74 "i:\\7.Projekty\\5.Arduino\\M8_MS_DS18B20SensorManager\\M8_MS_DS18B20SensorManager.ino" 2

class DS18B20Manager
{
public:
  DS18B20Manager(uint8_t pin,
                 unsigned short scanIntervalInSeconds,
                 uint16_t _conversionWait,
                 bool _sendSensorIdMessage = false)
  {
    if_init = false;
    scanInterval = ((scanIntervalInSeconds)<(300)?(scanIntervalInSeconds):(300));
    conversionWait = ((_conversionWait)>(750)?(_conversionWait):(750));
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
        presentToControler(i);
    initAllSensors();
  }

  void sensorsCheckLoop()
  {
    sensorsCheck(false);
  }

  void addSensor(uint8_t a1, uint8_t a2, uint8_t a3, uint8_t a4,
                 uint8_t a5, uint8_t a6, uint8_t a7, uint8_t a8,
                 const char *DS18B20Name)
  {
    addSensorToManager(pAddr(a1, a2, a3, a4, a5, a6, a7, a8),
                       DS18B20Name,
                       nullptr,
                       nullptr);
  }

  void addSensor(uint8_t a1, uint8_t a2, uint8_t a3, uint8_t a4,
                 uint8_t a5, uint8_t a6, uint8_t a7, uint8_t a8,
                 const char *DS18B20Name,
                 void (*temperatureReadPtr)(float),
                 void (*temperatureReadErrorPtr)())

  {
    addSensorToManager(pAddr(a1, a2, a3, a4, a5, a6, a7, a8),
                       DS18B20Name,
                       temperatureReadPtr,
                       temperatureReadErrorPtr);
  }

  void addSensor(uint8_t *DS18B20Adress,
                 const char *DS18B20Name)
  {
    addSensorToManager(DS18B20Adress,
                       DS18B20Name,
                       nullptr,
                       nullptr);
  }

private:
  const bool REPORT_ONLY_ON_CHANGE = false; //event fire only if temperature is change
  bool if_init;
  uint8_t scanInterval; // in seconds
  uint16_t conversionWait; //in miliseconds
  uint8_t onewirePin;
  unsigned long lastScanInit;
  unsigned long lastTempRequest;
  uint8_t lastIdTempRequest;
  bool requestTemp;
  const uint8_t START_ID = 134;
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
    void (*temperatureReadPtr)(float);
    void (*temperatureReadErrorPtr)();
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
      if_init = false;
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
          DS18B20List[lastIdTempRequest].temperatureReadErrorPtr();
        }

        char hex[17];
        getAdress(DS18B20List[i].DS18B20Adress, hex);
        logMsg('The sensor with the given address was not found on the bus: ');
        logMsg(hex);
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
                          void (*_temperatureReadPtr)(float),
                          void (*_temperatureReadErrorPtr)())
  {
    bool exist = false;
    if (DS18B20List.length() > 0)
      for (byte i = 0; i < DS18B20List.length(); i++)
        if (compareAdress(DS18B20List[i].DS18B20Adress, DS18B20Adress))
        {
          exist = true;

          if (_temperatureReadErrorPtr != nullptr)
          {
            _temperatureReadErrorPtr();
          }

          char hex[17];
          getAdress(DS18B20Adress, hex);
          logMsg('The sensor with the given address already exists: ');
          logMsg(hex);
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

      DS18B20List.push_back(DS18B20);
    };
  }

  void sensorsCheck(bool firstRead)
  {
    if (!if_init)
    {
      return;
    }

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
      if (DS18B20List[lastIdTempRequest].init && DS18B20List[lastIdTempRequest].requestTemp)
      {
        float tempVal;
        DS18B20List[lastIdTempRequest].requestTemp = false;
        DS18B20List[lastIdTempRequest].lastRead = false;

        tempVal = dallas->getTempC(DS18B20List[lastIdTempRequest].DS18B20Adress);

        // error read
        if (tempVal == -127 || tempVal == -127.00 || tempVal == 85.00)
        {
          if (DS18B20List[lastIdTempRequest].temperatureReadErrorPtr != nullptr)
          {
            DS18B20List[lastIdTempRequest].temperatureReadErrorPtr();
          }
          logMsg('Błąd odczytu czujnika');
        }
        else
        {
          //read ok
          if ((REPORT_ONLY_ON_CHANGE && tempVal != DS18B20List[lastIdTempRequest].temperature) ||
              !REPORT_ONLY_ON_CHANGE)
          {
            sendStateToController(lastIdTempRequest);
            if (DS18B20List[lastIdTempRequest].temperatureReadPtr != nullptr)
            {
              DS18B20List[lastIdTempRequest].temperatureReadPtr(tempVal);
            }
          }

          DS18B20List[lastIdTempRequest].temperature = tempVal;
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
    if (!requestTemp && ((timeNow > lastScanInit + scanInterval * 1000) || firstRead))
    {
      lastScanInit = timeNow;
      lastTempRequest = timeNow;
      lastIdTempRequest = getNextID(-1);
      requestTemp = true;
      DS18B20List[lastIdTempRequest].requestTemp = true;
      dallas->requestTemperaturesByAddress(DS18B20List[lastIdTempRequest].DS18B20Adress);
    }
  }

  bool compareAdress(uint8_t ad1[8], uint8_t ad2[8])
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
  void getAdress(uint8_t adress[8], char *stringadress)
  {
    for (int i = 0; i != 8; i++)
      sprintf(&stringadress[2 * i], "%02X", adress[i]);
    stringadress[16] = '\0';
  }

  void logMsg(const char *msg)
  {
    //Serial.println(msg);
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

/* inicjalize DS18B20Manager

DS18B20Manager(numer_pinu, 

          interwał_odczytu_temp(w sek), 

          czas konwersji (w milisek) - domyślnie (750) przy błędach zwiekszamy w góre do 1000-1500,

          czy wysyłać id sensora do kontrolera (nie działa poprawnie wszędzie)); */
# 396 "i:\\7.Projekty\\5.Arduino\\M8_MS_DS18B20SensorManager\\M8_MS_DS18B20SensorManager.ino"
DS18B20Manager myDS18B20Manager = DS18B20Manager(4, 6, 1500, true);

/*  End of M8_MS_DS18B20SensorManager */

void before()
{
  /* M8_MS_DS18B20SensorManager */
  myDS18B20Manager.addSensor(0x28, 0xEE, 0xAF, 0x47, 0x1A, 0x16, 0x01, 0x26, "Kuchnia", nullptr, nullptr); // M8_MS_DS18B20SensorManager
}

void setup() {}

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("DS18B20 Sensor Manager", "1.2");

  myDS18B20Manager.presentAllToControler(); //M8_MS_DS18B20SensorManager
}

void loop()
{
  myDS18B20Manager.sensorsCheckLoop(); //M8_MS_DS18B20SensorManager
}

void receive(const MyMessage &message) {}

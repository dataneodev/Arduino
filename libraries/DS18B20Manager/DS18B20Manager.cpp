
/*
   dataneo @2019 - M8_MS_DS18B20SensorManager
   MySensors DS18B20 Sensor Manager 1.1
   see https://sites.google.com/site/dataneosoftware/arduino/mysensors-ds18b20-sensor-manager
*/
#define MY_GATEWAY_SERIAL
#include <Arduino.h>
#include "DS18B20Manager.h"




DS18B20Manager::DS18B20Manager(uint8_t pin, unsigned short scanIntervalInSeconds, uint16_t _conversionWait, bool _sensorIdMessage)
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
    sensorIdMessage = _sensorIdMessage;
}

void DS18B20Manager::presentAllToControler()
{
    if (DS18B20List.length() > 0)
        for (byte i = 0; i < DS18B20List.length(); i++)
            presentToControler(i);
    initAllSensors();
}

void DS18B20Manager::sensorsCheck(bool firstRead)
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
            tempVal = dallas->getTempC(DS18B20List[lastIdTempRequest].DS18B20Adress);
            DS18B20List[lastIdTempRequest].requestTemp = false;
            if (tempVal != DS18B20List[lastIdTempRequest].temperature && tempVal != -127.00 && tempVal != 85.00)
            {
                DS18B20List[lastIdTempRequest].temperature = tempVal;
                sendStateToController(lastIdTempRequest);
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

void DS18B20Manager::addSensor(uint8_t *DS18B20Adress, const char *DS18B20_name)
{
    bool exist = false;
    if (DS18B20List.length() > 0)
        for (byte i = 0; i < DS18B20List.length(); i++)
            if (compareAdress(DS18B20List[i].DS18B20Adress, DS18B20Adress))
            {
                exist = true;
                char hex[17];
                getAdress(DS18B20Adress, hex);
                logMsg("The sensor with the given address already exists: ");
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
        DS18B20.requestTemp = false;
        DS18B20List.push_back(DS18B20);
    };
}

void DS18B20Manager::presentToControler(uint8_t id)
{
    present(DS18B20List[id].ControlerID, S_TEMP, DS18B20List[id].DS18B20name);
}

void DS18B20Manager::initAllSensors()
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
            char hex[17];
            getAdress(DS18B20List[i].DS18B20Adress, hex);
            logMsg("The sensor with the given address was not found on the bus: ");
            logMsg(hex);
        }
    if_init = true;
    sensorsCheck(true);
}

void DS18B20Manager::sendStateToController(uint8_t id)
{
    msgTemperature.setSensor(DS18B20List[id].ControlerID);
    send(msgTemperature.set(DS18B20List[id].temperature, 1));
    if (sensorIdMessage)
    {
        msgId.setSensor(DS18B20List[id].ControlerID);
        ;
        char hex[17];
        getAdress(DS18B20List[id].DS18B20Adress, hex);
        send(msgId.set(hex));
    }
};

bool DS18B20Manager::compareAdress(uint8_t* ad1, uint8_t* ad2)
{
    bool result = true;
    for (uint8_t i = 0; i < 8; i++)
        if (ad1[i] != ad2[i])
            result = false;
    return result;
}
uint8_t DS18B20Manager::getNextID(int8_t id)
{
    if (id < DS18B20List.length() - 1)
        for (int i = id + 1; i < DS18B20List.length(); i++)
            if (DS18B20List[i].init)
                return i;
    return 0;
}
void DS18B20Manager::getAdress(uint8_t* adress, char* stringadress)
{
    for (int i = 0; i != 8; i++)
        sprintf(&stringadress[2 * i], "%02X", adress[i]);
    stringadress[16] = '\0';
}

void DS18B20Manager::logMsg(const char*)
{
    //Serial.println(msg);
}

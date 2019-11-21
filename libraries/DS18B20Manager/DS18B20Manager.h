


#ifndef DS18B20Manager_h
#define DS18B20Manager_h

#include "OneWire.h"
#include "DallasTemperature.h"
#include "QList.h"
#include "MySensors.h"

class DS18B20Manager
{
public:
    DS18B20Manager(uint8_t,
                   unsigned short,
                   uint16_t,
                   bool = false);
    void presentAllToControler(void);
    void sensorsCheck(bool = false);
    void addSensor(uint8_t*, const char*);


    bool if_init;
    uint8_t scanInterval;    // in seconds
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
        bool requestTemp;
    };
    typedef struct dS18B20Single DS18B20Single;
    QList<DS18B20Single> DS18B20List;

    void presentToControler(uint8_t);
    void initAllSensors();
    void sendStateToController(uint8_t);
    bool compareAdress(uint8_t*, uint8_t*);
    uint8_t getNextID(int8_t );
    void getAdress(uint8_t*, char*);
    void logMsg(const char*);
};
#endif

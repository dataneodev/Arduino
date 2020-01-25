/*
   dataneo @2019 - PZEM016Device
   MySensors PZEM016 Manager 1.0
   see https://sites.google.com/site/dataneosoftware/arduino/mysensors-pzem016-manager
*/

class PZEM016Device
{
public:
    PZEM016Device() {}
    PZEM016Device(uint8_t pzemSlaveAddr, 
                 uint8_t childID, 
                 bool powerToController, 
                 bool multimeterToController, 
                 const char *PZEM016Name)
    {
        PZEM016Device(pzemSlaveAddr, childID, powerToController, multimeterToController, PZEM016Name, nullptr, nullptr);
    }
    PZEM016Device(uint8_t pzemSlaveAddr, 
                 uint8_t childID, 
                 bool powerToController, 
                 bool multimeterToController, 
                 const char *PZEM016Name,
                 void (*deviceReadPtr_)(uint8_t, bool, float, float, float, float, float),
                 void (*deviceReadErrorPtr_)(uint8_t, bool))
    {
        _pzemSlaveAddr = pzemSlaveAddr;
        _childID = childID;
        _powerToController = powerToController;
        _multimeterToController = multimeterToController;
        _PZEM016name = PZEM016Name;
        deviceReadPtr = deviceReadPtr_;
        deviceReadErrorPtr = deviceReadErrorPtr_;
    }
    uint8_t getPzemSlaveAddr()
    {
        return _pzemSlaveAddr;
    }
    uint8_t getChildID()
    {
        return _childID;
    }
    bool getIsPower()
    {
        return _powerToController;
    }
    bool getIsMultimeter()
    {
        return _multimeterToController;
    }
    const char *getName()
    {
        return _PZEM016name;
    }

    //measure
    bool _lastReadStatus = false;
    float _lastCurrent = 0.001;    // in A
    float _lastVoltage = 0.1;      // in V
    float _lastPower = 0.1;        // in Watt
    float _lastEnergy = 0.001;     // in kWh
    float _lastPowerFactor = 0.01; // eg 0.5

    float _lastCurrentSent = 0.001;    // in A
    float _lastVoltageSent = 0.1;      // in V
    float _lastPowerSent = 0.1;        // in Watt
    float _lastEnergySent = 0.001;     // in kWh
    float _lastPowerFactorSent = 0.01; // eg 0.5

    void (*deviceReadPtr)(uint8_t, bool, float, float, float, float, float);
    void (*deviceReadErrorPtr)(uint8_t, bool);

private:
    uint8_t _pzemSlaveAddr;
    uint8_t _childID;
    bool _powerToController;
    bool _multimeterToController;
    const char *_PZEM016name;
};
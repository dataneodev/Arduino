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
   dataneo @2019 - M10_MS_PRZEM016Manager
   MySensors PZEM016 Manager 1.0
   see https://sites.google.com/site/dataneosoftware/arduino/mysensors-pzem016-manager
*/

#include "ModbusMasterDMod.h"
#include <QList.h>

class PZEM016Device {
  public:
    PZEM016Device() {}
    PZEM016Device(uint8_t pzemSlaveAddr, uint8_t childID, bool powerToController = true, bool multimeterToController = false, const char* PZEM016Name = "") {
      _pzemSlaveAddr = pzemSlaveAddr;
      _childID = childID;
      _powerToController = powerToController;
      _multimeterToController = multimeterToController;
      _PZEM016name = PZEM016Name;
    }
    uint8_t getPzemSlaveAddr() {
      return _pzemSlaveAddr;
    }
    uint8_t getChildID() {
      return _childID;
    }
    bool getIsPower() {
      return _powerToController;
    }
    bool getIsMultimeter() {
      return _multimeterToController;
    }
    const char* getName() {
      return _PZEM016name;
    }

    //measure
    bool _lastReadStatus = false;
    float _lastCurrent = 0.001; // in A
    float _lastVoltage = 0.1; // in V
    float _lastPower = 0.1; // in Watt
    float _lastEnergy = 0.001; // in kWh
    float _lastPowerFactor = 0.01; // eg 0.5

    float _lastCurrentSent = 0.001; // in A
    float _lastVoltageSent = 0.1; // in V
    float _lastPowerSent = 0.1; // in Watt
    float _lastEnergySent = 0.001; // in kWh
    float _lastPowerFactorSent = 0.01; // eg 0.5
  private:
    uint8_t _pzemSlaveAddr;
    uint8_t _childID;
    bool _powerToController;
    bool _multimeterToController;
    const char* _PZEM016name;
};

class PZEM016Manager {
  public:
    PZEM016Manager(HardwareSerial &serial, uint8_t txRxChangePin = 0, uint8_t pzem016Check = 2, uint8_t controllerReport = 30) {
      _serial = &serial;
      _txRxChangePin = txRxChangePin;
      _pzem016Check = pzem016Check;
      _controllerReport = controllerReport;
      _serial -> begin(9600, SERIAL_8N1);
      PZEMNode = ModbusMasterDMod();
      PZEMNode.begin(1, serial);
      if (txRxChangePin > 0) {
        pinMode(txRxChangePin, OUTPUT);
        PZEMNode.preTransmission(preTransmission);
        PZEMNode.postTransmission(postTransmission);
      }
      last016Check = 0;
      lastControllerReport = 0;
    }

    void addPZEM016Device(uint8_t pzemSlaveAddr, bool presentToController = true, bool multimeterToController = false, const char* PZEM016Name = "") {
      if (pzemSlaveAddr == 0) {
        logMsg("PZEM adress no can not be 0");
        return;
      }
      bool addSuccesfull = true;
      if (PZEM016DeviceList.length() > 0)
        for (byte i = 0; i < PZEM016DeviceList.length(); i++)
          if (PZEM016DeviceList[i].getPzemSlaveAddr() == pzemSlaveAddr)
          {
            addSuccesfull = false;
            break;
          }
      if (addSuccesfull) {
        PZEM016Device newDevice(pzemSlaveAddr, _lastID, presentToController, multimeterToController, PZEM016Name);
        PZEM016DeviceList.push_back(newDevice);
        _lastID = _lastID + 2;
      }
    }

    void presentAllToControler() {
      if (PZEM016DeviceList.length() > 0) {
        for (byte i = 0; i < PZEM016DeviceList.length(); i++)
        {
          uint8_t groupNo = getGroupNo(PZEM016DeviceList[i].getPzemSlaveAddr());
          if (groupNo == 0 || (groupNo > 0 && isFirstOnGroup(PZEM016DeviceList[i].getPzemSlaveAddr(), groupNo))) {
            if (PZEM016DeviceList[i].getIsPower())
              present(PZEM016DeviceList[i].getChildID(), S_POWER, PZEM016DeviceList[i].getName());
            if (PZEM016DeviceList[i].getIsMultimeter())
              present(PZEM016DeviceList[i].getChildID() + 1, S_MULTIMETER, PZEM016DeviceList[i].getName());
          }
        }
        checkPZEM(true);
      }
    }

    void checkPZEM(bool forceCheck = false) {
      unsigned long timeNow = millis();
      if (last016Check > timeNow) { // time overload
        last016Check = timeNow;
        lastControllerReport = timeNow;
        return;
      }

      if (forceCheck || timeNow >= last016Check + (_pzem016Check * 1000)) {
        if (PZEM016DeviceList.length() > 0)
          for (byte i = 0; i < PZEM016DeviceList.length(); i++)
            readPZEM016Device(PZEM016DeviceList[i]);
        last016Check = timeNow;
      }

      if (forceCheck || timeNow >= lastControllerReport + (_controllerReport * 1000)) {
        if (PZEM016DeviceList.length() > 0)
          for (byte i = 0; i < PZEM016DeviceList.length(); i++) {
            uint8_t groupNo = getGroupNo(PZEM016DeviceList[i].getPzemSlaveAddr());
            if (groupNo == 0) {
              sendPZEM016Device(PZEM016DeviceList[i]);
            }
            else {
              if (isFirstOnGroup(PZEM016DeviceList[i].getPzemSlaveAddr(), groupNo))
                sendGroup(getGroupNo(PZEM016DeviceList[i].getPzemSlaveAddr()));
            }
          }
        lastControllerReport = timeNow;
      }
    }

    static void preTransmission()
    {
      digitalWrite(PZEM016Manager::_txRxChangePin, 1);
    }

    static void postTransmission()
    {
      digitalWrite(PZEM016Manager::_txRxChangePin, 0);
    }

    void addEmpty() {
      _lastID = _lastID + 2;
    }

    float getPower(uint8_t pzemAddr) {
      if (PZEM016DeviceList.length() > 0)
        for (byte i = 0; i < PZEM016DeviceList.length(); i++)
          if (PZEM016DeviceList[i].getPzemSlaveAddr() == pzemAddr)
            return PZEM016DeviceList[i]._lastPower;
      return 0;
    }
    float getCurrent(uint8_t pzemAddr) {
      if (PZEM016DeviceList.length() > 0)
        for (byte i = 0; i < PZEM016DeviceList.length(); i++)
          if (PZEM016DeviceList[i].getPzemSlaveAddr() == pzemAddr)
            return PZEM016DeviceList[i]._lastCurrent;
      return 0;
    }
    float getEnergy(uint8_t pzemAddr) {
      if (PZEM016DeviceList.length() > 0)
        for (byte i = 0; i < PZEM016DeviceList.length(); i++)
          if (PZEM016DeviceList[i].getPzemSlaveAddr() == pzemAddr)
            return PZEM016DeviceList[i]._lastEnergy;
      return 0;
    }
    void addGroup(uint8_t groupNo, uint8_t pzemAddr) {
      if (groupNo == 0) {
        logMsg("Group no can not be 0");
        return;
      }
      if (pzemAddr == 0) {
        logMsg("PZEM adress no can not be 0");
        return;
      }
      if (PRZEM016GroupList.length() > 0)
        for (byte i = 0; i < PRZEM016GroupList.length(); i++)
          if (PRZEM016GroupList[i].groupNo == groupNo && PRZEM016GroupList[i].przemSlaveId == pzemAddr) {
            logMsg("PZEM adress allready exists in this group");
            return;
          }
      PRZEM016Group newGroup;
      newGroup.groupNo = groupNo;
      newGroup.przemSlaveId = pzemAddr;
      PRZEM016GroupList.push_back(newGroup);
    }
  private:
    // const
    const uint8_t START_ID = 164;
    // variable
    QList<PZEM016Device> PZEM016DeviceList;
    ModbusMasterDMod PZEMNode;
    MyMessage powerMessageWatt =  MyMessage(1, V_WATT);
    MyMessage powerMessagekWh =  MyMessage(1, V_KWH);
    MyMessage powerMessagekPf =  MyMessage(1, V_POWER_FACTOR);
    MyMessage powerMessageVoltage =  MyMessage(1, V_VOLTAGE);
    MyMessage powerMessageCurrent =  MyMessage(1, V_CURRENT);

    HardwareSerial* _serial;
    uint8_t _pzem016Check; // in seconds
    uint8_t _controllerReport; // in seconds
    unsigned long last016Check; // in miliseconds
    unsigned long lastControllerReport; // in miliseconds
    uint8_t _lastID  = START_ID;
    static uint8_t _txRxChangePin;

    struct przem016group {
      uint8_t groupNo;
      uint8_t przemSlaveId;
    };
    typedef struct przem016group PRZEM016Group;
    QList<PRZEM016Group> PRZEM016GroupList;

    void readPZEM016Device(PZEM016Device &device) {
      PZEMNode.SetSlaveID(device.getPzemSlaveAddr());
      uint8_t result = PZEMNode.readInputRegisters(0x0000, 9); //read the 9 registers of the PZEM-014 / 016
      if (result == PZEMNode.ku8MBSuccess) {
        device._lastReadStatus = true;
        uint16_t tempWord;
        float tempFloat;
        /*
          RegAddr Description                 Resolution
          0x0000  Voltage value               1LSB correspond to 0.1V
          0x0001  Current value low 16 bits   1LSB correspond to 0.001A
          0x0002  Current value high 16 bits
          0x0003  Power value low 16 bits     1LSB correspond to 0.1W
          0x0004  Power value high 16 bits
          0x0005  Energy value low 16 bits    1LSB correspond to 1Wh
          0x0006  Energy value high 16 bits
          0x0007  Frequency value  virtual            1LSB correspond to 0.1Hz
          0x0008  Power factor value          1LSB correspond to 0.01
          0x0009  Alarm status  0xFFFF is alarm，0x0000is not alarm
        */
        // voltage
        tempFloat = PZEMNode.getResponseBuffer(0x0000) / 10.0;
        device._lastVoltage = tempFloat;

        //power
        tempWord = 0x0000;
        tempWord |= PZEMNode.getResponseBuffer(0x0003);       //LowByte
        tempWord |= PZEMNode.getResponseBuffer(0x0004) << 8;  //highByte
        if (tempWord >= 1000)
          tempFloat = tempWord;
        else
          tempFloat = tempWord / 10.0;
        device._lastPower = tempFloat;

        //current
        tempWord = 0x0000;
        tempWord |= PZEMNode.getResponseBuffer(0x0001);       //LowByte
        tempWord |= PZEMNode.getResponseBuffer(0x0002) << 8;  //highByte
        tempFloat = tempWord / 1000.0;
        device._lastCurrent = tempFloat;

        //energy
        tempWord = 0x0000;
        tempWord |= PZEMNode.getResponseBuffer(0x0005);       //LowByte
        tempWord |= PZEMNode.getResponseBuffer(0x0006) << 8;  //highByte
        if (tempWord >= 10000)
          tempFloat = tempWord / 100.0;
        else
          tempFloat = tempWord / 1000.0;
        device._lastEnergy = tempFloat;

        //powerPractor
        tempWord = PZEMNode.getResponseBuffer(0x0008);
        tempFloat = tempWord / 100.0;
        device._lastPowerFactor = tempFloat;
      }
      else {
        device._lastReadStatus = false;
        logMsg("Error during read device");
      }
    }

    void sendPZEM016Device(PZEM016Device &device) {
      if (!device._lastReadStatus) {
        return;
      }
      if (device.getIsMultimeter() && device._lastVoltage != device._lastVoltageSent) {
        device._lastVoltageSent = device._lastVoltage;
        powerMessageVoltage.setSensor(device.getChildID() + 1);
        send(powerMessageVoltage.set(device._lastVoltage, 1));
      }

      if (device.getIsPower() && device._lastPowerSent != device._lastPower) {
        device._lastPowerSent = device._lastPower;
        powerMessageWatt.setSensor(device.getChildID());
        send(powerMessageWatt.set(device._lastPower, 1));
      }

      if (device.getIsMultimeter() && device._lastCurrentSent != device._lastCurrent) {
        device._lastCurrentSent = device._lastCurrent;
        powerMessageCurrent.setSensor(device.getChildID() + 1);
        send(powerMessageCurrent.set(device._lastCurrent, 3));
      }

      if (device.getIsPower() && device._lastEnergySent != device._lastEnergy) {
        device._lastEnergySent = device._lastEnergy;
        powerMessagekWh.setSensor(device.getChildID());
        send(powerMessagekWh.set(device._lastEnergy, 3));
      }

      if (device.getIsPower() && device._lastPowerFactorSent != device._lastPowerFactor) {
        device._lastPowerFactorSent = device._lastPowerFactor;
        powerMessagekPf.setSensor(device.getChildID());
        send(powerMessagekPf.set(device._lastPowerFactor, 2));
      }
    }
    void sendGroup(uint8_t groupNo) {
      uint8_t groupCount = 0;
      uint8_t firstInGroupIndex;
      bool isAllLastSuccess = true;

      float _tempCurrent = 0.000; // in A
      float _tempVoltage = 0.0; // in V
      float _tempPower = 0.0; // in Watt
      float _tempEnergy = 0.000; // in kWh
      float _tempPowerFactor = 0.00; // eg 0.5

      if (PRZEM016GroupList.length() > 0)
        for (byte i = 0; i < PRZEM016GroupList.length(); i++)
          if (PRZEM016GroupList[i].groupNo == groupNo)
            for (byte j = 0; j < PZEM016DeviceList.length(); j++)
              if (PZEM016DeviceList[j].getPzemSlaveAddr() == PRZEM016GroupList[i].przemSlaveId) {
                groupCount++;
                if (groupCount == 1) {
                  firstInGroupIndex = j;
                }
                if (!PZEM016DeviceList[j]._lastReadStatus)
                  isAllLastSuccess = false;
                _tempCurrent += PZEM016DeviceList[j]._lastCurrent;
                _tempVoltage += PZEM016DeviceList[j]._lastVoltage;
                _tempPower += PZEM016DeviceList[j]._lastPower;
                _tempEnergy += PZEM016DeviceList[j]._lastEnergy;
                _tempPowerFactor += PZEM016DeviceList[j]._lastPowerFactor;
              }

      if (!isAllLastSuccess) {
        logMsg("sendGroup -> not all devices read correct!");
        return;
      }
      _tempVoltage = _tempVoltage / groupCount;
      _tempPowerFactor = _tempPowerFactor / groupCount;

      PZEM016Device PZEM016Temp(1, PZEM016DeviceList[firstInGroupIndex].getChildID(),
                                PZEM016DeviceList[firstInGroupIndex].getIsPower(),
                                PZEM016DeviceList[firstInGroupIndex].getIsMultimeter(),
                                PZEM016DeviceList[firstInGroupIndex].getName());

      PZEM016Temp._lastReadStatus = true;
      PZEM016Temp._lastCurrent = _tempCurrent;
      PZEM016Temp._lastVoltage = _tempVoltage;
      PZEM016Temp._lastPower = _tempPower;
      PZEM016Temp._lastEnergy = _tempEnergy;
      PZEM016Temp._lastPowerFactor = _tempPowerFactor;

      PZEM016Temp._lastCurrentSent = PZEM016DeviceList[firstInGroupIndex]._lastCurrentSent;
      PZEM016Temp._lastVoltageSent = PZEM016DeviceList[firstInGroupIndex]._lastVoltageSent;
      PZEM016Temp._lastPowerSent = PZEM016DeviceList[firstInGroupIndex]._lastPowerSent;
      PZEM016Temp._lastEnergySent = PZEM016DeviceList[firstInGroupIndex]._lastEnergySent;
      PZEM016Temp._lastPowerFactorSent = PZEM016DeviceList[firstInGroupIndex]._lastPowerFactorSent;

      sendPZEM016Device(PZEM016Temp);

      PZEM016DeviceList[firstInGroupIndex]._lastCurrentSent = PZEM016Temp._lastCurrentSent;
      PZEM016DeviceList[firstInGroupIndex]._lastVoltageSent = PZEM016Temp._lastVoltageSent;
      PZEM016DeviceList[firstInGroupIndex]._lastPowerSent = PZEM016Temp._lastPowerSent;
      PZEM016DeviceList[firstInGroupIndex]._lastEnergySent = PZEM016Temp._lastEnergySent;
      PZEM016DeviceList[firstInGroupIndex]._lastPowerFactorSent = PZEM016Temp._lastPowerFactorSent;
    }

    uint8_t getGroupNo(uint8_t pzemAddr) {
      if (PRZEM016GroupList.length() > 0)
        for (byte i = 0; i < PRZEM016GroupList.length(); i++)
          if (PRZEM016GroupList[i].przemSlaveId == pzemAddr)
            return PRZEM016GroupList[i].groupNo;
      return 0;
    }
    bool isFirstOnGroup(uint8_t pzemAddr, uint8_t groupNo) {
      if (PRZEM016GroupList.length() > 0)
        for (byte i = 0; i < PRZEM016GroupList.length(); i++)
          if (PRZEM016GroupList[i].groupNo == groupNo)
            if (PRZEM016GroupList[i].przemSlaveId == pzemAddr)
              return true;
            else
              return false;
      return false;
    }

    void logMsg(char* msg) {
      Serial.println(msg);
    }
};
uint8_t PZEM016Manager::_txRxChangePin = 0;

/*
  Configuration:

  PZEM016Manager(HardwareSerial &serial, uint8_t txRxChangePin = 0, uint8_t pzem016Check = 2, uint8_t controllerReport = 30) ;
  serial - communication serial
  txRxChangePin - pinout for control rx/tx; use 0 for disable
  pzem016Check - time in seconds between read PZEM016
  controllerReport - time in seonds between send raport to controller
*/

PZEM016Manager PZEM016ManagerObj = PZEM016Manager(Serial1, 10, 2, 10);
/*  End of M10_MS_PRZEM016Manager */

void before()
{
  //void addPZEM016Device(uint8_t pzemSlaveAddr, bool presentToController = true, bool multimeterToController = false, const char* PZEM016Name = "")
  PZEM016ManagerObj.addPZEM016Device(1, true, true, "PZEM016 Grupa Testowa"); //M10_MS_PRZEM016Manager
  //PZEM016ManagerObj.addPZEM016Device(2, true, true, "PZEM016 Test"); //M10_MS_PRZEM016Manager

  //void addGroup(uint8_t groupNo, uint8_t pzemAddr)
  PZEM016ManagerObj.addGroup(1, 1);
}

void setup() {}

void presentation()
{
  sendSketchInfo("PZEM016 Sensor Manager", "1.1");
  PZEM016ManagerObj.presentAllToControler(); // M10_MS_PRZEM016Manager
}

void loop() {
  PZEM016ManagerObj.checkPZEM(); // M10_MS_PRZEM016Manager
}

void receive(const MyMessage &message) { }

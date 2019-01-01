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

 *******************************

   REVISION HISTORY
   Version 1.0 - Henrik Ekblad

   DESCRIPTION
   Example sketch showing how to control physical relays.
   This example will remember relay state after power failure.
   http://www.mysensors.org/build/relay
*/

// Enable debug prints to serial monitor
#define MY_DEBUG

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
   dataneo @2018 - M1_MS_RelayManager
   MySensors Relay Manager 1.1
   see https://sites.google.com/site/dataneosoftware/arduino/mysensors-relay-manager
*/

#include <Bounce2.h>
#include <QList.h>

enum STATE_METHOD {
  SAVE_TO_EEPROM,
  LOAD_FROM_CONTROLLERS,
  START_IN_HIGH,
  START_IN_LOW,
};

enum RELAY_STATE {
  RELAY_ON_HIGH,
  RELAY_ON_LOW,
};

enum CONTROLLER_TYPE{
  DOMOTICZ,
  HOMEASSISTANT,
  OTHER, // NOT TESTED
};

//Simple relay class
class RelaySimple {
  public:
    RelaySimple() {
      _relay_pin_no = 0;
      _save_state = START_IN_HIGH;
      _relay_on_state = RELAY_ON_HIGH;
    };
    RelaySimple(byte relay_pin_no, STATE_METHOD save_state, RELAY_STATE relay_on_state, const char* relay_name) {
      _relay_pin_no = relay_pin_no;
      _save_state = save_state;
      _relay_on_state = relay_on_state;
      _relay_name = relay_name;
    };
    byte relayPin() {
      return _relay_pin_no;
    }
    void initPin() {
      pinMode(_relay_pin_no, OUTPUT);
      if (_save_state == SAVE_TO_EEPROM) {
        _relay_state = loadState(_relay_pin_no);
        digitalWrite(_relay_pin_no, getGPIOState(_relay_state));
        sendStateToController();
      }
      if (_save_state == LOAD_FROM_CONTROLLERS) {
        // request load state from controler
        request(_relay_pin_no, V_STATUS);
      }
      if (_save_state == START_IN_HIGH) {
        _relay_state = true;
        digitalWrite(_relay_pin_no, getGPIOState(_relay_state));
        sendStateToController();
      }
      if (_save_state == START_IN_LOW) {
        _relay_state = false;
        sendStateToController();
      }
    }

    void presentToControler() {
      present(_relay_pin_no, S_BINARY, _relay_name);
    }

    void setStateFromControler(bool relay_state, CONTROLLER_TYPE controller) {
      if (_relay_state == relay_state)
        return;
      _relay_state = relay_state;
      digitalWrite(_relay_pin_no, getGPIOState(_relay_state));
      if (_save_state == SAVE_TO_EEPROM)
        saveState(_relay_pin_no, _relay_state);
      if(controller == HOMEASSISTANT)
        sendStateToController();
    }

    void setButtonState() {
      _relay_state = ! _relay_state;
      digitalWrite(_relay_pin_no, getGPIOState(_relay_state));
      if (_save_state == SAVE_TO_EEPROM)
        saveState(_relay_pin_no, _relay_state);
      sendStateToController();
    }
  private:
    bool _relay_state; // current relay state on or off
    byte _relay_pin_no; // gpio pin for relay
    const char* _relay_name;
    STATE_METHOD _save_state;
    RELAY_STATE _relay_on_state;

    byte getGPIOState(bool relay_state) {
      return (_relay_on_state == RELAY_ON_HIGH) ? relay_state : ! relay_state;
    }
    void sendStateToController() {
      MyMessage mMessage(_relay_pin_no, V_STATUS);
      send(mMessage.set(_relay_state ? "1" : "0"));
    }
};

class ButtonSimple {
  public:
    ButtonSimple(){
      _button_pin_no = 0;
    }
    ButtonSimple(byte button_pin_no) {
      _button_pin_no = button_pin_no;
    }
    byte getButtonPinNo() {
      return _button_pin_no;
    }
    bool checkButton() {
      if (_button_pin_no == 0)
        return false;
      _debouncer.update();
      if (_debouncer.fell())
        return true;
      return false;
    }
    void initPin() {
      if (_button_pin_no != 0) {
        pinMode(_button_pin_no, INPUT_PULLUP);
        _debouncer = Bounce();
        _debouncer.attach(_button_pin_no);
        _debouncer.interval(5);
      }
    }
  private:
    byte _button_pin_no;
    Bounce _debouncer;
};

class RelayButtonPair {
  public:
    RelayButtonPair() {
      _relay_pin_no = 0;
      _button_pin_no = 0;
    }
    RelayButtonPair(byte relay_pin_no, byte button_pin_no) {
      _relay_pin_no = relay_pin_no;
      _button_pin_no = button_pin_no;
    }
    byte relayPin() {
      return _relay_pin_no;
    }
    byte getButtonPinNo() {
      return _button_pin_no;
    }
  private:
    byte _relay_pin_no;
    byte _button_pin_no;
};

class RelayManager {
  public:
    RelayManager(CONTROLLER_TYPE controller) {
      if_init = false;
      _controller = controller;
    }
    //Simple Relay change status in RelayList
    void setStateOnRelayListFromControler(int relay_pin_no, bool relay_state) {
      if (relayList.length() > 0 && if_init)
        for (byte i = 0; i < relayList.length(); i++)
          if (relayList[i].relayPin() == relay_pin_no)
            relayList[i].setStateFromControler(relay_state, _controller);
    }
    void presentAllToControler() {
      if (relayList.length() > 0)
        for (byte i = 0; i < relayList.length(); i++)
          relayList[i].presentToControler();
      initAllPins();
    }
    void buttonCheckState() {
      if (buttonList.length() > 0 && if_init)
        for (byte i = 0; i < buttonList.length(); i++)
          if (buttonList[i].checkButton())
            if (relayButtonPairList.length() > 0)
              for (byte j = 0; j < relayButtonPairList.length(); j++)
                if (relayButtonPairList[j].getButtonPinNo() == buttonList[i].getButtonPinNo())
                  if (relayList.length() > 0)
                    for (byte k = 0; k < relayList.length(); k++)
                      if (relayList[k].relayPin() == relayButtonPairList[j].relayPin())
                        relayList[k].setButtonState();
    }

    void addRelay(byte relay_pin_no) {
      addRelay(relay_pin_no, 0, SAVE_TO_EEPROM, RELAY_ON_HIGH, '\0');
    }

    void addRelay(byte relay_pin_no, byte button_pin_no) {
      addRelay(relay_pin_no, button_pin_no, SAVE_TO_EEPROM, RELAY_ON_HIGH, '\0');
    }

    void addRelay(byte relay_pin_no, byte button_pin_no, STATE_METHOD save_state, RELAY_STATE relay_on_state) {
      addRelay(relay_pin_no, button_pin_no, save_state, relay_on_state, '\0');
    }

    void addRelay(byte relay_pin_no, byte button_pin_no, STATE_METHOD save_state, RELAY_STATE relay_on_state, const char* relay_name) {
      if(relayList.length()>= MAX_PIN || buttonList.length() >= MAX_PIN)
        return;
      if(_controller == HOMEASSISTANT && save_state == LOAD_FROM_CONTROLLERS)
        save_state = START_IN_LOW;
        
      //check if relay exists
      bool exist = false;
      if (relayList.length() > 0)
        for (byte i = 0; i < relayList.length(); i++)
          if (relayList[i].relayPin() == relay_pin_no)
            exist = true;
      if (!exist) 
        relayList.push_back(RelaySimple(relay_pin_no, save_state, relay_on_state, relay_name));

      //button check
      exist = false;
      if (buttonList.length() > 0)
        for (byte i = 0; i < buttonList.length(); i++)
          if (buttonList[i].getButtonPinNo() == button_pin_no)
            exist = true;
      if (!exist && button_pin_no != 0)
        buttonList.push_back(ButtonSimple(button_pin_no));
      
      //pair exists
      exist = false;
      if (relayButtonPairList.length() > 0)
        for (byte i = 0; i < relayButtonPairList.length(); i++)
          if (relayButtonPairList[i].relayPin() == relay_pin_no && 
              relayButtonPairList[i].getButtonPinNo() == button_pin_no)
            exist = true;
      if (!exist)
        relayButtonPairList.push_back(RelayButtonPair(relay_pin_no, button_pin_no));
    }
  private:
    bool if_init;
    const byte MAX_PIN = 70;
    CONTROLLER_TYPE _controller;
    QList<ButtonSimple> buttonList;
    QList<RelaySimple> relayList;
    QList<RelayButtonPair> relayButtonPairList;

    void initAllPins() {
      if (relayList.length() > 0 && !if_init)
        for (byte i = 0; i < relayList.length(); i++)
          relayList[i].initPin();
      if (buttonList.length() > 0 && !if_init)
        for (byte i = 0; i < buttonList.length(); i++)
          buttonList[i].initPin();
      if_init = true;
    }
};

RelayManager myRelayController = RelayManager(HOMEASSISTANT);
/*
   End of Simple relay sketch
*/

/*
   dataneo @2018 - M2_MS_Heartbeat
   MySensors Heartbeat Manager 1.0
   see https://sites.google.com/site/dataneosoftware/arduino/mysensors-heartbeat
*/
class HeartBeatManager {
  public:
    HeartBeatManager(){ HeartBeatManager(0);};
    HeartBeatManager(byte alarmBuzzerPin) {
      _lastSend = 0;
      _lastRecive = 0;
      _lastBuzzerAcive = 0;
      _alarmBuzzerPin = alarmBuzzerPin;
      if (_alarmBuzzerPin != 0)
        pinMode(_alarmBuzzerPin, OUTPUT);
    }
    void HeartBeat() {
      _currentTime = millis();
      if (_currentTime < _lastSend) {
        _lastRecive = 0;
        _lastBuzzerAcive = 0;
        SendHeart();
        return;
      }
      BuzzerCheck();
      if (_currentTime < _lastSend + _timeSendPeriod)  return;
      SendHeart();
    }
    void ControllerReciveMsg() {
      _currentTime = millis();
      _lastRecive = _currentTime;
    }
    
  private:
    byte _alarmBuzzerPin;
    unsigned long _currentTime;
    unsigned long _lastSend;
    unsigned long _lastRecive;
    unsigned long _lastBuzzerAcive;
    const unsigned short _timeSendPeriod = 30 * 1000; // miliseconds
    const unsigned short _timeReciveTimeout = 10 * 1000; // miliseconds
    const unsigned short _toneLength = 500;
    const unsigned short _toneBreak = 500;
    const unsigned short _toneHz = 4000;
    
    void SendHeart(){
      _lastSend = _currentTime;
      sendHeartbeat();
      if (_alarmBuzzerPin != 0)
        requestTime();  
    }
    void BuzzerCheck() {
      if (_alarmBuzzerPin != 0 &&
          _lastSend != 0 &&
          _lastRecive < _lastSend &&
          ((_currentTime - _lastSend > _timeReciveTimeout) || (_currentTime - _lastRecive > _timeSendPeriod +_timeReciveTimeout)) &&
          _lastBuzzerAcive + _toneLength + _toneBreak < _currentTime) {
            _lastBuzzerAcive = _currentTime;
            tone(_alarmBuzzerPin, _toneHz, _toneLength);
      }
    }
};

HeartBeatManager myHeartBeatManager = HeartBeatManager(13);

void receiveTime(uint32_t ts) {
  myHeartBeatManager.ControllerReciveMsg();
}
/*
   End of MySensors Heartbeat Manager
*/

/*
   dataneo @2018 - M4_MS_SwitchSensorManager
   MySensors Switch Sensor Manager 1.0
   Mechanical switch manager with debouncer
   see https://sites.google.com/site/dataneosoftware/arduino/switch-sensor-manager
*/
enum SWITCH_STATE {
  SWITCH_NORMAL_OPEN,
  SWITCH_NORMAL_CLOSE,
};

//Switch Sensor Manager
class SwitchSimple {
  public:
    SwitchSimple() {
      _switch_pin_no = 0;
      _switch_value = 0;
      _switch_state = SWITCH_NORMAL_OPEN;
    };
    SwitchSimple(byte switch_pin_no, SWITCH_STATE switchState, const char* switch_name) {
      _switch_pin_no = switch_pin_no;
      _switch_value = 0;
      _switch_state = switchState;
      _switch_name = switch_name;
    };
    byte switchPin() {
      return _switch_pin_no;
    }
    bool checkSwitch(bool forceSendToController = false) {
      _debouncer.update();
      bool readValue = _debouncer.read();
      if(readValue != _switch_value || forceSendToController){
        _switch_value = readValue;
        sendStateToController();
      }
      return readValue;
    }
    void initPin() {
      pinMode(_switch_pin_no, INPUT_PULLUP);
      //digitalWrite(_switch_pin_no,HIGH);
      mMessage = MyMessage(_switch_pin_no, V_TRIPPED);
      _debouncer = Bounce();
      _debouncer.attach(_switch_pin_no);
      _debouncer.interval(5);
      checkSwitch(true);
    }
    void presentToControler() {
      present(_switch_pin_no, S_DOOR, _switch_name);
    }
  private:
    MyMessage mMessage;
    Bounce _debouncer;  
    SWITCH_STATE _switch_state;
    bool _switch_value;
    byte _switch_pin_no; // gpio pin for switch
    const char* _switch_name;  

    void sendStateToController() {
      bool state = _switch_value;
      if(_switch_state == SWITCH_NORMAL_OPEN) 
        state = !state;
      send(mMessage.set(state ? "1" : "0"));
    }
};

class SwitchManager {
  public:
    SwitchManager() {
      if_init = false;
    }
    void presentAllToControler() {
      if (switchList.length() > 0)
        for (byte i = 0; i < switchList.length(); i++)
          switchList[i].presentToControler();
      initAllPins();
    }
    void switchCheckState() {
      if (switchList.length() > 0 && if_init)
        for (byte i = 0; i < switchList.length(); i++)
          switchList[i].checkSwitch(false);
    }
    void addSwitch(byte switch_pin_no) {
      addSwitch(switch_pin_no, SWITCH_NORMAL_CLOSE, '\0');
    }
    void addSwitch(byte switch_pin_no, SWITCH_STATE switch_state) {
      addSwitch(switch_pin_no, SWITCH_NORMAL_CLOSE, '\0');
    }    
    void addSwitch(byte switch_pin_no, SWITCH_STATE switch_state, const char* switch_name) {
      if(switchList.length()>= MAX_PIN) return;        
      //check if switch exists
      bool exist = false;
      if (switchList.length() > 0)
        for (byte i = 0; i < switchList.length(); i++)
          if (switchList[i].switchPin() == switch_pin_no)
            exist = true;
      if (!exist) 
        switchList.push_back(SwitchSimple(switch_pin_no, switch_state, switch_name));
    }
  private:
    bool if_init;
    const byte MAX_PIN = 70;
    QList<SwitchSimple> switchList;

    void initAllPins() {
      if (switchList.length() > 0 && !if_init)
        for (byte i = 0; i < switchList.length(); i++)
          switchList[i].initPin();
      if_init = true;
    }
};

SwitchManager mySwitchManager = SwitchManager();
/*
   End of M4_MS_SwitchSensorManager
*/

/*
   dataneo @2018 - M5_MS_MotionSensorManager
   MySensors Motion Sensor Manager 1.2
   see https://sites.google.com/site/dataneosoftware/arduino/mysensors-motion-sensor-manager
*/
#include <QList.h>

enum MOTION_STATE {
  SENSOR_ON_LOW,
  SENSOR_ON_HIGH,
};

//Motion Sensor Manager
class MotionSimple {
  public:
    MotionSimple() {
      _motion_pin_no = 0;
      _motion_value = 0;
      _motion_state = SENSOR_ON_HIGH;
      _sensor = S_MOTION;
      _armed = true;
    };
    MotionSimple(byte motion_pin_no, MOTION_STATE motion_state, const char* motion_name, mysensors_sensor_t sensor_type) {
      _motion_pin_no = motion_pin_no;
      _motion_value = 0;
      _motion_state = motion_state;
      _motion_name = motion_name;
      _sensor = sensor_type;
      _armed = true;
    };
    byte motionPin() {
      return _motion_pin_no;
    }
    bool checkmotion(bool forceSendToController = false) {
      if (_motion_pin_no == 0) return false;
      bool readValue = _armed ? (digitalRead(_motion_pin_no) == HIGH ? true : false) : false;
      if (readValue != _motion_value || forceSendToController) {
        _motion_value = readValue;
        sendStateToController();
      }
      return readValue;
    }
    void initPin() {
      if (_motion_pin_no == 0) return;
      pinMode(_motion_pin_no, INPUT);
      mMessage = MyMessage(_motion_pin_no, V_TRIPPED);
      mMessageArmed = MyMessage(_motion_pin_no, V_ARMED);
      checkmotion(true);
    }
    void presentToControler() {
      if (_motion_pin_no == 0) return;
      present(_motion_pin_no, _sensor, _motion_name);
    }
    void setStateFromControler(bool state, CONTROLLER_TYPE controller) {
      _armed = state;
      if (controller == HOMEASSISTANT)
        sendStateToController();
    }
  private:
    MyMessage mMessage;
    MyMessage mMessageArmed;
    MOTION_STATE _motion_state;
    mysensors_sensor_t _sensor;
    bool _motion_value;
    byte _motion_pin_no;
    bool _armed;
    const char* _motion_name;

    void sendStateToController() {
      bool state = _motion_value;
      if (_motion_state == SENSOR_ON_LOW)
        state = !state;
      send(mMessage.set(state ? "1" : "0"));
      send(mMessageArmed.set(_armed ? "1" : "0"));
    }
};

class MotionManager {
  public:
    MotionManager(CONTROLLER_TYPE controller) {
      if_init = false;
      _controller = controller;
    }
    void presentAllToControler() {
      if (motionList.length() > 0)
        for (byte i = 0; i < motionList.length(); i++)
          motionList[i].presentToControler();
      initAllPins();
    }
    void motionCheckState() {
      if (motionList.length() > 0 && if_init)
        for (byte i = 0; i < motionList.length(); i++)
          motionList[i].checkmotion(false);
    }
    void addMotion(byte motion_pin_no) {
      addMotion(motion_pin_no, SENSOR_ON_HIGH, '\0', S_MOTION);
    }
    void addMotion(byte motion_pin_no, MOTION_STATE motion_state) {
      addMotion(motion_pin_no, SENSOR_ON_HIGH, '\0', S_MOTION);
    }
    void addMotion(byte motion_pin_no, MOTION_STATE motion_state, const char* motion_name, mysensors_sensor_t _sensor = S_MOTION) {
      if (motionList.length() >= MAX_PIN) return;
      //check if motion exists
      bool exist = false;
      if (motionList.length() > 0)
        for (byte i = 0; i < motionList.length(); i++)
          if (motionList[i].motionPin() == motion_pin_no)
            exist = true;
      if (!exist)
        motionList.push_back(MotionSimple(motion_pin_no, motion_state, motion_name, _sensor));
    }
    void setStatetFromControler(int motion_pin_no, bool state) {
      if (motionList.length() > 0 && if_init)
        for (byte i = 0; i < motionList.length(); i++)
          if (motionList[i].motionPin() == motion_pin_no)
            motionList[i].setStateFromControler(state, _controller);
    }
  private:
    bool if_init;
    const byte MAX_PIN = 70;
    CONTROLLER_TYPE _controller;
    QList<MotionSimple> motionList;
    void initAllPins() {
      if (motionList.length() > 0 && !if_init)
        for (byte i = 0; i < motionList.length(); i++)
          motionList[i].initPin();
      if_init = true;
    }
};

MotionManager myMotionManager = MotionManager(HOMEASSISTANT);
/*  End of M5_MS_MotionSensorManager */

/*
   dataneo @2018 - M6_MS_BME280SensorManager
   MySensors BME280 Sensor Manager 1.0
   see https://sites.google.com/site/dataneosoftware/arduino/mysensors-bme280-sensor-manager
*/
#include <Wire.h>
#include <BlueDot_BME280.h>
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
      //temp
      tempVar = initCorrect ? bme280Obj.readTempC() : 0;
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
      tempVar = initCorrect ? bme280Obj.readPressure() : 0;
      if (tempVar != _pressure || forceSendToController) {
        _pressure = tempVar;
        sendStateToController(S_BARO);
      }
    }
    void initSensor() {
      msgHumidity = MyMessage(getChildID(S_HUM), V_HUM);
      msgTemperature = MyMessage(getChildID(S_TEMP), V_TEMP);
      msgPressure = MyMessage(getChildID(S_BARO), V_PRESSURE);
      //msgForecast = MyMessage(getChildID(S_BARO), V_FORECAST);
      bme280Obj.parameter.communication = 0;
      bme280Obj.parameter.I2CAddress = bme280DeviceAdress;
      bme280Obj.parameter.sensorMode = 0b11;
      bme280Obj.parameter.IIRfilter = 0b100;
      bme280Obj.parameter.humidOversampling = 0b101;
      bme280Obj.parameter.tempOversampling = 0b101;
      bme280Obj.parameter.pressOversampling = 0b101;
      if (TCA9548ADeviceAdress > 0)
        tcaSelect(TCA9548ADeviceAdress, TCA9548APortNo);
      if (bme280Obj.init() != 0x60) 
        initCorrect = false;
      else 
        initCorrect = true;
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
    MyMessage msgHumidity;
    MyMessage msgTemperature;
    MyMessage msgPressure;
    //MyMessage msgForecast;
    BlueDot_BME280 bme280Obj;
    uint8_t TCA9548ADeviceAdress; // from 0x70 to 0x77 - use 0 for disable TCA9548A
    uint8_t TCA9548APortNo; // from 0 to 7
    uint8_t bme280DeviceAdress; // 0x76 or 0x77
    float _temperature;
    float _humidity;
    float _pressure;
    const char* _bmp280_name;
    bool initCorrect;
    void sendStateToController(mysensors_sensor_t sensor) {
      if (sensor == S_HUM)
        send(msgHumidity.set(_humidity, 1));
      if (sensor == S_TEMP)
        send(msgTemperature.set(_temperature, 1));
      if (sensor == S_BARO){
        send(msgPressure.set(_pressure, 1));
       // send(msgForecast.set("sunny"));  
      }
        
    };

    uint8_t getChildID(mysensors_sensor_t sensor) {
      uint8_t id = 70 + (bme280DeviceAdress - 0x76)*3; // start from 70 to 134
      if (TCA9548ADeviceAdress > 0){
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
      if(lastScan > timeNow){ // overload
        lastScan = timeNow;
        return;
      }
      if(timeNow < lastScan + scanInterval*1000)
        return;
      lastScan = timeNow;
      if (bmp280List.length() > 0 && if_init)
      for (byte i = 0; i < bmp280List.length(); i++)
        bmp280List[i].readSensor(false);
    }
    void addSensor() {
      addSensor(0x76, 0, 0, '\0');
    }
    void addSensor(uint8_t bme280Adress, uint8_t TCA9548AAdress, uint8_t TCA9548APort, const char* bmp280_name) {
      if (bmp280List.length() >= MAX_SENSORS) return;
      if(bme280Adress != 0x76 && bme280Adress != 0x77) return;
      if(TCA9548AAdress > 0 && (TCA9548AAdress < 0x70 || TCA9548AAdress > 0x77)) return;
      if(TCA9548APort < 0 || TCA9548APort > 7) return;
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
      if (bmp280List.length() > 0 && !if_init)
        for (byte i = 0; i < bmp280List.length(); i++)
          bmp280List[i].initSensor();
      if_init = true;
    }
};

BME280Manager myBME280Manager = BME280Manager(30); // set scan interval in seconds
/*  End of M6_MS_BME280SensorManager */

/*
   dataneo @2019 - M7_MS_BH1750SensorManager
   MySensors BH1750 Sensor Manager 1.0
   see https://sites.google.com/site/dataneosoftware/arduino/mysensors-bh1750-sensor-manager
*/
#include <Wire.h>
#include <ErriezBH1750.h>
#include <QList.h>

class BH1750Simple {
  public:
    BH1750Simple() {
      BH1750DeviceAdress = 0x23;
      TCA9548ADeviceAdress = 0;
      TCA9548APortNo = 0;
      initCorrect = false;
    };
    BH1750Simple(uint8_t BH1750Adress, uint8_t TCA9548AAdress, uint8_t TCA9548APort, const char* bh1750_name) {
      BH1750DeviceAdress = BH1750Adress;
      TCA9548ADeviceAdress = TCA9548AAdress;
      TCA9548APortNo = TCA9548APort;
      _bh1750_name = bh1750_name;
      initCorrect = false;
    };
    void readSensor(bool forceSendToController = false) {
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
      bh1750Obj.isConversionCompleted();
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
  private:
    uint8_t TCA9548ADeviceAdress; // from 0x70 to 0x77 - use 0 for disable TCA9548A
    uint8_t TCA9548APortNo; // from 0 to 7
    uint8_t BH1750DeviceAdress; // 0x23 or 0x5C
    BH1750 bh1750Obj;
    float _lux;
    const char* _bh1750_name;
    bool initCorrect;
    void sendStateToController() {
      MyMessage msgLighLevel(getChildID(), V_LEVEL);
      send(msgLighLevel.set(_lux, 1));
    };

    uint8_t getChildID() {
      byte f = BH1750DeviceAdress == 0x23 ? 0 : 1;
      uint8_t id = 135 + f; // start from 135 to 134
      if (TCA9548ADeviceAdress > 0) {
        id = 137;
        id = id + (TCA9548ADeviceAdress - 0x70) * 8 + TCA9548APortNo;
      }
      return id;
    };
    void tcaSelect(uint8_t TCA9548A, uint8_t i) {
      if (i > 7) return;
      Wire.beginTransmission(TCA9548A);
      Wire.write(1 << i);
      Wire.endTransmission();
    }
};

class BH1750Manager {
  public:
    BH1750Manager(uint8_t scanIntervalInSeconds) {
      if_init = false;
      scanInterval = scanIntervalInSeconds;
      lastScan = 0;
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
          BH1750List[i].readSensor(false);
    }
    void addSensor() {
      addSensor(0x23, 0, 0, '\0');
    }
    void addSensor(uint8_t BH1750Adress, uint8_t TCA9548AAdress, uint8_t TCA9548APort, const char* bh1750_name) {
      if (BH1750List.length() >= MAX_SENSORS) return;
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
      if (!exist)
        BH1750List.push_back(BH1750Simple(BH1750Adress, TCA9548AAdress, TCA9548APort, bh1750_name));
    }
  private:
    bool if_init;
    uint8_t scanInterval; // in seconds
    unsigned long lastScan;
    const byte MAX_SENSORS = 32;
    QList<BH1750Simple> BH1750List;
    void initAllSensors() {
      Wire.begin();
      if (BH1750List.length() > 0 && !if_init)
        for (byte i = 0; i < BH1750List.length(); i++)
          BH1750List[i].initSensor();
      if_init = true;
    }
};

BH1750Manager myBH1750Manager(30); // set scan interval in seconds
/*  End of M7_MS_BH1750SensorManager */

void before() {
  //M1_MS_RelayManager
  myRelayController.addRelay(23, A8, LOAD_FROM_CONTROLLERS, RELAY_ON_HIGH, "lampka"); // ch1
  myRelayController.addRelay(23, A7);
  myRelayController.addRelay(23, A6);

  mySwitchManager.addSwitch(A0, SWITCH_NORMAL_CLOSE, "drzwi kuchnia");  // M4_MS_SwitchSensorManager
  
  myMotionManager.addMotion(7, SENSOR_ON_HIGH, "Czujnik ruchu salon", S_MOTION);  // M5_MS_MotionSensorManager

  myBME280Manager.addSensor(0x76, 0, 0, "Kuchnia");  // M6_MS_BME280SensorManager

  /* M7_MS_BH1750SensorManager */
  myBH1750Manager.addSensor(0x23, 0, 0, "Czujnik poziomu oświetlenia");  // M7_MS_BH1750SensorManager
}

void setup() { }

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("A1_MS_Test", "1.0");
  myRelayController.presentAllToControler(); //M1_MS_RelayManager
  mySwitchManager.presentAllToControler(); //M4_MS_SwitchSensorManager
  myMotionManager.presentAllToControler(); //M5_MS_MotionSensorManager
  myBME280Manager.presentAllToControler(); //M6_MS_BME280SensorManager
  myBH1750Manager.presentAllToControler(); //M7_MS_BH1750SensorManager
}

void loop()
{
  myRelayController.buttonCheckState(); //M1_MS_RelayManager
  mySwitchManager.switchCheckState(); //M4_MS_SwitchSensorManager
  myMotionManager.motionCheckState(); //M5_MS_MotionSensorManager
  myHeartBeatManager.HeartBeat(); // Heartbeat Manager
  myBME280Manager.sensorsCheck(); //M6_MS_BME280SensorManager
  myBH1750Manager.sensorsCheck(); //M7_MS_BH1750SensorManager
  //wait(1, C_SET, V_STATUS);
}

void receive(const MyMessage &message)
{
  myHeartBeatManager.ControllerReciveMsg();// Heartbeat Manager

  if (message.type == V_ARMED && ! message.isAck()){
    myMotionManager.setStatetFromControler(message.sensor, message.getBool());  
  }
  //M1_MS_RelayManager
  if (message.type == V_STATUS && ! message.isAck()){
    myRelayController.setStateOnRelayListFromControler(message.sensor, message.getBool());
  }    
}

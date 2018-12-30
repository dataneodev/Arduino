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
      mMessage = MyMessage(_relay_pin_no, V_STATUS);
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
    MyMessage mMessage;
    STATE_METHOD _save_state;
    RELAY_STATE _relay_on_state;

    byte getGPIOState(bool relay_state) {
      return (_relay_on_state == RELAY_ON_HIGH) ? relay_state : ! relay_state;
    }
    void sendStateToController() {
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
   MySensors Motion Sensor Manager 1.0
   see https://sites.google.com/site/dataneosoftware/arduino/mysensors-motion-sensor-manager
*/

enum MOTION_STATE {
  MOTION_NORMAL_OPEN,
  MOTION_NORMAL_CLOSE,
};

//Motion Sensor Manager
class MotionSimple {
  public:
    MotionSimple() {
      _motion_pin_no = 0;
      _motion_value = 0;
      _motion_state = MOTION_NORMAL_OPEN;
    };
    MotionSimple(byte motion_pin_no, MOTION_STATE motion_state, const char* motion_name) {
      _motion_pin_no = motion_pin_no;
      _motion_value = 0;
      _motion_state = motion_state;
      _motion_name = motion_name;
    };
    byte motionPin() {
      return _motion_pin_no;
    }
    bool checkmotion(bool forceSendToController = false) {
      if(_motion_pin_no == 0) return false;
      bool readValue = digitalRead(_motion_pin_no) == HIGH ? true : false;
      if(readValue != _motion_value || forceSendToController){
        _motion_value = readValue;
        sendStateToController();
      }
      return readValue;
    }
    void initPin() {
      if(_motion_pin_no == 0) return;
      pinMode(_motion_pin_no, INPUT);
      mMessage = MyMessage(_motion_pin_no, V_TRIPPED);
      checkmotion(true);
    }
    void presentToControler() {
      if(_motion_pin_no == 0) return;
      present(_motion_pin_no, S_MOTION, _motion_name);
    }
  private:
    MyMessage mMessage;
    MOTION_STATE _motion_state;
    bool _motion_value;
    byte _motion_pin_no; 
    const char* _motion_name;  

    void sendStateToController() {
      bool state = _motion_value;
      if(_motion_state == MOTION_NORMAL_OPEN) 
        state = !state;
      send(mMessage.set(state ? "1" : "0"));
    }
};

class MotionManager {
  public:
    MotionManager() {
      if_init = false;
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
      addMotion(motion_pin_no, MOTION_NORMAL_CLOSE, '\0');
    }
    void addMotion(byte motion_pin_no, MOTION_STATE motion_state) {
      addMotion(motion_pin_no, MOTION_NORMAL_CLOSE, '\0');
    }
    void addMotion(byte motion_pin_no, MOTION_STATE motion_state, const char* motion_name) {
      if(motionList.length()>= MAX_PIN) return;        
      //check if motion exists
      bool exist = false;
      if (motionList.length() > 0)
        for (byte i = 0; i < motionList.length(); i++)
          if (motionList[i].motionPin() == motion_pin_no)
            exist = true;
      if (!exist) 
        motionList.push_back(MotionSimple(motion_pin_no, motion_state, motion_name));
    }
  private:
    bool if_init;
    const byte MAX_PIN = 70;
    QList<MotionSimple> motionList;
    void initAllPins() {
      if (motionList.length() > 0 && !if_init)
        for (byte i = 0; i < motionList.length(); i++)
          motionList[i].initPin();
      if_init = true;
    }
};

MotionManager myMotionManager = MotionManager();
/*  End of M5_MS_MotionSensorManager */

void before() {
  //M1_MS_RelayManager
  myRelayController.addRelay(23, A8, LOAD_FROM_CONTROLLERS, RELAY_ON_HIGH, "lampka"); // ch1
  myRelayController.addRelay(23, A7);
  myRelayController.addRelay(23, A6);

  mySwitchManager.addSwitch(A0, SWITCH_NORMAL_CLOSE, "drzwi kuchnia");  // M4_MS_SwitchSensorManager
  
  myMotionManager.addMotion(7, MOTION_NORMAL_CLOSE, "Czujnik ruchu salon");  // M5_MS_MotionSensorManager
}

void setup() { }

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("A1_MS_Test", "1.0");
  myRelayController.presentAllToControler(); //M1_MS_RelayManager
  mySwitchManager.presentAllToControler(); //M4_MS_SwitchSensorManager
  myMotionManager.presentAllToControler(); //M5_MS_MotionSensorManager
}

void loop()
{
  myRelayController.buttonCheckState(); //M1_MS_RelayManager
  mySwitchManager.switchCheckState(); //M4_MS_SwitchSensorManager
  myMotionManager.motionCheckState(); //M5_MS_MotionSensorManager
  myHeartBeatManager.HeartBeat(); // Heartbeat Manager
  //wait(1, C_SET, V_STATUS);
}

void receive(const MyMessage &message)
{
  myHeartBeatManager.ControllerReciveMsg();// Heartbeat Manager
  
  //M1_MS_RelayManager
  if (message.type == V_STATUS && ! message.isAck()) 
    myRelayController.setStateOnRelayListFromControler(message.sensor, message.getBool());
}

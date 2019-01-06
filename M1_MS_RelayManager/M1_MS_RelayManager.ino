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
   End of M1_MS_RelayManager
*/

void before()
{
  /* M1_MS_RelayManager definition list
     Define your relay here
     myRelayController.addRelay(byte relay_pin_no)
     myRelayController.addRelay(byte relay_pin_no, byte button_pin_no)
     myRelayController.addRelay(byte relay_pin_no, byte button_pin_no, STATE_METHOD save_state, RELAY_STATE relay_on_state)
     myRelayController.addRelay(byte relay_pin_no, byte button_pin_no, STATE_METHOD save_state, RELAY_STATE relay_on_state, const char* relay_name)

     relay_pin_no - gpio pin with relay connected
     button_pin_no - gpio pin with button switch connected; use 0 - for no button
     
     STATE_METHOD is one of 
      SAVE_TO_EEPROM - default
      LOAD_FROM_CONTROLLERS - don't work on homeassistant
      START_IN_HIGH
      START_IN_LOW
    
    RELAY_STATE is one of 
      RELAY_ON_HIGH - default
      RELAY_ON_LOW 
  */
  myRelayController.addRelay(23, A8, LOAD_FROM_CONTROLLERS, RELAY_ON_HIGH, "lampka"); // ch1
  myRelayController.addRelay(23, A7);
  myRelayController.addRelay(23, A6);
  myRelayController.addRelay(38, A8, SAVE_TO_EEPROM, RELAY_ON_HIGH, "ch8"); //ch8
  myRelayController.addRelay(40, A9, START_IN_HIGH, RELAY_ON_HIGH, "ch7"); //ch7
  myRelayController.addRelay(44, A10, START_IN_LOW, RELAY_ON_HIGH, "ch6"); //ch6
  myRelayController.addRelay(46, A11, START_IN_LOW, RELAY_ON_LOW, "ch5"); //ch5
}

void setup() { }

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("MySensorsRelayManager", "1.1");

  myRelayController.presentAllToControler(); //M1_MS_RelayManager
}

void loop()
{
  myRelayController.buttonCheckState(); //M1_MS_RelayManager
  wait(1, C_SET, V_STATUS);
}

void receive(const MyMessage &message)
{
  //M1_MS_RelayManager
  if (message.type == V_STATUS && ! message.isAck()) 
    myRelayController.setStateOnRelayListFromControler(message.sensor, message.getBool());
}

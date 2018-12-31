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
   dataneo @2018 - M4_MS_SwitchSensorManager
   MySensors Switch Sensor Manager 1.0
   Mechanical switch manager with debouncer
   see https://sites.google.com/site/dataneosoftware/arduino/switch-sensor-manager
*/
#include <Bounce2.h>
#include <QList.h>

enum SWITCH_STATE {
  NORMAL_OPEN,
  NORMAL_CLOSE,
};

//Switch Sensor Manager
class SwitchSimple {
  public:
    SwitchSimple() {
      _switch_pin_no = 0;
      _switch_value = 0;
      _switch_state = NORMAL_OPEN;
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
      if(_switch_state == NORMAL_OPEN) 
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
      addSwitch(switch_pin_no, NORMAL_CLOSE, '\0');
    }
    void addSwitch(byte switch_pin_no, SWITCH_STATE switch_state) {
      addSwitch(switch_pin_no, NORMAL_CLOSE, '\0');
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

void before()
{
  /* M4_MS_SwitchSensorManager */
  mySwitchManager.addSwitch(A0, NORMAL_CLOSE, "drzwi kuchnia");  // M4_MS_SwitchSensorManager

}

void setup() { }

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Switch Sensor Manager", "1.0");

  mySwitchManager.presentAllToControler(); //M4_MS_SwitchSensorManager
}

void loop()
{
  mySwitchManager.switchCheckState(); //M4_MS_SwitchSensorManager
}

void receive(const MyMessage &message){ 
if (message.type == V_STATUS && ! message.isAck())
  return;
}

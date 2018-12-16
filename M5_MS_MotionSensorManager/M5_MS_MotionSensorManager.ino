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
   dataneo @2018 - M5_MS_MotionSensorManager
   MySensors Motion Sensor Manager 1.0
   see https://sites.google.com/site/dataneosoftware/arduino/motion-sensor-manager
*/
#include <QList.h>

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

void before()
{
  /* M5_MS_MotionSensorManager */
  myMotionManager.addMotion(7, MOTION_NORMAL_CLOSE, "Czujnik ruchu salon");  // M5_MS_MotionSensorManager

}

void setup() { }

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Motion Sensor Manager", "1.0");

  myMotionManager.presentAllToControler(); //M5_MS_MotionSensorManager
}

void loop()
{
  myMotionManager.motionCheckState(); //M5_MS_MotionSensorManager
  wait(1, C_SET, V_STATUS);
}

void receive(const MyMessage &message){ 
if (message.type == V_STATUS && ! message.isAck())
  return;
}

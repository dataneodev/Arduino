/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0 - Henrik Ekblad
 *
 * DESCRIPTION
 * Example sketch showing how to control physical relays.
 * This example will remember relay state after power failure.
 * http://www.mysensors.org/build/relay
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
   dataneo @2018
   Simple relay sketch
*/

#define BOUNCE_LOCK_OUT
#include <Bounce2.h>

//Simple relay class
 class RelaySimple{
  public:
    RelaySimple(int relay_pin_no):RelaySimple(relay_pin_no,0,true, true){}
    RelaySimple(int relay_pin_no, int button_pin_no):RelaySimple(relay_pin_no, button_pin_no,true, true){}
    RelaySimple(int relay_pin_no, int button_pin_no, bool save_state, bool relay_on_state){
      Serial.print("RelaySimple: contructor");
      _relay_pin_no = relay_pin_no;
      _button_pin_no = button_pin_no;
      _save_state = save_state;
      _relay_on_state = relay_on_state;
    };

    int relayPin(){ return _relay_pin_no; }
    void initPin(){
      Serial.print("RelaySimple: initPins");
      pinMode(_relay_pin_no, OUTPUT);
      if(_button_pin_no != 0){
        pinMode(_button_pin_no, INPUT_PULLUP);
        _debouncer = Bounce();
        _debouncer.attach(_button_pin_no);
        _debouncer.interval(10);
      }
      mMessage = MyMessage(_relay_pin_no, V_LIGHT); 
      if(_save_state){
        _relay_state = loadState(_relay_pin_no);
        digitalWrite(_relay_pin_no, getGPIOState(_relay_state)); 
        // send load state to controller
        sendStateToController();
      } else {
        // request load state from controler
        request(_relay_pin_no, V_LIGHT);      
      }
    }
    
    void presentToControler(){
      Serial.print("RelaySimple: presentToControler");
      present(_relay_pin_no, S_LIGHT);
    }
    
    void setStateFromControler(bool relay_state){
      Serial.print("RelaySimple: setStateFromControler");
      if(_relay_state == relay_state)
        return;
      _relay_state = relay_state;
      digitalWrite(_relay_pin_no, getGPIOState(_relay_state)); 
      if(_save_state)
        saveState(_relay_pin_no, _relay_state);
    }
    
    void checkButton(){
      if(_button_pin_no == 0)
        return;
      _debouncer.update();
      if( _debouncer.fell() ) {  // Call code if button transitions from HIGH to LOW
        _relay_state = ! _relay_state;
        digitalWrite(_relay_pin_no, getGPIOState(_relay_state));
        if(_save_state) 
          saveState(_relay_pin_no, _relay_state);
          sendStateToController();
      }  
    }
  private: 
    bool _relay_state; // current relay state on or off
    int _relay_pin_no; // gpio pin for relay
    int _button_pin_no; // gpio pin for relay button; 0 - for no button
    bool _save_state; //true - save relay state to emprrom
    bool _relay_on_state; // true -> (_relay_state = true - gpio = 1); false -> (relay_on = true - gpio = 0) 
    Bounce _debouncer;  
    MyMessage mMessage;
    int getGPIOState(bool relay_state){
       return _relay_on_state ? relay_state : ! relay_state;
    }
    void sendStateToController(){
      send(mMessage.set(_relay_state)); 
    }
};
    
class RelayManager{
  public:
    RelayManager(){
      if_init = false;  
    }
    //Simple Relay change status in RelayList
    void setStateOnRelayListFromControler(int relay_pin_no, bool relay_state){
      Serial.print("RelaySimple: setStateOnRelayListFromControler");
      if(sizeof RelayList > 0 && if_init)
        for (int i=0; i<sizeof RelayList/sizeof RelayList[0]; i++)
          if(RelayList[i].relayPin() == relay_pin_no)
            RelayList[i].setStateFromControler(relay_state);
    }
    void presentAllToControler(){
      if(sizeof RelayList > 0)
        for (int i=0; i<sizeof RelayList/sizeof RelayList[0]; i++)
          RelayList[i].presentToControler();    
      initAllPins();
    }
    void buttonCheckState(){
    if(sizeof RelayList > 0 && if_init)
        for (int i=0; i<sizeof RelayList/sizeof RelayList[0]; i++) 
          RelayList[i].checkButton();    
    }
  private:
    bool if_init;
    void initAllPins(){
      if(sizeof RelayList > 0 && !if_init)
        for (int i=0; i<sizeof RelayList/sizeof RelayList[0]; i++) 
          RelayList[i].initPin();  
      if_init = true;
    }
    
    /* Simple Relay definition list
       Define your relay here
       possible constructors
       RelaySimple(int relay_pin_no)
       RelaySimple(int relay_pin_no, int button_pin_no)
       RelaySimple(int relay_pin_no, int button_pin_no, bool save_state, bool relay_start_state, bool relay_on_state)
    */
    RelaySimple RelayList[2] = {
      RelaySimple(23, A8, false, true),
      RelaySimple(A1, 0, false, true),
    };
};

RelayManager myRelayController = RelayManager();

void before(){ }
void setup(){ }

void presentation()
{
	// Send the sketch version information to the gateway and Controller
	sendSketchInfo("SimpleRelay_dataneo", "1.0");

 //Simple Relay presentAllToControler
  myRelayController.presentAllToControler(); 
}

void loop()
{
  //Simple Relay buttonCheckState
  myRelayController.buttonCheckState();
  
  wait(1); // loop 1000 times per second
}

void receive(const MyMessage &message)
{
	// We only expect one type of message from controller. But we better check anyway.
	if (message.type==V_STATUS) {
    //Simple Relay foward status 
    myRelayController.setStateOnRelayListFromControler(message.sensor, message.getBool());
	}
 
}

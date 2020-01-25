/*
   dataneo @2019 - M1_MS_RelayManager
   MySensors Relay Manager 1.3
   see https://sites.google.com/site/dataneosoftware/arduino/mysensors-relay-manager
*/

#define MY_GATEWAY_SERIAL // Enable serial gateway

#include "I:\7.Projekty\5.Arduino\M_Library\RelayManager\RelayManager.h"

RelayManager myRelayController = RelayManager(HOMEASSISTANT, true); //controller type, pinMode: INPUT_PULLUP - true, INPUT false

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
      SAVE_TO_24C32 
      LOAD_FROM_CONTROLLERS - don't work on homeassistant
      START_IN_HIGH
      START_IN_LOW

      BUTTON_TYPE
        BISTABLE - default
        MONOSTABLE

    RELAY_STATE is one of
      RELAY_ON_HIGH - default
      RELAY_ON_LOW
  */
  myRelayController.addRelay(6, 7, LOAD_FROM_CONTROLLERS, RELAY_ON_HIGH, MONOSTABLE, "lampka"); // ch1
  myRelayController.addRelay(23, A7);
  myRelayController.addRelay(23, A6);
  myRelayController.addRelay(38, A8, SAVE_TO_EEPROM, RELAY_ON_HIGH, BISTABLE, "ch8"); //ch8
  myRelayController.addRelay(40, A9, START_IN_HIGH, RELAY_ON_HIGH, BISTABLE, "ch7");  //ch7
  myRelayController.addRelay(44, A10, START_IN_LOW, RELAY_ON_HIGH, BISTABLE, "ch6");  //ch6
  myRelayController.addRelay(46, A11, START_IN_LOW, RELAY_ON_LOW, BISTABLE, "ch5");   //ch5
}

void setup() {}

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("MySensorsRelayManager", "1.3");

  myRelayController.presentAllToControler(); //M1_MS_RelayManager
}

void loop()
{
  myRelayController.buttonCheckState(); //M1_MS_RelayManager
  wait(1, C_SET, V_STATUS);
}

void receive(const MyMessage &message)
{
  myRelayController.setStateOnRelayListFromControler(message); //M1_MS_RelayManager
}

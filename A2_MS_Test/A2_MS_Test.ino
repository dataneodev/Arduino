//A2_MS_Test

#define MY_GATEWAY_SERIAL // Enable serial gateway
#include "I:\7.Projekty\5.Arduino\M_Library\RelayManager\RelayManager.h"

RelayManager myRelayController = RelayManager(HOMEASSISTANT, false);
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
  myRelayController.addRelay(22, 23, START_IN_LOW, RELAY_ON_HIGH, BISTABLE, "22-23"); //ch1
  myRelayController.addRelay(24, 25, START_IN_LOW, RELAY_ON_HIGH, MONOSTABLE, "24-25"); //ch2
  myRelayController.addRelay(26, 27, START_IN_LOW, RELAY_ON_HIGH, MONOSTABLE, "26-27"); //ch3
  myRelayController.addRelay(28, 29, START_IN_LOW, RELAY_ON_HIGH, MONOSTABLE, "28-29"); //ch4
  myRelayController.addRelay(30, 31, START_IN_LOW, RELAY_ON_HIGH, MONOSTABLE, "30-31"); //ch5
  myRelayController.addRelay(32, 33, START_IN_LOW, RELAY_ON_HIGH, MONOSTABLE, "32-33"); //ch6
  myRelayController.addRelay(34, 35, START_IN_LOW, RELAY_ON_HIGH, MONOSTABLE, "34-35"); //ch7
  myRelayController.addRelay(36, 37, START_IN_LOW, RELAY_ON_HIGH, MONOSTABLE, "36-37"); //ch8
}

void setup() { }

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

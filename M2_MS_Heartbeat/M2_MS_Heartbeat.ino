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
   dataneo @2018 - M2_MS_Heartbeat
   MySensors Heartbeat Manager 1.0
   see https://sites.google.com/site/dataneosoftware/arduino/mysensors-heartbeat
*/
class HeartBeatManager {
  public:
    HeartBeatManager() {
      HeartBeatManager(0);
    };
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
    const unsigned short _toneLength = 500; // length of buzzer active
    const unsigned short _toneBreak = 500; // length of buzzer break
    const unsigned short _toneHz = 4000; // buzzer

    void SendHeart() {
      _lastSend = _currentTime;
      sendHeartbeat();
      if (_alarmBuzzerPin != 0)
        requestTime();
    }
    void BuzzerCheck() {
      if (_alarmBuzzerPin != 0 &&
          _lastSend != 0 &&
          _lastRecive < _lastSend &&
          ((_currentTime - _lastSend > _timeReciveTimeout) || (_currentTime - _lastRecive > _timeSendPeriod + _timeReciveTimeout)) &&
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
   End of M2_MS_Heartbeat
*/

void before() { }

void setup() { }

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("M2_MS_Heartbeat", "1.0");
}

void loop()
{
  myHeartBeatManager.HeartBeat(); // M2_MS_Heartbeat
  wait(1, C_SET, V_STATUS);
}

void receive(const MyMessage &message)
{
  myHeartBeatManager.ControllerReciveMsg();// M2_MS_Heartbeat
}

/*
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2022 Sensnology AB
 * Full contributor list: https://github.com/mysensors/MySensors/graphs/contributors
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
 * Version 1.1 - GizMoCuz
 * Version 1.2 - Paolo Rendano
 *                * factory reset
 *                * automatic home assistant entities creation
 *                * counter correction from home assistant using
 *                  service notify.mysensors
 *                * fixed counter automatically incremented by 1
 *                  at each device restart (due to arduino library
 *                  interrupt bug)
 *                * other tiny improvements
 * Version 1.3 - Paolo Rendano
 *                * change flow measurement unit to m3/h to adopt
 *                  in home assistant as standard measurement unit
 *                  for V_FLOW
 *
 * DESCRIPTION
 * Use this sensor to measure volume and flow of your house water meter.
 * You need to set the correct pulsefactor of your meter (pulses per m3).
 * The sensor starts by fetching current volume reading from gateway (VAR 1).
 * Reports both volume and flow back to gateway.
 *
 * Unfortunately millis() won't increment when the Arduino is in
 * sleepmode. So we cannot make this sensor sleep if we also want
 * to calculate/report flow.
 * http://www.mysensors.org/build/pulse_water
 */

// Enable debug prints to serial monitor
#define MY_DEBUG
#define APP_DEBUG

// Enable factory reset to REINIT the board with a different ID
//#define FORCE_FACTORY_RESET

// uncomment to rejoin to a previous assigned node id
//#define MY_NODE_ID 58

// Enable and select radio type attached
#define MY_RADIO_RF24
//#define MY_RADIO_NRF5_ESB
//#define MY_RADIO_RFM69
//#define MY_RADIO_RFM95
//#define MY_PJON
#define MY_SPLASH_SCREEN_DISABLED

#include <MySensors.h>

// The digital input you attached your sensor.  (Only 2 and 3 generates interrupt!)
#define DIGITAL_INPUT_SENSOR 3

// Arduino Uno/Nano: INTF0 for DIGITAL_INPUT_SENSOR = 2; INTF1 for DIGITAL_INPUT_SENSOR = 3
// Arduino AtMega2560: INTF0->INTF7 see datasheet based on the input you want to attach
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define DIGITAL_INPUT_SENSOR_INTF INTF1
#endif

// Number of blinks per m3 of your meter (One rotation/liter)
#define PULSE_FACTOR 1000.0d

// flowvalue can only be reported when sleep mode is false.
#define SLEEP_MODE false

// Max flow (m3/h) value to report. This filters outliers.
#define MAX_FLOW 2.4d

// Timeout (in milliseconds) to reset to 0 the flow
// information (assuming no pulses if no flow)
#define FLOW_RESET_TO_ZERO_TIMEOUT 120000

// Id of the sensor child
#define CHILD_ID 1

// Id of the sensor child for counter pulse addition
#define CHILD_ID_VAR1 2

// Minimum time between send (in milliseconds). We don't want to spam the gateway.
#define SEND_FREQUENCY 30000

// Save on board if the home assistant counters have been initialized
// (sufficient condition to see the entity in hass)
#define FIRST_VALUE_SENT_FLAG_POSITION 0
#define FIRST_VALUE_SENT_FLAG_YES 1
#define FIRST_VALUE_SENT_FLAG_NO 255

MyMessage flowMsg(CHILD_ID,V_FLOW);
MyMessage volumeMsg(CHILD_ID,V_VOLUME);
MyMessage lastCounterMsg(CHILD_ID,V_VAR1);
MyMessage volumeAdd(CHILD_ID_VAR1,V_TEXT);

// Pulses per liter
double ppl = ((double)PULSE_FACTOR)/1000;

volatile uint32_t pulseCount = 0;
volatile uint32_t lastBlink = 0;
volatile double flow = 0;
bool pcReceived = false;
uint32_t oldPulseCount = 0;
double oldflow = 0;
double oldvolume =0;
uint32_t lastSend =0;
uint32_t lastPulse =0;
bool firstValuesMessageSent = false;

void IRQ_HANDLER_ATTR onPulse()
{
	if (!SLEEP_MODE) {
		uint32_t newBlink = micros();
		uint32_t interval = newBlink-lastBlink;

		if (interval!=0) {
			lastPulse = millis();
			if (interval<500000L) {
				// Sometimes we get interrupt on RISING,
        // 500000 = 0.5 second debounce ( max 7.2 m3/h)
				return;
			}
			flow = (3600000.0 /interval) / ppl;
		}
		lastBlink = newBlink;
	}
	pulseCount++;
}

void setup()
{
  #ifdef FORCE_FACTORY_RESET
  // got from mysensors clear e2p sketch
  for (uint16_t i=0; i<EEPROM_LOCAL_CONFIG_ADDRESS; i++) {
		hwWriteConfig(i,0xFF);
	}
  // reset state for this sensor
  forceFactoryReset();
  Serial.println("Factory reset complete");
  while (true);
  #endif

	// initialize our digital pins internal pullup resistor so one pulse
  // switches from high to low (less distortion)
	pinMode(DIGITAL_INPUT_SENSOR, INPUT_PULLUP);

	pulseCount = oldPulseCount = 0;

  // initialize home assistant values
  checkAndFirstTimeInitValuesOnHomeAssistant();

	// Fetch last known pulse count value from gw
	request(CHILD_ID, V_VAR1);

	lastSend = lastPulse = millis();

  // fix for arduino library bug calling ISR function
  // at startup see https://github.com/arduino/ArduinoCore-avr/issues/244
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  EIFR |= (1 << DIGITAL_INPUT_SENSOR_INTF);
#endif

	attachInterrupt(digitalPinToInterrupt(DIGITAL_INPUT_SENSOR), onPulse, FALLING);
}

void presentation()
{
	// Send the sketch version information to the gateway and Controller
	sendSketchInfo("Water Meter", "1.3");

	// Register this device as Water flow sensor
	present(CHILD_ID, S_WATER);

  // Add info entity to manage corrections on pulse counter
  // (on Home Assistant this can be hidden)
  present(CHILD_ID_VAR1, S_INFO);
}

void loop()
{
	uint32_t currentTime = millis();

	// Only send values at a maximum frequency or woken up from sleep
	if (SLEEP_MODE || (currentTime - lastSend > SEND_FREQUENCY)) {
		lastSend=currentTime;

		if (!pcReceived) {
			// Last Pulsecount not yet received from controller,
      // request it again
			request(CHILD_ID, V_VAR1);
			return;
		}

		if (!SLEEP_MODE && flow != oldflow) {
			oldflow = flow;
#ifdef APP_DEBUG
			Serial.print("m3/h:");
			Serial.println(flow);
#endif
			// Check that we don't get unreasonable large flow value.
			// could happen when long wraps or false interrupt triggered
			if (flow<((uint32_t)MAX_FLOW)) {
        // Send flow value to gw
				send(flowMsg.set(flow, 4));
			}
		}

		// No Pulse count received for a defined time
		if(currentTime - lastPulse > FLOW_RESET_TO_ZERO_TIMEOUT) {
			flow = 0;
		}

		// Pulse count has changed
		if ((pulseCount != oldPulseCount)||(!SLEEP_MODE)) {
			oldPulseCount = pulseCount;
#ifdef APP_DEBUG
			Serial.print("pulsecount:");
			Serial.println(pulseCount);
#endif
      // Send pulsecount value to gw in VAR1
			send(lastCounterMsg.set(pulseCount));

			double volume = pulseCount / PULSE_FACTOR;
			if ((volume != oldvolume)||(!SLEEP_MODE)) {
				oldvolume = volume;

#ifdef APP_DEBUG
				Serial.print("volume:");
				Serial.println(volume, 3);
#endif
        // Send volume value to gw
				send(volumeMsg.set(volume, 3));
			}
		}
	}
	if (SLEEP_MODE) {
		sleep(SEND_FREQUENCY, false);
	}
}

// clearing eeprom with mysensors sketch is not enough since this
// doesn't cover the saved state values. Call this function once
// to force factory reset
void forceFactoryReset() {
  saveState(FIRST_VALUE_SENT_FLAG_POSITION, FIRST_VALUE_SENT_FLAG_NO);
}

void checkAndFirstTimeInitValuesOnHomeAssistant() {
  // check local e2p if this sensor has already sent
  // initial value (0). If not will send the init value
  uint8_t state = loadState(FIRST_VALUE_SENT_FLAG_POSITION);
  if (state==FIRST_VALUE_SENT_FLAG_NO) {
    // never sent anything. No value in HASS is assumed
#ifdef APP_DEBUG
    Serial.println("First time init");
#endif
    firstValuesMessageSent = true;
    // Send flow value to gw
    send(flowMsg.set(0.0d, 4));
    // Send volume value to gw
    send(volumeMsg.set(0.0d, 3));
    // Send pulsecount value to gw in VAR1
	  send(lastCounterMsg.set((uint32_t)0));
    // Send volumeAdd value to gw in V_TEXT
    send(volumeAdd.set(""));
  }
}

void receive(const MyMessage &message)
{
	if (message.getType()==V_VAR1) {
    if (firstValuesMessageSent) {
      // ack saving value on board that HomeAssistant
      // got first init values. Will never be sent again
      saveState(FIRST_VALUE_SENT_FLAG_POSITION,
                FIRST_VALUE_SENT_FLAG_YES);
    }

		uint32_t gwPulseCount=message.getULong();
		pulseCount += gwPulseCount;
		flow=oldflow=0.0d;

#ifdef APP_DEBUG
		Serial.print("Received last pulse count from gw:");
		Serial.println(pulseCount);
#endif

		pcReceived = true;
	}

  // incoming message to correct pulsecount
  // used to add or remove pulses to the current reading
  if (message.getType()==V_TEXT) {
      long val = atol(message.getString());
      if ((val+(int32_t)pulseCount)<0) {
        // TODO: this is not covering all the ranges problems
#ifdef APP_DEBUG
        Serial.println("out of range. Won't add");
#endif
      }
      else {
        pulseCount+=val;
        // reset only if ok
        send(volumeAdd.set(""));
#ifdef APP_DEBUG
        Serial.print("New pulseCount: ");
        Serial.println(pulseCount);
#endif
      }
  }
}

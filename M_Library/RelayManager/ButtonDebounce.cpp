#include "Arduino.h"
#include "ButtonDebounce.h"

ButtonDebounce::ButtonDebounce(u_int32_t pin){
  pinMode(pin, INPUT_PULLDOWN);
  
  _pin = digitalPinToPinName(pin);
  _lastDebounceTime = 0;
  _lastStateBtn = HIGH;
}

bool ButtonDebounce::isTimeToUpdate(){
  return (millis() - _lastDebounceTime) > DELAY;
}

void ButtonDebounce::update(){
  if(!isTimeToUpdate()) return;
  _lastDebounceTime = millis();
  int btnState = (_pin == NC) ? 0 : digitalReadFast(_pin);
  if(btnState == _lastStateBtn) return;
  _lastStateBtn = btnState;
  if(this->callback) this->callback(_lastStateBtn);
}

byte ButtonDebounce::state(){
  return _lastStateBtn;
}

void ButtonDebounce::setCallback(BTN_CALLBACK){
  this->callback = callback;
}
#include "Arduino.h"
#include "GKButtonDebounce.h"

unsigned long GKButtonDebounce::Delay = 100;

GKButtonDebounce::GKButtonDebounce(u_int32_t pin){
  pinMode(pin, INPUT_PULLDOWN);
  
  _pin = digitalPinToPinName(pin);
  _lastDebounceTime = 0;
  _lastStateBtn = HIGH;
}

bool GKButtonDebounce::isTimeToUpdate(){
  return (millis() - _lastDebounceTime) > Delay;
}

void GKButtonDebounce::update(){
  if(!isTimeToUpdate()) return;
  _lastDebounceTime = millis();
  int btnState = (_pin == NC) ? 0 : digitalReadFast(_pin);
  if(btnState == _lastStateBtn) return;
  _lastStateBtn = btnState;
  if(this->callback) this->callback(_lastStateBtn);
}

byte GKButtonDebounce::state(){
  return _lastStateBtn;
}

void GKButtonDebounce::setCallback(GKBTN_CALLBACK_STD cb){
  this->callback = cb;
}
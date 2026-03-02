#include "Arduino.h"
#include "STMButtonDebounce.h"

unsigned long STMButtonDebounce::Delay = 100;

STMButtonDebounce::STMButtonDebounce(u_int32_t pin){
  pinMode(pin, INPUT_PULLDOWN);
  
  _pin = digitalPinToPinName(pin);
  _lastDebounceTime = 0;
  _lastStateBtn = HIGH;
}

bool STMButtonDebounce::isTimeToUpdate(){
  return (millis() - _lastDebounceTime) > Delay;
}

void STMButtonDebounce::update(){
  if(!isTimeToUpdate()) return;
  _lastDebounceTime = millis();
  int btnState = (_pin == NC) ? 0 : digitalReadFast(_pin);
  if(btnState == _lastStateBtn) return;
  _lastStateBtn = btnState;
  if(this->callback) this->callback(_lastStateBtn);
}

byte STMButtonDebounce::state(){
  return _lastStateBtn;
}

void STMButtonDebounce::setCallback(STMBTN_CALLBACK_STD cb){
  this->callback = cb;
}
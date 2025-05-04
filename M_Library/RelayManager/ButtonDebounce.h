#include "wiring_constants.h"
#include <sys/types.h>
/*
  ButtonDebounce.h - Library for Button Debounce.
  Created by Maykon L. Capellari, September 30, 2017.
  Released into the public domain.
*/
#ifndef ButtonDebounce_h
#define ButtonDebounce_h

#include "Arduino.h"

#define BTN_CALLBACK void (*callback)(int)
#define DELAY 100

class ButtonDebounce{
  private:
    PinName _pin;
    unsigned long _lastDebounceTime;
    byte _lastStateBtn;
    BTN_CALLBACK;
    bool isTimeToUpdate();
  public:
    ButtonDebounce(u_int32_t pin);
    void update();
    byte state();
    void setCallback(BTN_CALLBACK);
};

#endif

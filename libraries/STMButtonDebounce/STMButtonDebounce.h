#include "wiring_constants.h"
#include <sys/types.h>
/*
  STMButtonDebounce.h 
*/
#ifndef STMButtonDebounce_h
#define STMButtonDebounce_h

#include "Arduino.h"

typedef std::function<void(int)> STMBTN_CALLBACK_STD;

class STMButtonDebounce{
  private:
    PinName _pin;
    unsigned long _lastDebounceTime;
    byte _lastStateBtn;
    STMBTN_CALLBACK_STD callback ;
    bool isTimeToUpdate();
  public:
    STMButtonDebounce(u_int32_t pin);
    void update();
    byte state();
    void setCallback(STMBTN_CALLBACK_STD cb);
	static unsigned long Delay;
};

#endif

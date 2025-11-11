#include "wiring_constants.h"
#include <sys/types.h>
/*
  GKButtonDebounce.h 
*/
#ifndef GKButtonDebounce_h
#define GKButtonDebounce_h

#include "Arduino.h"

typedef std::function<void(int)> GKBTN_CALLBACK_STD;

class GKButtonDebounce{
  private:
    PinName _pin;
    unsigned long _lastDebounceTime;
    byte _lastStateBtn;
    GKBTN_CALLBACK_STD callback ;
    bool isTimeToUpdate();
  public:
    GKButtonDebounce(u_int32_t pin);
    void update();
    byte state();
    void setCallback(GKBTN_CALLBACK_STD cb);
	static unsigned long Delay;
};

#endif

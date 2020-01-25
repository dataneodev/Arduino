#include <Arduino.h>
#line 1 "i:\\7.Projekty\\5.Arduino\\M14_BuzzerManager\\M14_BuzzerManager.ino"
/*
   dataneo @2019 - M14_BuzzerManager
   Buzzer Manager 1.0
*/

#include <BuzzerManager.h>

BuzzerManager buzzer = BuzzerManager(13);

void setup() 
{ 
    buzzer.ActiveBuzzerMelody1();
}

void loop() { }

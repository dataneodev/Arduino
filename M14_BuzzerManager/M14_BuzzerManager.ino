/*
   dataneo @2019 - M14_BuzzerManager
   Buzzer Manager 1.0
*/

#include "I:\7.Projekty\5.Arduino\M_Library\BuzzerManager\BuzzerManager.h"

BuzzerManager buzzer = BuzzerManager(13);

void setup() 
{ 
    buzzer.ActiveBuzzerMelody1();
}

void loop() { }

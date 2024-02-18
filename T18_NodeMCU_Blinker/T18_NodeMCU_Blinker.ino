
#include <Blinkenlight.h>
#include <Fadinglight.h>


Blinkenlight led1(D5);
Blinkenlight led3(D6);
Fadinglight led2(D6, false, 2); 

void setup() {
  // put your setup code here, to run once:

//led1.blink();

//oczekiwanie
analogWriteFreq(100);
SpeedSetting waitingFade = {
    .on_ms = 1500,
    .off_ms = 1500,
    .pause_ms = 3000,
    .ending_ms = 6000,
};
led2.blink(waitingFade);


}

bool oneNoweSet = false;
void loop() {
  // put your main code here, to run repeatedly:
  led1.update();
led2.update();
led3.update();

if(!oneNoweSet && millis() > 15000){

led2.off();

//error
led1.setSpeed(SPEED_RAPID );
led1.blink();


//opening
const SpeedSetting  openingSetting = {
    .on_ms = 400,
    .off_ms = 400,
    .pause_ms = 800,
    .ending_ms = 1600,
};
led3.setSpeed(openingSetting);
led3.blink();

oneNoweSet = true;



}
}

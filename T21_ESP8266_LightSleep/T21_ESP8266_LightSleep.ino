//include files ****************************************************************************************

//WiFi
#include <ESP8266WiFi.h>
//WakeUp
extern "C" {
#include "user_interface.h"
#include "gpio.h"
}
//******************************************************************************************************

//const
const char* ssid = "mySSID";
const char* password = "myPassword";


//var
#define INPUT_PIN 2
int count = 0;
//******************************************************************************************************

void setup() {
//Serial
//Serial.begin(115200);
//Serial.println("Initialization");

pinMode(D1, OUTPUT);
pinMode(D6, OUTPUT);
pinMode(D8, OUTPUT);

pinMode(3, OUTPUT);
pinMode(1, OUTPUT);

digitalWrite(D1, LOW);
digitalWrite(D6, LOW);
digitalWrite(D8, LOW);
digitalWrite(GPIO3, LOW);
digitalWrite(GPIO1, LOW);

WiFi.disconnect();
WiFi.mode(WIFI_OFF);
WiFi.setSleepMode(WIFI_MODEM_SLEEP);
WiFi.forceSleepBegin();


}//setup

void light_sleep(){
   wifi_station_disconnect();
   wifi_set_opmode_current(NULL_MODE);
   wifi_fpm_set_sleep_type(LIGHT_SLEEP_T); // set sleep type, the above    posters wifi_set_sleep_type() didnt seem to work for me although it did let me compile and upload with no errors 
   wifi_fpm_open(); // Enables force sleep
   //gpio_pin_wakeup_enable(GPIO_ID_PIN(2), GPIO_PIN_INTR_LOLEVEL); // GPIO_ID_PIN(2) corresponds to GPIO2 on ESP8266-01 , GPIO_PIN_INTR_LOLEVEL for a logic low, can also do other interrupts, see gpio.h above
   wifi_fpm_do_sleep(0xFFFFFFF); // Sleep for longest possible time
 }


void loop() {

 // Serial.println("Loop start");
  delay(10000);


//Serial.println("I'm ready to Detect => going to lightSleep");

light_sleep();

delay(10000);

}//loop
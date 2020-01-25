
#include <Wire.h>
#include <ESP8266WiFi.h>

extern "C" { 
#include "twi.h"  // from Wire library, so we can do bus scanning
}

#define TCAADDR 0x70
bool initLoop = false;

void setup() {}

void loop() {
  if(!initLoop)
  {
    WiFi.mode(WIFI_OFF);    //This also works
    // put your setup code here, to run once:
    Serial.begin(115200);
    delay(2000);
    Serial.println("TCA9548A I2C scanner ready!");
    Wire.begin(D1, D2);
    
    initLoop = true;
  }
  
  for (uint8_t t=0; t<8; t++) {
      tcaselect(t);
      Serial.print("TCA Port #"); Serial.println(t);
 
      for (uint8_t addr = 0; addr<=127; addr++) {
        if (addr == TCAADDR) continue;
      
        uint8_t data;
        if (! twi_writeTo(addr, &data, 0, 1)) {
           Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
        }
      }
    }
    Serial.println("\ndone");

  delay(5000);           // wait 5 seconds for next scan
}

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

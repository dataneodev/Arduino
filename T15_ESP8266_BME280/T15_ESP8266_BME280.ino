
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;
#include <ESP8266WiFi.h>

extern "C" { 
#include "twi.h"  // from Wire library, so we can do bus scanning
}

#define TCAADDR 0x70

void setup() {
  
  WiFi.mode(WIFI_OFF);    //This also works
    // put your setup code here, to run once:
    Serial.begin(115200);
    delay(2000);
    Serial.println("TCA9548A I2C scanner ready!");
    Wire.begin(D1, D2);

  tcaselect(6);

  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  }

void loop() {
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println("*C");

  Serial.print("Pressure = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println("hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println("m");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println("%");

  Serial.println();
  delay(5000);        // wait 5 seconds for next scan
}

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

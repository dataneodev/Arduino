

#include <OneWire.h>
#include <DallasTemperature.h>


OneWire oneWire(4); //PodĹ‚Ä…czenie do A5
DallasTemperature sensors(&oneWire); //Przekazania inf

int numberOfDevices;
DeviceAddress tempDeviceAddress;

DeviceAddress termometrZewnetrzny = { 0x10, 0x7A, 0x31, 0x99, 0x1, 0x8, 0x0, 0x4A };

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  sensors.begin(); //Inicjalizacja czujnikow
  sensors.setResolution(12);// 9 to 12
  // report parasite power requirements
  Serial.print("Parasite power is: "); 
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");

  numberOfDevices = sensors.getDeviceCount();
  
  Serial.print("Ilość urządzeń: " + numberOfDevices);

  for (byte i = 0; i < numberOfDevices; i++){
    if(sensors.getAddress(tempDeviceAddress, i)){
      Serial.print("Sensor : ");
      Serial.println(i+1, DEC);
      Serial.print("Sensor adress : ");  
      printAddress(tempDeviceAddress);
      Serial.print("Aktualna temperatura: ");
      //requestTemperatures(void); for all
      sensors.requestTemperaturesByAddress(tempDeviceAddress);
      Serial.println(sensors.getTempC(tempDeviceAddress));
    }    
  }
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  Serial.print("{");
  for (uint8_t i = 0; i < 8; i++)
  {
    Serial.print("0x");
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
    if( i < 7)
      Serial.print(", ");
  }
  Serial.println("} ");
}

void loop() {
  // put your main code here, to run repeatedly:   
  Serial.print("Aktualna temperatura: ");
  sensors.requestTemperaturesByIndex(0);
  Serial.println(sensors.getTempCByIndex(0));  //Wyswietlenie informacji
  delay(5000);
}

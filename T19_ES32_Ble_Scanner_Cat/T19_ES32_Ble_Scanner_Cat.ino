/*
   Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleScan.cpp
   Ported to Arduino ESP32 by Evandro Copercini
*/

#define DEBUG_GK  // for tests
#define MIN_RSSI -80

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEAddress.h>

#include "DeviceDef.h"
#include "BLEScanner.h"

DeviceDef devices[] = {
  DeviceDef(1, new BLEAddress("CB:F7:92:0F:3B:2E"), "Test")
};

BLEScanner ScannerGK(&devices[0], sizeof(devices) / sizeof(*devices));

void setup() {
  Serial.begin(115200);
  Serial.println("Scanning...");
  ScannerGK.init();
}

void loop() {

  ScannerGK.scan();


  if(ScannerGK.isAuth()){
Serial.println("isAuth");
Serial.print("Id: ");
Serial.println(ScannerGK.getAuthDeviceId());

Serial.print("Name: ");
const char* name = ScannerGK.getAuthDeviceName();
Serial.println(name);
  }
}
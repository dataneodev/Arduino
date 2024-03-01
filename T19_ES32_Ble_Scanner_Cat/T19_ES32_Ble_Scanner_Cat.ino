/*
   Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleScan.cpp
   Ported to Arduino ESP32 by Evandro Copercini
*/

#define DEBUG_GK  // for tests

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEAddress.h>

#include "DeviceDef.h"

class BLEScannerGk {
public:
  BLEScannerGk(DeviceDef* dev)
    : _dev(dev) {

    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan();  //create new scan
    pBLEScan->setActiveScan(true);    //active scan uses more power, but get results faster
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);  // less or equal setInterval value
  }

  void scann() {
    uint32_t current = millis();

    if (current < _lastScann) {
      _lastScann = current;
      return;
    }

    if (current < _lastScann + 2000) {
      return;
    }

    _lastScann = current;
    _deviceId = 0;
    _rssi = 0;

    Serial.print("Definied devices: ");
    Serial.println(getBLEDevicesCount());

    BLEScanResults* foundDevices = pBLEScan->start(scanTime, true);

    int findedCount = foundDevices->getCount();

    if (findedCount > 0 && getBLEDevicesCount()) {
      for(int i = 0; i < findedCount; i++){
        BLEAdvertisedDevice  device = foundDevices->getDevice(i);
        BLEAddress adress = device.getAddress();
      }
    }
    Serial.print("Devices found: ");
    Serial.println(findedCount);
    Serial.println("Scan done!");



    pBLEScan->clearResults();  // delete results fromBLEScan buffer to release memory
  }

  int getBLEDevicesCount() {
    return sizeof(_dev) / sizeof(*_dev);
  }

private:
  DeviceDef* _dev;
  uint32_t _lastScann;

  int scanTime = 5;  //In seconds
  BLEScan* pBLEScan;

  int _deviceId;
  int _rssi;
};


DeviceDef devices[] = {
  DeviceDef(1, new BLEAddress("CB:F7:92:0F:3B:2E"), "Test")
};

BLEScannerGk ScannerGK(&devices[0]);



void setup() {
  Serial.begin(115200);
  Serial.println("Scanning...");
}

void loop() {
  ScannerGK.scann();
}
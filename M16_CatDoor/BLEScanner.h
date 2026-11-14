#pragma once

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

class BLEScanner {
public:
  static int scanTime; // W sekundach

  BLEScanner(Storage* storage) {
    _storage = storage;
  }

  // Lekka struktura do bezpiecznego przekazania 6 bajtów adresu MAC z wątku BLE do loop()
  struct ScannedDevice {
    uint8_t address[6];
    int rssi;
  };

  // Wątek BLE robi TU ABSOLUTNE MINIMUM - zero Serial.print i zero odczytu z Storage/Flash!
  class MyScanCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) override {
      if (_bufferCount >= MAX_SCAN_DEVICES) {
        return;
      }

      // W ESP32 Core 3.3.12 getNative() zwraca uint8_t*
      uint8_t* nativeAddr = advertisedDevice.getAddress().getNative();
      if (nativeAddr == nullptr) {
        return;
      }

      int rssi = advertisedDevice.getRSSI();

      // Jeśli ten adres jest już w buforze, zaktualizuj tylko jego najsilniejszy RSSI
      for (int i = 0; i < _bufferCount; i++) {
        if (memcmp(_buffer[i].address, nativeAddr, 6) == 0) {
          if (rssi > _buffer[i].rssi) {
            _buffer[i].rssi = rssi;
          }
          return;
        }
      }

      // Dodaj nowe urządzenie do bufora w RAM
      memcpy(_buffer[_bufferCount].address, nativeAddr, 6);
      _buffer[_bufferCount].rssi = rssi;
      _bufferCount++;
    }
  };

  // Wywołuj w loop() - tu bezpiecznie przetwarzamy wyniki, sprawdzamy Storage i wyłączamy BLE
  static bool isScanComplete() {
    if (_bleNeedsShutdown) {
      shutdownBLE();
      processResults(); // Przetwarzanie wyników w bezpiecznym wątku głównym loop()!
      _bleNeedsShutdown = false;
      _scanComplete = true;
      return true;
    }

    // Timeout bezpieczeństwa
    if (_isBleInitialized && !_scanComplete) {
      unsigned long maxDurationMs = ((unsigned long)scanTime * 1000UL) + 5500UL;
      if (millis() - _scanStartTime >= maxDurationMs) {
#if defined(DEBUG_GK)
        Serial.println("[BLE] Scan TIMEOUT! Forcing shutdown...");
#endif
        shutdownBLE();
        processResults();
        _bleNeedsShutdown = false;
        _scanComplete = true;
        return true;
      }
    }

    return _scanComplete;
  }

  static void scanCompleteCB(BLEScanResults foundDevices) {
    if (!_isBleInitialized) {
      return;
    }
    _bleNeedsShutdown = true;
  }

  static void scan() {
    _scanComplete = false;
    _bleNeedsShutdown = false;
    _lastRssi = 0;
    _lastDeviceId = 0;
    _bufferCount = 0;

    if (!_storage->isAuth()) {
      _scanComplete = true;
      return;
    }

    if (_isBleInitialized) {
      shutdownBLE();
    }

#if defined(DEBUG_GK)
    Serial.println("[BLE] Init stack & start scan...");
#endif

    BLEDevice::init("GK");
    _pBLEScan = BLEDevice::getScan();
    _pBLEScan->setAdvertisedDeviceCallbacks(&_callbacks, true);
    _pBLEScan->setActiveScan(true);
    _pBLEScan->setInterval(100);
    _pBLEScan->setWindow(99);

    _scanStartTime = millis();
    _isBleInitialized = true;

    bool started = _pBLEScan->start(scanTime, scanCompleteCB, false);
    if (!started) {
#if defined(DEBUG_GK)
      Serial.println("[BLE] Failed to start scan!");
#endif
      shutdownBLE();
      _scanComplete = true;
    }
  }

  bool isAuth() {
    if (!_storage->isAuth()) {
      return true;
    }
    return _lastDeviceId > 1;
  }

  int getAuthDeviceId() {
    if (!_storage->isAuth() || _lastDeviceId < 2) {
      return 1;
    }
    return _lastDeviceId;
  }

private:
  static constexpr int MAX_SCAN_DEVICES = 32;
  static ScannedDevice _buffer[MAX_SCAN_DEVICES];
  static volatile int _bufferCount;

  static Storage* _storage;
  static BLEScan* _pBLEScan;
  static MyScanCallbacks _callbacks;

  static volatile bool _scanComplete;
  static volatile bool _bleNeedsShutdown;
  static bool _isBleInitialized;
  static unsigned long _scanStartTime;

  static int _lastDeviceId;
  static int _lastRssi;

  // Przetwarzanie zebranych urządzeń w głównym wątku loop() (po wyłączeniu radia!)
  static void processResults() {
    int count = _bufferCount;

#if defined(DEBUG_GK)
    Serial.print("[BLE] Processing unique devices: ");
    Serial.println(count);
#endif

    for (int i = 0; i < count; i++) {
      BLEAddress adress(_buffer[i].address);
      int rssi = _buffer[i].rssi;

#if defined(DEBUG_GK)
      Serial.print("[BLE] Device: ");
      Serial.print(adress.toString().c_str());
      Serial.print(" | RSSI: ");
      Serial.println(rssi);
#endif

      int devId = getDeviceId(adress);
      if (devId == 0) {
        continue;
      }

      if (_lastDeviceId != 0 && rssi < _lastRssi) {
        continue;
      }

      if (rssi < _storage->getMinRSSI() * -1) {
#if defined(DEBUG_GK)
        Serial.print("[BLE] Weak RSSI Id: ");
        Serial.println(devId);
#endif
        continue;
      }

      _lastRssi = rssi;
      _lastDeviceId = devId;
    }

#if defined(DEBUG_GK)
    if (_lastDeviceId > 0) {
      Serial.print("[BLE] >>> AUTH MATCH Id: ");
      Serial.print(_lastDeviceId);
      Serial.print(" | RSSI: ");
      Serial.println(_lastRssi);
    }
#endif
  }

  static void shutdownBLE() {
    if (!_isBleInitialized) return;

    if (_pBLEScan != nullptr) {
      _pBLEScan->stop();
      _pBLEScan->clearResults();
      _pBLEScan = nullptr;
    }

    BLEDevice::deinit(false);
    _isBleInitialized = false;

#if defined(DEBUG_GK)
    Serial.println("[BLE] Radio & stack OFF.");
#endif
  }

  static int getDeviceId(BLEAddress address) {
    return _storage->getBleDeviceId(&address);
  }
};

// Inicjalizacja zmiennych statycznych
int BLEScanner::scanTime = 5;
BLEScanner::ScannedDevice BLEScanner::_buffer[BLEScanner::MAX_SCAN_DEVICES];
volatile int BLEScanner::_bufferCount = 0;
Storage* BLEScanner::_storage = nullptr;
BLEScan* BLEScanner::_pBLEScan = nullptr;
BLEScanner::MyScanCallbacks BLEScanner::_callbacks;
volatile bool BLEScanner::_scanComplete = false;
volatile bool BLEScanner::_bleNeedsShutdown = false;
bool BLEScanner::_isBleInitialized = false;
unsigned long BLEScanner::_scanStartTime = 0;
int BLEScanner::_lastDeviceId = 0;
int BLEScanner::_lastRssi = 0;

class BLEScanner {
public:
  static int scanTime;  //In seconds

  BLEScanner(Storage* storage) {
    _storage = storage;
  }

  bool isScanComplete() {
    return _scanComplete;
  }

  static void scanCompleteCB(BLEScanResults foundDevices) {
    int findedCount = foundDevices.getCount();

#if defined(DEBUG_GK)
    Serial.println("Start ble scan end");
    Serial.print("Found devices: ");
    Serial.println(findedCount);
#endif

    if (findedCount > 0) {
      for (int i = 0; i < findedCount; i++) {
        BLEAdvertisedDevice device = foundDevices.getDevice(i);
        BLEAddress adress = device.getAddress();

#if defined(DEBUG_GK)
        Serial.print("Device: ");
        Serial.println(adress.toString().c_str());
#endif

        int devId = getDeviceId(adress);

        if (devId == 0) {
          continue;
        }

        int rssi = device.getRSSI();

        if (_lastDeviceId != 0 && rssi < _lastRssi) {
          continue;
        }

        if (rssi < _storage->getMinRSSI() * -1) {
#if defined(DEBUG_GK)
          Serial.print("Found match device with weak RSSI Id: ");
          Serial.println(devId);

          Serial.print("RSSI: ");
          Serial.println(rssi);
#endif
          continue;
        }

        _lastRssi = rssi;
        _lastDeviceId = devId;
      }
    }

#if defined(DEBUG_GK)
    if (_lastDeviceId > 0) {
      Serial.print("Match device Id: ");
      Serial.println(_lastDeviceId);

      Serial.print("Match RSSI: ");
      Serial.println(_lastRssi);
    }
#endif

    foundDevices.dump();
    _scanComplete = true;
  }

  static void init() {
    BLEDevice::init("");
    _pBLEScan = BLEDevice::getScan();
    _pBLEScan->setActiveScan(true);
    _pBLEScan->setInterval(100);
    _pBLEScan->setWindow(99);
  }

  static void scan() {
    _scanComplete = false;
    _lastRssi = 0;
    _lastDeviceId = 0;

    if (!_storage->isAuth()) {
      return;
    }

#if defined(DEBUG_GK)
    Serial.println("Init ble scan");
#endif
    _pBLEScan->start(scanTime, scanCompleteCB, false);

#if defined(DEBUG_GK)
    Serial.println("Init ble scan end");
#endif
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
  static Storage* _storage;
  static BLEScan* _pBLEScan;

  static bool _scanComplete;
  static int _lastDeviceId;
  static int _lastRssi;

  static int getDeviceId(BLEAddress address) {
    return _storage->getBleDeviceId(&address);
  }
};

int BLEScanner::scanTime = 5;
Storage* BLEScanner::_storage = nullptr;
BLEScan* BLEScanner::_pBLEScan = nullptr;
bool BLEScanner::_scanComplete = false;
int BLEScanner::_lastDeviceId = 0;
int BLEScanner::_lastRssi = 0;
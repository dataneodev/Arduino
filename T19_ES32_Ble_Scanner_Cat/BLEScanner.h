class BLEScanner {
public:

  BLEScanner(DeviceDef* dev, int deviceCount) {
    _defindedDevices = dev;
    _defindedDevicesCount = deviceCount;
  }

  void init() {
    if (_defindedDevicesCount == 0) {
      return;
    }

    BLEDevice::init("");
    _pBLEScan = BLEDevice::getScan();
    _pBLEScan->setActiveScan(true);
    _pBLEScan->setInterval(100);
    _pBLEScan->setWindow(99);
  }

  void newScan(){
    _lastScan = o;
    _lastDeviceId = 0;
    _lastRssi = 0;

    scan() ;
  }

  void scan() {
    if (_defindedDevicesCount == 0) {
      return;
    }

    uint32_t current = millis();

    if (current < _lastScan) {
      _lastScan = current;
      return;
    }

    if (current < _lastScan + scanPeriod) {
      return;
    }

#if defined(DEBUG_GK)
    Serial.println("BLE scan start");
#endif  

    _pBLEScan->start(scanTime, scanCompleted);

#if defined(DEBUG_GK)
    Serial.println("BLE scan ended");
#endif
  }

  bool isAuth() {
    if (_defindedDevicesCount == 0) {
      return true;
    }

    if (millis() > _lastScan + scanPeriod * 2) {
      return false;
    }

    return BLEScanner::_lastDeviceId > 1;
  }

  int getAuthDeviceId() {
    if (_defindedDevicesCount == 0 || BLEScanner::_lastDeviceId < 2) {
      return 1;
    }

    return BLEScanner::_lastDeviceId;
  }

  const char* getAuthDeviceName() {
    if (_defindedDevicesCount == 0 || BLEScanner::_lastDeviceId < 2) {
      return "dsfsdf";
    }

    for (int i = 0; i < _defindedDevicesCount; i++) {
      DeviceDef* curr = _defindedDevices + i * sizeof(DeviceDef);

      if (curr->GetId() == BLEScanner::_lastDeviceId) {
        return curr->GetName();
      }
    }

    return "dsfsdf";
  }

private:
  static int _defindedDevicesCount;
  static DeviceDef* _defindedDevices;

  int scanPeriod = 3000;  // milisecond
  int scanTime = 2;       //In seconds

  BLEScan* _pBLEScan;

  uint32_t _lastScan;
  static int _lastDeviceId;
  static int _lastRssi;

  static void scanCompleted(BLEScanResults foundDevices) {
    int findedCount = foundDevices.getCount();

#if defined(DEBUG_GK)
    Serial.println("Scan completed!");
    Serial.print("Devices found: ");
    Serial.println(findedCount);
#endif

    if (findedCount > 0) {
      for (int i = 0; i < findedCount; i++) {
        BLEAdvertisedDevice device = foundDevices.getDevice(i);
        BLEAddress adress = device.getAddress();

        int devId = getDeviceId(adress);
        if (devId == 0) {
          continue;
        }

        int rssi = device.getRSSI();

        if (_lastDeviceId != 0 && rssi < _lastRssi) {
          continue;
        }

        if (rssi < MIN_RSSI) {
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
  }

  static int getDeviceId(BLEAddress address) {
    if (_defindedDevicesCount == 0) {
      return 0;
    }

    for (int i = 0; i < _defindedDevicesCount; i++) {
      DeviceDef* curr = _defindedDevices + i * sizeof(DeviceDef);

      if (curr->IsEquals(&address)) {
        return curr->GetId();
      }
    }

    return 0;
  }
};

int BLEScanner::_lastDeviceId = 0;
int BLEScanner::_lastRssi = 0;
int BLEScanner::_defindedDevicesCount = 0;
DeviceDef* BLEScanner::_defindedDevices = nullptr;
class BLEScanner : public BLEAdvertisedDeviceCallbacks {
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
   // _pBLEScan->setAdvertisedDeviceCallbacks(this);
    _pBLEScan->setActiveScan(true);
    _pBLEScan->setInterval(100);
    _pBLEScan->setWindow(99);
  }

  int getDefindedDevicesCount() {
    return _defindedDevicesCount;
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

    _lastScan = current;
    _lastRssi = 0;
    _lastDeviceId = 0;

#if defined(DEBUG_GK)
    Serial.println("Start ble scan");
#endif

    BLEScanResults* foundDevices = _pBLEScan->start(scanTime, false);

    int findedCount = foundDevices->getCount();

#if defined(DEBUG_GK)
    Serial.println("Start ble scan end");
    Serial.print("Found devices: ");
    Serial.println(findedCount);
#endif

    if (findedCount > 0) {
      for (int i = 0; i < findedCount; i++) {
        BLEAdvertisedDevice device = foundDevices->getDevice(i);
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
    _pBLEScan->clearResults();
  }

  void onResult(BLEAdvertisedDevice advertisedDevice) {
    BLEAddress adress = advertisedDevice.getAddress();

#if defined(DEBUG_GK)
    Serial.print("BLEAdvertised device: ");
    Serial.println(adress.toString().c_str());
#endif

    int devId = getDeviceId(adress);
    if (devId == 0) {
       return;
    }

    int rssi = advertisedDevice.getRSSI();

    if (_lastDeviceId != 0 && rssi < _lastRssi) {
      return;
    }

    if (rssi < MIN_RSSI) {
#if defined(DEBUG_GK)
      Serial.print("Found match device with weak RSSI Id: ");
      Serial.println(devId);

      Serial.print("RSSI: ");
      Serial.println(rssi);
#endif
       return;
    }

    _lastRssi = rssi;
    _lastDeviceId = devId;
  }

  bool isAuth() {
    if (_defindedDevicesCount == 0) {
      return true;
    }

    return _lastDeviceId > 1;
  }

  int getAuthDeviceId() {
    if (_defindedDevicesCount == 0 || _lastDeviceId < 2) {
      return 1;
    }

    return _lastDeviceId;
  }

  const char* getAuthDeviceName() {
    if (_defindedDevicesCount == 0 || _lastDeviceId < 2) {
      return SKETCH_NAME;
    }

    for (int i = 0; i < _defindedDevicesCount; i++) {
      DeviceDef curr = _defindedDevices[i];

      if (curr.GetId() == _lastDeviceId) {
        return curr.GetName();
      }
    }

    return SKETCH_NAME;
  }

private:
  int _defindedDevicesCount;
  DeviceDef* _defindedDevices;

  int scanPeriod = 7000;  // milisecond
  int scanTime = 4;       //In seconds

  BLEScan* _pBLEScan;

  uint32_t _lastScan;
  int _lastDeviceId;
  int _lastRssi;

  int getDeviceId(BLEAddress address) {
    if (_defindedDevicesCount == 0) {
      return 0;
    }

    for (int i = 0; i < _defindedDevicesCount; i++) {
      DeviceDef curr = _defindedDevices[i];

      if (curr.IsEquals(&address)) {
        return curr.GetId();
      }
    }

    return 0;
  }
};
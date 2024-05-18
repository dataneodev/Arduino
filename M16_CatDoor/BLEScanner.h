class BLEScanner {
public:
  static int scanTime;  //In seconds

  BLEScanner(DeviceDef* dev, int deviceCount) {
    _defindedDevices = dev;
    _defindedDevicesCount = deviceCount;
  }

  static bool isAnyDeviceDefined() {
    return _defindedDevicesCount > 0;
  }

  int getDefindedDevicesCount() {
    return _defindedDevicesCount;
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

    if (!BLEScanner::isAnyDeviceDefined()) {
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
    if (!BLEScanner::isAnyDeviceDefined()) {
      return true;
    }

    return _lastDeviceId > 1;
  }

  int getAuthDeviceId() {
    if (!BLEScanner::isAnyDeviceDefined() || _lastDeviceId < 2) {
      return 1;
    }

    return _lastDeviceId;
  }

  const char* getAuthDeviceName() {
    if (!BLEScanner::isAnyDeviceDefined() || _lastDeviceId < 2) {
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
  static int _defindedDevicesCount;
  static DeviceDef* _defindedDevices;

  static BLEScan* _pBLEScan;

  static bool _scanComplete;
  static int _lastDeviceId;
  static int _lastRssi;

  static int getDeviceId(BLEAddress address) {
    if (!BLEScanner::isAnyDeviceDefined()) {
      return 0;
    }

    for (int i = 0; i < BLEScanner::_defindedDevicesCount; i++) {
      DeviceDef curr = BLEScanner::_defindedDevices[i];

      if (curr.IsEquals(&address)) {
        return curr.GetId();
      }
    }

    return 0;
  }
};

int BLEScanner::scanTime = 5;
int BLEScanner::_defindedDevicesCount = 0;
DeviceDef* BLEScanner::_defindedDevices = nullptr;
BLEScan* BLEScanner::_pBLEScan = nullptr;
bool BLEScanner::_scanComplete = false;
int BLEScanner::_lastDeviceId = 0;
int BLEScanner::_lastRssi = 0;
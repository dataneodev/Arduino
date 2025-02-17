#include "24C32.h"
#include "DeviceStorage.h"

#define TRUE_VALUE 0x55
#define FALSE_VALUE 0x45

class Storage {
public:
  Storage(EE* eprom)
    : EEPROM24C32(eprom) {}

  void Inicjalize() {
    EEPROM24C32->begin(0x50, false);

    _isInicjalized = EEPROM24C32->checkPresence();

    if (_isInicjalized) {
      readAll();
    }
  }

  bool IsInicjalized() {
    return _isInicjalized;
  }

  bool isDoorOpen() {
    return _isDoorOpen;
  }

  bool isDoorAlwaysOpen() {
    return _alwaysOpen;
  }

  bool isDoorAlwaysClose() {
    return _alwaysClose;
  }

  bool useAthorizationBle() {
    return _useBleAuth;
  }

  uint32_t getDoorOpenCount() {
    return _doorOpenCount;
  }

  bool useLight() {
    return _useLight;
  }

  void setDoorOpen(bool isOpen) {
    _isDoorOpen = isOpen;
    EEPROM24C32->writeByte(106, isOpen ? TRUE_VALUE : FALSE_VALUE, false, false);

    if (isOpen) {
      _doorOpenCount += 1;
      EEPROM24C32->writeUInt32(110, _doorOpenCount, false, false);
    }
  }

  void setDoorAlwaysOpen(bool isAlwaysOpen) {
    _alwaysOpen = isAlwaysOpen;
    EEPROM24C32->writeByte(114, isAlwaysOpen ? TRUE_VALUE : FALSE_VALUE, false, false);
  }

  void setDoorAlwaysClose(bool isAlwaysClose) {
    _alwaysClose = isAlwaysClose;
    EEPROM24C32->writeByte(115, isAlwaysClose ? TRUE_VALUE : FALSE_VALUE, false, false);
  }

  void setAthorizationBle(bool bleAuth) {
    _useBleAuth = bleAuth;
    EEPROM24C32->writeByte(116, bleAuth ? TRUE_VALUE : FALSE_VALUE, false, false);
  }

  void setLight(bool light) {
    _useLight = light;
    EEPROM24C32->writeByte(117, light ? TRUE_VALUE : FALSE_VALUE, false, false);
  }

  void setMinRSSI(uint8_t minRssi) {
    if (minRssi > 100) {
      minRssi = 100;
    }

    if (minRssi < 5) {
      minRssi = 5;
    }

    _minRSSI = minRssi;
    EEPROM24C32->writeByte(118, minRssi, false, false);
  }

  void setDoorLockTime(uint32_t lockTime) {
    if (lockTime > 3600) {
      lockTime = 3600;
    }

    if (lockTime < 0) {
      lockTime = 0;
    }

    _doorLockTime = lockTime;
    EEPROM24C32->writeUInt32(122, lockTime, false, false);
  }

  uint8_t getBleDevicesCount() {
    return _deviceCount;
  }

  uint8_t getBleDeviceId(BLEAddress* address) {
    for (int i = 0; i < _deviceCount; i++) {
      if (!_devices[i].isEquals(address) || !_devices[i].isEnabled()) {
        continue;
      }

      return _devices[i].getId();
    }

    return 0;
  }

  void addNewBleAddress(BLEAddress* address) {
    for (int i = 0; i < _deviceCount; i++) {
      if (_devices[i].isEquals(address)) {

        if (!isBleEnabled(i)) {
          _devices[i].setEnabled(true);
          writeBleEnabled(i, true);
        }

        return;
      }
    }

    uint8_t availableId = getAvailableId();
    if (availableId == 255) {
      return;
    }

    esp_bd_addr_t* adr = address->getNative();

    uint8_t a[6];
    memcpy(&a, adr, 6);

    _devices[availableId].setNewAddress(a[0], a[1], a[2], a[3], a[4], a[5]);
    _devices[availableId].setEnabled(true);

    writeBleAddress(availableId, a[0], a[1], a[2], a[3], a[4], a[5]);
    writeBleEnabled(availableId, true);
  }

  void deleteBleDevice(BLEAddress* address) {
    for (int i = 0; i < _deviceCount; i++) {
      if (!_devices[i].isEquals(address)) {
        continue;
      }

      if (!isBleEnabled(i)) {
        return;
      }

      _devices[i].setNewAddress(255, 255, 255, 255, 255, 255);
      _devices[i].setEnabled(false);
      
      writeBleAddress(i, 255, 255, 255, 255, 255, 255);
      writeBleEnabled(i, false);
    }
  }

  bool isBleEnabled(uint8_t lp) {
    if (lp > _deviceCount) {
      return false;
    }

    return _devices[lp].isEnabled();
  }

  uint8_t getBleId(uint8_t lp) {
    if (lp > _deviceCount) {
      return false;
    }

    return _devices[lp].getId();
  }

  BLEAddress* getBleAddress(uint8_t lp) {
    return _devices[lp].getAddress();
  }

  bool isAnyDeviceDefined() {
    for (int i = 0; i < _deviceCount; i++) {
      if (_devices[i].isEnabled()) {
        return true;
      }
    }

    return false;
  }

  bool isAuth() {
    return useAthorizationBle() && isAnyDeviceDefined();
  }

  uint8_t getMinRSSI() {
    return _minRSSI;
  }

  uint32_t getDoorLockTime() {
    return _doorLockTime;
  }

private:
  EE* EEPROM24C32;

  bool _isInicjalized = false;

  uint32_t _doorOpenCount = 0;
  bool _isDoorOpen = false;
  bool _alwaysClose = false;
  bool _alwaysOpen = false;
  bool _useBleAuth = false;
  bool _useLight = true;
  uint8_t _minRSSI = 60;
  uint32_t _doorLockTime = 0;

  static const uint8_t _deviceCount = 10;

  DeviceStorage _devices[_deviceCount] = {
    DeviceStorage(2, false, new BLEAddress("FF:FF:FF:FF:FF:FF")),
    DeviceStorage(3, false, new BLEAddress("FF:FF:FF:FF:FF:FF")),
    DeviceStorage(4, false, new BLEAddress("FF:FF:FF:FF:FF:FF")),
    DeviceStorage(5, false, new BLEAddress("FF:FF:FF:FF:FF:FF")),
    DeviceStorage(6, false, new BLEAddress("FF:FF:FF:FF:FF:FF")),
    DeviceStorage(7, false, new BLEAddress("FF:FF:FF:FF:FF:FF")),
    DeviceStorage(8, false, new BLEAddress("FF:FF:FF:FF:FF:FF")),
    DeviceStorage(9, false, new BLEAddress("FF:FF:FF:FF:FF:FF")),
    DeviceStorage(10, false, new BLEAddress("FF:FF:FF:FF:FF:FF")),
    DeviceStorage(11, false, new BLEAddress("FF:FF:FF:FF:FF:FF")),
  };

  void readAll() {
    if (EEPROM24C32->readByte(105) != CHECK_NUMBER) {
#if defined(DEBUG_GK)
      Serial.println("RESET SETTINGS");
#endif
      EEPROM24C32->writeByte(105, CHECK_NUMBER, false, false);  //check value
      EEPROM24C32->writeByte(106, FALSE_VALUE, false, false);   // is door open

      EEPROM24C32->writeUInt32(110, 0, false, false);  // open door count

      EEPROM24C32->writeByte(114, FALSE_VALUE, false, false);  // open always door
      EEPROM24C32->writeByte(115, FALSE_VALUE, false, false);  // close always door
      EEPROM24C32->writeByte(116, FALSE_VALUE, false, false);  // auth
      EEPROM24C32->writeByte(117, TRUE_VALUE, false, false);   // light
      EEPROM24C32->writeByte(118, 60, false, false);           // min RSSI
      EEPROM24C32->writeUInt32(122, 0, false, false);          // open door count

      writeDefaultBle();  //ble
    }

#if defined(DEBUG_GK)
    Serial.println("READ SETTINGS");
#endif

    _isDoorOpen = EEPROM24C32->readByte(106) == TRUE_VALUE;
    _doorOpenCount = EEPROM24C32->readUInt32(110);
    _alwaysOpen = EEPROM24C32->readByte(114) == TRUE_VALUE;
    _alwaysClose = EEPROM24C32->readByte(115) == TRUE_VALUE;
    _useBleAuth = EEPROM24C32->readByte(116) == TRUE_VALUE;
    _useLight = EEPROM24C32->readByte(117) == TRUE_VALUE;
    _minRSSI = EEPROM24C32->readByte(118);
    _doorLockTime = EEPROM24C32->readUInt32(122);

    readBleAllAddress();

#if defined(DEBUG_GK)
    Serial.print("Door open count:");
    Serial.println(_doorOpenCount);
#endif
  }

  void readBleAllAddress() {
    for (int i = 0; i < _deviceCount; i++) {
      readBleAddress(i);
    }
  }

  void readBleAddress(uint8_t lp) {
    uint16_t addressStart = 200 + lp * 50;

    bool enabled = EEPROM24C32->readByte(addressStart) == TRUE_VALUE;

    uint8_t a1 = EEPROM24C32->readByte(addressStart + 1);
    uint8_t a2 = EEPROM24C32->readByte(addressStart + 2);
    uint8_t a3 = EEPROM24C32->readByte(addressStart + 3);
    uint8_t a4 = EEPROM24C32->readByte(addressStart + 4);
    uint8_t a5 = EEPROM24C32->readByte(addressStart + 5);
    uint8_t a6 = EEPROM24C32->readByte(addressStart + 6);

    _devices[lp].setNewAddress(a1, a2, a3, a4, a5, a6);
    _devices[lp].setEnabled(enabled);
  }

  void writeDefaultBle() {
    for (int i = 0; i < _deviceCount; i++) {
      writeBleAddress(i, 255, 255, 255, 255, 255, 255);
      writeBleEnabled(i, false);
    }
  }

  void writeBleAddress(uint8_t lp, uint8_t a1, uint8_t a2, uint8_t a3, uint8_t a4, uint8_t a5, uint8_t a6) {
    uint16_t addressStart = 200 + lp * 50;

    EEPROM24C32->writeByte(addressStart + 1, a1, false, false);
    EEPROM24C32->writeByte(addressStart + 2, a2, false, false);
    EEPROM24C32->writeByte(addressStart + 3, a3, false, false);
    EEPROM24C32->writeByte(addressStart + 4, a4, false, false);
    EEPROM24C32->writeByte(addressStart + 5, a5, false, false);
    EEPROM24C32->writeByte(addressStart + 6, a6, false, false);
  }

  void writeBleEnabled(uint8_t lp, bool enabled) {
    uint16_t addressStart = 200 + lp * 50;

    EEPROM24C32->writeByte(addressStart, enabled ? TRUE_VALUE : FALSE_VALUE, false, false);
  }

  uint8_t getAvailableId() {

    for (int i = 0; i < _deviceCount; i++) {
      if (!_devices[i].isEnabled()) {
        return i;
      }
    }

    return 255;
  }
};
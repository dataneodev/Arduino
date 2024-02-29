#include "24C32.h"



#define TRUE_VALUE 0x55
#define FALSE_VALUE 0x45

class Storage {
public:
  Storage(EE *eprom)
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

  void setDoorOpen(bool isOpen) {
    if (!_isInicjalized) {
      return;
    }

    _isDoorOpen = isOpen;
    EEPROM24C32->writeByte(106, isOpen ? TRUE_VALUE : FALSE_VALUE, false, false);

    if (isOpen) {
      _doorOpenCount += 1;
      EEPROM24C32->writeUInt32(110, _doorOpenCount, false, false);
    }
  }

  void setDoorAlwaysOpen(bool isAlwaysOpen) {
    if (!_isInicjalized) {
      return;
    }

    _alwaysOpen = isAlwaysOpen;
    EEPROM24C32->writeByte(114, isAlwaysOpen ? TRUE_VALUE : FALSE_VALUE, false, false);
  }


  void setDoorAlwaysClose(bool isAlwaysClose) {
    if (!_isInicjalized) {
      return;
    }

    _alwaysClose = isAlwaysClose;
    EEPROM24C32->writeByte(115, isAlwaysClose ? TRUE_VALUE : FALSE_VALUE, false, false);
  }

  void setAthorizationBle(bool bleAuth) {
    if (!_isInicjalized) {
      return;
    }

    _useBleAuth = bleAuth;
    EEPROM24C32->writeByte(116, bleAuth ? TRUE_VALUE : FALSE_VALUE, false, false);
  }

private:
  EE *EEPROM24C32;

  bool _isInicjalized = false;

  uint32_t _doorOpenCount = 0;
  bool _isDoorOpen = false;
  bool _alwaysClose = false;
  bool _alwaysOpen = false;
  bool _useBleAuth = false;

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

#ifdef BLE_AUTH
      EEPROM24C32->writeByte(116, TRUE_VALUE, false, false);  // auth
#else
      EEPROM24C32->writeByte(116, FALSE_VALUE, false, false);  // auth
#endif
    }

#if defined(DEBUG_GK)
    Serial.println("READ SETTINGS");
#endif

    _isDoorOpen = EEPROM24C32->readByte(106) == TRUE_VALUE;
    _doorOpenCount = EEPROM24C32->readUInt32(110) ;
    _alwaysOpen = EEPROM24C32->readByte(114) == TRUE_VALUE;
    _alwaysClose = EEPROM24C32->readByte(115) == TRUE_VALUE;
    _useBleAuth = EEPROM24C32->readByte(116) == TRUE_VALUE;

#if defined(DEBUG_GK)
    Serial.print("Door open count:");
    Serial.println(_doorOpenCount);
#endif
  }
};
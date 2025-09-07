#include <BLEAddress.h>

class DeviceStorage {
public:
  DeviceStorage(int id, bool enable, BLEAddress* address)
    : _address(address) {
    _id = id;
    _enable = enable;
  }

  bool isEquals(BLEAddress* address) {
    return address->equals(*_address);
  }

  int getId() {
    return _id;
  }

  bool isEqualsEditNo(uint8_t no) {
    return getEditNoId() == no;
  }

  int getEditNoId() {
    return _id + MS_DOOR_EDIT_START_ID;
  }

  BLEAddress* getAddress() {
    return _address;
  }

  bool isEnabled() {
    return _enable;
  }

  void setNewAddress(uint8_t a1, uint8_t a2, uint8_t a3, uint8_t a4, uint8_t a5, uint8_t a6) {
    delete _address;

    esp_bd_addr_t addr = { a1, a2, a3, a4, a5, a6 };

    _address = new BLEAddress(addr);
  }

  void setEnabled(bool enable) {
    _enable = enable;
  }

  void deleteDevice(BLEAddress* address) {
    _enable = false;
  }

private:
  int _id;
  bool _enable;
  BLEAddress* _address;
};
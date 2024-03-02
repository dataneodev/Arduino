#include <BLEAddress.h>

class DeviceDef {
public:
  DeviceDef(int id, BLEAddress* address, const char* _name)
    : _address(address) {
    _id = id + 1;
    _name = _name;
  }

bool IsEquals(BLEAddress* address){
  return address->equals(*_address);
}

int GetId(){
  return _id;
}

const char* GetName(){
  return _name;
}

private:
  int _id;
  BLEAddress* _address;
  const char* _name;
};
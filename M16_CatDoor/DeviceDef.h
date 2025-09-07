#include <BLEAddress.h>

class DeviceDef {
public:


  DeviceDef(int id, BLEAddress* address, const char* name)
    : _address(address) {
    _id = id + 1;
    _name = name;
  }

bool IsEquals(BLEAddress* address){
  return address->equals(*_address);
}

bool IsEqualsEditNo(uint8_t no){
  return GetEditNo() == no;
}

int GetEditNo(){
  return _id + MS_DOOR_EDIT_START_ID;
}

int GetId(){
  return _id;
}

const char* GetName(){
  return _name;
}

private:
  int _id;
  const char * _name;
  BLEAddress* _address;
  
};
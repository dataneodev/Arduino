class DeviceDef {
  public:
    
  int Id;
  int BLEAddres[4];
  const char* Name;

  DeviceDef(int _id, int ble1, int ble2,int ble3,int ble4,const char* _name){
      Id = _id;

      BLEAddres[0] = ble1;
      BLEAddres[1] = ble2;
      BLEAddres[2] = ble3;
      BLEAddres[3] = ble4;

      Name = _name;
    }
} ;
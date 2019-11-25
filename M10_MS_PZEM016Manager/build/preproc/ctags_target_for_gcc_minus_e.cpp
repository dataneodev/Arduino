# 1 "i:\\7.Projekty\\5.Arduino\\M10_MS_PZEM016Manager\\M10_MS_PZEM016Manager.ino"
/*

   dataneo @2019 - M10_MS_PRZEM016Manager

   MySensors PZEM016 Manager 1.0

   see https://sites.google.com/site/dataneosoftware/arduino/mysensors-pzem016-manager

*/
# 9 "i:\\7.Projekty\\5.Arduino\\M10_MS_PZEM016Manager\\M10_MS_PZEM016Manager.ino"
# 10 "i:\\7.Projekty\\5.Arduino\\M10_MS_PZEM016Manager\\M10_MS_PZEM016Manager.ino" 2

/*

  Configuration:



  PZEM016Manager(HardwareSerial &serial, uint8_t txRxChangePin = 0, uint8_t pzem016Check = 2, uint8_t controllerReport = 30) ;

  serial - communication serial

  txRxChangePin - pinout for control rx/tx; use 0 for disable

  pzem016Check - time in seconds between read PZEM016

  controllerReport - time in seonds between send raport to controller

*/
# 21 "i:\\7.Projekty\\5.Arduino\\M10_MS_PZEM016Manager\\M10_MS_PZEM016Manager.ino"
PZEM016Manager PZEM016ManagerObj = PZEM016Manager(Serial1, 10, 2, 10);
/*  End of M10_MS_PRZEM016Manager */

void before()
{
  //void addPZEM016Device(uint8_t pzemSlaveAddr, bool presentToController = true, bool multimeterToController = false, const char* PZEM016Name = "")
  PZEM016ManagerObj.addPZEM016Device(1, true, true, "PZEM016 Grupa Testowa"); //M10_MS_PRZEM016Manager
  //PZEM016ManagerObj.addPZEM016Device(2, true, true, "PZEM016 Test"); //M10_MS_PRZEM016Manager

  //void addGroup(uint8_t groupNo, uint8_t pzemAddr)
  // first device in group define name, setting and events
  PZEM016ManagerObj.addGroup(1, 1);
}

void setup() {}

void presentation()
{
  sendSketchInfo("PZEM016 Sensor Manager", "1.1");
  PZEM016ManagerObj.presentAllToControler(); // M10_MS_PRZEM016Manager
}

void loop()
{
  PZEM016ManagerObj.checkPZEM(); // M10_MS_PRZEM016Manager
}

void receive(const MyMessage &message) {}

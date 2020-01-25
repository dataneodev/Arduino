# 1 "i:\\7.Projekty\\5.Arduino\\M12_MS_Heater\\M12_MS_Heater.ino"
/*

   dataneo @2019

   MySensors Heater 1.0

*/
# 9 "i:\\7.Projekty\\5.Arduino\\M12_MS_Heater\\M12_MS_Heater.ino" 2
# 10 "i:\\7.Projekty\\5.Arduino\\M12_MS_Heater\\M12_MS_Heater.ino" 2
# 11 "i:\\7.Projekty\\5.Arduino\\M12_MS_Heater\\M12_MS_Heater.ino" 2

PZEM016Manager PZEM016ManagerObj = PZEM016Manager(Serial1, 10, 2, 10);

void before()
{
}

void setup() {}

void presentation()
{
    sendSketchInfo("MySensors House Heater", "1.0");
}

void loop()
{
}

void receive(const MyMessage &message) {}

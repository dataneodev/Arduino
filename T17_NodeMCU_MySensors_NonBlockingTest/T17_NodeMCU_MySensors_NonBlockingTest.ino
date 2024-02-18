//RS485
//#define MY_DISABLED_SERIAL         //manual configure Serial1
#define MY_RS485                   // Enable RS485 transport layer
#define MY_RS485_DE_PIN 22         // Define this to enables DE-pin management on defined pin
#define MY_RS485_BAUD_RATE 9600    // Set RS485 baud rate to use
#define MY_RS485_HWSERIAL Serial  //
#define MY_TRANSPORT_WAIT_READY_MS 1

#define MY_DISABLED_SERIAL
//#define MY_GATEWAY_SERIAL

#include <MySensors.h>

MyMessage mMessage;

uint beforeExe = 0;
void before() {

//  Serial.println("before");

  beforeExe = millis();
}

uint preHwInitExe = 0;
void preHwInit() {

   // Serial.begin(115200);
  Serial.println("preHwInit");

  preHwInitExe = millis();
}

uint presentationExe = 0;
void presentation()  //MySensors
{
  Serial.println("presentation");
  sendSketchInfo("T17_NodeMCU_MySensors_NonBlockingTest", "1.0");

  presentationExe = millis();
}


uint setupExe = 0;
void setup() {
  setupExe = millis();


  Serial.println("T17_NodeMCU_MySensors_NonBlockingTest");

  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Loop");

  Serial.print("preHwInit execute: ");
  Serial.println(preHwInitExe);

  Serial.print("beforeExe execute: ");
  Serial.println(beforeExe);

  Serial.print("setupExe execute: ");
  Serial.println(setupExe);

  Serial.print("presentationExe execute: ");
  Serial.println(presentationExe);

    mMessage.setSensor(1);
  send(mMessage.set("0"));

  sleep(4000);
}

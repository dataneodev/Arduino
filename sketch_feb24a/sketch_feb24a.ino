
#include "esp_random.h"  //brakuje w plikach mysensors dla esp32, sprawdzicz y mozna usunąć w nowych wersjach

#define OPEN 1
#define CLOSE 0
#define CHILD_ID 1
#define MY_GATEWAY_SERIAL
#define MY_DISABLED_SERIAL         // manual configure Serial1

#include <MySensors.h>

MyMessage msg(1, V_TRIPPED);

uint8_t value = OPEN;

void presentation()
{
  present(1, S_DOOR);
}

void loop()
{

}

void setup() {
  // put your setup code here, to run once:

}


void receive(const MyMessage &message) {

}
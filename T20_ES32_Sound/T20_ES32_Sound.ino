#define SOFTWARE_VERION "1.0"
#define SKETCH_NAME "M15_Cat_Door"

#pragma region INSTALATION

#include "esp_random.h"  //brakuje w plikach mysensors dla esp32, sprawdzicz y mozna usunąć w nowych wersjach
/*

dla ESP32-C6 zakomentować w pliku:
C:\Users\Smith\Documents\Arduino\libraries\MySensors\hal\architecture\ESP32\MyHwESP32.h

static __inline__ void __psRestore(const uint32_t *__s)
{
  //XTOS_RESTORE_INTLEVEL(*__s);
}

*/
#pragma endregion INSTALATION

#pragma region CONFIGURATION



#define ALARM_ENABLED  // w przypadku błędow uruchamiać alarm dzwiękowy
//#define OUT_2_ENABLED  // czy dioda 2 jest zainstalowana - czerwona błędu - inne zachowanie jak są 2 diody

#define BLE_AUTH  // autoryzacja ble wymagana aby otworzyć drzwi - sterowane przez mysensors, aby zmienic trzeba

#define USE_M1_M2_ON_DOOR_CLOSING  // czy wykrycie ruchy przez m1 i m2 także przerywa zamykanie drzwi

#define MOTION_1_DELAY 4 * 1000       // czas pomiędzy pierwszym wykryciem ruchu a kolejnym wykryciem uruchamiajacym otwarcie drzwi dla sensoru 1,
#define MOTION_1_DELAY_WAIT 4 * 1000  // czas oczekiwania na 2 wykrycie ruchu dla sensoru 1,

#define MOTION_2_DELAY 4 * 1000       // czas pomiędzy pierwszym wykryciem ruchu a wykryciem uruchamiajacym otwarcie dla sensoru 2,
#define MOTION_2_DELAY_WAIT 4 * 1000  // czas oczekiwania na 2 wykrycie ruchu dla sensoru 2,

#define OPENING_DOOR_TIME 11 * 1000                 // czas otwierania drzwi
#define OPEN_DOOR_TIME 10 * 1000                    // czas oczekiwania na zamknięcie drzwi od ostatnieo wykrycia ruchu
#define TO_LONG_OPEN_DOOR_TIME 100 * 1000           // czas zbyt długiego otwarcia drzwi aby włączyc alarm
#define TIME_SLEEP_AFTER_LAST_DETECTION 120 * 1000  // czas przejscia w deep sleep od ostatniego wykrycia ruchu
#define DOOR_INTERRUPTED_WAITING 4 * 1000           // czas zatrzymania w przypadku wykrycia ruchy przy zamykaniu - po tym czasie następuje otwarcie

#define MY_NODE_ID 95  // id wezła dla my sensors

#define CHECK_NUMBER 0x68  //zmienic aby zresetować ustawienia zapisane w pamięci
#define DEBUG_GK           // for tests
#define FADE 2
#define FADE_OFF 100000
#define MIN_RSSI -60
#pragma endregion CONFIGURATION

#pragma region BOARD_PIN_CONFIGURATION
#define MOTION_SENSOR_1_PIN 20
#define MOTION_SENSOR_2_PIN 19
#define MOTION_SENSOR_3_PIN 18

#define OUPUT_1_PIN 5
#define OUPUT_2_PIN 0
#define OUPUT_3_PIN 1

#define POWER_PIN 2

#define OPEN_DOOR_PIN 10   // pin otwarcia
#define CLOSE_DOOR_PIN 11  // pin zamkniecia

#define SDA_PIN 6
#define SCL_PIN 7

#define RX_PIN 21
#define TX_PIN 23
#pragma endregion BOARD_PIN_CONFIGURATION

#pragma region MY_SENSORS_CONFIGURATION
// RS485
#define MY_DISABLED_SERIAL         // manual configure Serial1
#define MY_RS485                   // Enable RS485 transport layer
#define MY_RS485_DE_PIN 22         // Define this to enables DE-pin management on defined pin
#define MY_RS485_BAUD_RATE 9600    // Set RS485 baud rate to use
#define MY_RS485_HWSERIAL Serial1  //
#define MY_TRANSPORT_WAIT_READY_MS 1

#define MS_OPEN_DOOR_COUNT_ID 20
#define MS_OPEN_DOOR_ID 21
#define MS_CLOSE_DOOR_ID 22
#define MS_AUTH_BLE_ID 23
#pragma endregion MY_SENSORS_CONFIGURATION

#pragma region GLOBAL_VARIABLE

#include "Blinkenlight.h"
#include "Fadinglight.h"


Blinkenlight Out3(OUPUT_3_PIN);

#pragma endregion GLOBAL_VARIABLE

#pragma region MAIN
void setup() {
  Serial.begin(115200);
}

const SpeedSetting s1 = {
  .on_ms = 150,
    .off_ms = 150,
    .pause_ms = 200,
    .ending_ms = 400,
};

bool first = false;
bool sec = false;
void loop() {
  if (!first) {
    first = true;
    Serial.println("Sound 1");

    Out3.pattern(2, s1, false);
  }


  if (!sec && millis() > 10000) {
    Serial.println("Sound 2");
    sec = true;


    Out3.pattern(3, s1, false);
  }

  Out3.update();
}

#pragma endregion MAIN

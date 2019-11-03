//#include <registers.h>
//#include <pin_magic.h>

#include <Adafruit_TFTLCD.h>
#include <Adafruit_GFX.h>

#define LCD_CS D3
#define LCD_WR D1
#define LCD_RD D5


#define LCD_CD D2


#define LCD_RESET D4

#define RED 0xF800
#define BLUE 0x001F


//Tworzymy instancję wyswietlacza:
Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

void setup(){

  tft.reset();
  tft.begin(0x154);

  tft.fillScreen(RED);
}

void loop(){
  //sleep(5000);
  //tft.fillRect(50, 50, 100, 100, BLUE); 
}

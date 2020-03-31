#ifndef MAIN_H
#define MAIN_H

class MainScreen
{
public:
    MainScreen(Adafruit_TFTLCD *gfx, DataLayer *dataLayer) : _gfx(gfx), _dataLayer(dataLayer) {}

    void UpdateMainInfo()
    {
    }

    void RegenScreen()
    {
        _gfx->fillScreen(ST7735_YELLOW);
        _gfx->drawFastHLine(10, 10, 200, ST7735_BLUE);
    }

private:
    DataLayer *_dataLayer;
    Adafruit_TFTLCD *_gfx;

    // char *getErrorCodeDesc(uint8_t errorCode)
    // {
    //     switch ( errorCode )
    //       {
    //          case 1:
    //             char* name = "Bład odczytu pamięci";
    //             return name;
    //          default:
    //             return  &'None';
    //       }
    //       return  &'None';
    // }
};
#endif
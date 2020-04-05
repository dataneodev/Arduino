#ifndef MAIN_H
#define MAIN_H

#define POWER_SEGMENT_Y 160
#define POWER_SEGMENT_X 2
#define POWER_SEGMENT_OFFSET_DESC 2
#define POWER_SEGMENT_OFFSET_MAIN 12

#define STATIC_FIRSTCOLUMN 6
#define STATIC_SECONDCOLUMN 20
#define STATIC_THIRDCOLUMN 70
#define STATIC_FOURTCOLUMN 90
#define STATIC_SEGMENT_OFFSET_STATIC 4

#define FONT9T_HEIGHT 13
#define FONT18T_HEIGHT 27
#define FONT24T_HEIGHT 37
#define FONT9T &FreeSans9pt7b
#define FONT24T &FreeSansBold24pt7b

class MainScreen
{
public:
    MainScreen(Adafruit_TFTLCD *gfx, DS18B20Manager *ds18b20Manager, DataLayer *dataLayer) : _gfx(gfx), _ds18b20Manager(ds18b20Manager), _dataLayer(dataLayer) {}

    void UpdateMainInfo(bool clearScreen = false)
    {
        Serial.println("UpdateMainInfo");
        uint16_t x = PrintWeatcherTemp(POWER_SEGMENT_X, clearScreen);
        x = PrintPower(3999, x, true, clearScreen);
        PrintStatus(0, x, true, clearScreen);
        PrintStatic(clearScreen);
    }

    void RegenScreen()
    {
        Serial.println("RegenScreen");
        _gfx->fillScreen(ST7735_YELLOW);
        UpdateMainInfo(true);
    }

private:
    DataLayer *_dataLayer;
    Adafruit_TFTLCD *_gfx;
    DS18B20Manager *_ds18b20Manager;

    uint16_t PrintStatic(bool clearScreen = false)
    {
        if (clearScreen)
            _gfx->drawRoundRect(1, 1, 150, 125, 10, ST7735_BLUE);

        uint8_t x = 4;

        _gfx->setFont(FONT9T);
        _gfx->setTextColor(ST7735_BLACK);
        _gfx->setTextSize(1);
        _gfx->setTextWrap(false);

        //PV info
        if (clearScreen)
        {
            _gfx->setCursor(STATIC_FIRSTCOLUMN, x + FONT9T_HEIGHT);
            _gfx->print("PV");
        }

        _gfx->fillCircle(STATIC_FIRSTCOLUMN + 20 + FONT9T_HEIGHT / 2 + 6,
                         x + FONT9T_HEIGHT / 2 + 1,
                         FONT9T_HEIGHT / 2,
                         ST7735_GREEN);

        _gfx->setCursor(STATIC_FIRSTCOLUMN + 20 + FONT9T_HEIGHT + 10, x + FONT9T_HEIGHT);
        _gfx->print("Aktywny");

        x = x + FONT9T_HEIGHT + STATIC_SEGMENT_OFFSET_STATIC;

        //I i U
        if (clearScreen)
        {
            _gfx->setCursor(STATIC_FIRSTCOLUMN, x + FONT9T_HEIGHT);
            _gfx->print("I:");
            _gfx->setCursor(STATIC_THIRDCOLUMN, x + FONT9T_HEIGHT);
            _gfx->print("U:");
        }

        if (!clearScreen)
        {
            _gfx->fillRect(STATIC_SECONDCOLUMN, x - 1, STATIC_SECONDCOLUMN - STATIC_SECONDCOLUMN, FONT9T_HEIGHT + 2, ST7735_YELLOW);
            _gfx->fillRect(STATIC_FOURTCOLUMN, x - 1, STATIC_FOURTCOLUMN - STATIC_THIRDCOLUMN, FONT9T_HEIGHT + 2, ST7735_YELLOW);
        }

        _gfx->setCursor(STATIC_SECONDCOLUMN, x + FONT9T_HEIGHT);
        _gfx->print("10.5A");

        _gfx->setCursor(STATIC_FOURTCOLUMN, x + FONT9T_HEIGHT);
        _gfx->print("234V");
        x = x + FONT9T_HEIGHT + STATIC_SEGMENT_OFFSET_STATIC;

        //P i T
        if (clearScreen)
        {
            _gfx->setCursor(STATIC_FIRSTCOLUMN, x + FONT9T_HEIGHT);
            _gfx->print("T:");
            _gfx->setCursor(STATIC_THIRDCOLUMN, x + FONT9T_HEIGHT);
            _gfx->print("P:");
        }

        if (!clearScreen)
        {
            _gfx->fillRect(STATIC_SECONDCOLUMN, x - 1, STATIC_SECONDCOLUMN - STATIC_SECONDCOLUMN, FONT9T_HEIGHT + 2, ST7735_YELLOW);
            _gfx->fillRect(STATIC_FOURTCOLUMN, x - 1, STATIC_FOURTCOLUMN - STATIC_THIRDCOLUMN, FONT9T_HEIGHT + 2, ST7735_YELLOW);
        }

        _gfx->setCursor(STATIC_SECONDCOLUMN, x + FONT9T_HEIGHT);
        _gfx->print("107C");

        _gfx->setCursor(STATIC_FOURTCOLUMN, x + FONT9T_HEIGHT);
        _gfx->print("2999W");
        x = x + FONT9T_HEIGHT + STATIC_SEGMENT_OFFSET_STATIC;

        //PWM
        if (clearScreen)
        {
            _gfx->setCursor(STATIC_FIRSTCOLUMN, x + FONT9T_HEIGHT);
            _gfx->print("PWM:");
        }

        if (!clearScreen)
        {
            _gfx->fillRect(STATIC_THIRDCOLUMN, x - 1, STATIC_FOURTCOLUMN - STATIC_THIRDCOLUMN, FONT9T_HEIGHT + 2, ST7735_YELLOW);
        }

        _gfx->setCursor(STATIC_THIRDCOLUMN, x + FONT9T_HEIGHT);
        _gfx->print("100%");
        x = x + FONT9T_HEIGHT + STATIC_SEGMENT_OFFSET_STATIC;

        //P.dzis
        if (clearScreen)
        {
            _gfx->setCursor(STATIC_FIRSTCOLUMN, x + FONT9T_HEIGHT);
            _gfx->print("P.dzis:");
        }

        if (!clearScreen)
        {
            _gfx->fillRect(STATIC_THIRDCOLUMN, x - 1, STATIC_FOURTCOLUMN - STATIC_THIRDCOLUMN, FONT9T_HEIGHT + 2, ST7735_YELLOW);
        }

        _gfx->setCursor(STATIC_THIRDCOLUMN, x + FONT9T_HEIGHT);
        _gfx->print("2100W");
        x = x + FONT9T_HEIGHT + STATIC_SEGMENT_OFFSET_STATIC;

        //E.dzis
        if (clearScreen)
        {
            _gfx->setCursor(STATIC_FIRSTCOLUMN, x + FONT9T_HEIGHT);
            _gfx->print("E.dzis:");
        }

        if (!clearScreen)
        {
            _gfx->fillRect(STATIC_THIRDCOLUMN, x - 1, STATIC_FOURTCOLUMN - STATIC_THIRDCOLUMN, FONT9T_HEIGHT + 2, ST7735_YELLOW);
        }

        _gfx->setCursor(STATIC_THIRDCOLUMN, x + FONT9T_HEIGHT);
        _gfx->print("245kWh");
        x = x + FONT9T_HEIGHT + STATIC_SEGMENT_OFFSET_STATIC;
        //E.7dni
        if (clearScreen)
        {
            _gfx->setCursor(STATIC_FIRSTCOLUMN, x + FONT9T_HEIGHT);
            _gfx->print("E.7dni:");
        }

        if (!clearScreen)
        {
            _gfx->fillRect(STATIC_THIRDCOLUMN, x - 1, STATIC_FOURTCOLUMN - STATIC_THIRDCOLUMN, FONT9T_HEIGHT + 2, ST7735_YELLOW);
        }

        _gfx->setCursor(STATIC_THIRDCOLUMN, x + FONT9T_HEIGHT);
        _gfx->print("1245kWh");
        x = x + FONT9T_HEIGHT + STATIC_SEGMENT_OFFSET_STATIC;

        //230V
        if (clearScreen)
            _gfx->drawRoundRect(1, 130, 150, 80, 10, ST7735_BLUE);

        x = 135;

        //230V info
        if (clearScreen)
        {
            _gfx->setCursor(STATIC_FIRSTCOLUMN, x + FONT9T_HEIGHT);
            _gfx->print("230V");
        }

        _gfx->fillCircle(STATIC_FIRSTCOLUMN + 37 + FONT9T_HEIGHT / 2 + 6,
                         x + FONT9T_HEIGHT / 2 + 1,
                         FONT9T_HEIGHT / 2,
                         ST7735_RED);

        _gfx->setCursor(STATIC_FIRSTCOLUMN + 35 + FONT9T_HEIGHT + 10, x + FONT9T_HEIGHT);
        _gfx->print("Wyłączony");

        x = x + FONT9T_HEIGHT + STATIC_SEGMENT_OFFSET_STATIC;

        //P i T
        if (clearScreen)
        {
            _gfx->setCursor(STATIC_FIRSTCOLUMN, x + FONT9T_HEIGHT);
            _gfx->print("T:");
        }

        if (!clearScreen)
        {
            _gfx->fillRect(STATIC_SECONDCOLUMN, x - 1, STATIC_SECONDCOLUMN - STATIC_SECONDCOLUMN, FONT9T_HEIGHT + 2, ST7735_YELLOW);
        }

        _gfx->setCursor(STATIC_SECONDCOLUMN, x + FONT9T_HEIGHT);
        _gfx->print("107C");
        x = x + FONT9T_HEIGHT + STATIC_SEGMENT_OFFSET_STATIC;

        //E.dzis
        if (clearScreen)
        {
            _gfx->setCursor(STATIC_FIRSTCOLUMN, x + FONT9T_HEIGHT);
            _gfx->print("E.dzis:");
        }

        if (!clearScreen)
        {
            _gfx->fillRect(STATIC_THIRDCOLUMN, x - 1, STATIC_FOURTCOLUMN - STATIC_THIRDCOLUMN, FONT9T_HEIGHT + 2, ST7735_YELLOW);
        }

        _gfx->setCursor(STATIC_THIRDCOLUMN, x + FONT9T_HEIGHT);
        _gfx->print("245kWh");
        x = x + FONT9T_HEIGHT + STATIC_SEGMENT_OFFSET_STATIC;
        //E.7dni
        if (clearScreen)
        {
            _gfx->setCursor(STATIC_FIRSTCOLUMN, x + FONT9T_HEIGHT);
            _gfx->print("E.7dni:");
        }

        if (!clearScreen)
        {
            _gfx->fillRect(STATIC_THIRDCOLUMN, x - 1, STATIC_FOURTCOLUMN - STATIC_THIRDCOLUMN, FONT9T_HEIGHT + 2, ST7735_YELLOW);
        }

        _gfx->setCursor(STATIC_THIRDCOLUMN, x + FONT9T_HEIGHT);
        _gfx->print("2365kWh");

        x = x + FONT9T_HEIGHT + STATIC_SEGMENT_OFFSET_STATIC;
        return x;
    }

    /* #region  Power */
    float _lastTemp;
    uint16_t PrintWeatcherTemp(uint16_t x, bool clearScreen = false)
    {
        uint8_t sensorAdress[8];
        float temp;

        _ds18b20Manager->GetSensorAdress(TempWatherID, &sensorAdress[0]);
        bool valueCorrent = _ds18b20Manager->getLastTemperatureRead(&sensorAdress[0], &temp);

        if (!valueCorrent)
            temp = -134.541;

        if (!clearScreen && temp == _lastTemp)
        {
            x = x + FONT9T_HEIGHT + POWER_SEGMENT_OFFSET_DESC;
            x = x + FONT24T_HEIGHT + POWER_SEGMENT_OFFSET_MAIN;
            return x;
        }

        _lastTemp = temp;

        if (clearScreen)
        {
            _gfx->setFont(FONT9T);
            _gfx->setCursor(POWER_SEGMENT_Y, x + FONT9T_HEIGHT);
            _gfx->setTextColor(ST7735_BLACK);
            _gfx->setTextSize(1);
            _gfx->setTextWrap(false);
            _gfx->print("Temperatura wody");
        }
        x = x + FONT9T_HEIGHT + POWER_SEGMENT_OFFSET_DESC;

        _gfx->setFont(FONT24T);
        if (!clearScreen)
        {
            _gfx->fillRect(POWER_SEGMENT_Y, x - 1, 320 - POWER_SEGMENT_Y, FONT24T_HEIGHT + 2, ST7735_YELLOW);
        }

        _gfx->setCursor(POWER_SEGMENT_Y, x + FONT24T_HEIGHT);
        x = x + FONT24T_HEIGHT + POWER_SEGMENT_OFFSET_MAIN;
        _gfx->setTextSize(1);
        _gfx->setTextWrap(false);

        if (valueCorrent)
        {
            if (temp <= 40)
            {
                _gfx->setTextColor(ST7735_BLUE);
            }

            if (temp > 40 && temp <= 60)
            {
                _gfx->setTextColor(ST7735_ORANGE);
            }

            if (temp > 60)
            {
                _gfx->setTextColor(ST7735_RED);
            }

            Serial.println("Temp read");
            Serial.println(temp);
            _gfx->print(temp, 1);
            _gfx->print('C');
        }
        else
        {
            _gfx->setTextColor(ST7735_BLUE);
            _gfx->print("-C");
        }

        return x;
    }

    uint16_t PrintPower(uint16_t power, uint16_t x, bool valueCorrent = true, bool clearScreen = false)
    {
        uint16_t offsetx = 4;

        if (clearScreen)
        {
            _gfx->setFont(FONT9T);
            _gfx->setCursor(POWER_SEGMENT_Y, x + FONT9T_HEIGHT);
            _gfx->setTextColor(ST7735_BLACK);
            _gfx->setTextSize(1);
            _gfx->setTextWrap(false);
            _gfx->print("Moc dostarczana");
        }
        x = x + FONT9T_HEIGHT + POWER_SEGMENT_OFFSET_DESC;

        _gfx->setFont(FONT24T);
        _gfx->setCursor(POWER_SEGMENT_Y, x + FONT24T_HEIGHT);
        x = x + FONT24T_HEIGHT + POWER_SEGMENT_OFFSET_MAIN;

        _gfx->setTextColor(ST7735_RED);
        _gfx->setTextSize(1);
        _gfx->setTextWrap(false);

        if (valueCorrent)
        {
            _gfx->print(power);
            _gfx->print('W');
        }
        else
        {
            _gfx->print("-W");
        }
        return x;
    }

    uint16_t PrintStatus(uint8_t errorCode, uint16_t x, bool valueCorrent = true, bool clearScreen = false)
    {
        uint16_t offsetx = 4;

        if (clearScreen)
        {
            _gfx->setFont(FONT9T);
            _gfx->setCursor(POWER_SEGMENT_Y, x + FONT9T_HEIGHT);
            _gfx->setTextColor(ST7735_BLACK);
            _gfx->setTextSize(1);
            _gfx->setTextWrap(false);
            _gfx->print("Status sterownika");
        }
        x = x + FONT9T_HEIGHT + POWER_SEGMENT_OFFSET_DESC;

        _gfx->setFont(FONT9T);
        _gfx->setCursor(POWER_SEGMENT_Y, x + FONT9T_HEIGHT * 2);
        x = x + FONT9T_HEIGHT * 2 + POWER_SEGMENT_OFFSET_MAIN;

        _gfx->setTextColor(ST7735_GREEN);
        _gfx->setTextSize(2);
        _gfx->setTextWrap(false);
        _gfx->print("Pracuje");
        return x;
    }
    /* #endregion */

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
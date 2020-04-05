#ifndef DISPLAY_H
#define DISPLAY_H

class Display
{
public:
    Display(Adafruit_TFTLCD *gfx, navRoot *menu, UserAction *userAction, MainScreen *mainScreen, DataLayer *dataLayer) : _gfx(gfx), _menu(menu), _userAction(userAction), _mainScreen(mainScreen), _dataLayer(dataLayer) {}

    enum LCD_STATUS_ENUM
    {
        OFF,
        MAIN,
        MENU
    };

    void Inicjalize()
    {
        PUBLIC_LCD_STATUS = OFF;
        menuInicjalization();
        setLcdStatus(MAIN);
    }

    void Pool(bool cycle)
    {
        bool userDoAction = checkUserAction();

        bool backMenuClicked = false;
        bool enterMenuClicked = false;

        if (userDoAction)
        {
            backMenuClicked = _userAction->IsBackMenu();
            enterMenuClicked = _userAction->IsEnterMenu();
        }

        if (PUBLIC_LCD_STATUS == OFF)
        {
            if (ERROR_CODE != 0)
            {
                setLcdStatus(MAIN);
                return;
            }

            if (userDoAction)
            {
                setLcdStatus(MAIN);
                return;
            }
        }

        if (PUBLIC_LCD_STATUS == MAIN)
        {
            if (enterMenuClicked)
            {
                setLcdStatus(MENU);
                return;
            }

            if (cycle && checkDisplayOff())
            {
                setLcdStatus(OFF);
                return;
            }

            if (cycle)
            {
                updateMainInfo();
                return;
            }
        }

        if (PUBLIC_LCD_STATUS == MENU)
        {
            if (backMenuClicked)
            {
                checkMenuLeave();
                setLcdStatus(MAIN);
                return;
            }

            if (cycle && checkMenuLeave())
            {
                setLcdStatus(MAIN);
                return;
            }

            _menu->poll();
            return;
        }
    }

    bool CheckForEnterMenu()
    {
        return PUBLIC_LCD_STATUS == MAIN;
    }

    bool MenuActive()
    {
        return PUBLIC_LCD_STATUS == MENU;
    }

    LCD_STATUS_ENUM GetLCDStatus()
    {
        return PUBLIC_LCD_STATUS;
    }

private:
    Adafruit_TFTLCD *_gfx;
    navRoot *_menu;
    UserAction *_userAction;
    DataLayer *_dataLayer;
    MainScreen *_mainScreen;

    LCD_STATUS_ENUM PUBLIC_LCD_STATUS = OFF;
    unsigned long LAST_USER_ACTIVE_MILLS;

    bool checkUserAction()
    {
        bool action = LAST_USER_ACTIVE_MILLS != _userAction->GetLastUserActionMillis();
        if (action)
        {
            if (_userAction->GetMillis() + 100 > LAST_USER_ACTIVE_MILLS ||
                _userAction->GetMillis() < LAST_USER_ACTIVE_MILLS)
            {
                LAST_USER_ACTIVE_MILLS = _userAction->GetLastUserActionMillis();
                return true;
            }
        }
        return false;
    }

    bool checkMenuLeave()
    {
        unsigned long RTM = RETURN_TO_MAIN_FROM_MENU;
        RTM += _userAction->GetLastUserActionMillis();
        if (_userAction->GetMillis() > RTM)
            return true;
        return false;
    }

    bool checkDisplayOff()
    {
        unsigned long lcdTimeoutUL = _dataLayer->getLcdTimeOff();
        if (lcdTimeoutUL < 30)
            return false;

        lcdTimeoutUL = (lcdTimeoutUL * 1000) + _userAction->GetLastUserActionMillis();

        if (_userAction->GetMillis() > lcdTimeoutUL)
            return true;
        return false;
    }

    void setLcdStatus(LCD_STATUS_ENUM LCD_STATUS)
    {
        if (LCD_STATUS == PUBLIC_LCD_STATUS)
        {
            return;
        }

        if (PUBLIC_LCD_STATUS == OFF && LCD_STATUS != OFF)
        {
            Serial.println("set MAIN");
            PUBLIC_LCD_STATUS = MAIN;
            lcdOn();
            regenMainInfo();
            _userAction->BeginRegisterEnterMenu();
            return;
        }

        if (PUBLIC_LCD_STATUS != OFF && LCD_STATUS == OFF)
        {
            Serial.println("set OFF");
            PUBLIC_LCD_STATUS = OFF;
            lcdOff();
            _userAction->EndRegisterEnterMenu();
            return;
        }

        if (LCD_STATUS == MAIN)
        {
            Serial.println("set MAIN");
            PUBLIC_LCD_STATUS = MAIN;
            regenMainInfo();
            _userAction->BeginRegisterEnterMenu();
            return;
        }

        if (LCD_STATUS == MENU)
        {
            Serial.println("set MENU");
            PUBLIC_LCD_STATUS = LCD_STATUS;
            menuShow();
            _gfx->setFont();
            _gfx->setTextSize(textScale);
            _menu->reset();
            _menu->poll();
            _userAction->EndRegisterEnterMenu();
            return;
        }

        PUBLIC_LCD_STATUS = LCD_STATUS;
    }

    void regenMainInfo()
    {
        _mainScreen->RegenScreen();
    }

    void updateMainInfo()
    {
        _mainScreen->UpdateMainInfo();
    }

    void lcdStartup()
    {
        _gfx->reset();

        uint16_t identifier = _gfx->readID();
        _gfx->begin(identifier);
        _gfx->setRotation(3);
        _gfx->setTextSize(textScale); //test scalling
        _gfx->setTextWrap(false);
        _gfx->fillScreen(ST7735_YELLOW);
    }

    void lcdOff()
    {
        digitalWrite(LCD_ON_OFF, LOW);
    }

    void lcdOn()
    {
        digitalWrite(LCD_ON_OFF, HIGH);
        lcdStartup();
    }
};

#endif

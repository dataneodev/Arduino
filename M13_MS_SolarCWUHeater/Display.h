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

    void Pool()
    {
        bool cycle = checkCycleTime();
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
            if (userDoAction && enterMenuClicked)
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
            if (userDoAction && backMenuClicked)
            {
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
    unsigned long LAST_PROCESS_MILLS;
    unsigned long LAST_USER_ACTIVE_MILLS;

    const uint16_t CYCLE_TIME = 2000;                // co ile aktualizować mainDisplay
    const uint16_t RETURN_TO_MAIN_FROM_MENU = 30000; // 15 SEK.

    bool checkCycleTime()
    {
        //over
        if (_userAction->GetMillis() < LAST_PROCESS_MILLS)
        {
            LAST_PROCESS_MILLS = _userAction->GetMillis();
            return true;
        }

        if (_userAction->GetMillis() > LAST_PROCESS_MILLS + ((unsigned long)CYCLE_TIME))
        {
            LAST_PROCESS_MILLS = _userAction->GetMillis();
            return true;
        }
        return false;
    }

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
        if (_userAction->GetMillis() >
            _userAction->GetLastUserActionMillis() + ((unsigned long)RETURN_TO_MAIN_FROM_MENU))
        {
            return true;
        }
        return false;
    }

    bool checkDisplayOff()
    {
        uint8_t lcdTimeout = _dataLayer->getLcdTimeOff();
        if (lcdTimeout < 30)
        {
            return false;
        }

        if (_userAction->GetMillis() >
            _userAction->GetLastUserActionMillis() + ((unsigned long)lcdTimeout) * 1000)
        {
            return true;
        }
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
            PUBLIC_LCD_STATUS = MAIN;
            lcdOn();
            regenMainInfo();
            _userAction->BeginRegisterEnterMenu();
            return;
        }

        if (PUBLIC_LCD_STATUS != OFF && LCD_STATUS == OFF)
        {
            PUBLIC_LCD_STATUS = OFF;
            lcdOff();
            _userAction->EndRegisterEnterMenu();
            return;
        }

        if (LCD_STATUS == MAIN)
        {
            PUBLIC_LCD_STATUS = LCD_STATUS;
            regenMainInfo();
            _userAction->BeginRegisterEnterMenu();
            return;
        }

        if (LCD_STATUS == MENU)
        {
            PUBLIC_LCD_STATUS = LCD_STATUS;
            menuShow();
            _gfx->setFont();
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

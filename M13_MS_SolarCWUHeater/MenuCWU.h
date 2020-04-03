#ifndef MENUCWU
#define MENUCWU

#include <menu.h>
#include <menuIO/adafruitGfxOut.h>
#include <TimerOne.h>
#include <menuIO/clickEncoderIn.h>

#include <menuIO/keyIn.h>
#include <menuIO/chainStream.h>

using namespace Menu;

/* #region  ColorDefine */
#define ST7735_GRAY RGB565(128, 128, 128)
#define ST7735_BLACK RGB565(0, 0, 0)
#define ST7735_BLUE RGB565(0, 100, 255)
#define ST7735_WHITE RGB565(255, 255, 255)
#define ST7735_YELLOW RGB565(255, 255, 0)
#define ST7735_RED RGB565(255, 0, 0)

#define ST7735_GRAY_A RGB565(64, 64, 64)
#define ST7735_BLUE_A RGB565(0, 50, 125)

//each color is in the format:
//  {{disabled normal,disabled selected},{enabled normal,enabled selected, enabled editing}}

const colorDef<uint16_t> colors[6] MEMMODE = {
    {{(uint16_t)ST7735_BLACK, (uint16_t)ST7735_BLACK}, {(uint16_t)ST7735_BLACK, (uint16_t)ST7735_GRAY_A, (uint16_t)ST7735_BLUE_A}}, //bgColor
    {{(uint16_t)ST7735_GRAY, (uint16_t)ST7735_GRAY}, {(uint16_t)ST7735_WHITE, (uint16_t)ST7735_WHITE, (uint16_t)ST7735_WHITE}},     //fgColor
    {{(uint16_t)ST7735_WHITE, (uint16_t)ST7735_BLACK}, {(uint16_t)ST7735_YELLOW, (uint16_t)ST7735_YELLOW, (uint16_t)ST7735_RED}},   //valColor
    {{(uint16_t)ST7735_WHITE, (uint16_t)ST7735_BLACK}, {(uint16_t)ST7735_WHITE, (uint16_t)ST7735_YELLOW, (uint16_t)ST7735_YELLOW}}, //unitColor
    {{(uint16_t)ST7735_WHITE, (uint16_t)ST7735_GRAY}, {(uint16_t)ST7735_BLACK, (uint16_t)ST7735_BLUE, (uint16_t)ST7735_WHITE}},     //cursorColor
    {{(uint16_t)ST7735_WHITE, (uint16_t)ST7735_YELLOW}, {(uint16_t)ST7735_BLUE, (uint16_t)ST7735_RED, (uint16_t)ST7735_RED}},       //titleColor
};

/* #endregion */

ClickEncoderStream encStream(clickEncoder, 2);
MENU_INPUTS(in, &encStream);

/* #region  weatcherTemp */
uint8_t weatcherTemp = DL.getMaxWeatherTemp();
result setWeatherTemp()
{
    DL.setMaxWeatherTemp(weatcherTemp);
}
/* #endregion */

/* #region  restartSystem */
// customizing a menu prompt look
class confirmRestart : public menu
{
public:
    confirmRestart(constMEM menuNodeShadow &shadow) : menu(shadow) {}
    Used printTo(navRoot &root, bool sel, menuOut &out, idx_t idx, idx_t len, idx_t p) override
    {
        return idx < 0 ? //idx will be -1 when printing a menu title or a valid index when printing as option
                   menu::printTo(root, sel, out, idx, len, p)
                       :                                                     //when printing title
                   out.printRaw((constText *)F("Restartuj sterownik"), len); //when printing as regular option
    }
};

result systemRestart()
{
    digitalWrite(RESET_PIN, LOW);
}

altMENU(confirmRestart, restartMenu, "Restartuj sterownik?", doNothing, noEvent, wrapStyle, (Menu::_menuData | Menu::_canNav), OP("Tak", systemRestart, enterEvent), EXIT("Anuluj"));
/* #endregion */

/* #region  subtempSensorsMenu */
/* #region  tempSensorViewName */
uint8_t *pAddr(uint8_t a1, uint8_t a2, uint8_t a3, uint8_t a4, uint8_t a5, uint8_t a6, uint8_t a7, uint8_t a8)
{
    uint8_t *c = new uint8_t[8];
    c[0] = a1;
    c[1] = a2;
    c[2] = a3;
    c[3] = a4;
    c[4] = a5;
    c[5] = a6;
    c[6] = a7;
    c[7] = a8;
    return c;
}

void static getAdress(uint8_t *adress, char *stringadress)
{
    sprintf(&stringadress[0], "Adres ", 0);

    for (int i = 0; i != 8; i++)
        sprintf(&stringadress[6 + 2 * i], "%02X", adress[i]);
    stringadress[22] = '\0';
}

class tempSensorAdres : public prompt
{
public:
    tempSensorAdres(constMEM promptShadow &p) : prompt(p) {}
    Used printTo(navRoot &root, bool sel, menuOut &out, idx_t idx, idx_t len, idx_t panelNr) override
    {
        char test[23];
        getAdress(pAddr(
                      DL.getTempAdressSensor(1),
                      DL.getTempAdressSensor(2),
                      DL.getTempAdressSensor(3),
                      DL.getTempAdressSensor(4),
                      DL.getTempAdressSensor(5),
                      DL.getTempAdressSensor(6),
                      DL.getTempAdressSensor(7),
                      DL.getTempAdressSensor(8)),
                  test);

        len -= out.print(test);
        return len;
    }
};
/* #endregion */

/* #region  subSubtempSensorsMenu */
bool compareAdress(uint8_t ad1[8], uint8_t ad2[8])
{
    bool result = true;
    for (uint8_t i = 0; i < 8; i++)
        if (ad1[i] != ad2[i])
            result = false;
    return result;
}

bool detectNewTempSensor()
{
    DeviceAddress tempDeviceAddress;
    int numberOfDevices = myDS18B20Manager.GetDallasTemperature()->getDeviceCount();

    if (numberOfDevices > 0)
        for (byte i = 0; i < numberOfDevices; i++)
        {
            if (myDS18B20Manager.GetDallasTemperature()->getAddress(tempDeviceAddress, i))
            {
                uint8_t tem1p[8];
                myDS18B20Manager.GetSensorAdress(135, &tem1p[0]);

                uint8_t tem2p[8];
                myDS18B20Manager.GetSensorAdress(136, &tem2p[0]);

                uint8_t tem3p[8];
                myDS18B20Manager.GetSensorAdress(137, &tem3p[0]);

                if (!compareAdress(tem1p, tempDeviceAddress) &&
                    !compareAdress(tem2p, tempDeviceAddress) &&
                    !compareAdress(tem3p, tempDeviceAddress) &&
                    tempDeviceAddress[0] != DL.getTempAdressSensor(1) &&
                    tempDeviceAddress[1] != DL.getTempAdressSensor(2) &&
                    tempDeviceAddress[2] != DL.getTempAdressSensor(3) &&
                    tempDeviceAddress[3] != DL.getTempAdressSensor(4) &&
                    tempDeviceAddress[4] != DL.getTempAdressSensor(5) &&
                    tempDeviceAddress[5] != DL.getTempAdressSensor(6) &&
                    tempDeviceAddress[6] != DL.getTempAdressSensor(7) &&
                    tempDeviceAddress[7] != DL.getTempAdressSensor(8))
                {
                    DL.setTempAdressSensor(1, tempDeviceAddress[0]);
                    DL.setTempAdressSensor(2, tempDeviceAddress[1]);
                    DL.setTempAdressSensor(3, tempDeviceAddress[2]);
                    DL.setTempAdressSensor(4, tempDeviceAddress[3]);
                    DL.setTempAdressSensor(5, tempDeviceAddress[4]);
                    DL.setTempAdressSensor(6, tempDeviceAddress[5]);
                    DL.setTempAdressSensor(7, tempDeviceAddress[6]);
                    DL.setTempAdressSensor(8, tempDeviceAddress[7]);
                    return true;
                }
            }
        }

    return false;
}

class detectNewTempResult : public prompt
{
public:
    detectNewTempResult(constMEM promptShadow &p) : prompt(p) {}
    Used printTo(navRoot &root, bool sel, menuOut &out, idx_t idx, idx_t len, idx_t p) override
    {
        if (detectNewTempSensor())
            len -= out.print("Wykryto nowy czujnik");
        else
            len -= out.print("Brak nowego czujnika");
        return len;
    }
};

MENU(detectNewTempMenu, "Wykryj nowy czujnik", doNothing, noEvent, noStyle,
     altOP(detectNewTempResult, "", doNothing, noEvent),
     EXIT("<Wstecz"));
/* #endregion */

MENU(subTempSensorMenu, "Opcje czujnika temperatury", doNothing, noEvent, noStyle,
     altOP(tempSensorAdres, "", doNothing, noEvent),
     SUBMENU(detectNewTempMenu),
     SUBMENU(restartMenu),
     EXIT("<Wstecz"));
/* #endregion */

/* #region  relayPVEnabled */
bool relayPVEnabled = DL.getPVRelay();
result setrelayPVEnabled();
SELECT(relayPVEnabled, relayPVEnableMenu, "Zasilanie z PV", setrelayPVEnabled, exitEvent, noStyle,
       VALUE("Wlaczone", true, doNothing, noEvent),
       VALUE("Wylaczone", false, doNothing, noEvent));
/* #endregion */

/* #region  subRelayPVMenu */
MENU(subRelayPVMenu, "Opcje zasilania PV", doNothing, noEvent, noStyle,
     OP("Op", doNothing, noEvent),
     EXIT("<Back"));
/* #endregion */

/* #region  relay230vEnabled */
bool relay230VEnagled = DL.get230VRelay();
result setrelay230VEnagled();
SELECT(relay230VEnagled, relay230VEnableMenu, "Zasilanie z 230V", setrelay230VEnagled, exitEvent, noStyle,
       VALUE("Wlaczone", true, doNothing, noEvent),
       VALUE("Wylaczone", false, doNothing, noEvent));
/* #endregion */

/* #region  subRelay230VMenu */
bool relay230VAdvenceEnable = DL.getRELAY_230V_ADVENCE();
result setrelay230VAdvenceEnable()
{
    DL.setRELAY_230V_ADVENCE(relay230VAdvenceEnable);
    return proceed;
};

SELECT(relay230VAdvenceEnable, relay230VAdvenceEnableMenu, "Opcje zaaw.", setrelay230VAdvenceEnable, exitEvent, noStyle,
       VALUE("Wlaczone", true, doNothing, noEvent),
       VALUE("Wylaczone", false, doNothing, noEvent));

// char *constMEM hexDigit MEMMODE = "0123456789";
// char *constMEM hexNr[] MEMMODE = {hexDigit, ":", hexDigit};
// char bufPo[] = "17:11";
// char bufPrzed[] = "19:11";

// uint8_t hourStart = 17;
// uint8_t minuteStart = 10;

// uint8_t hourEnd = 17;
// uint8_t minuteEnd = 10;

// prompt *padData[] = {
//     new menuField<typeof(hourStart)>(hourStart, "", "", 00, 23, 1, 0, doNothing, noEvent),
//     new menuField<typeof(minuteStart)>(minuteStart, "/", "", 0, 59, 1, 0, doNothing, noEvent)};

// menuNode padMenu(
//     "Zacznij po",
//     sizeof(padData) / sizeof(prompt *),
//     padData,
//     doNothing,
//     noEvent,
//     noStyle,
//     (systemStyles)(_asPad | Menu::_menuData | Menu::_canNav | _parentDraw));

uint16_t mocGrzalki = DL.getPOWER230V();
result setPOWER230V()
{
    DL.setPOWER230V(mocGrzalki);
    return proceed;
};

uint8_t RELAY_230_TEMP_START = DL.getRELAY_230_TEMP_START();
result setRELAY_230_TEMP_START()
{
    DL.setRELAY_230_TEMP_START(RELAY_230_TEMP_START);
    return proceed;
};

uint8_t RELAY_230_TEMP_HEAT_UP = DL.getRELAY_230_TEMP_HEAT_UP();
result setRELAY_230_TEMP_HEAT_UP()
{
    DL.setRELAY_230_TEMP_HEAT_UP(RELAY_230_TEMP_HEAT_UP);
    return proceed;
};

MENU(subRelay230VMenu, "Opcje zasilania 230V", doNothing, noEvent, noStyle,
     FIELD(mocGrzalki, "Moc grzalki ", " W", 100, 4500, 100, 1, setPOWER230V, exitEvent, noStyle),
     SUBMENU(relay230VAdvenceEnableMenu),
     FIELD(RELAY_230_TEMP_START, "Zacznij grzac ponizej", " C", 0, 85, 1, 0, setRELAY_230_TEMP_START, exitEvent, noStyle),
     FIELD(RELAY_230_TEMP_HEAT_UP, "Grzej do", " C", 0, 85, 1, 0, setRELAY_230_TEMP_HEAT_UP, exitEvent, noStyle),
     EXIT("<Wstecz"));

/* #endregion */

/* #region  mySensorsEnabled */
bool mySensorsEnabled = DL.getMySensorsEnable();
result setgetMySensorsEnable()
{
    DL.setMySensorsEnable(mySensorsEnabled);
    return proceed;
}
SELECT(mySensorsEnabled, mysensorsMenu, "Komunikacja RS485", setgetMySensorsEnable, exitEvent, noStyle,
       VALUE("Wlaczona", true, doNothing, noEvent),
       VALUE("Wylaczona", false, doNothing, noEvent));
/* #endregion */

/* #region   lcdTimeOfMenu*/
uint16_t czasUspieniaEkranu = DL.getLcdTimeOff();
result setCzasUspieniaEkranu()
{
    DL.setLcdTimeOff(czasUspieniaEkranu);
    return proceed;
}
SELECT(czasUspieniaEkranu, lcdTimeOfMenu, "Uspienie ekranu", setCzasUspieniaEkranu, exitEvent, noStyle,
       VALUE("Zawsze wl.", 0, doNothing, noEvent),
       VALUE("1 min.", 60, doNothing, noEvent),
       VALUE("2 min.", 120, doNothing, noEvent),
       VALUE("3 min.", 180, doNothing, noEvent),
       VALUE("4 min.", 240, doNothing, noEvent),
       VALUE("5 min.", 300, doNothing, noEvent),
       VALUE("10 min.", 600, doNothing, noEvent),
       VALUE("15 min.", 900, doNothing, noEvent),
       VALUE("20 min.", 1200, doNothing, noEvent),
       VALUE("25 min.", 1500, doNothing, noEvent),
       VALUE("30 min.", 1800, doNothing, noEvent));
/* #endregion */

/* #region  backToMain */
result backToMain()
{
    userAction.SetBackMenu();
    return proceed;
}
/* #endregion */

MENU(mainMenu, "Opcje sterownika", doNothing, noEvent, wrapStyle,
     FIELD(weatcherTemp, "Temperatura wody ", " C", 30, 85, 1, 0, setWeatherTemp, exitEvent, noStyle),
     SUBMENU(subTempSensorMenu),
     SUBMENU(relayPVEnableMenu),
     SUBMENU(subRelayPVMenu),
     SUBMENU(relay230VEnableMenu),
     SUBMENU(subRelay230VMenu),
     SUBMENU(mysensorsMenu),
     SUBMENU(lcdTimeOfMenu),
     SUBMENU(restartMenu),
     OP("Wersja oprogramowania:1.0", doNothing, noEvent),
     OP("Wyjscie z menu", backToMain, enterEvent));

MENU_OUTPUTS(out, 4, ADAGFX_OUT(gfx, colors, 6 * textScale, 9 * textScale, {0, 0, 27, 14}), NONE);
NAVROOT(navMenu, mainMenu, 4, in, out);

void menuInicjalization()
{
    navMenu.showTitle = true;
}

void menuShow()
{
    weatcherTemp = DL.getMaxWeatherTemp();
    czasUspieniaEkranu = DL.getLcdTimeOff();
    mySensorsEnabled = DL.getMySensorsEnable();
    relayPVEnabled = DL.getPVRelay();
    relay230VEnagled = DL.get230VRelay();

    if (relayPVEnabled)
        mainMenu[3].enable();
    else
        mainMenu[3].disable();

    if (relay230VEnagled)
        mainMenu[5].enable();
    else
        mainMenu[5].disable();
    mainMenu[9].disable();

    relay230VAdvenceEnable = DL.getRELAY_230V_ADVENCE();
    mocGrzalki = DL.getPOWER230V();
    RELAY_230_TEMP_START = DL.getRELAY_230_TEMP_START();
    RELAY_230_TEMP_HEAT_UP = DL.getRELAY_230_TEMP_HEAT_UP();
}

/* #region  relayPVEnabled */
result setrelayPVEnabled()
{
    if (relayPVEnabled)
        mainMenu[3].enable();
    else
        mainMenu[3].disable();

    DL.setPVRelay(relayPVEnabled);
    return proceed;
}
/* #endregion */

/* #region  relay230vEnabled */
result setrelay230VEnagled()
{
    if (relay230VEnagled)
        mainMenu[5].enable();
    else
        mainMenu[5].disable();

    DL.set230VRelay(relay230VEnagled);
    return proceed;
}
/* #endregion */

//config myOptions('*','-',defaultNavCodes,false);
/* #endregion MenuConfiguration */

#endif
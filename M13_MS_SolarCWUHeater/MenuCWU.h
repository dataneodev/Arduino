#ifndef MENUCWU
#define MENUCWU

#include <menu.h>
#include <menuIO/adafruitGfxOut.h>
#include <TimerOne.h>
#include <menuIO/clickEncoderIn.h>

#include <menuIO/keyIn.h>
#include <menuIO/chainStream.h>

using namespace Menu;

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

ClickEncoderStream encStream(clickEncoder, 2);
MENU_INPUTS(in, &encStream);

/* #region  weatcherTemp */
uint8_t weatcherTemp = DL.getMaxWeatherTemp();
result setWeatherTemp()
{
    DL.setMaxWeatherTemp(weatcherTemp);
}
/* #endregion */

/* #region  subtempSensorsMenu */
MENU(subTempSensorMenu, "Opcje czujnika temperatury", doNothing, noEvent, noStyle,
     OP("Op", doNothing, noEvent),
     EXIT("<Back"));
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
char *constMEM hexDigit MEMMODE = "0123456789";
char *constMEM hexNr[] MEMMODE = {hexDigit, ":", hexDigit};
char buf1[] = "10x11";

MENU(subRelay230VMenu, "Opcje zasilania 230V", doNothing, noEvent, noStyle,
     OP("Op", doNothing, noEvent),
     EXIT("<Back"));
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
     FIELD(weatcherTemp, "Temperatura wody ", " C", 30, 85, 10, 1, setWeatherTemp, exitEvent, noStyle),
     SUBMENU(subTempSensorMenu),
     SUBMENU(relayPVEnableMenu),
     SUBMENU(subRelayPVMenu),
     SUBMENU(relay230VEnableMenu),
     SUBMENU(subRelay230VMenu),
     SUBMENU(mysensorsMenu),
     SUBMENU(lcdTimeOfMenu),
     OP("Restartuj sterownik", doNothing, noEvent),
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
#ifndef MENUCWU
#define MENUCWU

#include <menu.h>
#include <menuIO/adafruitGfxOut.h>
#include <TimerOne.h>
#include <menuIO/clickEncoderIn.h>

#include <menuIO/keyIn.h>
#include <menuIO/chainStream.h>

using namespace Menu;

//customizing a prompt look!
//by extending the prompt class
class altPrompt : public prompt
{
public:
    altPrompt(constMEM promptShadow &p) : prompt(p) {}
    Used printTo(navRoot &root, bool sel, menuOut &out, idx_t idx, idx_t len, idx_t) override
    {
        return out.printRaw(F("special prompt!"), len);
        ;
    }
};

#define ST7735_GRAY RGB565(128, 128, 128)
#define ST7735_BLACK RGB565(0, 0, 0)
#define ST7735_BLUE RGB565(0, 100, 255)
#define ST7735_WHITE RGB565(255, 255, 255)
#define ST7735_YELLOW RGB565(255, 255, 0)
#define ST7735_RED RGB565(255, 0, 0)

const colorDef<uint16_t> colors[6] MEMMODE = {
    {{(uint16_t)ST7735_BLACK, (uint16_t)ST7735_BLACK}, {(uint16_t)ST7735_BLACK, (uint16_t)ST7735_BLUE, (uint16_t)ST7735_BLUE}},     //bgColor
    {{(uint16_t)ST7735_GRAY, (uint16_t)ST7735_GRAY}, {(uint16_t)ST7735_WHITE, (uint16_t)ST7735_WHITE, (uint16_t)ST7735_WHITE}},     //fgColor
    {{(uint16_t)ST7735_WHITE, (uint16_t)ST7735_BLACK}, {(uint16_t)ST7735_YELLOW, (uint16_t)ST7735_YELLOW, (uint16_t)ST7735_RED}},   //valColor
    {{(uint16_t)ST7735_WHITE, (uint16_t)ST7735_BLACK}, {(uint16_t)ST7735_WHITE, (uint16_t)ST7735_YELLOW, (uint16_t)ST7735_YELLOW}}, //unitColor
    {{(uint16_t)ST7735_WHITE, (uint16_t)ST7735_GRAY}, {(uint16_t)ST7735_BLACK, (uint16_t)ST7735_BLUE, (uint16_t)ST7735_WHITE}},     //cursorColor
    {{(uint16_t)ST7735_WHITE, (uint16_t)ST7735_YELLOW}, {(uint16_t)ST7735_BLUE, (uint16_t)ST7735_RED, (uint16_t)ST7735_RED}},       //titleColor
};

ClickEncoderStream encStream(clickEncoder, 2);
MENU_INPUTS(in, &encStream);

#define MAX_DEPTH 4
#define textScale 2

result doAlert(eventMask e, prompt &item);

int test = 55;

int ledCtrl = LOW;

result myLedOn()
{
    ledCtrl = HIGH;
    return proceed;
}
result myLedOff()
{
    ledCtrl = LOW;
    return proceed;
}

TOGGLE(ledCtrl, setLed, "Led: ", doNothing, noEvent, noStyle //,doExit,enterEvent,noStyle
       ,
       VALUE("On", HIGH, doNothing, noEvent), VALUE("Off", LOW, doNothing, noEvent));

int selTest = 0;
SELECT(selTest, selMenu, "Select", doNothing, noEvent, noStyle, VALUE("Zero", 0, doNothing, noEvent), VALUE("One", 1, doNothing, noEvent), VALUE("Two", 2, doNothing, noEvent));

int chooseTest = -1;
CHOOSE(chooseTest, chooseMenu, "Choose", doNothing, noEvent, noStyle, VALUE("First", 1, doNothing, noEvent), VALUE("Second", 2, doNothing, noEvent), VALUE("Third", 3, doNothing, noEvent), VALUE("Last", -1, doNothing, noEvent));

MENU(subMenu, "Sub-Menu", doNothing, noEvent, noStyle, altOP(altPrompt, "", doNothing, noEvent), OP("Op", doNothing, noEvent), EXIT("<Back"));

char *constMEM hexDigit MEMMODE = "0123456789ABCDEF";
char *constMEM hexNr[] MEMMODE = {"0", "x", hexDigit, hexDigit};
char buf1[] = "0x11";

MENU(mainMenu, "Main menu", doNothing, noEvent, wrapStyle, OP("Op1", doNothing, noEvent), OP("Op2", doNothing, noEvent)
     // ,FIELD(test,"Test","%",0,100,10,1,doNothing,noEvent,wrapStyle)
     ,
     SUBMENU(subMenu), SUBMENU(setLed), OP("LED On", myLedOn, enterEvent), OP("LED Off", myLedOff, enterEvent), SUBMENU(selMenu), SUBMENU(chooseMenu)
     //,OP("Alert test",doAlert,enterEvent)
     ,
     EDIT("Hex", buf1, hexNr, doNothing, noEvent, noStyle), EXIT("<Back"));

// define menu colors --------------------------------------------------------
//  {{disabled normal,disabled selected},{enabled normal,enabled selected, enabled editing}}
//monochromatic color table

MENU_OUTPUTS(out, MAX_DEPTH, ADAGFX_OUT(gfx, colors, 6 * textScale, 9 * textScale, {0, 0, 27, 13}), NONE);

NAVROOT(navMenu, mainMenu, MAX_DEPTH, in, out);

void menuInicjalization()
{
    navMenu.showTitle = true; //show menu title?
                              //mainMenu[1].disable();
                              //outGfx.usePreview=true;//reserve one panel for preview?
}

//config myOptions('*','-',defaultNavCodes,false);
/* #endregion MenuConfiguration */

#endif
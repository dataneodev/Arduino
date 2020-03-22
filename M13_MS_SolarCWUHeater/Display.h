#ifndef DISPLAY_H
#define DISPLAY_H



void displayInicjalization()
{
}

void lcdStartup()
{
    gfx.reset();

    uint16_t identifier = gfx.readID();
    gfx.begin(identifier);

    gfx.setRotation(3);
    gfx.setTextSize(textScale); //test scalling
    gfx.setTextWrap(false);
    gfx.fillScreen(ST7735_YELLOW);
}

#endif

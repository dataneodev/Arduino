/*
24C32.h - Library for EEPROM 24C32
Franz Stoiber 2020

History -----------------------------------------------------------
2020-01-05 V1
           check if device is present in function begin
2020-01-07 V2
           check if device is present now in function checkPresence
           new function logInfo can be activated in function begin
2020-01-26 V3
           new function readUInt32
           new function writeUInt32
           new function readInt32
           new function writeInt32
2020-02-23 V4
-------------------------------------------------------------------
*/
 
#ifndef EE_h
#define EE_h

#include "Arduino.h"

#define EEPROM24C32_DEFAULT_ADDRESS  0x57
#define EEPROM24C32__SIZE            4096
#define EEPROM24C32__PAGES            128
#define EEPROM24C32__PAGE_SIZE         32
#define I2C_WRITE_BUFFER_SIZE          30
#define I2C_READ_BUFFER_SIZE           32


class EE {
  public:
    const uint8_t Version = 4;
  
    void begin(uint8_t I2CAdr, bool Log = false);
    bool checkPresence();
    bool eraseData(uint8_t Val, bool Check);
    bool writeByte(uint16_t Adr, uint8_t Data, bool Check, bool Update);
    bool writeFloat(uint16_t Adr, float Val, bool Check, bool Update);
    bool writeDouble(uint16_t Adr, double Val, bool Check, bool Update);
    bool writeUInt32(uint16_t Adr, uint32_t Val, bool Check, bool Update);
    bool writeInt32(uint16_t Adr, int32_t Val, bool Check, bool Update);
    bool writeBytes(uint16_t Adr, int16_t Len, uint8_t *Data, bool Check);
    uint8_t readByte(uint16_t Adr);
    float readFloat(uint16_t Adr);
    double readDouble(uint16_t Adr);
    uint32_t readUInt32(uint16_t Adr);
    int32_t readInt32(uint16_t Adr);
    void readBytes(uint16_t Adr, int16_t Len, uint8_t *Data);
    
  private:
    uint8_t DeviceAdr;
    bool SerialLog;
    
    bool writePage (uint8_t PageNr, uint8_t StartByteNr, int8_t Len, uint8_t *Data, uint16_t &Pos, bool Check);
    void waitReady();
    void logInfo(const char *Item);
};

#endif

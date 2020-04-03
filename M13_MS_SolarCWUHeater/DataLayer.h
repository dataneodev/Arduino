#ifndef DATALAYER
#define DATALAYER

class DataLayer
{
public:
  DataLayer(EE *eprom) : EEPROM(eprom) {}

  void Inicjalize(uint8_t address)
  {
    Wire.begin();
    Wire.setClock(400000);

    EEPROM->begin(address, true);
    IsInicjalize = EEPROM->checkPresence();
    if (IsInicjalize)
      readAll();
  }

  bool IsEepromInicjalize()
  {
    return IsInicjalize;
  }

  /* #region   uint8_t LCD_DISPLAY_TIMEOFF*/
  uint16_t getLcdTimeOff()
  {
    return LCD_DISPLAY_TIMEOFF;
  }

  void setLcdTimeOff(uint16_t newValue, bool forceSave = false)
  {
    bool needToSave = newValue != LCD_DISPLAY_TIMEOFF;
    LCD_DISPLAY_TIMEOFF = newValue;

    if (!IsInicjalize)
      return;

    if (needToSave || forceSave)
      EEPROM->writeUInt32(101, (uint32_t)LCD_DISPLAY_TIMEOFF, false, false);
  }
  /* #endregion */

  /* #region  uint8_t MAX_WEATHER_TEMPERATURE */
  uint8_t getMaxWeatherTemp()
  {
    return MAX_WEATHER_TEMPERATURE;
  }

  void setMaxWeatherTemp(uint8_t newValue, bool forceSave = false)
  {
    bool needToSave = newValue != MAX_WEATHER_TEMPERATURE;
    MAX_WEATHER_TEMPERATURE = newValue;

    if (!IsInicjalize)
      return;

    if (needToSave || forceSave)
      EEPROM->writeByte(105, MAX_WEATHER_TEMPERATURE, false, false);
  }
  /* #endregion */

  /* #region  Relay */
  bool getPVRelay()
  {
    return RELAY_PV_ENABLE;
  }

  void setPVRelay(bool newValue, bool forceSave = false)
  {
    bool needToSave = newValue != RELAY_PV_ENABLE;
    RELAY_PV_ENABLE = newValue;

    if (!IsInicjalize)
      return;

    if (needToSave || forceSave)
      EEPROM->writeByte(106, newValue ? 0x05 : 0x00, false, false);
  }

  bool get230VRelay()
  {
    return RELAY_230V_ENABLE;
  }

  void set230VRelay(bool newValue, bool forceSave = false)
  {
    bool needToSave = newValue != RELAY_230V_ENABLE;
    RELAY_230V_ENABLE = newValue;

    if (!IsInicjalize)
      return;

    if (needToSave || forceSave)
      EEPROM->writeByte(107, newValue ? 0x05 : 0x00, false, false);
  }
  /* #endregion */

  /* #region  TempSensorAdress */
  uint8_t getTempAdressSensor(uint8_t lpAdress)
  {
    if (lpAdress >= 1 && lpAdress <= 8 && IsInicjalize)
    {
      return EEPROM->readByte(109 + lpAdress);
    }
    return 0x00;
  }

  void setTempAdressSensor(uint8_t lpAdress, uint8_t value)
  {
    if (lpAdress >= 1 && lpAdress <= 8 && IsInicjalize)
    {
      EEPROM->writeByte(109 + lpAdress, value, false, false);
    }
  }
  /* #endregion */

  bool getMySensorsEnable()
  {
    return MYSENSORS_ENABLE;
  }

  void setMySensorsEnable(bool newValue, bool forceSave = false)
  {
    bool needToSave = newValue != MYSENSORS_ENABLE;
    MYSENSORS_ENABLE = newValue;

    if (!IsInicjalize)
      return;

    if (needToSave || forceSave)
      EEPROM->writeByte(108, newValue ? 0x05 : 0x00, false, false);
  }

  bool getRELAY_230V_ADVENCE()
  {
    return RELAY_230V_ADVENCE;
  }

  void setRELAY_230V_ADVENCE(bool newValue, bool forceSave = false)
  {
    bool needToSave = newValue != RELAY_230V_ADVENCE;
    RELAY_230V_ADVENCE = newValue;

    if (!IsInicjalize)
      return;

    if (needToSave || forceSave)
      EEPROM->writeByte(120, newValue ? 0x05 : 0x00, false, false);
  }

  uint8_t getRELAY_230_TEMP_START()
  {
    return RELAY_230_TEMP_START;
  }

  void setRELAY_230_TEMP_START(uint8_t newValue, bool forceSave = false)
  {
    bool needToSave = newValue != RELAY_230_TEMP_START;
    RELAY_230_TEMP_START = newValue;

    if (!IsInicjalize)
      return;

    if (needToSave || forceSave)
      EEPROM->writeByte(121, newValue, false, false);
  }

  uint8_t getRELAY_230_TEMP_HEAT_UP()
  {
    return RELAY_230_TEMP_HEAT_UP;
  }

  void setRELAY_230_TEMP_HEAT_UP(uint8_t newValue, bool forceSave = false)
  {
    bool needToSave = newValue != RELAY_230_TEMP_HEAT_UP;
    RELAY_230_TEMP_HEAT_UP = newValue;

    if (!IsInicjalize)
      return;

    if (needToSave || forceSave)
      EEPROM->writeByte(122, newValue, false, false);
  }

  uint8_t getRELAY_230_ALLOW_START_H()
  {
    return RELAY_230_ALLOW_START_H;
  }

  void setRELAY_230_ALLOW_START_H(uint8_t newValue, bool forceSave = false)
  {
    bool needToSave = newValue != RELAY_230_ALLOW_START_H;
    RELAY_230_ALLOW_START_H = newValue;

    if (!IsInicjalize)
      return;

    if (needToSave || forceSave)
      EEPROM->writeByte(123, newValue, false, false);
  }

  uint8_t getRELAY_230_ALLOW_START_M()
  {
    return RELAY_230_ALLOW_START_M;
  }

  void setRELAY_230_ALLOW_START_M(uint8_t newValue, bool forceSave = false)
  {
    bool needToSave = newValue != RELAY_230_ALLOW_START_M;
    RELAY_230_ALLOW_START_M = newValue;

    if (!IsInicjalize)
      return;

    if (needToSave || forceSave)
      EEPROM->writeByte(124, newValue, false, false);
  }

  uint8_t getRELAY_230_ALLOW_END_H()
  {
    return RELAY_230_ALLOW_END_H;
  }

  void setRELAY_230_ALLOW_END_H(uint8_t newValue, bool forceSave = false)
  {
    bool needToSave = newValue != RELAY_230_ALLOW_END_H;
    RELAY_230_ALLOW_END_H = newValue;

    if (!IsInicjalize)
      return;

    if (needToSave || forceSave)
      EEPROM->writeByte(125, newValue, false, false);
  }

  uint8_t getRELAY_230_ALLOW_END_M()
  {
    return RELAY_230_ALLOW_END_M;
  }

  void setRELAY_230_ALLOW_END_M(uint8_t newValue, bool forceSave = false)
  {
    bool needToSave = newValue != RELAY_230_ALLOW_END_M;
    RELAY_230_ALLOW_END_M = newValue;

    if (!IsInicjalize)
      return;

    if (needToSave || forceSave)
      EEPROM->writeByte(126, newValue, false, false);
  }

  uint16_t getPOWER230V()
  {
    return POWER230V;
  }

  void setPOWER230V(uint16_t newValue, bool forceSave = false)
  {
    bool needToSave = newValue != POWER230V;
    POWER230V = newValue;

    if (!IsInicjalize)
      return;

    if (needToSave || forceSave)
      EEPROM->writeUInt32(130, (uint32_t)newValue, false, false);
  }

private:
  EE *EEPROM;
  bool IsInicjalize = false;
  const uint8_t readCheck = 0x53;

  //epprom setting
  uint16_t LCD_DISPLAY_TIMEOFF = 120;
  bool RELAY_PV_ENABLE = false;
  bool RELAY_230V_ENABLE = false;
  bool MYSENSORS_ENABLE = true;
  uint8_t MAX_WEATHER_TEMPERATURE = 50;

  bool RELAY_230V_ADVENCE = true;

  uint8_t RELAY_230_TEMP_START = 40;
  uint8_t RELAY_230_TEMP_HEAT_UP = 45;

  uint8_t RELAY_230_ALLOW_START_H = 17;
  uint8_t RELAY_230_ALLOW_START_M = 00;

  uint8_t RELAY_230_ALLOW_END_H = 19;
  uint8_t RELAY_230_ALLOW_END_M = 00;

  uint16_t POWER230V = 2100;

  void saveAll()
  {
    if (!IsInicjalize)
      return;

    setLcdTimeOff(LCD_DISPLAY_TIMEOFF, true);
    setMaxWeatherTemp(MAX_WEATHER_TEMPERATURE, true);
    setPVRelay(RELAY_PV_ENABLE, true);
    set230VRelay(RELAY_230V_ENABLE, true);
    setMySensorsEnable(MYSENSORS_ENABLE, true);
    setRELAY_230V_ADVENCE(RELAY_230V_ADVENCE, true);
    setRELAY_230_TEMP_START(RELAY_230_TEMP_START, true);
    setRELAY_230_TEMP_HEAT_UP(RELAY_230_TEMP_HEAT_UP, true);
    setRELAY_230_ALLOW_START_H(RELAY_230_ALLOW_START_H, true);
    setRELAY_230_ALLOW_START_M(RELAY_230_ALLOW_START_M, true);
    setRELAY_230_ALLOW_END_H(RELAY_230_ALLOW_END_H, true);
    setRELAY_230_ALLOW_END_M(RELAY_230_ALLOW_END_M, true);
    setPOWER230V(POWER230V, true);

    EEPROM->writeByte(100, readCheck, false, false);
  }

  void readAll()
  {
    if (!IsInicjalize)
      return;
    Serial.println("Odczytuje wszystkie ustawienia.");
    if (EEPROM->readByte(100) != readCheck)
    {
      saveAll();
      Serial.println("Zapisuje wszystkie ustawienia.");
      return;
    }

    readLcdTimeOff();
    readMaxWeatherTemp();
    readPVRelay();
    read230VRelay();
    readMySensorsEnable();
    readRelay230VAdvenceEnabled();
    readRELAY_230_TEMP_START();
    readRELAY_230_TEMP_HEAT_UP();
    readRELAY_230_ALLOW_START_H();
    readRELAY_230_ALLOW_START_M();
    readRELAY_230_ALLOW_END_H();
    readRELAY_230_ALLOW_END_M();
    readPOWER230V();
  }

  void readLcdTimeOff()
  {
    LCD_DISPLAY_TIMEOFF = (uint16_t)EEPROM->readUInt32(101);
  }

  void readMaxWeatherTemp()
  {
    MAX_WEATHER_TEMPERATURE = EEPROM->readByte(105);
  }

  void readPVRelay()
  {
    RELAY_PV_ENABLE = EEPROM->readByte(106) == 0x05;
  }

  void read230VRelay()
  {
    RELAY_230V_ENABLE = EEPROM->readByte(107) == 0x05;
  }

  void readMySensorsEnable()
  {
    MYSENSORS_ENABLE = EEPROM->readByte(108) == 0x05;
  }

  void readRelay230VAdvenceEnabled()
  {
    RELAY_230V_ADVENCE = EEPROM->readByte(120) == 0x05;
  }

  void readRELAY_230_TEMP_START()
  {
    RELAY_230_TEMP_START = EEPROM->readByte(121);
  }

  void readRELAY_230_TEMP_HEAT_UP()
  {
    RELAY_230_TEMP_HEAT_UP = EEPROM->readByte(122);
  }

  void readRELAY_230_ALLOW_START_H()
  {
    RELAY_230_ALLOW_START_H = EEPROM->readByte(123);
  }

  void readRELAY_230_ALLOW_START_M()
  {
    RELAY_230_ALLOW_START_M = EEPROM->readByte(124);
  }

  void readRELAY_230_ALLOW_END_H()
  {
    RELAY_230_ALLOW_END_H = EEPROM->readByte(125);
  }

  void readRELAY_230_ALLOW_END_M()
  {
    RELAY_230_ALLOW_END_M = EEPROM->readByte(126);
  }

  void readPOWER230V()
  {
    POWER230V = (uint16_t)EEPROM->readUInt32(130);
  }
};

#endif

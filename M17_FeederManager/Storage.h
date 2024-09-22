#include "24C32.h"

#define TRUE_VALUE 0x55
#define FALSE_VALUE 0x45

class Storage {
public:
  Storage(EE* eprom)
    : EEPROM24C32(eprom) {}

  void Inicjalize() {
    EEPROM24C32->begin(0x57, false);

    _isInicjalized = EEPROM24C32->checkPresence();

    if (_isInicjalized) {
      readAll();
    }
  }

  bool IsInicjalized() {
    return _isInicjalized;
  }

  bool enable24VOutput() {
    return _enable24VOutput;
  }

  bool enable5V1Output() {
    return _enable5V1Output;
  }

  bool enable5V2Output() {
    return _enable5V2Output;
  }

  bool enableMotionDetection() {
    return _enableMotionDetection;
  }

  bool enableClockSchedule() {
    return _enableClockSchedule;
  }

  uint16_t motionDetectedEnabledTime() {
    return _motionDetectedEnabledTime;
  }

  uint16_t clockScheduleEnabledTime() {
    return _clockScheduleEnabledTime;
  }

  uint8_t clockScheduleIntervalHour() {
    return _clockScheduleIntervalHour;
  }

  void setEnable24VOutput(bool state) {
    _enable24VOutput = state;
    EEPROM24C32->writeByte(106, state ? TRUE_VALUE : FALSE_VALUE, false, false);
  }

  void setEnable5V_1Output(bool state) {
    _enable5V1Output = state;
    EEPROM24C32->writeByte(107, state ? TRUE_VALUE : FALSE_VALUE, false, false);
  }

  void setEnable5V_2Output(bool state) {
    _enable5V2Output = state;
    EEPROM24C32->writeByte(108, state ? TRUE_VALUE : FALSE_VALUE, false, false);
  }

  void setEnableMotionDetection(bool state) {
    _enableMotionDetection = state;
    EEPROM24C32->writeByte(109, state ? TRUE_VALUE : FALSE_VALUE, false, false);
  }

  void setEnableClockSchedule(bool state) {
    _enableClockSchedule = state;
    EEPROM24C32->writeByte(110, state ? TRUE_VALUE : FALSE_VALUE, false, false);
  }

  void setMotionDetectedEnabledTime(uint16_t time) {
    if (time > 1200) {
      time = 1200;
    }

    if (time < 20) {
      time = 20;
    }

    _motionDetectedEnabledTime = time;
    EEPROM24C32->writeUInt32(111, time, false, false);
  }

  void setClockScheduleEnabledTime(uint16_t time) {
    if (time > 1200) {
      time = 1200;
    }

    if (time < 20) {
      time = 20;
    }

    _clockScheduleEnabledTime = time;
    EEPROM24C32->writeUInt32(115, time, false, false);
  }

  void setClockScheduleIntervalHour(uint8_t time) {
    if (time > 24) {
      time = 24;
    }

    if (time < 1) {
      time = 1;
    }

    _clockScheduleIntervalHour = time;
    EEPROM24C32->writeByte(119, time, false, false);
  }

private:
  EE* EEPROM24C32;

  bool _isInicjalized = false;

  bool _enable24VOutput = false;
  bool _enable5V1Output = false;
  bool _enable5V2Output = false;

  bool _enableMotionDetection = true;
  bool _enableClockSchedule = true;

  uint16_t _motionDetectedEnabledTime = 180;  //<20, 1200> sek.
  uint16_t _clockScheduleEnabledTime = 180;   //<20, 1200> 180 sek.
  uint8_t _clockScheduleIntervalHour = 3;     //<1-24> co 3 h, start from 00:00

  void readAll() {
    if (EEPROM24C32->readByte(105) != EEPROM_RESET) {
#if defined(DEBUG_GK)
      Serial.println("RESET SETTINGS");
#endif
      EEPROM24C32->writeByte(105, EEPROM_RESET, false, false);  //check value
      setEnable24VOutput(_enable24VOutput);
      setEnable5V_1Output(_enable5V1Output);
      setEnable5V_2Output(_enable5V2Output);
      setEnableMotionDetection(_enableMotionDetection);
      setEnableClockSchedule(_enableClockSchedule);
      setMotionDetectedEnabledTime(_motionDetectedEnabledTime);
      setClockScheduleEnabledTime(_clockScheduleEnabledTime);
      setClockScheduleIntervalHour(_clockScheduleIntervalHour);
    }

#if defined(DEBUG_GK)
    Serial.println("READ SETTINGS");
#endif

    _enable24VOutput = EEPROM24C32->readByte(106) == TRUE_VALUE;
    _enable5V1Output = EEPROM24C32->readByte(107) == TRUE_VALUE;
    _enable5V2Output = EEPROM24C32->readByte(108) == TRUE_VALUE;
    _enableMotionDetection = EEPROM24C32->readByte(109) == TRUE_VALUE;
    _enableClockSchedule = EEPROM24C32->readByte(110) == TRUE_VALUE;

    _motionDetectedEnabledTime = EEPROM24C32->readUInt32(111);
    _clockScheduleEnabledTime = EEPROM24C32->readUInt32(115);
    _clockScheduleIntervalHour = EEPROM24C32->readByte(119);
  }
};
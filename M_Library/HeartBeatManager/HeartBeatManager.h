/*
   dataneo @2018 - MS_Heartbeat
   MySensors Heartbeat Manager 1.0
   see https://sites.google.com/site/dataneosoftware/arduino/mysensors-heartbeat
*/

#include <MySensors.h>

class HeartBeatManager
{
public:
  HeartBeatManager()
  {
    HeartBeatManager(0);
  };
  HeartBeatManager(byte alarmBuzzerPin)
  {
    _lastSend = 0;
    _lastRecive = 0;
    _lastBuzzerAcive = 0;
    _alarmBuzzerPin = alarmBuzzerPin;
    if (_alarmBuzzerPin != 0)
      pinMode(_alarmBuzzerPin, OUTPUT);
  }
  void HeartBeat()
  {
    _currentTime = millis();
    if (_currentTime < _lastSend)
    {
      _lastRecive = 0;
      _lastBuzzerAcive = 0;
      SendHeart();
      return;
    }
    BuzzerCheck();
    if (_currentTime < _lastSend + _timeSendPeriod)
      return;
    SendHeart();
  }

  bool ComunicationIsOK()
  {
    return isAlive;
  }
  void ControllerReciveMsg()
  {
    _currentTime = millis();
    _lastRecive = _currentTime;
  }

private:
  byte _alarmBuzzerPin;
  bool isAlive;
  unsigned long _currentTime;
  unsigned long _lastSend;
  unsigned long _lastRecive;
  unsigned long _lastBuzzerAcive;
  const unsigned short _timeSendPeriod = 30 * 1000;    // miliseconds
  const unsigned short _timeReciveTimeout = 10 * 1000; //timeout miliseconds
  const unsigned short _toneLength = 500;              // time of buzzer active miliseconds
  const unsigned short _toneBreak = 500;               // time of buzzer break miliseconds
  const unsigned short _toneHz = 4000;                 // buzzer hz

  void SendHeart()
  {
    _lastSend = _currentTime;
    sendHeartbeat();
    if (_alarmBuzzerPin != 0)
      requestTime();
  }
  void BuzzerCheck()
  {
    if (_lastRecive < _lastSend &&
        ((_currentTime - _lastSend > _timeReciveTimeout) ||
         (_currentTime - _lastRecive > _timeSendPeriod + _timeReciveTimeout)))
    {
      isAlive = false;
      if (_alarmBuzzerPin != 0 && (_lastBuzzerAcive + _toneLength + _toneBreak < _currentTime))
      {
        _lastBuzzerAcive = _currentTime;
        tone(_alarmBuzzerPin, _toneHz, _toneLength);
      }
    }
    else
    {
      isAlive = true;
    }
  }
};

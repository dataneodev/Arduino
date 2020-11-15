/*
   dataneo @2019 - M14_BuzzerManager
   Buzzer Manager 1.0
*/

class BuzzerManager
{
public:
    BuzzerManager(byte alarmBuzzerPin)
    {
        _buzerActive = false;
        _lastBuzzerAcive = 0;
        _alarmBuzzerPin = alarmBuzzerPin;
        if (_alarmBuzzerPin != 0)
            pinMode(_alarmBuzzerPin, OUTPUT);
    }

    void ActiveBuzzerMelody1()
    {
        _buzerActive = true;
        _lastBuzzerAcive = 0;
        _actual = _m1;
    }

    void ActiveBuzzerMelody2()
    {
        _buzerActive = true;
        _lastBuzzerAcive = 0;
        _actual = _m2;
    }

    void ActiveBuzzerMelody3()
    {
        _buzerActive = true;
        _lastBuzzerAcive = 0;
        _actual = _m3;
    }

    void DeactiveBuzzer()
    {
        _buzerActive = false;
        _lastBuzzerAcive = 0;
        noTone(_alarmBuzzerPin);
    }

    void BuzzerCheck()
    {
        if (!_buzerActive)
            return;
        _currentTime = millis();
        if (_lastBuzzerAcive + *(_actual + 1) + *(_actual + 2) < _currentTime)
        {
            _lastBuzzerAcive = _currentTime;
            tone(_alarmBuzzerPin, *_actual, *(_actual + 1));
        }
    }

    bool IsBuzerActive()
    {
        return _buzerActive;
    }

private:
    unsigned short *_actual;
    unsigned short _m1[3] = {1000, 2000, 1500}; //{_toneHz, _toneLength, _toneBreak}
    unsigned short _m2[3] = {4000, 500, 500};
    unsigned short _m3[3] = {4000, 500, 500};

    byte _alarmBuzzerPin;
    bool _buzerActive;
    unsigned long _currentTime;
    unsigned long _lastBuzzerAcive;
};
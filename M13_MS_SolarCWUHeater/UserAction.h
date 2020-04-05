#ifndef USERACTION
#define USERACTION

class UserAction
{
public:
    UserAction(ClickEncoder *encoder) : _encoder(encoder) {}

    void Pool()
    {
        updateMillis();
    }

    unsigned long GetLastUserActionMillis()
    {
        return LAST_USER_ACTION_MILLS;
    }

    unsigned long GetMillis()
    {
        return NOW_MILLS;
    }

    bool GetMainCycle()
    {
        return checkCycleTime(MAIN_CYCLE_TIME);
    }

    bool IsEnterMenu()
    {
        if (_enterMenu)
        {
            _enterMenu = false;
            return true;
        }
        else
        {
            return false;
        }
    }

    bool IsBackMenu()
    {
        if (_backMenu)
        {
            _backMenu = false;
            return true;
        }
        else
        {
            return false;
        }
    }

    void SetBackMenu()
    {
        _backMenu = true;
        _enterMenu = false;
        updateMillis(true);
    }

    void BeginRegisterEnterMenu()
    {
        registerEnterMenu = true;
    }

    void EndRegisterEnterMenu()
    {
        registerEnterMenu = false;
    }

private:
    ClickEncoder *_encoder;

    unsigned long NOW_MILLS = 0;
    unsigned long LAST_USER_ACTION_MILLS = 0;
    unsigned long LAST_ACTIVE_CYCLE_TIME_MILLS;

    int16_t _lastRotaryValue;
    int16_t _currentRotaryValue;

    bool registerEnterMenu = false;
    bool _enterMenu = false;
    bool _backMenu = false;

    void updateMillis(bool forceUserActionUpdate = false)
    {
        unsigned long temp = millis();
        if (temp < NOW_MILLS)
        {
            LAST_USER_ACTION_MILLS = temp;
        }

        NOW_MILLS = temp;

        //check last user action
        if (forceUserActionUpdate)
        {
            LAST_USER_ACTION_MILLS = NOW_MILLS;
        }
        else
        {
            _currentRotaryValue += _encoder->getValue();
            if (_currentRotaryValue != _lastRotaryValue)
            {
                _lastRotaryValue = _currentRotaryValue;
                LAST_USER_ACTION_MILLS = NOW_MILLS;
            }
        }

        ClickEncoder::Button b = _encoder->getButton();
        if (b != ClickEncoder::Open)
        {
            LAST_USER_ACTION_MILLS = NOW_MILLS;

            if (registerEnterMenu)
            {
                switch (b)
                {
                case ClickEncoder::Clicked:
                case ClickEncoder::DoubleClicked:
                case ClickEncoder::Pressed:
                case ClickEncoder::Held:
                case ClickEncoder::Released:
                    _enterMenu = true;
                    break;
                }
            }
            else
            {
                _enterMenu = false;
            }
        }
    }

    bool checkCycleTime(unsigned long CYCLE_TIME)
    {
        //over
        if (NOW_MILLS < LAST_ACTIVE_CYCLE_TIME_MILLS)
        {
            LAST_ACTIVE_CYCLE_TIME_MILLS = NOW_MILLS;
            return true;
        }

        CYCLE_TIME += LAST_ACTIVE_CYCLE_TIME_MILLS;

        if (NOW_MILLS > CYCLE_TIME)
        {
            LAST_ACTIVE_CYCLE_TIME_MILLS = NOW_MILLS;
            return true;
        }
        return false;
    }
};

#endif
#ifndef CORE
#define CORE

//operation veriable
class Core
{
public:
    Core(DataLayer *DL, UserAction *userAction) : _dataLayer(DL), _userAction(userAction) {}

    void Inicjalize()
    {
        shutdownAll();
    }

    void Pool()
    {
        if (ERROR_CODE != 0)
        {
            shutdownAll();
            return;
        }

        if (!checkCycleTime())
            return;

        processCore();

        if (ERROR_CODE != 0)
        {
            shutdownAll();
            return;
        }
    }

private:
    DataLayer *_dataLayer;
    UserAction *_userAction;
    uint8_t PWM_DUTY = 0;
    const uint16_t CYCLE_TIME = 1000;
    unsigned long LAST_PROCESS_MILLS;

    bool checkCycleTime()
    {
        //over
        if (_userAction->GetMillis() < LAST_PROCESS_MILLS)
        {
            LAST_PROCESS_MILLS = _userAction->GetMillis();
            return true;
        }

        if (_userAction->GetMillis() > LAST_PROCESS_MILLS + ((unsigned long)CYCLE_TIME))
        {
            LAST_PROCESS_MILLS = _userAction->GetMillis();
            return true;
        }
        return false;
    }

    void processCore()
    {
    }

    void shutdownAll()
    {
        digitalWrite(PWM, LOW);
        digitalWrite(RELAY, LOW);
        PWM_DUTY = 0;
    }
};

#endif

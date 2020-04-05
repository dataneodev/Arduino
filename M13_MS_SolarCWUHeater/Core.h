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

    void Pool(bool cycle)
    {
        if (ERROR_CODE != 0)
        {
            shutdownAll();
            return;
        }

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

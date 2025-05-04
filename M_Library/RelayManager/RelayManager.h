/*
   dataneo @2019 - M1_MS_RelayManager
   MySensors Relay Manager 1.3
   see https://sites.google.com/site/dataneosoftware/arduino/mysensors-relay-manager
*/

#include <MySensors.h>
#include <ButtonDebounce.h>
#include <QList.h>
#include <24C32.h>
#include <Wire.h>

enum STATE_METHOD
{
  SAVE_TO_24C32,
  START_IN_HIGH,
  START_IN_LOW,
};

enum BUTTON_TYPE
{
  BISTABLE,
  MONOSTABLE
};

enum RELAY_STATE
{
  RELAY_ON_HIGH,
  RELAY_ON_LOW,
};

class Logger
{
public:
  static void logMsg(const char *msg)
  {
    Serial.println(msg);
  }
};

class EE24C32Loader
{
public:
EE24C32Loader(EE* eprom)
    : EEPROM24C32(eprom) {}

  static bool IsInicjalized()
  {
    if (!EEPROM24C32->checkPresence())
    {
      Logger::logMsg("EE24C32 initialization failed");
      return false;
    }
    return true;
  }


  static bool save24C32State(uint8_t _relay_pin_no, bool _relay_state)
  {
    if (_relay_pin_no > 0)
    {
      EEPROM24C32->write(_relay_pin_no, _relay_state ? 97 : 101);
    }
  }

  static bool loadStateFrom24C32(uint8_t _relay_pin_no)
  {
    if (EEPROM24C32->read(_relay_pin_no) == 97)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  private:
  EE* EEPROM24C32;

};


//Simple relay class
class RelaySimple
{
public:
  RelaySimple()
  {
    _relay_pin_no = 0;
    _save_state = START_IN_HIGH;
    _relay_on_state = RELAY_ON_HIGH;
  };

  RelaySimple(uint8_t relay_pin_no,
              STATE_METHOD save_state,
              RELAY_STATE relay_on_state,
              const char *relay_name)
  {
    _relay_pin_no = relay_pin_no;
    _save_state = save_state;
    _relay_on_state = relay_on_state;
    _relay_name = relay_name;
  };
  uint8_t relayPin()
  {
    return _relay_pin_no;
  }
  void initPin()
  {
    pinMode(_relay_pin_no, OUTPUT);
    if (_save_state == SAVE_TO_EEPROM)
    {
      _relay_state = loadState(_relay_pin_no);
      digitalWrite(_relay_pin_no, getGPIOState(_relay_state));
      sendStateToController();
    }

    if (_save_state == SAVE_TO_24C32)
    {
      _relay_state = EE24C32Loader::loadStateFrom24C32(_relay_pin_no);
      digitalWrite(_relay_pin_no, getGPIOState(_relay_state));
      sendStateToController();
    }

    if (_save_state == LOAD_FROM_CONTROLLERS)
    {
      // request load state from controler
      request(_relay_pin_no, V_STATUS);
    }
    if (_save_state == START_IN_HIGH)
    {
      _relay_state = true;
      digitalWrite(_relay_pin_no, getGPIOState(_relay_state));
      sendStateToController();
    }
    if (_save_state == START_IN_LOW)
    {
      _relay_state = false;
      sendStateToController();
    }
  }

  void presentToControler()
  {
    present(_relay_pin_no, S_BINARY, _relay_name);
  }

  void setStateFromControler(bool relay_state, CONTROLLER_TYPE controller)
  {
    if (_relay_state == relay_state)
      return;
    _relay_state = relay_state;
    digitalWrite(_relay_pin_no, getGPIOState(_relay_state));

    if (_save_state == SAVE_TO_EEPROM)
      saveState(_relay_pin_no, _relay_state);

    if (_save_state == SAVE_TO_24C32)
      EE24C32Loader::save24C32State(_relay_pin_no, _relay_state);

    if (controller == HOMEASSISTANT)
      sendStateToController();
  }

  void setButtonState()
  {
    _relay_state = !_relay_state;
    setRelayState(_relay_state);
  }

  void setButtonState(bool state)
  {
    _relay_state = state;
    setRelayState(_relay_state);
  }

  void setRelayState(bool relayState)
  {
    digitalWrite(_relay_pin_no, getGPIOState(_relay_state));

    if (_save_state == SAVE_TO_EEPROM)
      saveState(_relay_pin_no, _relay_state);

    if (_save_state == SAVE_TO_24C32)
      EE24C32Loader::save24C32State(_relay_pin_no, _relay_state);

    sendStateToController();
  }

  STATE_METHOD getStateMethod()
  {
    return _save_state;
  }

private:
  bool _relay_state;     // current relay state on or off
  uint8_t _relay_pin_no; // gpio pin for relay
  const char *_relay_name;
  static MyMessage mMessage;

  STATE_METHOD _save_state;
  RELAY_STATE _relay_on_state;

  uint8_t getGPIOState(bool relay_state)
  {
    return (_relay_on_state == RELAY_ON_HIGH) ? relay_state : !relay_state;
  }
  void sendStateToController()
  {
    mMessage.setSensor(_relay_pin_no);
    send(mMessage.set(_relay_state ? "1" : "0"));
  }
};

MyMessage RelaySimple::mMessage = MyMessage(1, V_STATUS);

class ButtonSimple
{
public:
  ButtonSimple()
  {
    _button_pin_no = 0;
  }
  ButtonSimple(uint8_t button_pin_no, BUTTON_TYPE button_type)
  {
    _button_pin_no = button_pin_no;
    _button_type = button_type;
  }
  uint8_t getButtonPinNo()
  {
    return _button_pin_no;
  }
  BUTTON_TYPE getButtonType()
  {
    return _button_type;
  }

  bool checkButton()
  {
    if (_button_pin_no == 0)
      return false;
    _debouncer.update();
    if (_button_type == BISTABLE)
    {
      if (_debouncer.fell())
        return true;
      return false;
    }
    else
    {
      bool readValue = _debouncer.read();
      if (readValue != lastState)
      {
        lastState = readValue;
        return true;
      }
      return false;
    }
  }

  bool getButtonState()
  {
    return lastState;
  }

  void initPin(bool pullUpPin)
  {
    if (_button_pin_no != 0)
    {
      if (pullUpPin)
      {
        pinMode(_button_pin_no, INPUT_PULLUP);
      }
      else
      {
        pinMode(_button_pin_no, INPUT);
      }

      _debouncer = Bounce();
      _debouncer.attach(_button_pin_no);
      _debouncer.interval(20);
    }
  }

private:
  uint8_t _button_pin_no;
  bool lastState = LOW;
  BUTTON_TYPE _button_type;
  Bounce _debouncer;
};

class RelayButtonPair
{
public:
  RelayButtonPair()
  {
    _relay_pin_no = 0;
    _button_pin_no = 0;
  }
  RelayButtonPair(uint8_t relay_pin_no, uint8_t button_pin_no)
  {
    _relay_pin_no = relay_pin_no;
    _button_pin_no = button_pin_no;
  }
  uint8_t relayPin()
  {
    return _relay_pin_no;
  }
  uint8_t getButtonPinNo()
  {
    return _button_pin_no;
  }

private:
  uint8_t _relay_pin_no;
  uint8_t _button_pin_no;
};

class RelayManager
{
public:
  RelayManager(CONTROLLER_TYPE controller,
               bool pullUpButtonPins = true,
               uint8_t EE24C32Address = 0x50)
  {
    if_init = false;
    _pullUpPins = pullUpButtonPins;
    _controller = controller;

    if (EE24C32Address != 0x50)
    {
      EE24C32Loader::SetAdress(EE24C32Address);
    }
  }
  //Simple Relay change status in RelayList
  void setStateOnRelayListFromControler(const MyMessage &message)
  {
    if (message.type != V_STATUS || message.isAck())
      return;
    if (relayList.length() > 0 && if_init)
      for (uint8_t i = 0; i < relayList.length(); i++)
        if (relayList[i].relayPin() == message.sensor)
          relayList[i].setStateFromControler(message.getBool(), _controller);
  }
  void presentAllToControler()
  {
    if (relayList.length() > 0)
      for (uint8_t i = 0; i < relayList.length(); i++)
        relayList[i].presentToControler();
    initAllPins();
  }
  void buttonCheckState()
  {
    if (buttonList.length() > 0 && if_init)
      for (uint8_t i = 0; i < buttonList.length(); i++)
        if (buttonList[i].checkButton())
          if (relayButtonPairList.length() > 0)
            for (uint8_t j = 0; j < relayButtonPairList.length(); j++)
              if (relayButtonPairList[j].getButtonPinNo() == buttonList[i].getButtonPinNo())
                if (relayList.length() > 0)
                  for (uint8_t k = 0; k < relayList.length(); k++)
                    if (relayList[k].relayPin() == relayButtonPairList[j].relayPin())
                    {
                      if (buttonList[i].getButtonType() == BISTABLE)
                      {
                        relayList[k].setButtonState();
                      }
                      else
                      {
                        relayList[k].setButtonState(buttonList[i].getButtonState());
                      }
                    }
  }

  void addRelay(uint8_t relay_pin_no)
  {
    addRelay(relay_pin_no, 0, SAVE_TO_EEPROM, RELAY_ON_HIGH, BISTABLE, "\0");
  }

  void addRelay(uint8_t relay_pin_no, uint8_t button_pin_no)
  {
    addRelay(relay_pin_no, button_pin_no, SAVE_TO_EEPROM, RELAY_ON_HIGH, BISTABLE, "\0");
  }

  void addRelay(uint8_t relay_pin_no, uint8_t button_pin_no, STATE_METHOD save_state, RELAY_STATE relay_on_state)
  {
    addRelay(relay_pin_no, button_pin_no, save_state, relay_on_state, BISTABLE, "\0");
  }

  void addRelay(uint8_t relay_pin_no,
                uint8_t button_pin_no,
                STATE_METHOD save_state,
                RELAY_STATE relay_on_state,
                BUTTON_TYPE button_type,
                const char *relay_name)
  {
    if (_controller == HOMEASSISTANT &&
        save_state == LOAD_FROM_CONTROLLERS)
    {
      save_state = START_IN_LOW;
    }

    //check if relay exists
    bool exist = false;
    if (relayList.length() > 0)
      for (uint8_t i = 0; i < relayList.length(); i++)
        if (relayList[i].relayPin() == relay_pin_no)
          exist = true;
    if (!exist)
    {
      relayList.push_back(RelaySimple(relay_pin_no,
                                      save_state,
                                      relay_on_state,
                                      relay_name));
    }

    //button check
    exist = false;
    if (buttonList.length() > 0)
      for (uint8_t i = 0; i < buttonList.length(); i++)
        if (buttonList[i].getButtonPinNo() == button_pin_no)
          exist = true;
    if (!exist && button_pin_no != 0)
      buttonList.push_back(ButtonSimple(button_pin_no, button_type));

    //pair exists
    exist = false;
    if (relayButtonPairList.length() > 0)
      for (uint8_t i = 0; i < relayButtonPairList.length(); i++)
        if (relayButtonPairList[i].relayPin() == relay_pin_no &&
            relayButtonPairList[i].getButtonPinNo() == button_pin_no)
          exist = true;
    if (!exist)
      relayButtonPairList.push_back(RelayButtonPair(relay_pin_no, button_pin_no));
  }

private:
  bool if_init;
  bool _pullUpPins;
  CONTROLLER_TYPE _controller;
  QList<ButtonSimple> buttonList;
  QList<RelaySimple> relayList;
  QList<RelayButtonPair> relayButtonPairList;

  void initAllPins()
  {
    if (if_init)
      return;

    if (relayList.length() > 0)
      for (uint8_t i = 0; i < relayList.length(); i++)
        relayList[i].initPin();

    if (buttonList.length() > 0)
      for (uint8_t i = 0; i < buttonList.length(); i++)
        buttonList[i].initPin(_pullUpPins);

    //inicjalize EE24C32
    bool ee24c32Exists = false;
    if (relayList.length() > 0)
      for (uint8_t i = 0; i < relayList.length(); i++)
        if (relayList[i].getStateMethod() == SAVE_TO_24C32)
        {
          ee24c32Exists = true;
          break;
        }

    if (ee24c32Exists)
    {
      if (!EE24C32Loader::IsInicjalized())
      {
        if_init = false;
        return;
      }
    }

    if_init = true;
  }
};

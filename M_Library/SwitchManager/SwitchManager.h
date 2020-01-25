/*
   dataneo @2018 - M4_MS_SwitchSensorManager
   MySensors Switch Sensor Manager 1.1
   Mechanical switch manager with debouncer
   see https://sites.google.com/site/dataneosoftware/arduino/switch-sensor-manager
*/

#include <MySensors.h>
#include <Bounce2.h>
#include <QList.h>

enum SWITCH_STATE {
  NORMAL_OPEN,
  NORMAL_CLOSE,
};

//Switch Sensor Manager
class SwitchSimple {
  public:
    SwitchSimple() {
      _switch_pin_no = 0;
      _switch_value = 0;
      _switch_state = NORMAL_OPEN;
    };
    SwitchSimple(byte switch_pin_no, SWITCH_STATE switchState, const char* switch_name) {
      _switch_pin_no = switch_pin_no;
      _switch_value = 0;
      _switch_state = switchState;
      _switch_name = switch_name;
    };
    byte switchPin() {
      return _switch_pin_no;
    }
    bool checkSwitch(bool forceSendToController = false) {
      _debouncer.update();
      bool readValue = _debouncer.read();
      if (readValue != _switch_value || forceSendToController) {
        _switch_value = readValue;
        sendStateToController();
      }
      return readValue;
    }
    void initPin() {
      pinMode(_switch_pin_no, INPUT_PULLUP);
      _debouncer = Bounce();
      _debouncer.attach(_switch_pin_no);
      _debouncer.interval(5);
      checkSwitch(true);
    }
    void presentToControler() {
      present(_switch_pin_no, S_DOOR, _switch_name);
    }
  private:
    Bounce _debouncer;
    SWITCH_STATE _switch_state;
    bool _switch_value;
    byte _switch_pin_no; // gpio pin for switch
    const char* _switch_name;
    static MyMessage mMessage;

    void sendStateToController() {
      bool state = _switch_value;
      if (_switch_state == NORMAL_OPEN)
        state = !state; 
      mMessage.setSensor(_switch_pin_no);
      send(mMessage.set(state ? "1" : "0"));
    }
};
MyMessage SwitchSimple::mMessage = MyMessage(1, V_TRIPPED);

class SwitchManager {
  public:
    SwitchManager() {
      if_init = false;
    }
    void presentAllToControler() {
      if (switchList.length() > 0)
        for (byte i = 0; i < switchList.length(); i++)
          switchList[i].presentToControler();
      initAllPins();
    }
    void switchCheckState() {
      if (switchList.length() > 0 && if_init)
        for (byte i = 0; i < switchList.length(); i++)
          switchList[i].checkSwitch(false);
    }
    void addSwitch(byte switch_pin_no) {
      addSwitch(switch_pin_no, NORMAL_CLOSE, "\0");
    }
    void addSwitch(byte switch_pin_no, SWITCH_STATE switch_state) {
      addSwitch(switch_pin_no, NORMAL_CLOSE, "\0");
    }
    void addSwitch(byte switch_pin_no, SWITCH_STATE switch_state, const char* switch_name) {
      //check if switch exists
      bool exist = false;
      if (switchList.length() > 0)
        for (byte i = 0; i < switchList.length(); i++)
          if (switchList[i].switchPin() == switch_pin_no)
            exist = true;
      if (!exist)
        switchList.push_back(SwitchSimple(switch_pin_no, switch_state, switch_name));
    }
  private:
    bool if_init;
    QList<SwitchSimple> switchList;

    void initAllPins() {
      if (switchList.length() > 0 && !if_init)
        for (byte i = 0; i < switchList.length(); i++)
          switchList[i].initPin();
      if_init = true;
    }
};

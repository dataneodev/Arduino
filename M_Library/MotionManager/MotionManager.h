/*
   dataneo @2018 - M5_MS_MotionSensorManager
   MySensors Motion Sensor Manager 1.2
   see https://sites.google.com/site/dataneosoftware/arduino/mysensors-motion-sensor-manager
*/
#include <QList.h>
#include <MySensors.h>

enum MOTION_STATE {
  SENSOR_ON_LOW,
  SENSOR_ON_HIGH,
};

enum CONTROLLER_TYPE {
  DOMOTICZ,
  HOMEASSISTANT,
  OTHER, // NOT TESTED
};

//Motion Sensor Manager
class MotionSimple {
  public:
    MotionSimple() {
      _motion_pin_no = 0;
      _motion_value = 0;
      _motion_state = SENSOR_ON_HIGH;
      _sensor = S_MOTION;
      _armed = true;
    };
    MotionSimple(byte motion_pin_no, MOTION_STATE motion_state, const char* motion_name, mysensors_sensor_t sensor_type, bool armedEnable = false) {
      _motion_pin_no = motion_pin_no;
      _motion_value = 0;
      _motion_state = motion_state;
      _motion_name = motion_name;
      _sensor = sensor_type;
      _armedEnable = armedEnable;
      _armed = true;
    };
    byte motionPin() {
      return _motion_pin_no;
    }
    bool checkmotion(bool forceSendToController = false) {
      if (_motion_pin_no == 0) return false;
      bool readValue = _armed ? (digitalRead(_motion_pin_no) == HIGH ? true : false) : false;
      if (readValue != _motion_value || forceSendToController) {
        _motion_value = readValue;
        sendStateToController();
      }
      return readValue;
    }
    void initPin() {
      if (_motion_pin_no == 0) return;
      pinMode(_motion_pin_no, INPUT);
      checkmotion(true);
    }
    void presentToControler() {
      if (_motion_pin_no == 0) return;
      present(_motion_pin_no, _sensor, _motion_name);
    }

    void setStateFromControler(bool state, CONTROLLER_TYPE controller) {
      _armed = state;
      sendStateToController();
    }
    bool IsArmedEnabled() {
      return _armedEnable;
    }
  private:
    MOTION_STATE _motion_state;
    mysensors_sensor_t _sensor;
    bool _motion_value;
    byte _motion_pin_no;
    bool _armedEnable;
    bool _armed;
    const char* _motion_name;
    static MyMessage mMessage;
    static MyMessage mMessageArmed;

    void sendStateToController() {
      bool state = _motion_value;
      if (_motion_state == SENSOR_ON_LOW)
        state = !state;
      mMessage.setSensor(_motion_pin_no);
      send(mMessage.set(state ? "1" : "0"));
      if (_armedEnable) {
        mMessageArmed.setSensor(_motion_pin_no);
        send(mMessageArmed.set(_armed ? "1" : "0"));
      }
    }
};

MyMessage MotionSimple::mMessage = MyMessage(1, V_TRIPPED);
MyMessage MotionSimple::mMessageArmed = MyMessage(1, V_ARMED);

class MotionManager {
  public:
    MotionManager(CONTROLLER_TYPE controller) {
      if_init = false;
      _controller = controller;
    }
    void presentAllToControler() {
      if (motionList.length() > 0)
        for (byte i = 0; i < motionList.length(); i++)
          motionList[i].presentToControler();
      initAllPins();
    }
    void motionCheckState() {
      if (motionList.length() > 0 && if_init)
        for (byte i = 0; i < motionList.length(); i++)
          motionList[i].checkmotion(false);
    }
    void addMotion(byte motion_pin_no) {
      addMotion(motion_pin_no, SENSOR_ON_HIGH, "\0", S_MOTION);
    }
    void addMotion(byte motion_pin_no, MOTION_STATE motion_state) {
      addMotion(motion_pin_no, SENSOR_ON_HIGH, "\0", S_MOTION);
    }
    void addMotion(byte motion_pin_no, MOTION_STATE motion_state, const char* motion_name, mysensors_sensor_t _sensor = S_MOTION) {
      //check if motion exists
      bool armed = false;
      if (_controller == HOMEASSISTANT)
        armed = true;
      bool exist = false;
      if (motionList.length() > 0)
        for (byte i = 0; i < motionList.length(); i++)
          if (motionList[i].motionPin() == motion_pin_no)
            exist = true;
      if (!exist)
        motionList.push_back(MotionSimple(motion_pin_no, motion_state, motion_name, _sensor, armed));
    }
    void setStatetFromControler(const MyMessage &message) {
      if (message.type != V_ARMED || message.isAck())
        return;
      if (motionList.length() > 0 && if_init)
        for (byte i = 0; i < motionList.length(); i++)
          if (motionList[i].motionPin() == message.sensor && motionList[i].IsArmedEnabled())
            motionList[i].setStateFromControler(message.getBool(), _controller);
    }
  private:
    bool if_init;
    CONTROLLER_TYPE _controller;
    QList<MotionSimple> motionList;
    void initAllPins() {
      if (motionList.length() > 0 && !if_init)
        for (byte i = 0; i < motionList.length(); i++)
          motionList[i].initPin();
      if_init = true;
    }
};

#ifndef RELAYMANAGER
#define RELAYMANAGER

#include <MySensors.h>
#include <QList.h>
#include "DataLayer.h"

enum CONTROLLER_TYPE
{
  DOMOTICZ,
  HOMEASSISTANT,
  OTHER, // NOT TESTED
};

//Simple relay class
class RelaySimple
{
public:
  RelaySimple(){};
  RelaySimple(uint8_t relay_id, const char *relay_name, DataLayer *dl)
  {
    _relay_id = relay_id;
    _relay_name = relay_name;
    loadState(_relay_id, dl);
  };

  uint8_t relayId()
  {
    return _relay_id;
  }

  void presentToControler()
  {
    present(_relay_id, S_BINARY, _relay_name);
    sendStateToController();
  }

  void setStateFromControler(bool relay_state, CONTROLLER_TYPE controller, DataLayer *dl)
  {
    if (_relay_state == relay_state)
      return;
    _relay_state = relay_state;

    saveState(_relay_id, _relay_state, dl);

    if (controller == HOMEASSISTANT)
      sendStateToController();
  }

  void setRelayState(bool relayState, DataLayer *dl)
  {
    _relay_state = relayState;
    saveState(_relay_id, _relay_state, dl);
    sendStateToController();
  }

private:
  bool _relay_state; // current relay state on or off
  uint8_t _relay_id; // gpio pin for relay
  const char *_relay_name;
  static MyMessage mMessage;

  void sendStateToController()
  {
    mMessage.setSensor(_relay_id);
    send(mMessage.set(_relay_state ? "1" : "0"));
  }

  void saveState(uint8_t relay_id, bool relayState, DataLayer *dl)
  {
    if (relay_id == RELAY_PV_ID)
      dl->setPVRelay(relayState);

    if (relay_id == RELAY_230V_ID)
      dl->set230VRelay(relayState);
  }

  void loadState(uint8_t relay_id, DataLayer *dl)
  {
    if (relay_id == RELAY_PV_ID)
      _relay_state = dl->getPVRelay();

    if (relay_id == RELAY_230V_ID)
      _relay_state = dl->get230VRelay();
  }
};

MyMessage RelaySimple::mMessage = MyMessage(1, V_STATUS);

class RelayManager
{
public:
  RelayManager(CONTROLLER_TYPE controller, DataLayer *dl) : dataLayer(dl)
  {
    _controller = controller;
  }
  //Simple Relay change status in RelayList
  void setStateOnRelayListFromControler(const MyMessage &message)
  {
    if (message.type != V_STATUS || message.isAck())
      return;
    setRelayState(message.sensor, message.getBool());
  }

  void setRelayState(uint8_t relayId, bool relayState)
  {
    if (relayList.length() > 0)
      for (uint8_t i = 0; i < relayList.length(); i++)
        if (relayList[i].relayId() == relayId)
          relayList[i].setStateFromControler(relayState, _controller, dataLayer);
  }

  void presentAllToControler()
  {
    if (relayList.length() > 0)
      for (uint8_t i = 0; i < relayList.length(); i++)
        relayList[i].presentToControler();
  }

  void addRelay(uint8_t relay_id,
                const char *relay_name)
  {
    //check if relay exists
    bool exist = false;
    if (relayList.length() > 0)
      for (uint8_t i = 0; i < relayList.length(); i++)
        if (relayList[i].relayId() == relay_id)
          exist = true;
    if (!exist)
      relayList.push_back(RelaySimple(relay_id, relay_name, dataLayer));
  }

private:
  CONTROLLER_TYPE _controller;
  DataLayer *dataLayer;
  QList<RelaySimple> relayList;
};

#endif
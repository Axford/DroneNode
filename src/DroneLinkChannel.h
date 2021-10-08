#ifndef DRONE_LINK_CHANNEL_H
#define DRONE_LINK_CHANNEL_H

#include "Arduino.h"
#include "LinkedList.h"

#include "DroneLinkMsg.h"
#include "DroneModule.h"
//#include "DroneLinkSubscriber.h"

#define DRONE_LINK_CHANNEL_QUEUE_LIMIT  30  // max number of queued messages


struct DroneLinkChannelSubscription {
  DroneModule* module;  // the subscribing module
  uint8_t param; // target param or ALL
};


class DroneLinkChannel
{
protected:
  uint8_t _node; // which node is associated with this channel
  uint8_t _id;  // channel id
  uint32_t _choked;
  IvanLinkedList::LinkedList<DroneLinkChannelSubscription> _subs;
  IvanLinkedList::LinkedList<DroneLinkMsg*> _queue;
  uint8_t _peakSize;

public:
    DroneLinkChannel(uint8_t node, uint8_t id):
      _node(node),
      _id(id),
      _subs(IvanLinkedList::LinkedList<DroneLinkChannelSubscription>()),
      _queue(IvanLinkedList::LinkedList<DroneLinkMsg*>())
    {
      Log.noticeln(F("Create channel: %d>%d"),node,id);
      _choked =0;
      _peakSize = 0;
    }

    uint8_t id() { return _id; }

    uint8_t node() { return _node; }

    uint8_t size() { return _queue.size(); }

    uint32_t choked() { return _choked; }

    uint8_t peakSize() { return _peakSize; }

    boolean publish(DroneLinkMsg msg) {
      if (_queue.size() < DRONE_LINK_CHANNEL_QUEUE_LIMIT) {
        //Log.errorln(F("Queue %d = %d"), _id, _queue.size());

        // TODO: do we pre-filter and only queue if there's a matching subscriber?
        // TODO: screen for matching sig already in queue

        // add to queue
        DroneLinkMsg *tmp = new DroneLinkMsg(msg);
        _queue.add(tmp);

        _peakSize = max(_peakSize, (uint8_t)_queue.size());

        return true;
      } else {
        Log.errorln(F("Channel queue full %d"), _id);
        _choked++;
        return false;
      }
    }

    void processQueue() {
      // pop a waiting message
      if (_queue.size() == 0) return;

      DroneLinkMsg *tmp = _queue.shift();

      // send msg to each subscriber
      DroneLinkChannelSubscription sub;
      for(int i = 0; i < _subs.size(); i++) {
        sub = _subs.get(i);
        // check paramMask
        //if (tmp->matchParam(sub.paramMask))
        if (tmp->param() == sub.param || sub.param == DRONE_LINK_PARAM_ALL)
          sub.module->handleLinkMessage(tmp);
      }

      // delete the tmp msg
      delete tmp;
    }

    void subscribe(DroneModule* subscriber, uint8_t param) {
      // check we don't already have this subscriber registered
      DroneLinkChannelSubscription sub;
      for(int i = 0; i < _subs.size(); i++) {
        sub = _subs.get(i);
        if (sub.module == subscriber && sub.param == param) return;
      }
      sub.module = subscriber;
      sub.param = param;
      _subs.add(sub);
    }
/*
    void printSubscribers() {
      Log.noticeln("DroneLinkChannel: %d", _id);
      DroneLinkChannelSubscription sub;
      for(int i = 0; i < _subs.size(); i++) {
        sub = _subs.get(i);
        Log.noticeln(sub.module->getName().c_str());
      }
    }*/
};


#endif

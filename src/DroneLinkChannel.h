#ifndef DRONE_LINK_CHANNEL_H
#define DRONE_LINK_CHANNEL_H

#include "Arduino.h"
#include "LinkedList.h"

#include "DroneLinkMsg.h"
#include "DroneModule.h"
#include <ESPAsyncWebServer.h>
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

    boolean publish(DroneLinkMsg &msg) {
      if (_queue.size() < DRONE_LINK_CHANNEL_QUEUE_LIMIT) {
        //Log.errorln(F("Queue %d = %d"), _id, _queue.size());

        // TODO: do we pre-filter and only queue if there's a matching subscriber?
        // TODO: prioritise numeric values in the custom param range (>7)

        // screen for matching sig already in queue
        DroneLinkMsg *scrub;
        boolean sameSig = false;
        for (uint8_t i=0; i<_queue.size(); i++) {
          scrub = _queue.get(i);
          if (scrub->sameSignature(&msg)) {
            Serial.print("Same sig in queue, overwriting: ");
            scrub->print();
            Serial.print( " with ");
            msg.print();

            sameSig = true;
            // overwrite message payload
            memcpy(scrub->_msg.payload.c, msg._msg.payload.c, msg.length());
            break;
          }
        }

        if (!sameSig) {
          DroneLinkMsg *tmp = new DroneLinkMsg(msg);

          if (msg.type() < DRONE_LINK_MSG_TYPE_CHAR) {
            // priority message, jump the queue
            // TODO: search back from the end until we find the first high priority item, then insert after it
            _queue.unshift(tmp); // quick cheat
          } else {
            // add to queue
            _queue.add(tmp);
          }

          _peakSize = max(_peakSize, (uint8_t)_queue.size());
        }

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

      //Log.noticeln("[DLC.pQ]");

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

    void serveChannelInfo(AsyncResponseStream *response) {
      DroneLinkChannelSubscription sub;
      for(int i = 0; i < _subs.size(); i++) {
        sub = _subs.get(i);

        response->printf("    %u: %s\n", sub.module->id(), sub.module->getName());
        //Log.noticeln(sub.module->getName().c_str());
      }
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

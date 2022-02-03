#include "DroneLinkChannel.h"
#include "DroneLinkManager.h"


DroneLinkChannel::DroneLinkChannel(DroneLinkManager* dlm, uint8_t node, uint8_t id):
  _dlm(dlm),
  _node(node),
  _id(id),
  _subs(IvanLinkedList::LinkedList<DroneLinkChannelSubscription*>()),
  _queue(IvanLinkedList::LinkedList<DroneLinkMsg*>())
{
  Log.noticeln(F("Create channel: %d>%d"),node,id);
  _choked =0;
  _peakSize = 0;
}

uint8_t DroneLinkChannel::id() {
  return _id;
} // channel id

uint8_t DroneLinkChannel::node() {
  return _node;
}

uint8_t DroneLinkChannel::size() {
  return _queue.size();
}

uint32_t DroneLinkChannel::choked() {
  return _choked;
}

uint8_t DroneLinkChannel::peakSize() {
  return _peakSize;
}


boolean DroneLinkChannel::publish(DroneLinkMsg &msg) {
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
        /*
        Serial.print("Same sig in queue, overwriting: ");
        scrub->print();
        Serial.print( " with ");
        msg.print();
        */

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

void DroneLinkChannel::processQueue() {
  // pop a waiting message
  if (_queue.size() == 0) return;

  DroneLinkMsg *tmp = _queue.shift();

  //Log.noticeln("[DLC.pQ] %u>%u", _node, _id);

  // send msg to each subscriber
  DroneLinkChannelSubscription *sub;
  for(int i = 0; i < _subs.size(); i++) {
    sub = _subs.get(i);
    // check paramMask
    //if (tmp->matchParam(sub.paramMask))
    if (tmp->param() == sub->param || sub->param == DRONE_LINK_PARAM_ALL) {
      //Log.noticeln("[DLC.pQ] to %s", sub.module->getName());
      if (sub->module) {
        // internal sub
        sub->module->handleLinkMessage(tmp);
      } else {
        // external sub - pass to DLM
        _dlm->sendDroneLinkMessage(sub->extNode, tmp);

      }
    }
  }

  // delete the tmp msg
  delete tmp;
  //Log.noticeln("[DLC.pQ] end");
}

void DroneLinkChannel::subscribe(uint8_t extNode, DroneModule* subscriber, uint8_t param) {
  // check we don't already have this subscriber registered
  DroneLinkChannelSubscription *sub;
  for(int i = 0; i < _subs.size(); i++) {
    sub = _subs.get(i);
    if (sub->extNode == extNode &&
        sub->module == subscriber &&
        sub->param == param)
        return;
  }

  sub = (DroneLinkChannelSubscription*)malloc(sizeof(DroneLinkChannelSubscription));
  sub->extNode = extNode;
  sub->module = subscriber;
  sub->param = param;
  sub->state = DRONE_LINK_CHANNEL_SUBSCRIPTION_PENDING;
  sub->timer = millis();
  _subs.add(sub);
}


void DroneLinkChannel::processExternalSubscriptions() {
  // not relevant if we represent a local channel
  if (_node == _dlm->node()) return;

  uint32_t loopTime = millis();

  // check all subs for pending external subs
  DroneLinkChannelSubscription *sub;
  for(int i = 0; i < _subs.size(); i++) {
    sub = _subs.get(i);
    if (sub->module != NULL) {
      if (sub->state == DRONE_LINK_CHANNEL_SUBSCRIPTION_PENDING) {
        // check timer
        if (loopTime > sub->timer + DRONE_LINK_CHANNEL_SUB_PENDING_RETRY_INTERVAL) {
          Log.noticeln("[DLC.pES] generating sub request");
          // ask DLM to generate a subscription request
          if (_dlm->generateSubscriptionRequest(_node, _id, sub->param)) {
            sub->state =DRONE_LINK_CHANNEL_SUBSCRIPTION_REQUESTED;
          }

          sub->timer = loopTime;
        }
      } else if (sub->state == DRONE_LINK_CHANNEL_SUBSCRIPTION_REQUESTED) {
        // check timer
        if (loopTime > sub->timer + DRONE_LINK_CHANNEL_SUB_REQUESTED_RETRY_INTERVAL) {
          Log.noticeln("[DLC.pES] retrying sub request");
          // ask DLM to generate a subscription request
          if (_dlm->generateSubscriptionRequest(_node, _id, sub->param)) {
            sub->state =DRONE_LINK_CHANNEL_SUBSCRIPTION_REQUESTED;
          }

          sub->timer = loopTime;
        }
      }
    }
  }

}


void DroneLinkChannel::confirmExternalSubscription(uint8_t param) {
  DroneLinkChannelSubscription *sub;
  for(int i = 0; i < _subs.size(); i++) {
    sub = _subs.get(i);
    if (sub->module != NULL &&
        sub->param == param) {

      sub->state = DRONE_LINK_CHANNEL_SUBSCRIPTION_CONFIRMED;
      return;
    }
  }
}


void DroneLinkChannel::removeExternalSubscriptions(uint8_t node) {

  // does this channel represent an external subscription?
  if (_node == node) {
    DroneLinkChannelSubscription *sub;
    // reset all subscription states
    for(int i = 0; i < _subs.size(); i++) {
      sub = _subs.get(i);
      sub->state = DRONE_LINK_CHANNEL_SUBSCRIPTION_PENDING;
    }
  } else {
    // perhaps the node that we've lost a route to was subscribed to us?
    DroneLinkChannelSubscription *sub;
    // reset all subscription states
    for(int i = 0; i < _subs.size(); i++) {
      sub = _subs.get(i);
      if (sub->module == NULL &&
          sub->extNode == node) {
        // need to remove the sub
        _subs.remove(i);
        // TODO - is this robust?
      }
    }
  }
}


void DroneLinkChannel::serveChannelInfo(AsyncResponseStream *response) {
  DroneLinkChannelSubscription *sub;
  for(int i = 0; i < _subs.size(); i++) {
    sub = _subs.get(i);

    if (sub->module) {
      if (_node != _dlm->node()) {
        response->printf("    module: %u: %s, state:%u\n", sub->module->id(), sub->module->getName(), sub->state);
      } else {
        response->printf("    module: %u: %s \n", sub->module->id(), sub->module->getName());
      }

    } else {
      response->printf("    node: %u\n", sub->extNode);
    }
  }
}

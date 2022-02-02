#ifndef DRONE_LINK_CHANNEL_H
#define DRONE_LINK_CHANNEL_H

#include "Arduino.h"
#include "LinkedList.h"

#include "DroneLinkMsg.h"
#include "DroneModule.h"
#include <ESPAsyncWebServer.h>
//#include "DroneLinkSubscriber.h"

#define DRONE_LINK_CHANNEL_QUEUE_LIMIT  30  // max number of queued messages

// subs are either internal modules or external (via the mesh)
struct DroneLinkChannelSubscription {
  uint8_t extNode;  // to indicate external subs
  DroneModule* module;  // the subscribing module, or NULL for external
  uint8_t param; // target param or ALL
  uint8_t state; // for external subscriptions
};

// external subscription states
#define DRONE_LINK_CHANNEL_SUBSCRIPTION_PENDING    0
#define DRONE_LINK_CHANNEL_SUBSCRIPTION_REQUESTED  1
#define DRONE_LINK_CHANNEL_SUBSCRIPTION_CONFIRMED  2

// forward declarations
class DroneLinkManager;


class DroneLinkChannel
{
protected:
  DroneLinkManager* _dlm;
  uint8_t _node; // which node is associated with this channel
  uint8_t _id;  // channel id
  uint32_t _choked;
  IvanLinkedList::LinkedList<DroneLinkChannelSubscription*> _subs;
  IvanLinkedList::LinkedList<DroneLinkMsg*> _queue;
  uint8_t _peakSize;

public:
    DroneLinkChannel(DroneLinkManager* dlm, uint8_t node, uint8_t id);

    uint8_t id();

    uint8_t node();

    uint8_t size();

    uint32_t choked();

    uint8_t peakSize();

    boolean publish(DroneLinkMsg &msg);

    void processQueue();

    void subscribe(uint8_t extNode, DroneModule* subscriber, uint8_t param);

    void processExternalSubscriptions();
    void confirmExternalSubscription(uint8_t extNode, uint8_t param);

    void serveChannelInfo(AsyncResponseStream *response);
};

#endif

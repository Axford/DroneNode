#ifndef DRONE_LINK_CHANNEL_H
#define DRONE_LINK_CHANNEL_H

#include "Arduino.h"
#include "LinkedList.h"

#include "DroneLinkMsg.h"
#include "DroneModule.h"
#include <ESPAsyncWebServer.h>
//#include "DroneLinkSubscriber.h"

#define DRONE_LINK_CHANNEL_QUEUE_LIMIT  30  // max number of queued messages

#define DRONE_LINK_CHANNEL_SUB_PENDING_RETRY_INTERVAL  1000
#define DRONE_LINK_CHANNEL_SUB_REQUESTED_RETRY_INTERVAL  5000
#define DRONE_LINK_CHANNEL_SUB_REDO_INTERVAL 60000


// subs are either internal modules or external (via the mesh)
struct DroneLinkChannelSubscription {
  uint8_t extNode;  // to indicate external subs
  DroneModule* module;  // the subscribing module, or NULL for external
  uint8_t param; // target param or ALL
  uint8_t state; // for external subscriptions
  uint32_t timer; // for external sub retries, etc
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
  uint8_t _maxSize;
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

    void resetExternalSubscriptions(uint8_t extNode);
    void processExternalSubscriptions();
    void confirmExternalSubscription(uint8_t param);
    void removeExternalSubscriptions(uint8_t node);

    void serveChannelInfo(AsyncResponseStream *response);
    void serveQueueInfo(AsyncResponseStream *response);
};

#endif

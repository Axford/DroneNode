#ifndef DRONE_LINK_MANAGER_H
#define DRONE_LINK_MANAGER_H

#include "Arduino.h"
#include "LinkedList.h"

#include "DroneLinkMsg.h"
#include "DroneLinkChannel.h"
#include <ESPAsyncWebServer.h>


struct DRONE_LINK_NODE_INFO {
  unsigned long lastHeard;
  boolean heard;
  uint8_t RSSI;  // signal strength of last packet, rounded to single digit and invert (e.g. -50dB becomes 50)
  // location?
  uint8_t interface;  // module ID of the network interface that heard this node
};

#define DRONE_LINK_NODE_PAGE_SIZE  16

struct DRONE_LINK_NODE_PAGE {
  DRONE_LINK_NODE_INFO nodeInfo[DRONE_LINK_NODE_PAGE_SIZE];
};

#define DRONE_LINK_NODE_PAGES  (256 / DRONE_LINK_NODE_PAGE_SIZE)

class DroneLinkManager
{
protected:
  uint8_t _node;  // local node id
  unsigned long _publishedMessages;
  IvanLinkedList::LinkedList<DroneLinkChannel*> _channels;

  // node map - nodes we've heard of and info about them
  DRONE_LINK_NODE_PAGE *_nodePages[DRONE_LINK_NODE_PAGES];

public:
    DroneLinkManager();

    void node(uint8_t node);
    uint8_t node();

    uint32_t getChokes();

    void subscribe(DRONE_LINK_ADDR *addr, DroneModule *subscriber);
    void subscribe(uint8_t channel, DroneModule *subscriber, uint8_t param);
    void subscribe(uint8_t node, uint8_t channel, DroneModule *subscriber, uint8_t param);

    bool publish(DroneLinkMsg msg);
    bool publishPeer(DroneLinkMsg msg, uint8_t RSSI, uint8_t interface);

    void processChannels();

    DroneLinkChannel* findChannel(uint8_t node, uint8_t chan);

    unsigned long publishedMessages();
    void resetPublishedMessages();

    void serveNodeInfo(AsyncWebServerRequest *request);
    void serveChannelInfo(AsyncWebServerRequest *request);
};


#endif

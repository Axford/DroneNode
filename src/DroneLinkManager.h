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
  char * name; // dynamically allocated to match heard name
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
  uint8_t _peerNodes;  // how many peer nodes have we heard
  uint8_t _minPeer;  // min id
  uint8_t _maxPeer;  // max id
  DRONE_LINK_NODE_PAGE *_nodePages[DRONE_LINK_NODE_PAGES];

public:
    DroneLinkManager();

    void node(uint8_t node);
    uint8_t node();

    uint32_t getChokes();

    void subscribe(DRONE_LINK_ADDR *addr, DroneModule *subscriber);
    void subscribe(uint8_t channel, DroneModule *subscriber, uint8_t param);
    void subscribe(uint8_t node, uint8_t channel, DroneModule *subscriber, uint8_t param);

    bool publish(DroneLinkMsg &msg);
    bool publishPeer(DroneLinkMsg &msg, int16_t RSSI, uint8_t interface);

    void processChannels();

    DroneLinkChannel* findChannel(uint8_t node, uint8_t chan);

    unsigned long publishedMessages();
    void resetPublishedMessages();

    uint8_t numPeers();
    uint8_t maxPeer();
    uint8_t minPeer();

    uint8_t getNodeByName(char * name);
    DRONE_LINK_NODE_INFO* getNodeInfo(uint8_t source);

    // get the interface associated with a source id
    uint8_t getSourceInterface(uint8_t source);

    void serveNodeInfo(AsyncWebServerRequest *request);
    void serveChannelInfo(AsyncWebServerRequest *request);
};


#endif

/*

DroneLinkManager

Manages the local set of pub/sub channels


*/
#ifndef DRONE_LINK_MANAGER_H
#define DRONE_LINK_MANAGER_H

#include "Arduino.h"
#include "LinkedList.h"

#include "DroneLinkMsg.h"
#include "DroneMeshMsg.h"
#include "DroneLinkChannel.h"
#include <ESPAsyncWebServer.h>

#include "droneModules/NetworkInterfaceModule.h"

// forward decl
class WiFiManager;

// aka routing entry
struct DRONE_LINK_NODE_INFO {
  unsigned long lastHeard;
  boolean heard;
  uint8_t metric;
  uint8_t seq;
  uint8_t nextHop;  // what node is the next hop
  NetworkInterfaceModule* interface;  // network interface that heard this node with the lowest metric
  char * name; // dynamically allocated to match heard name
};

#define DRONE_LINK_NODE_PAGE_SIZE  16

struct DRONE_LINK_NODE_PAGE {
  DRONE_LINK_NODE_INFO nodeInfo[DRONE_LINK_NODE_PAGE_SIZE];
};

#define DRONE_LINK_NODE_PAGES  (256 / DRONE_LINK_NODE_PAGE_SIZE)


#define DRONE_LINK_MANAGER_MAX_ROUTE_AGE    60000  // 60 sec


class DroneLinkManager
{
protected:
  WiFiManager *_wifiManager;
  uint8_t _node;  // local node id
  unsigned long _publishedMessages;
  IvanLinkedList::LinkedList<DroneLinkChannel*> _channels;
  DroneLinkMsg _receivedMsg;

  // routing table
  uint8_t _peerNodes;  // how many peer nodes have we heard
  uint8_t _minPeer;  // min id
  uint8_t _maxPeer;  // max id
  DRONE_LINK_NODE_PAGE *_nodePages[DRONE_LINK_NODE_PAGES];

  IvanLinkedList::LinkedList<NetworkInterfaceModule*> _interfaces;

public:
    DroneLinkManager(WiFiManager *wifiManager);

    void enableWiFi();
    void disableWiFi();
    boolean isWiFiEnabled();

    void node(uint8_t node);
    uint8_t node();

    uint32_t getChokes();

    void subscribe(DRONE_LINK_ADDR *addr, DroneModule *subscriber);
    void subscribe(uint8_t channel, DroneModule *subscriber, uint8_t param);
    void subscribe(uint8_t node, uint8_t channel, DroneModule *subscriber, uint8_t param);

    void subscribeExt(uint8_t extNode, uint8_t channel, uint8_t param);

    bool publish(DroneLinkMsg &msg);
    //bool publishPeer(DroneLinkMsg &msg, int16_t RSSI, uint8_t interface);

    void processChannels();
    void processExternalSubscriptions();

    void removeRoute(uint8_t node);

    void checkForOldRoutes();
    void loop();

    DroneLinkChannel* findChannel(uint8_t node, uint8_t chan);

    unsigned long publishedMessages();
    void resetPublishedMessages();

    uint8_t numPeers();
    uint8_t maxPeer();
    uint8_t minPeer();

    uint8_t getNodeByName(char * name);
    DRONE_LINK_NODE_INFO* getNodeInfo(uint8_t source, boolean heard);

    // get the interface associated with a source id
    NetworkInterfaceModule* getSourceInterface(uint8_t source);

    void serveNodeInfo(AsyncWebServerRequest *request);
    void serveChannelInfo(AsyncWebServerRequest *request);

    // mesh methods
    void registerInterface(NetworkInterfaceModule *interface);
    void receivePacket(NetworkInterfaceModule *interface, uint8_t *buffer, uint8_t metric);

    void receiveHello(NetworkInterfaceModule *interface, uint8_t *buffer, uint8_t metric);
    void receiveSubscription(NetworkInterfaceModule *interface, uint8_t *buffer, uint8_t metric);
    void receiveDroneLinkMsg(NetworkInterfaceModule *interface, uint8_t *buffer, uint8_t metric);
    void receiveTraceroute(NetworkInterfaceModule *interface, uint8_t *buffer, uint8_t metric);

    // standard forwarding mechanic for unicast packets
    virtual void hopAlong(uint8_t *buffer);

    virtual void generateResponse(uint8_t *buffer);

    boolean sendDroneLinkMessage(uint8_t extNode, DroneLinkMsg *msg);

    boolean generateSubscriptionRequest(uint8_t extNode, uint8_t channel, uint8_t param);

    // for terminal debugging
    void generateTraceroute(uint8_t destNode);
};


#endif

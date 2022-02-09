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
#include <FastCRC.h>

#include "droneModules/NetworkInterfaceModule.h"

// forward decl
class WiFiManager;


#define DRONE_LINK_MANAGER_MAX_TX_QUEUE    16

#define DRONE_LINK_MANAGER_HELLO_INTERVAL  5000
#define DRONE_LINK_MANAGER_SEQ_INTERVAL    30000

#define DRONE_LINK_MANAGER_MAX_RETRY_INTERVAL   5000
#define DRONE_LINK_MANAGER_MAX_RETRIES          10
#define DRONE_LINK_MANAGER_MAX_ACK_INTERVAL     500


// aka routing entry
struct DRONE_LINK_NODE_INFO {
  unsigned long lastHeard;
  unsigned long lastBroadcast;
  boolean heard;
  uint8_t metric;
  uint32_t uptime;  // how long has this node been up
  uint8_t seq; // last seq heard on route update (Hello)
  uint8_t nextHop;  // what node is the next hop
  NetworkInterfaceModule* interface;  // network interface that heard this node with the lowest metric originating on this node
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
  IvanLinkedList::LinkedList<DRONE_MESH_MSG_BUFFER*> _txQueue;

  uint8_t _helloSeq;
  uint32_t _helloTimer;
  uint32_t _seqTimer;
  FastCRC8 _CRC8;
  uint8_t _gSeq;  // seq number for guaranteed packets originating on this node

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

    void checkForOldRoutes(); // check for old routes and interface status
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

    void receiveAck(uint8_t *buffer);

    void receiveHello(NetworkInterfaceModule *interface, uint8_t *buffer, uint8_t metric);

    void receiveSubscriptionRequest(NetworkInterfaceModule *interface, uint8_t *buffer, uint8_t metric);
    void receiveSubscriptionResponse(NetworkInterfaceModule *interface, uint8_t *buffer, uint8_t metric);

    void receiveTracerouteRequest(NetworkInterfaceModule *interface, uint8_t *buffer, uint8_t metric);
    void receiveTracerouteResponse(NetworkInterfaceModule *interface, uint8_t *buffer, uint8_t metric);

    void receiveRouteEntryRequest(NetworkInterfaceModule *interface, uint8_t *buffer, uint8_t metric);
    void receiveRouteEntryResponse(NetworkInterfaceModule *interface, uint8_t *buffer, uint8_t metric);

    void receiveDroneLinkMsg(NetworkInterfaceModule *interface, uint8_t *buffer, uint8_t metric);

    // standard forwarding mechanic for unicast packets
    virtual void hopAlong(uint8_t *buffer);

    virtual void generateResponse(uint8_t *buffer, uint8_t newType);

    boolean sendDroneLinkMessage(uint8_t extNode, DroneLinkMsg *msg);

    boolean generateSubscriptionRequest(uint8_t extNode, uint8_t channel, uint8_t param);

    // for terminal debugging
    void generateTraceroute(uint8_t destNode);

    // ----------------------------------------------------------------
    // stuff brought in from Network Interface
    // ----------------------------------------------------------------

    uint8_t getTxQueueSize();
    DRONE_MESH_MSG_BUFFER* getTransmitBuffer(NetworkInterfaceModule *interface, uint8_t priority);
    void processTransmitQueue();

    boolean generateNextHop(NetworkInterfaceModule *interface, uint8_t *buffer, uint8_t nextHop);

    boolean generateAck(NetworkInterfaceModule *interface, uint8_t *buffer);

    void generateHellos();
    boolean generateHello(NetworkInterfaceModule *interface, uint8_t src, uint8_t seq, uint8_t metric, uint32_t uptime);

    boolean generateSubscriptionRequest(NetworkInterfaceModule *interface, uint8_t src, uint8_t next, uint8_t dest, uint8_t channel, uint8_t param);

    boolean generateTracerouteRequest(NetworkInterfaceModule *interface, uint8_t destNode, uint8_t nextNode);

    boolean generateRouteEntryResponse(NetworkInterfaceModule *interface,void * nodeInfo, uint8_t target, uint8_t dest, uint8_t nextHop);

    boolean sendDroneLinkMessage(NetworkInterfaceModule *interface, uint8_t destNode, uint8_t nextNode, DroneLinkMsg *msg);
};


#endif

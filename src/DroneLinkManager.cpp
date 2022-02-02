#include "Arduino.h"
#include "DroneLinkManager.h"
#include "WiFiManager.h"

DroneLinkManager::DroneLinkManager(WiFiManager *wifiManager):
  _wifiManager(wifiManager),
  _node(0),
  _channels(IvanLinkedList::LinkedList<DroneLinkChannel*>()),
  _interfaces(IvanLinkedList::LinkedList<NetworkInterfaceModule*>())
{
  _publishedMessages = 0;
  _peerNodes = 0;
  _minPeer = 255;
  _maxPeer = 0;
  for (uint8_t i=0; i<DRONE_LINK_NODE_PAGES; i++) {
    _nodePages[i] = NULL;
  }
}

void DroneLinkManager::enableWiFi() {
  _wifiManager->enable();
}

void DroneLinkManager::disableWiFi() {
  _wifiManager->disable();
}

boolean DroneLinkManager::isWiFiEnabled() {
  return _wifiManager->isEnabled();
}

void DroneLinkManager::node(uint8_t node) {
  _node = node;
}

uint8_t DroneLinkManager::node() {
  return _node;
}

uint32_t DroneLinkManager::getChokes() {
  uint32_t res = 0;
  DroneLinkChannel* c;
  for(int i = 0; i < _channels.size(); i++){
    c = _channels.get(i);
    res += c->choked();
  }
  return res;
}


void DroneLinkManager::subscribe(DRONE_LINK_ADDR *addr, DroneModule *subscriber) {
  subscribe(addr->node, addr->channel, subscriber, addr->param);
}


void DroneLinkManager::subscribe(uint8_t channel, DroneModule *subscriber, uint8_t param) {
  subscribe(_node, channel, subscriber, param);
}

void DroneLinkManager::subscribe(uint8_t node, uint8_t channel, DroneModule *subscriber, uint8_t param) {

  if (node == 255 || channel == 255 || param == 255) return;  // invalid address

  // locate a matching channel (or create it if it doesn't exist)
  DroneLinkChannel* c = findChannel(node, channel);
  if (!c) {
    Log.noticeln(F("Creating channel: %u > %u"), node, channel);
    c = new DroneLinkChannel(this, node, channel);
    _channels.add(c);
  }

  // wire up the subscriber to the channel
  c->subscribe(_node, subscriber, param);

  // print new set of subscribers
  //c->printSubscribers();
}


void DroneLinkManager::subscribeExt(uint8_t extNode, uint8_t channel, uint8_t param) {
  if (extNode == 255 || channel == 255 || param == 255) return;  // invalid address

  // locate a matching channel (ignore this request if doesnt exist)
  // given this is an external request, it must be for a channel on this _node
  DroneLinkChannel* c = findChannel(_node, channel);
  if (c) {
    // wire up the subscriber to the channel
    c->subscribe(extNode, NULL, param);
  }
}


bool DroneLinkManager::publish(DroneLinkMsg &msg) {
  // see if there's a matching channel... or a catchall channel
  bool f = false;

  _publishedMessages++;

  if (msg.type() == DRONE_LINK_MSG_TYPE_NAMEQUERY) {
    Serial.print("Publish: ");
    msg.print();
  }
  //Log.noticeln(F("Pub: %d>%d.%d"),msg.node(), msg.channel(), msg.param());

  // and publish to any matches
  DroneLinkChannel* c;
  for(int i = 0; i < _channels.size(); i++){
    c = _channels.get(i);
    if ((c->id() == msg.channel() || c->id() == DRONE_LINK_CHANNEL_ALL) && (c->node() == msg.node())) {
      //
      c->publish(msg);
      f = true;
    }
  }

  return f;
}

/*
bool DroneLinkManager::publishPeer(DroneLinkMsg &msg, int16_t RSSI, uint8_t interface) {

  // update nodeInfo map
  if (msg.source() != node()) {
    // get page
    uint8_t pageIndex = msg.source() >> 4;  // div by 16
    uint8_t nodeIndex = msg.source() & 0xF;

    // see if page exists
    DRONE_LINK_NODE_PAGE* page = _nodePages[pageIndex];

    if (page == NULL) {
      // create page
      _nodePages[pageIndex] = (DRONE_LINK_NODE_PAGE*)malloc(sizeof(DRONE_LINK_NODE_PAGE));
      page = _nodePages[pageIndex];

      // init
      for (uint8_t i=0; i<DRONE_LINK_NODE_PAGE_SIZE; i++) {
        page->nodeInfo[i].heard = false;
        page->nodeInfo[i].name = NULL;
      }
    }

    // update node info
    if (!page->nodeInfo[nodeIndex].heard) _peerNodes++;
    page->nodeInfo[nodeIndex].heard = true;
    page->nodeInfo[nodeIndex].lastHeard = millis();
    page->nodeInfo[nodeIndex].RSSI = abs(RSSI);
    page->nodeInfo[nodeIndex].interface = interface;

    _minPeer = min(_minPeer, msg.source());
    _maxPeer = max(_maxPeer, msg.source());

    // is this a node name message ?
    if (msg.channel() == 1 &&
        msg.param() == 8 &&
        msg.type() == DRONE_LINK_MSG_TYPE_CHAR) {
          // do we need to allocate memory for the name?
          if (page->nodeInfo[nodeIndex].name == NULL) {
            page->nodeInfo[nodeIndex].name = (char*)malloc(17);
          }

          //copy the new name
          memcpy(page->nodeInfo[nodeIndex].name, msg._msg.payload.c, msg.length());
          page->nodeInfo[nodeIndex].name[msg.length()] = '\0';
        }
  }

  return publish(msg);
}
*/

void DroneLinkManager::processChannels() {
  DroneLinkChannel* c;
  for(int i = 0; i < _channels.size(); i++){
    //Log.noticeln("[DLM.pC] %u", i);
    c = _channels.get(i);
    c->processQueue();
  }
}


void DroneLinkManager::processExternalSubscriptions() {
  // check for pending subscriptions

  DroneLinkChannel* c;
  for(int i = 0; i < _channels.size(); i++){
    c = _channels.get(i);
    c->processExternalSubscriptions();
  }
}


void DroneLinkManager::removeRoute(uint8_t node) {
  DRONE_LINK_NODE_INFO* nodeInfo = getNodeInfo(node, false);
  if (nodeInfo && nodeInfo->heard) {
    Log.noticeln("[DLM.rR] Removing route to %u", node);
    nodeInfo->heard = false;

    // udpate state for any associated ext subs
    DroneLinkChannel* c;
    for(int i = 0; i < _channels.size(); i++){
      c = _channels.get(i);
      c->removeExternalSubscriptions(node);
    }
  }
}


void DroneLinkManager::checkForOldRoutes() {
  DRONE_LINK_NODE_PAGE *page;
  for (uint8_t i=0; i<DRONE_LINK_NODE_PAGES; i++) {
    page = _nodePages[i];
    if (page != NULL) {
      for (uint8_t j=0; j<DRONE_LINK_NODE_PAGE_SIZE; j++) {
        if (page->nodeInfo[j].heard) {
          uint8_t n = i*DRONE_LINK_NODE_PAGE_SIZE + j;

          // check age
          if (millis() - page->nodeInfo[j].lastHeard > DRONE_LINK_MANAGER_MAX_ROUTE_AGE) {
            // remove route
            removeRoute(n);
          }
        }
      }
    }
  }
}


void DroneLinkManager::loop() {
  static uint32_t processTimer = 0;
  uint32_t loopTime = millis();

  // process local channel messages
  processChannels();

  if (loopTime > processTimer + 1000) {
    // process ext subs
    processExternalSubscriptions();

    checkForOldRoutes();

    processTimer = loopTime;
  }
}


DroneLinkChannel* DroneLinkManager::findChannel(uint8_t node, uint8_t chan) {
  DroneLinkChannel* c;
  for(int i = 0; i < _channels.size(); i++){
    c = _channels.get(i);
    if ((c->id() == chan) && (c->node() == node)) {
      return c;
    }
  }
  return NULL;
}


unsigned long DroneLinkManager::publishedMessages() {
  return _publishedMessages;
}

void DroneLinkManager::resetPublishedMessages() {
  _publishedMessages = 0;
}


uint8_t DroneLinkManager::numPeers() {
  return _peerNodes;
}

uint8_t DroneLinkManager::maxPeer() {
  return _maxPeer;
}

uint8_t DroneLinkManager::minPeer() {
  return _minPeer;
}

uint8_t DroneLinkManager::getNodeByName(char * name) {
  DRONE_LINK_NODE_PAGE *page;
  for (uint8_t i=0; i<DRONE_LINK_NODE_PAGES; i++) {
    page = _nodePages[i];
    if (page != NULL) {
      for (uint8_t j=0; j<DRONE_LINK_NODE_PAGE_SIZE; j++) {
        if (page->nodeInfo[j].name != NULL) {
          if (strcmp(page->nodeInfo[j].name, name) ==0) {
            uint8_t id = (i << 4) + j;
            return id;
          }
        }
      }
    }
  }
  return 0;
}

DRONE_LINK_NODE_INFO* DroneLinkManager::getNodeInfo(uint8_t source, boolean heard) {
  // get page
  uint8_t pageIndex = source >> 4;  // div by 16
  uint8_t nodeIndex = source & 0xF;

  // see if page exists
  DRONE_LINK_NODE_PAGE* page = _nodePages[pageIndex];

  if (page == NULL && heard) {
    // create page
    _nodePages[pageIndex] = (DRONE_LINK_NODE_PAGE*)malloc(sizeof(DRONE_LINK_NODE_PAGE));
    page = _nodePages[pageIndex];

    // init
    for (uint8_t i=0; i<DRONE_LINK_NODE_PAGE_SIZE; i++) {
      page->nodeInfo[i].heard = false;
      page->nodeInfo[i].seq = 0;
      page->nodeInfo[i].metric = 255;
      page->nodeInfo[i].name = NULL;
      page->nodeInfo[i].interface = NULL;
      page->nodeInfo[i].nextHop = 0;
    }
  }

  if (page != NULL) {

    if (heard) {
      if (!page->nodeInfo[nodeIndex].heard) {
        _peerNodes++;
        page->nodeInfo[nodeIndex].heard = true;
      }
      page->nodeInfo[nodeIndex].lastHeard = millis();
    }

    return &page->nodeInfo[nodeIndex];
  } else
    return NULL;
}

NetworkInterfaceModule* DroneLinkManager::getSourceInterface(uint8_t source) {
  DRONE_LINK_NODE_INFO* nodeInfo = getNodeInfo(source, false);

  if (nodeInfo == NULL) return NULL;
  return nodeInfo->interface;
}


void DroneLinkManager::serveNodeInfo(AsyncWebServerRequest *request) {
  AsyncResponseStream *response = request->beginResponseStream("text/text");
  response->addHeader("Server","ESP Async Web Server");

  response->print(F("Interfaces: \n"));

  for (uint8_t i=0; i < _interfaces.size(); i++) {
    NetworkInterfaceModule* interface = _interfaces.get(i);

    response->printf("  %s, queue size: %u\n", interface->getName(), interface->getTxQueueSize() );
  }


  response->print(F("\n\nRouting table: \n"));

  unsigned long loopTime = millis();

  DRONE_LINK_NODE_PAGE *page;
  for (uint8_t i=0; i<DRONE_LINK_NODE_PAGES; i++) {
    page = _nodePages[i];
    if (page != NULL) {
      for (uint8_t j=0; j<DRONE_LINK_NODE_PAGE_SIZE; j++) {
        if (page->nodeInfo[j].heard) {
          uint8_t id = (i << 4) + j;
          unsigned int age = (loopTime - page->nodeInfo[j].lastHeard) / 1000;
          char *interfaceName = NULL;
          if (page->nodeInfo[j].interface) interfaceName = page->nodeInfo[j].interface->getName();
          response->printf("  %u > ", id);
          if (page->nodeInfo[j].name == NULL) {
            response->print("???");
          } else
            response->printf("%s", page->nodeInfo[j].name);
          response->printf(", Seq: %u, Metric: %u, Next Hop: %u, Age: %u sec, Int: %s\n", page->nodeInfo[j].seq, page->nodeInfo[j].metric, page->nodeInfo[j].nextHop, age, interfaceName);
        }
      }
    }
  }

  //send the response last
  request->send(response);
}


void DroneLinkManager::serveChannelInfo(AsyncWebServerRequest *request) {

  AsyncResponseStream *response = request->beginResponseStream("text/text");
  response->addHeader("Server","ESP Async Web Server");
  response->print(F("Channels, queues and subscribers \n"));

  DroneLinkChannel* c;
  for(int i = 0; i < _channels.size(); i++){
    c = _channels.get(i);
    response->printf("%u>%u = size: %u (peak: %u)\n", c->node(), c->id(), c->size(), c->peakSize());

    c->serveChannelInfo(response);
    response->print("\n");
  }


  //send the response last
  request->send(response);
}


// -----------------------------------------------------------------------------
// mesh methods
// -----------------------------------------------------------------------------

void DroneLinkManager::registerInterface(NetworkInterfaceModule *interface) {
  _interfaces.add(interface);
}


void DroneLinkManager::receivePacket(NetworkInterfaceModule *interface, uint8_t *buffer, uint8_t metric) {
  // this is the meaty stuff...
  // do something appropriate with it depending on its routing mode, type, etc
  uint8_t type = getDroneMeshMsgType(buffer);

  // update lastHeard for txNode
  DRONE_MESH_MSG_HEADER *header = (DRONE_MESH_MSG_HEADER*)buffer;
  DRONE_LINK_NODE_INFO* txNodeInfo = getNodeInfo(header->txNode, false);
  if (txNodeInfo) txNodeInfo->lastHeard = millis();

  // pass to receive handler
  switch (type) {
    case DRONE_MESH_MSG_TYPE_HELLO: receiveHello(interface, buffer, metric); break;
    case DRONE_MESH_MSG_TYPE_SUBSCRIPTION: receiveSubscription(interface, buffer, metric); break;
    case DRONE_MESH_MSG_TYPE_DRONELINKMSG: receiveDroneLinkMsg(interface, buffer, metric); break;
    case DRONE_MESH_MSG_TYPE_TRACEROUTE: receiveTraceroute(interface, buffer, metric); break;
  }
}


void DroneLinkManager::receiveHello(NetworkInterfaceModule *interface, uint8_t *buffer, uint8_t metric) {
  DRONE_MESH_MSG_HEADER *header = (DRONE_MESH_MSG_HEADER*)buffer;

  if (getDroneMeshMsgDirection(buffer) != DRONE_MESH_MSG_REQUEST) return;

  Log.noticeln("[DLM.rP] Hello from %u tx by %u", header->srcNode, header->txNode);

  DRONE_MESH_MSG_HELLO *hello = (DRONE_MESH_MSG_HELLO*)buffer;

  // ignore it if this hello packet is from us
  if (header->srcNode == _node) return;

  // calc total metric, inc RSSI to us
  uint32_t newMetric = constrain(hello->metric + metric, 0, 255);


  // fetch node info (routing entry), create if needed
  DRONE_LINK_NODE_INFO* nodeInfo = getNodeInfo(header->srcNode, true);

  if (nodeInfo) {

    // if its a brand new route entry it will have metric 255... so good to overwrite
    boolean feasibleRoute = nodeInfo->metric == 255;

    if (interface != nodeInfo->interface && newMetric < nodeInfo->metric) {
      feasibleRoute = true;
    } else {
      // is this a new sequence (allow for wrap-around)
      if ((header->seq > nodeInfo->seq) || (nodeInfo->seq > 128 && (header->seq < nodeInfo->seq - 128))) {
        feasibleRoute = true;
        Log.noticeln("New seq %u", header->seq);
      }

      // or is it the same, but with a better metric
      if (header->seq == nodeInfo->seq &&
          newMetric < nodeInfo->metric) {
        feasibleRoute = true;
        Log.noticeln("Better metric %u", newMetric);
      }
    }

    if (feasibleRoute) {
      Log.noticeln("Updating route info");
      nodeInfo->seq = header->seq;
      nodeInfo->metric = newMetric;
      nodeInfo->interface = interface;
      nodeInfo->nextHop = header->txNode;

      // if metric < 255 then retransmit the Hello on all interfaces
      if (newMetric < 255) {
        for (uint8_t i=0; i < _interfaces.size(); i++) {
          NetworkInterfaceModule* interface = _interfaces.get(i);
          interface->generateHello(header->srcNode, header->seq, newMetric);
        }
      }

    } else {
      Log.noticeln("New route infeasible");
    }
  }
}


void DroneLinkManager::receiveSubscription(NetworkInterfaceModule *interface, uint8_t *buffer, uint8_t metric) {
  DRONE_MESH_MSG_HEADER *header = (DRONE_MESH_MSG_HEADER*)buffer;
  DRONE_MESH_MSG_SUBCSRIPTION *subBuffer = (DRONE_MESH_MSG_SUBCSRIPTION*)buffer;

  Log.noticeln("[DLM.rS] Sub from %u to %u", header->srcNode, header->destNode);

  if (getDroneMeshMsgDirection(buffer) == DRONE_MESH_MSG_REQUEST) {
    // request
    Log.noticeln("  Request:");

    // check if we are the nextNode... otherwise ignore it
    if (header->nextNode == _node) {
      // are we the destination?
      if (header->destNode == _node) {
        Log.noticeln("[DLM.rS] Reached destination");

        // make a note that we need to send stuff to the subscribing (src) node
        subscribeExt(header->srcNode, subBuffer->channel, subBuffer->param);

        // generate a response
        generateResponse(buffer);

      } else {
        Log.noticeln("[DLM.rS] Intermediate hop");

        hopAlong(buffer);
      }
    }

  } else {
    // response
    Log.noticeln("  Response:");

    // check if we are the destination... otherwise ignore it
    if (header->destNode == _node) {

      // find matching sub channel and update state
      DroneLinkChannel* c = findChannel(header->srcNode, subBuffer->channel);
      if (c) {
        c->confirmExternalSubscription(subBuffer->param);
      }

    } else {
      hopAlong(buffer);
    }
  }
}


void DroneLinkManager::receiveDroneLinkMsg(NetworkInterfaceModule *interface, uint8_t *buffer, uint8_t metric) {
  DRONE_MESH_MSG_HEADER *header = (DRONE_MESH_MSG_HEADER*)buffer;

  Log.noticeln("[DLM.rDlM] DLM from %u to %u", header->srcNode, header->destNode);

  if (getDroneMeshMsgDirection(buffer) == DRONE_MESH_MSG_REQUEST) {
    // check if we are the nextNode... otherwise ignore it
    if (header->nextNode == _node) {
      // are we the destination?
      if (header->destNode == _node) {
        Log.noticeln("[DLM.rDlM] Reached destination");

        // upwrap contained msg
        uint8_t payloadSize = getDroneMeshMsgPayloadSize(buffer);

        memcpy((uint8_t*)&_receivedMsg._msg, &buffer[sizeof(DRONE_MESH_MSG_HEADER)], payloadSize);

        // publish contained message on local bus
        publish(_receivedMsg);

      } else {
        Log.noticeln("[DLM.rDlM] Intermediate hop");

        hopAlong(buffer);
      }
    }

  }
}


void DroneLinkManager::receiveTraceroute(NetworkInterfaceModule *interface, uint8_t *buffer, uint8_t metric) {
  DRONE_MESH_MSG_HEADER *header = (DRONE_MESH_MSG_HEADER*)buffer;

  Log.noticeln("[DLM.rT] Traceroute from %u to %u", header->srcNode, header->destNode);

  if (getDroneMeshMsgDirection(buffer) == DRONE_MESH_MSG_REQUEST) {
    // check if we are the nextNode... otherwise ignore it
    if (header->nextNode == _node) {

      // tweak buffer to add our traceroute info
      uint8_t payloadLen = getDroneMeshMsgPayloadSize(buffer);

      if (payloadLen < DRONE_MESH_MSG_MAX_PAYLOAD_SIZE-2) {
        // calc index of insertion point
        uint8_t p = sizeof(DRONE_MESH_MSG_HEADER) + payloadLen;

        // add our info
        buffer[p] = _node;
        buffer[p+1] = metric;

        // update payload size
        setDroneMeshMsgPayloadSize(buffer, payloadLen+2);

        // no need to recalc CRC, as that will be dealt with by generateResponse or hopAlong
      }

      // are we the destination?
      if (header->destNode == _node) {
        Log.noticeln("  Reached destination");

        // generate a response
        generateResponse(buffer);

      } else {
        Log.noticeln("  Intermediate hop");

        hopAlong(buffer);
      }
    }

  } else {
    // response
    //Log.noticeln("  Response:");

    // check if we are the destination... otherwise ignore it
    if (header->destNode == _node) {
      Log.noticeln("  Traceroute received:");

      // list traceroute info
      uint8_t payloadLen = getDroneMeshMsgPayloadSize(buffer);

      uint8_t p = sizeof(DRONE_MESH_MSG_HEADER);
      for (uint8_t i=0; i < payloadLen/2; i++) {
        Log.noticeln("    %u, metric: %u", buffer[p], buffer[p+1]);
        p += 2;
      }

    } else {
      Log.noticeln("  Response - hopAlong");
      hopAlong(buffer);
    }
  }
}


void DroneLinkManager::hopAlong(uint8_t *buffer) {
  DRONE_MESH_MSG_HEADER *header = (DRONE_MESH_MSG_HEADER*)buffer;

  Log.noticeln("[DLM.hA] %u --> %u", header->srcNode, header->destNode);

  // check if we are the nextNode... otherwise ignore it
  if (header->nextNode == _node) {
    // are we the destination?
    if (header->destNode == _node) {
      //Log.noticeln("[DLM.rS] Reached destination");

    } else {
      //Log.noticeln("[DLM.hA] Intermediate hop");

      // get next hop
      DRONE_LINK_NODE_INFO* nodeInfo = getNodeInfo(header->destNode, false);
      if (nodeInfo && nodeInfo->heard) {
        // hand to interface to manage the next hop
        if (nodeInfo->interface) {
          if (nodeInfo->interface->generateNextHop(buffer, nodeInfo->nextHop)) {
            // next hop generated ok
          } else {
            // unable to generate next hop... e.g. queue full or interface down
          }
        }
      } else {
        // now what?
        Log.errorln("[DLM.rS] No route to destination %u", header->destNode);
      }

    }
  }
}


void DroneLinkManager::generateResponse(uint8_t *buffer) {
  // generate a standard response by copying the packet and swapping the src/dest

  DRONE_MESH_MSG_HEADER *header = (DRONE_MESH_MSG_HEADER*)buffer;

  Log.noticeln("[DLM.gR] Responding %u --> %u", header->destNode, header->srcNode);

  // flip src and dest
  uint8_t oldSrc = header->srcNode;
  header->srcNode = header->destNode;
  header->destNode = oldSrc;

  // change direction to response
  header->typeDir = getDroneMeshMsgType(buffer) | DRONE_MESH_MSG_RESPONSE;

  // set nextNode to ourself so we can use hopAlong logic
  header->nextNode = _node;

  // use hopAlong to start the response
  hopAlong(buffer);
}


boolean DroneLinkManager::sendDroneLinkMessage(uint8_t extNode, DroneLinkMsg *msg) {
  // see if we have a valid route to the target node
  DRONE_LINK_NODE_INFO* nodeInfo = getNodeInfo(extNode, false);
  if (nodeInfo && nodeInfo->heard) {
    // generate a subscription request on the relevant interface
    if (nodeInfo->interface)
      return nodeInfo->interface->sendDroneLinkMessage(extNode, nodeInfo->nextHop, msg);
  }

  return false;
}


boolean DroneLinkManager::generateSubscriptionRequest(uint8_t extNode, uint8_t channel, uint8_t param) {

  // see if we have a valid route to the target node
  DRONE_LINK_NODE_INFO* nodeInfo = getNodeInfo(extNode, false);
  if (nodeInfo && nodeInfo->heard) {
    // generate a subscription request on the relevant interface
    if (nodeInfo->interface &&
        nodeInfo->interface->generateSubscriptionRequest(_node, nodeInfo->nextHop, extNode, channel, param)) {
      return true;
    } else {
      Log.warningln("[DLM.gSR] failed to generate sub request %u", extNode);
    }
  } else {
    // no valid route
    Log.warningln("[DLM.gSR] no valid route to %u", extNode);
  }

  return false;
}


void DroneLinkManager::generateTraceroute(uint8_t destNode) {
  // see if we have a valid route to the target node
  DRONE_LINK_NODE_INFO* nodeInfo = getNodeInfo(destNode, false);
  if (nodeInfo && nodeInfo->heard) {
    // generate a traceroute request on the relevant interface
    if (nodeInfo->interface) {
        nodeInfo->interface->generateTraceroute(destNode, nodeInfo->nextHop);
    }
  }
}

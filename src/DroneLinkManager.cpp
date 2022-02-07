#include "Arduino.h"
#include "DroneLinkManager.h"
#include "WiFiManager.h"

DroneLinkManager::DroneLinkManager(WiFiManager *wifiManager):
  _wifiManager(wifiManager),
  _node(0),
  _channels(IvanLinkedList::LinkedList<DroneLinkChannel*>()),
  _interfaces(IvanLinkedList::LinkedList<NetworkInterfaceModule*>()),
  _txQueue(IvanLinkedList::LinkedList<DRONE_MESH_MSG_BUFFER*>())
{
  _publishedMessages = 0;
  _peerNodes = 0;
  _minPeer = 255;
  _maxPeer = 0;
  for (uint8_t i=0; i<DRONE_LINK_NODE_PAGES; i++) {
    _nodePages[i] = NULL;
  }
  _helloSeq = 0;
  _helloTimer = 0;
  _seqTimer = 0;
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

  // locate a matching channel, create if it doesn't exist
  // given this is an external request, it must be for a channel on this _node
  DroneLinkChannel* c = findChannel(_node, channel);

  if (!c) {
    Log.noticeln(F("Creating channel: %u > %u"), _node, channel);
    c = new DroneLinkChannel(this, _node, channel);
    _channels.add(c);
  }

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

          // check age && interface status
          if ((millis() - page->nodeInfo[j].lastHeard > DRONE_LINK_MANAGER_MAX_ROUTE_AGE) ||
          (!page->nodeInfo[j].interface->getInterfaceState())) {
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

  if (loopTime > _helloTimer + DRONE_LINK_MANAGER_HELLO_INTERVAL || _helloTimer == 0) {
    generateHellos();
    _helloTimer = loopTime;
  }

  processTransmitQueue();
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
      page->nodeInfo[i].gSeq = 0;
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

  response->printf(("Tx Queue:  (size %u) \n"), getTxQueueSize());

  // print detailed queue info
  for (uint8_t i=0; i<_txQueue.size(); i++) {
    DRONE_MESH_MSG_BUFFER* buffer = _txQueue.get(i);

    response->printf("    %u: state=%u, int:%s, \n", i, buffer->state, buffer->interface->getName());

    if (buffer->state > DRONE_MESH_MSG_BUFFER_STATE_EMPTY) {
      // decode message in buffer
      response->print("      (");
      response->print( isDroneMeshMsgAck(buffer->data) ? "S" : "A" );
      response->print(", ");
      response->printf(", %u) ", getDroneMeshMsgPayloadType(buffer->data));
      response->printf("%u > %u > %u > %u ", getDroneMeshMsgSrcNode(buffer->data), getDroneMeshMsgTxNode(buffer->data), getDroneMeshMsgNextNode(buffer->data),
    getDroneMeshMsgDestNode(buffer->data));
      response->printf("seq=%u [%u]\n", getDroneMeshMsgSeq(buffer->data), getDroneMeshMsgPayloadSize(buffer->data));
    }

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
          response->printf(", Seq: %u, Metric: %u, Next Hop: %u, Age: %u sec, Uptime: %u, Int: %s\n", page->nodeInfo[j].seq, page->nodeInfo[j].metric, page->nodeInfo[j].nextHop, age, page->nodeInfo[j].uptime, interfaceName);
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
  // validate packet
  uint8_t len = getDroneMeshMsgTotalSize(buffer);
  uint8_t crc = _CRC8.smbus((uint8_t*)buffer, len-1);
  if (buffer[len-1] != crc) {
    Serial.println("[DLM.rP] CRC fail");
    return;
  }

  Log.noticeln("[DLM.rP] Rec %u bytes with metric %u on int: ", len, metric, interface->getName());

  // this is the meaty stuff...
  // do something appropriate with it depending on its routing mode, type, etc
  uint8_t type = getDroneMeshMsgPayloadType(buffer);

  /*
  // decode packet header
  Log.noticeln("  Mode: %u", getDroneMeshMsgMode(buffer));
  Log.noticeln("  Guaranteed: %b", isDroneMeshMsgGuaranteed(buffer));
  Log.noticeln("  Size: %u", getDroneMeshMsgPayloadSize(buffer));
  Log.noticeln("  txNode: %u", getDroneMeshMsgTxNode(buffer));
  Log.noticeln("  srcNode: %u", getDroneMeshMsgSrcNode(buffer));
  Log.noticeln("  nextNode: %u", getDroneMeshMsgNextNode(buffer));
  Log.noticeln("  destNode: %u", getDroneMeshMsgDestNode(buffer));
  Log.noticeln("  seq: %u", getDroneMeshMsgSeq(buffer));
  Log.noticeln("  Type: %u", getDroneMeshMsgType(buffer));
  Log.noticeln("  Direction: %u", getDroneMeshMsgDirection(buffer));
*/

  // update lastHeard for txNode
  DRONE_MESH_MSG_HEADER *header = (DRONE_MESH_MSG_HEADER*)buffer;
  DRONE_LINK_NODE_INFO* txNodeInfo = getNodeInfo(header->txNode, false);
  if (txNodeInfo) txNodeInfo->lastHeard = millis();

  // pass to receive handler
  switch (type) {
    case DRONE_MESH_MSG_TYPE_HELLO: receiveHello(interface, buffer, metric); break;

    case DRONE_MESH_MSG_TYPE_SUBSCRIPTION_REQUEST: receiveSubscriptionRequest(interface, buffer, metric); break;
    case DRONE_MESH_MSG_TYPE_SUBSCRIPTION_RESPONSE: receiveSubscriptionResponse(interface, buffer, metric); break;

    case DRONE_MESH_MSG_TYPE_TRACEROUTE_REQUEST: receiveTracerouteRequest(interface, buffer, metric); break;
    case DRONE_MESH_MSG_TYPE_TRACEROUTE_RESPONSE: receiveTracerouteResponse(interface, buffer, metric); break;

    case DRONE_MESH_MSG_TYPE_ROUTEENTRY_REQUEST: receiveRouteEntryRequest(interface, buffer, metric); break;
    case DRONE_MESH_MSG_TYPE_ROUTEENTRY_RESPONSE: receiveRouteEntryResponse(interface, buffer, metric); break;

    case DRONE_MESH_MSG_TYPE_DRONELINKMSG: receiveDroneLinkMsg(interface, buffer, metric); break;

  }
}


void DroneLinkManager::receiveHello(NetworkInterfaceModule *interface, uint8_t *buffer, uint8_t metric) {
  DRONE_MESH_MSG_HEADER *header = (DRONE_MESH_MSG_HEADER*)buffer;

  unsigned long loopTime = millis();

  Log.noticeln("[DLM.rP] Hello from %u tx by %u", header->srcNode, header->txNode);

  DRONE_MESH_MSG_HELLO *hello = (DRONE_MESH_MSG_HELLO*)buffer;

  // ignore it if this hello packet is from us
  if (header->srcNode == _node || header->txNode == _node) return;

  // calc total metric, inc RSSI to us
  uint32_t newMetric = constrain(hello->metric + metric, 0, 255);


  // fetch node info (routing entry), create if needed
  DRONE_LINK_NODE_INFO* nodeInfo = getNodeInfo(header->srcNode, true);

  if (nodeInfo) {

    // if its a brand new route entry it will have metric 255... so good to overwrite
    boolean feasibleRoute = nodeInfo->metric == 255;

    // if new uptime is less than current uptime
    if (hello->uptime < nodeInfo->uptime) {
      feasibleRoute = true;
      Log.noticeln("Lower uptime %u", hello->uptime);
    }

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
      nodeInfo->lastBroadcast = loopTime;
      nodeInfo->uptime = hello->uptime;

      // if metric < 255 then retransmit the Hello on all interfaces
      if (newMetric < 255) {
        for (uint8_t i=0; i < _interfaces.size(); i++) {
          NetworkInterfaceModule* interface = _interfaces.get(i);
          generateHello(interface, header->srcNode, header->seq, newMetric, hello->uptime);
        }
      }

    } else {
      Log.noticeln("New route infeasible");

      if (loopTime > nodeInfo->lastBroadcast + DRONE_LINK_MANAGER_HELLO_INTERVAL) {
        // retransmit our current best metric on all interfaces
        for (uint8_t i=0; i < _interfaces.size(); i++) {
          NetworkInterfaceModule* interface = _interfaces.get(i);
          generateHello(interface, header->srcNode, header->seq, nodeInfo->metric, nodeInfo->uptime);
        }

        nodeInfo->lastBroadcast = loopTime;
      }


    }
  }
}


void DroneLinkManager::receiveSubscriptionRequest(NetworkInterfaceModule *interface, uint8_t *buffer, uint8_t metric) {
  DRONE_MESH_MSG_HEADER *header = (DRONE_MESH_MSG_HEADER*)buffer;
  DRONE_MESH_MSG_SUBCSRIPTION_REQUEST *subBuffer = (DRONE_MESH_MSG_SUBCSRIPTION_REQUEST*)buffer;

  Log.noticeln("[DLM.rS] Sub request from %u to %u", header->srcNode, header->destNode);

  // check if we are the nextNode... otherwise ignore it
  if (header->nextNode == _node) {
    // are we the destination?
    if (header->destNode == _node) {
      Log.noticeln("[DLM.rS] Reached destination");

      // make a note that we need to send stuff to the subscribing (src) node
      subscribeExt(header->srcNode, subBuffer->channel, subBuffer->param);

      // generate a response
      generateResponse(buffer, DRONE_MESH_MSG_TYPE_SUBSCRIPTION_RESPONSE);

    } else {
      Log.noticeln("[DLM.rS] Intermediate hop");

      hopAlong(buffer);
    }
  }


}


void DroneLinkManager::receiveSubscriptionResponse(NetworkInterfaceModule *interface, uint8_t *buffer, uint8_t metric) {
  DRONE_MESH_MSG_HEADER *header = (DRONE_MESH_MSG_HEADER*)buffer;
  DRONE_MESH_MSG_SUBCSRIPTION_REQUEST *subBuffer = (DRONE_MESH_MSG_SUBCSRIPTION_REQUEST*)buffer;

  Log.noticeln("[DLM.rS] Sub response from %u to %u", header->srcNode, header->destNode);

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


void DroneLinkManager::receiveDroneLinkMsg(NetworkInterfaceModule *interface, uint8_t *buffer, uint8_t metric) {
  DRONE_MESH_MSG_HEADER *header = (DRONE_MESH_MSG_HEADER*)buffer;

  Log.noticeln("[DLM.rDlM] DLM from %u to %u", header->srcNode, header->destNode);

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


void DroneLinkManager::receiveTracerouteRequest(NetworkInterfaceModule *interface, uint8_t *buffer, uint8_t metric) {
  DRONE_MESH_MSG_HEADER *header = (DRONE_MESH_MSG_HEADER*)buffer;

  Log.noticeln("[DLM.rT] Traceroute request from %u to %u", header->srcNode, header->destNode);

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
      generateResponse(buffer, DRONE_MESH_MSG_TYPE_TRACEROUTE_RESPONSE);

    } else {
      Log.noticeln("  Intermediate hop");

      hopAlong(buffer);
    }
  }
}


void DroneLinkManager::receiveTracerouteResponse(NetworkInterfaceModule *interface, uint8_t *buffer, uint8_t metric) {
  DRONE_MESH_MSG_HEADER *header = (DRONE_MESH_MSG_HEADER*)buffer;

  Log.noticeln("[DLM.rT] Traceroute response from %u to %u", header->srcNode, header->destNode);

  // check if we are the destination... otherwise hopAlong
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


void DroneLinkManager::receiveRouteEntryRequest(NetworkInterfaceModule *interface, uint8_t *buffer, uint8_t metric) {
  DRONE_MESH_MSG_HEADER *header = (DRONE_MESH_MSG_HEADER*)buffer;

  Log.noticeln("[DLM.rT] Route Entry request from %u to %u", header->srcNode, header->destNode);

  // check if we are the nextNode... otherwise ignore it
  if (header->nextNode == _node) {

    // are we the destination?
    if (header->destNode == _node) {
      Log.noticeln("  Reached destination");

      DRONE_MESH_MSG_ROUTEENTRY_REQUEST* req = (DRONE_MESH_MSG_ROUTEENTRY_REQUEST*)buffer;

      // see if we have routing info for the requested node
      DRONE_LINK_NODE_INFO* nodeInfo = getNodeInfo(req->node, false);
      if (nodeInfo && nodeInfo->heard) {
        // generate a response

        // get nodeInfo for return route to requestor
        DRONE_LINK_NODE_INFO* srcInfo = getNodeInfo(header->srcNode, false);
        if (srcInfo && srcInfo->heard) {
          if (srcInfo->interface) {
            if (generateRouteEntryResponse(srcInfo->interface, nodeInfo, req->node, header->srcNode, srcInfo->nextHop)) {
              // next hop generated ok
            } else {
              // unable to generate next hop... e.g. queue full or interface down
            }
          }
        }

      }

    } else {
      Log.noticeln("  Intermediate hop");
      hopAlong(buffer);
    }
  }
}


void DroneLinkManager::receiveRouteEntryResponse(NetworkInterfaceModule *interface, uint8_t *buffer, uint8_t metric) {
  DRONE_MESH_MSG_HEADER *header = (DRONE_MESH_MSG_HEADER*)buffer;

  Log.noticeln("[DLM.rT] Route Entry response from %u to %u", header->srcNode, header->destNode);

  // ignore if we're not the next node
  if (header->nextNode == _node) {
    // check if we are the destination...
    if (header->destNode == _node) {
      Log.noticeln("  RouteEntry received:");

      // do nothing... this shouldn't ever happen on a node

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
      // should never reach this point, as should already have been procesed by a receive handler

    } else {
      //Log.noticeln("[DLM.hA] Intermediate hop");

      // get next hop
      DRONE_LINK_NODE_INFO* nodeInfo = getNodeInfo(header->destNode, false);
      if (nodeInfo && nodeInfo->heard) {
        // hand to interface to manage the next hop
        if (nodeInfo->interface) {
          if (generateNextHop(nodeInfo->interface, buffer, nodeInfo->nextHop)) {
            // next hop generated ok
          } else {
            // unable to generate next hop... e.g. queue full or interface down
            // abandon packet :(
          }
        }
      } else {
        // now what?
        Log.errorln("[DLM.rS] No route to destination %u", header->destNode);
      }

    }
  }
}


void DroneLinkManager::generateResponse(uint8_t *buffer, uint8_t newType) {
  // generate a standard response by copying the packet and swapping the src/dest

  DRONE_MESH_MSG_HEADER *header = (DRONE_MESH_MSG_HEADER*)buffer;

  Log.noticeln("[DLM.gR] Responding %u --> %u", header->destNode, header->srcNode);

  // flip src and dest
  uint8_t oldSrc = header->srcNode;
  header->srcNode = header->destNode;
  header->destNode = oldSrc;

  // update type
  setDroneMeshMsgPayloadType(buffer, newType);

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
      return sendDroneLinkMessage(nodeInfo->interface, extNode, nodeInfo->nextHop, msg);
  }

  return false;
}


boolean DroneLinkManager::generateSubscriptionRequest(uint8_t extNode, uint8_t channel, uint8_t param) {

  // see if we have a valid route to the target node
  DRONE_LINK_NODE_INFO* nodeInfo = getNodeInfo(extNode, false);
  if (nodeInfo && nodeInfo->heard) {
    // generate a subscription request on the relevant interface
    if (nodeInfo->interface &&
        generateSubscriptionRequest(nodeInfo->interface, _node, nodeInfo->nextHop, extNode, channel, param)) {
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
        generateTracerouteRequest(nodeInfo->interface, destNode, nodeInfo->nextHop);
    }
  }
}

// -------------------------------------------------
// REFACTOR - from Network Interface
// -------------------------------------------------


uint8_t DroneLinkManager::getTxQueueSize() {
  return _txQueue.size();
}


DRONE_MESH_MSG_BUFFER* DroneLinkManager::getTransmitBuffer(NetworkInterfaceModule *interface, uint8_t priority) {
  DRONE_MESH_MSG_BUFFER *buffer = NULL;

  //if (!getInterfaceState()) return NULL;

  // see if we have an empty transmit buffer than can be used
  for (uint8_t i=0; i<_txQueue.size(); i++) {
    DRONE_MESH_MSG_BUFFER *b = _txQueue.get(i);
    if (b->state == DRONE_MESH_MSG_BUFFER_STATE_EMPTY) {
      buffer = b;
      break;
    }
  }

  // if none available, see if we have space to create one
  if (!buffer && _txQueue.size() < DRONE_LINK_MANAGER_MAX_TX_QUEUE) {
    buffer = (DRONE_MESH_MSG_BUFFER*)malloc(sizeof(DRONE_MESH_MSG_BUFFER));
    _txQueue.add(buffer);
  }

  // failing that, lets see if there's one of lower priority we can repurpose
  if (!buffer) {
    for (uint8_t i=0; i<_txQueue.size(); i++) {
      DRONE_MESH_MSG_BUFFER *b = _txQueue.get(i);
      if (getDroneMeshMsgPriority(b->data) < priority) {
        buffer = b;
      }
    }
  }

  if (buffer) {
    // update state
    buffer->state = DRONE_MESH_MSG_BUFFER_STATE_READY;
    // set interface
    buffer->interface = interface;
  }

  return buffer;
}


void DroneLinkManager::processTransmitQueue() {
  // look through txQueue and see if anything is Ready to send
  for (uint8_t i=0; i<_txQueue.size(); i++) {
    DRONE_MESH_MSG_BUFFER *b = _txQueue.get(i);
    if (b->state == DRONE_MESH_MSG_BUFFER_STATE_READY) {
      if (b->interface->sendPacket(b->data)) {
        // if this is guaranteed, then flag to wait for a reply
        if (isDroneMeshMsgGuaranteed(b->data)) {
          //b->state = DRONE_MESH_MSG_BUFFER_STATE_WAITING;
          // TODO - actually implement guaranteed delivery
          b->state = DRONE_MESH_MSG_BUFFER_STATE_EMPTY;
        } else {
          // otherwise set to empty
          b->state = DRONE_MESH_MSG_BUFFER_STATE_EMPTY;
        }
      }
    }
  }
}


boolean DroneLinkManager::generateNextHop(NetworkInterfaceModule *interface, uint8_t *pbuffer, uint8_t nextHop) {
  //if (!getInterfaceState()) return false;

  // request a new buffer in the transmit queue
  DRONE_MESH_MSG_BUFFER *buffer = getTransmitBuffer(interface, getDroneMeshMsgPriority(pbuffer));

  // if successful
  if (buffer) {
    // get message size from previous buffer
    uint8_t len = getDroneMeshMsgTotalSize(pbuffer);

    // copy previous buffer to new
    memcpy(buffer->data, pbuffer, len);

    // update next hop
    DRONE_MESH_MSG_HEADER *header = (DRONE_MESH_MSG_HEADER*)buffer->data;
    header->nextNode = nextHop;

    // and tx node
    header->txNode = node();

    // update CRC
    buffer->data[len-1] = _CRC8.smbus((uint8_t*)buffer, len-1);

    return true;
  }

  return false;
}


void DroneLinkManager::generateHellos() {
  Serial.println("[DLM.gH]");

  uint32_t loopTime = millis();

  // generate a hello for each active interface
  for (uint8_t i=0; i<_interfaces.size(); i++) {
    NetworkInterfaceModule *interface = _interfaces.get(i);

    if (interface->getInterfaceState()) {
      if (generateHello(interface, node(), _helloSeq, 0, loopTime)) {
        // ?
      }
    }
  }

  // generate a new hello seq number every now and again
  if (loopTime > _seqTimer + DRONE_LINK_MANAGER_SEQ_INTERVAL) {
    _helloSeq++;
    _seqTimer = loopTime;
  }
}


boolean DroneLinkManager::generateHello(NetworkInterfaceModule *interface, uint8_t src, uint8_t seq, uint8_t metric, uint32_t uptime) {

  // request a new buffer in the transmit queue
  DRONE_MESH_MSG_BUFFER *buffer = getTransmitBuffer(interface, DRONE_MESH_MSG_PRIORITY_CRITICAL);

  // if successful
  if (buffer) {
    DRONE_MESH_MSG_HELLO *helloBuffer = (DRONE_MESH_MSG_HELLO*)buffer->data;

    // populate with a Hello packet
    helloBuffer->header.typeGuaranteeSize = DRONE_MESH_MSG_SEND |  DRONE_MESH_MSG_NOT_GUARANTEED | (5-1) ;  // payload is 1 byte... sent as n-1
    helloBuffer->header.txNode = node();
    helloBuffer->header.srcNode = src;
    helloBuffer->header.nextNode = 0;
    helloBuffer->header.destNode = 0;
    helloBuffer->header.seq = seq;
    setDroneMeshMsgPriorityAndPayloadType(buffer->data, DRONE_MESH_MSG_PRIORITY_CRITICAL, DRONE_MESH_MSG_TYPE_HELLO);
    helloBuffer->metric = metric;
    helloBuffer->uptime = uptime;

    // calc CRC
    helloBuffer->crc = _CRC8.smbus((uint8_t*)helloBuffer, sizeof(DRONE_MESH_MSG_HELLO)-1);

    return true;
  }

  return false;
}


boolean DroneLinkManager::generateSubscriptionRequest(NetworkInterfaceModule *interface, uint8_t src, uint8_t next, uint8_t dest, uint8_t channel, uint8_t param) {
  //if (!getInterfaceState()) return false;

  // request a new buffer in the transmit queue
  DRONE_MESH_MSG_BUFFER *buffer = getTransmitBuffer(interface, DRONE_MESH_MSG_PRIORITY_MEDIUM);

  // if successful
  if (buffer) {
    DRONE_MESH_MSG_SUBCSRIPTION_REQUEST *subBuffer = (DRONE_MESH_MSG_SUBCSRIPTION_REQUEST*)buffer->data;

    // populate with a subscription request packet
    subBuffer->header.typeGuaranteeSize = DRONE_MESH_MSG_SEND | DRONE_MESH_MSG_GUARANTEED | 1 ;  // payload is 2 byte... sent as n-1
    subBuffer->header.txNode = src;
    subBuffer->header.srcNode = node();
    subBuffer->header.nextNode = next;
    subBuffer->header.destNode = dest;
    subBuffer->header.seq = 0;
    //subBuffer->header.type = DRONE_MESH_MSG_TYPE_SUBSCRIPTION_REQUEST;
    setDroneMeshMsgPriorityAndPayloadType(buffer->data, DRONE_MESH_MSG_PRIORITY_MEDIUM, DRONE_MESH_MSG_TYPE_SUBSCRIPTION_REQUEST);
    subBuffer->channel = channel;
    subBuffer->param = param;

    // calc CRC
    subBuffer->crc = _CRC8.smbus((uint8_t*)subBuffer, sizeof(DRONE_MESH_MSG_SUBCSRIPTION_REQUEST)-1);

    return true;
  }

  return false;
}


boolean DroneLinkManager::generateTracerouteRequest(NetworkInterfaceModule *interface, uint8_t destNode, uint8_t nextNode) {
  //if (!getInterfaceState()) return false;

  // request a new buffer in the transmit queue
  DRONE_MESH_MSG_BUFFER *buffer = getTransmitBuffer(interface, DRONE_MESH_MSG_PRIORITY_LOW);

  // if successful
  if (buffer) {
    DRONE_MESH_MSG_TRACEROUTE_REQUEST *tBuffer = (DRONE_MESH_MSG_TRACEROUTE_REQUEST*)buffer->data;

    // populate with a Hello packet
    tBuffer->header.typeGuaranteeSize = DRONE_MESH_MSG_SEND| DRONE_MESH_MSG_GUARANTEED | 1 ;  // payload is 2 byte... sent as n-1
    tBuffer->header.txNode = node();;
    tBuffer->header.srcNode = node();
    tBuffer->header.nextNode = nextNode;
    tBuffer->header.destNode = destNode;
    tBuffer->header.seq = 0;
    //tBuffer->header.type = DRONE_MESH_MSG_TYPE_TRACEROUTE_REQUEST;
    setDroneMeshMsgPriorityAndPayloadType(buffer->data, DRONE_MESH_MSG_PRIORITY_LOW, DRONE_MESH_MSG_TYPE_TRACEROUTE_REQUEST);
    tBuffer->dummyNode = node();
    tBuffer->dummyMetric = 0;

    // calc CRC
    tBuffer->crc = _CRC8.smbus((uint8_t*)tBuffer, sizeof(DRONE_MESH_MSG_TRACEROUTE_REQUEST)-1);

    return true;
  }

  return false;
}


boolean DroneLinkManager::generateRouteEntryResponse(NetworkInterfaceModule *interface, void * nodeInfo, uint8_t target, uint8_t dest, uint8_t nextHop) {
  DRONE_LINK_NODE_INFO *ni = (DRONE_LINK_NODE_INFO*)nodeInfo;

  //if (!getInterfaceState()) return false;

  // request a new buffer in the transmit queue
  DRONE_MESH_MSG_BUFFER *buffer = getTransmitBuffer(interface, DRONE_MESH_MSG_PRIORITY_LOW);

  // if successful
  if (buffer) {
    DRONE_MESH_MSG_ROUTEENTRY_RESPONSE *rBuffer = (DRONE_MESH_MSG_ROUTEENTRY_RESPONSE*)buffer->data;

    // populate packet
    rBuffer->header.typeGuaranteeSize = DRONE_MESH_MSG_SEND | DRONE_MESH_MSG_GUARANTEED | (sizeof(DRONE_MESH_MSG_ROUTEENTRY_RESPONSE) - sizeof(DRONE_MESH_MSG_HEADER) - 2) ;
    rBuffer->header.txNode = node();;
    rBuffer->header.srcNode = node();
    rBuffer->header.nextNode = nextHop;
    rBuffer->header.destNode = dest;
    rBuffer->header.seq = 0;
    //rBuffer->header.type = DRONE_MESH_MSG_TYPE_ROUTEENTRY_RESPONSE;
    setDroneMeshMsgPriorityAndPayloadType(buffer->data, DRONE_MESH_MSG_PRIORITY_LOW, DRONE_MESH_MSG_TYPE_ROUTEENTRY_RESPONSE);

    // populate route entry info
    rBuffer->src = node();
    rBuffer->node = target;
    rBuffer->seq = ni->seq;
    rBuffer->metric = ni->metric;
    rBuffer->interfaceType = ni->interface->getInterfaceType();
    rBuffer->nextHop = ni->nextHop;
    rBuffer->age = ni->lastHeard - millis();
    rBuffer->uptime = ni->uptime;

    // calc CRC
    rBuffer->crc = _CRC8.smbus((uint8_t*)rBuffer, sizeof(DRONE_MESH_MSG_ROUTEENTRY_RESPONSE)-1);

    return true;
  }

  return false;
}


boolean DroneLinkManager::sendDroneLinkMessage(NetworkInterfaceModule *interface, uint8_t destNode, uint8_t nextNode, DroneLinkMsg *msg) {
  //if (!getInterfaceState()) return false;

  // request a new buffer in the transmit queue
  DRONE_MESH_MSG_BUFFER *buffer = getTransmitBuffer(interface, DRONE_MESH_MSG_PRIORITY_HIGH);

  // if successful
  if (buffer) {
    uint8_t payloadSize = msg->totalSize();
    uint8_t totalSize = payloadSize + sizeof(DRONE_MESH_MSG_HEADER) + 1;

    DRONE_MESH_MSG_HEADER *header = (DRONE_MESH_MSG_HEADER*)buffer->data;

    // populate with a DroneLinkMsg packet
    header->typeGuaranteeSize = DRONE_MESH_MSG_SEND | DRONE_MESH_MSG_GUARANTEED | (payloadSize-1) ;
    header->txNode = node();
    header->srcNode = node();
    header->nextNode = nextNode;
    header->destNode = destNode;
    header->seq = 0;
    //header->type = DRONE_MESH_MSG_TYPE_DRONELINKMSG;
    // TODO - read priority from DLM
    setDroneMeshMsgPriorityAndPayloadType(buffer->data, DRONE_MESH_MSG_PRIORITY_HIGH, DRONE_MESH_MSG_TYPE_DRONELINKMSG);

    // copy msg data
    memcpy(&buffer->data[sizeof(DRONE_MESH_MSG_HEADER)], (uint8_t*)&msg->_msg, payloadSize);

    // calc CRC
    buffer->data[totalSize-1] = _CRC8.smbus((uint8_t*)buffer->data, totalSize-1);

    return true;
  }

  return false;
}

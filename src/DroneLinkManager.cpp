#include "Arduino.h"
#include "DroneLinkManager.h"
#include "WiFiManager.h"
#include <Update.h>

DroneLinkManager::DroneLinkManager(WiFiManager *wifiManager, DroneFS* fs):
  _wifiManager(wifiManager),
  _fs(fs),
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
  _gSeq = 0;
  _packetsSent = 0;
  _packetsReceived = 0;
  _choked = 0;
  _kicked = 0;
  _chokeRate = 0;
  _kickRate = 0;
  _utilisation = 0;

  _firmwareStarted = false;
  _firmwareSize = 0;
  _firmwarePos = 0;
  _firmwareComplete = false;
  _firmwareLastRewind = 0;
  _firmwareSrc = 0;
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
  subscribe(addr->node, addr->channel, subscriber, addr->paramPriority);
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
  c->subscribe(_node, subscriber, getDroneLinkMsgParam(param));

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
    // UPDATE: removing subscriptions is unhelpful - e.g. if routes are briefly down then we lose subs, e.g. to the server
    /*
    DroneLinkChannel* c;
    for(int i = 0; i < _channels.size(); i++){
      c = _channels.get(i);
      c->removeExternalSubscriptions(node);
    }
    */
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
              (!page->nodeInfo[j].interface->getInterfaceState()) ||
              (!page->nodeInfo[j].interface->isEnabled()) ) {
            // remove route
            removeRoute(n);
          }
        }
      }
    }
  }
}


void DroneLinkManager::checkDirectLinks() {
  DRONE_LINK_NODE_PAGE *page;
  for (uint8_t i=0; i<DRONE_LINK_NODE_PAGES; i++) {
    page = _nodePages[i];
    if (page != NULL) {
      for (uint8_t j=0; j<DRONE_LINK_NODE_PAGE_SIZE; j++) {
        if (page->nodeInfo[j].heard) {
          uint8_t n = i*DRONE_LINK_NODE_PAGE_SIZE + j;

          // see if this is a directly connected
          if (millis() < page->nodeInfo[j].lastHello + 10 * DRONE_LINK_MANAGER_HELLO_INTERVAL) {
            // see how long since we last had an Ack from this node?
            if (millis() > page->nodeInfo[j].lastAck + DRONE_LINK_MANAGER_LINK_CHECK_INTERVAL) {
              if (page->nodeInfo[j].helloInterface &&
                  page->nodeInfo[j].helloInterface->getInterfaceState()) {
                generateLinkCheckRequest(page->nodeInfo[j].helloInterface, n, n);
                // update lastAck so we don't try again too soon
                page->nodeInfo[j].lastAck = millis();
              }
            }
          } else {
            // taper off avgAttempts info ready for potential future connection?
            page->nodeInfo[j].avgAttempts *= 0.9;
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
    checkDirectLinks();

    // update _utilisation
    uint8_t utilCount = 0;
    for (uint8_t i=0; i<_txQueue.size(); i++) {
      DRONE_MESH_MSG_BUFFER *b = _txQueue.get(i);

      // check age of packet
      if (b->state > DRONE_MESH_MSG_BUFFER_STATE_EMPTY) {
        utilCount++;
      }
    }

    // 10 second average
    if (_txQueue.size() > 0)
      _utilisation = (_utilisation * 9 + ((float)utilCount / _txQueue.size())) / 10;

    processTimer = loopTime;
  }

  if (loopTime > _helloTimer + DRONE_LINK_MANAGER_HELLO_INTERVAL || _helloTimer == 0) {
    generateHellos();
    _helloTimer = loopTime;
  }

  processTransmitQueue();

  processFirmwareUpdate();
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
      page->nodeInfo[i].gSeq = 0;
      page->nodeInfo[i].metric = 255;
      page->nodeInfo[i].helloMetric = 255;
      page->nodeInfo[i].name = NULL;
      page->nodeInfo[i].interface = NULL;
      page->nodeInfo[i].nextHop = 0;
      page->nodeInfo[i].givenUp = 0;
      page->nodeInfo[i].avgAttempts = 0;
      page->nodeInfo[i].avgTxTime = 0;
      page->nodeInfo[i].lastAck = 0;
      page->nodeInfo[i].lastHello = 0;
      page->nodeInfo[i].helloInterface = NULL;
      page->nodeInfo[i].avgAckTime = 0;
      page->nodeInfo[i].gSequencer = NULL;
    }
  }

  if (page != NULL) {

    if (heard) {
      if (!page->nodeInfo[nodeIndex].heard) {
        _peerNodes++;
        page->nodeInfo[nodeIndex].heard = true;
        // init sequencer if needed
        if (!page->nodeInfo[nodeIndex].gSequencer) {
          page->nodeInfo[nodeIndex].gSequencer = new DroneLinkMeshMsgSequencer();
        } else {
          page->nodeInfo[nodeIndex].gSequencer->clear();
        }
      }
      page->nodeInfo[nodeIndex].lastHeard = millis();
    }

    return &page->nodeInfo[nodeIndex];
  } else
    return NULL;
}


uint8_t DroneLinkManager::getMetricFromNodeInfo(DRONE_LINK_NODE_INFO* nodeInfo) {
  uint8_t metric = 20;
  if (nodeInfo) {
    metric = min((int)ceil(2*nodeInfo->avgAttempts + 0.1), 20);
  }
  return constrain(metric, 1, 20);
}


NetworkInterfaceModule* DroneLinkManager::getSourceInterface(uint8_t source) {
  DRONE_LINK_NODE_INFO* nodeInfo = getNodeInfo(source, false);

  if (nodeInfo == NULL) return NULL;
  return nodeInfo->interface;
}


void DroneLinkManager::serveNodeInfo(AsyncWebServerRequest *request) {
  AsyncResponseStream *response = request->beginResponseStream("text/text");
  response->addHeader("Server","ESP Async Web Server");

  response->print(F("Routing table: \n"));

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
          response->printf(", Sq: %u, Mt: %u, Nx: %u, Ag: %u sec, Up: %u, I: %s (%u), At: %.1f, Tx: %.0fms, Ack: %.0fms, GU: %u\n", page->nodeInfo[j].seq, page->nodeInfo[j].metric, page->nodeInfo[j].nextHop, age, page->nodeInfo[j].uptime, interfaceName, page->nodeInfo[j].interface->getInterfaceType(), page->nodeInfo[j].avgAttempts, page->nodeInfo[j].avgTxTime, page->nodeInfo[j].avgAckTime, page->nodeInfo[i].givenUp);
        }
      }
    }
  }

  response->printf(("\n\nTx Queue: %.1f (size %u), kicked: %u (%.1f), choked: %u (%.1f) \n"), (100*_utilisation), getTxQueueSize(), _kicked, _kickRate, _choked, _chokeRate);

  // print detailed queue info
  for (uint8_t i=0; i<_txQueue.size(); i++) {
    DRONE_MESH_MSG_BUFFER* buffer = _txQueue.get(i);

    response->printf("    %u: state=%u, int:%s, \n", i, buffer->state, buffer->interface->getName());

    if (buffer->state > DRONE_MESH_MSG_BUFFER_STATE_EMPTY) {
      // decode message in buffer
      response->print("      (");
      response->print( isDroneMeshMsgAck(buffer->data) ? "A" : "S" );
      response->printf(", %u) ", getDroneMeshMsgPayloadType(buffer->data));
      response->printf("%u > %u > %u > %u ", getDroneMeshMsgSrcNode(buffer->data), getDroneMeshMsgTxNode(buffer->data), getDroneMeshMsgNextNode(buffer->data),
    getDroneMeshMsgDestNode(buffer->data));
      response->printf("seq=%u [%u]", getDroneMeshMsgSeq(buffer->data), getDroneMeshMsgPayloadSize(buffer->data));

      // if this is a DLM ... decode that too
      if (getDroneMeshMsgPayloadType(buffer->data) == DRONE_MESH_MSG_TYPE_DRONELINKMSG) {
        // upwrap contained msg
        //uint8_t payloadSize = getDroneMeshMsgPayloadSize(buffer);

        //memcpy((uint8_t*)&tempMsg._msg, &buffer[sizeof(DRONE_MESH_MSG_HEADER)], payloadSize);

      }

      response->print("\n");
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

  // ignore it if this packet is from us
  if (getDroneMeshMsgSrcNode(buffer) == _node || getDroneMeshMsgTxNode(buffer) == _node) return;

  // ignore it if this packet is not destined for us (unless its a broadcast)
  uint8_t nn = getDroneMeshMsgNextNode(buffer);
  if (nn != 0 && nn != _node) return;

  //Log.noticeln("[DLM.rP] Rec %u bytes with metric %u on int: ", len, metric, interface->getName());

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

  _packetsReceived++;

  // is this an Ack?
  if (isDroneMeshMsgAck(buffer)) {
    //Log.noticeln("isAck");
    receiveAck(buffer);

  } else {
    // generate Ack
    if (isDroneMeshMsgGuaranteed(buffer)) {
      //Log.noticeln("genAck");

      // even if we've received this packet before, we should try to send another Ack, as the sending node may not have heard our previous Ack and be stuck re-transmitting
      // however, we need to make sure we're not in the middle of transmitting an Ack... how? ... let generateAck take care of that with a scrub

      generateAck(interface, buffer);

      // check to see if we've already received this packet

      DRONE_LINK_NODE_INFO* srcNodeInfo = getNodeInfo(header->srcNode, false);
      if (srcNodeInfo) {
        if (srcNodeInfo->gSequencer &&
            srcNodeInfo->gSequencer->isDuplicate(header->seq)) {
          return;
        }

        //if (header->seq <= srcNodeInfo->gSeq && header->seq > 5) return;

        //srcNodeInfo->gSeq = header->seq;
      }
    }

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

      case DRONE_MESH_MSG_TYPE_ROUTER_REQUEST: receiveRouterRequest(interface, buffer, metric); break;
      case DRONE_MESH_MSG_TYPE_ROUTER_RESPONSE: receiveRouterResponse(interface, buffer, metric); break;

      case DRONE_MESH_MSG_TYPE_LINK_CHECK_REQUEST: receiveLinkCheckRequest(interface, buffer, metric); break;

      // firmware updates
      case DRONE_MESH_MSG_TYPE_FIRMWARE_START_REQUEST: receiveFirmwareStartRequest(interface, buffer, metric); break;
      case DRONE_MESH_MSG_TYPE_FIRMWARE_WRITE: receiveFirmwareWrite(interface, buffer, metric); break;

      // filesystem
      case DRONE_MESH_MSG_TYPE_FS_FILE_REQUEST: receiveFSFileRequest(interface, buffer, metric); break;

    default:
      // if not a broadcast then just provide default routing using hopAlong
      if (nn != 0) {
        hopAlong(buffer);
      }
    }
  }
}


void DroneLinkManager::receiveAck(uint8_t *buffer) {
  // search tx queue for a matching waiting buffer
  for (uint8_t i=0; i<_txQueue.size(); i++) {
    DRONE_MESH_MSG_BUFFER *b = _txQueue.get(i);
    // find waiting buffers
    if (b->state == DRONE_MESH_MSG_BUFFER_STATE_WAITING) {
      // see if this matches the Ack we've received
      // dest should equal src, and src equal dest
      // same seq number

      if ( getDroneMeshMsgSeq(b->data) == getDroneMeshMsgSeq(buffer) &&
           getDroneMeshMsgSrcNode(b->data) == getDroneMeshMsgDestNode(buffer) &&
           getDroneMeshMsgDestNode(b->data) == getDroneMeshMsgSrcNode(buffer) ) {
        // clear waiting buffer
        b->state = DRONE_MESH_MSG_BUFFER_STATE_EMPTY;

        // update stats on nextNode
        DRONE_LINK_NODE_INFO* nextNodeInfo = getNodeInfo(getDroneMeshMsgTxNode(buffer), false);
        if (nextNodeInfo) {
          nextNodeInfo->lastAck = millis();
          nextNodeInfo->avgAttempts = (nextNodeInfo->avgAttempts * (DRONE_LINK_MANAGER_AVG_SAMPLES-1) + b->attempts) / DRONE_LINK_MANAGER_AVG_SAMPLES;
          nextNodeInfo->avgAckTime = (nextNodeInfo->avgAckTime * (DRONE_LINK_MANAGER_AVG_SAMPLES-1) + (millis() - b->created)) / DRONE_LINK_MANAGER_AVG_SAMPLES;
        }
      }
    }
  }
}


void DroneLinkManager::receiveHello(NetworkInterfaceModule *interface, uint8_t *buffer, uint8_t metric) {
  DRONE_MESH_MSG_HEADER *header = (DRONE_MESH_MSG_HEADER*)buffer;

  unsigned long loopTime = millis();

  Log.noticeln("[DLM.rP] Hello from %u tx by %u", header->srcNode, header->txNode);

  DRONE_MESH_MSG_HELLO *hello = (DRONE_MESH_MSG_HELLO*)buffer;

  // ignore it if this hello packet is from us
  if (header->srcNode == _node || header->txNode == _node) return;

  // set an initial new total metric, inc RSSI to us
  uint32_t newMetric = constrain(hello->metric + metric, 0, 255);

  // lookup info on tx Node... to calc a better metric than RSSI
  DRONE_LINK_NODE_INFO* txNodeInfo = getNodeInfo(header->txNode, false);
  if (txNodeInfo) {
    // use avgAttempts to txNode to update total metric
    newMetric = constrain(hello->metric + getMetricFromNodeInfo(txNodeInfo), 0, 255);
    // update last hello
    txNodeInfo->lastHello = millis();
    txNodeInfo->helloInterface = interface;
  }


  // fetch node info (routing entry), create if needed
  DRONE_LINK_NODE_INFO* nodeInfo = getNodeInfo(header->srcNode, true);

  if (nodeInfo) {

    // if its a brand new route entry it will have metric 255... so good to overwrite
    boolean feasibleRoute = false;
    if (nodeInfo->metric == 255) {
      feasibleRoute = true;
    } else {
      // update existing metric info based on latest link quality
      DRONE_LINK_NODE_INFO* nextHopInfo = getNodeInfo(nodeInfo->nextHop, false);
      if (nextHopInfo) {
        // use avgAttempts to nexthop to update total metric
        nodeInfo->metric = constrain(nodeInfo->helloMetric + getMetricFromNodeInfo(nextHopInfo), 0, 255);
      }
    }

    // if new uptime is significantly less than current uptime
    if (hello->uptime < nodeInfo->uptime * 0.9) {
      feasibleRoute = true;
      Log.noticeln("Lower uptime %u", hello->uptime);
    }

    // if the interface has changed and there's a lower metric
    if (interface != nodeInfo->interface && newMetric < nodeInfo->metric) {
      feasibleRoute = true;
    } else {
      // is this a new sequence (allow for wrap-around)
      if ((header->seq > nodeInfo->seq) || (nodeInfo->seq > 128 && (header->seq < nodeInfo->seq - 128))) {
        feasibleRoute = true;
        Log.noticeln("New seq %u", header->seq);
      }

      // or is it the same seq, but with a better metric
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
      nodeInfo->helloMetric = hello->metric;
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
        // retransmit our current best route on all interfaces
        for (uint8_t i=0; i < _interfaces.size(); i++) {
          NetworkInterfaceModule* interface = _interfaces.get(i);
          generateHello(interface, header->srcNode, nodeInfo->seq, nodeInfo->metric, nodeInfo->uptime);
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


void DroneLinkManager::receiveRouterRequest(NetworkInterfaceModule *interface, uint8_t *buffer, uint8_t metric) {
  DRONE_MESH_MSG_HEADER *header = (DRONE_MESH_MSG_HEADER*)buffer;

  Log.noticeln("[DLM.rT] Router request from %u to %u", header->srcNode, header->destNode);

  // check if we are the nextNode... otherwise ignore it
  if (header->nextNode == _node) {

    // are we the destination?
    if (header->destNode == _node) {
      Log.noticeln("  Reached destination");

      //DRONE_MESH_MSG_ROUTER_REQUEST* req = (DRONE_MESH_MSG_ROUTER_REQUEST*)buffer;

      // get nodeInfo for return route to requestor
      DRONE_LINK_NODE_INFO* srcInfo = getNodeInfo(header->srcNode, false);
      if (srcInfo && srcInfo->heard) {
        if (srcInfo->interface) {
          if (generateRouterResponse(srcInfo->interface, header->srcNode, srcInfo->nextHop)) {
            // next hop generated ok
          } else {
            // unable to generate next hop... e.g. queue full or interface down
          }
        }
      }



    } else {
      Log.noticeln("  Intermediate hop");
      hopAlong(buffer);
    }
  }
}


void DroneLinkManager::receiveFirmwareStartRequest(NetworkInterfaceModule *interface, uint8_t *buffer, uint8_t metric) {
  DRONE_MESH_MSG_HEADER *header = (DRONE_MESH_MSG_HEADER*)buffer;

  Log.noticeln("[DLM.rFSR] Firmware Start request from %u", header->srcNode);

  // are we the destination?
  if (header->destNode == _node || header->destNode == 0) {
    DRONE_MESH_MSG_FIRMWARE_START_REQUEST* firmwareStart = (DRONE_MESH_MSG_FIRMWARE_START_REQUEST*)buffer;

    // get size
    _firmwareSize = firmwareStart->size;

    //Use cli() to disable interrupts and sei() to
    /// reenable them.
    cli();

    if (Update.isRunning()) {
      Update.abort();
      Update.end();
    }

    _firmwareStarted = Update.begin(_firmwareSize);
    sei();

    _firmwarePos = 0;

    _firmwareComplete = false;

    _firmwareSrc = header->srcNode;

    Log.noticeln("[DLM.rFSR] Primed to receive: %u bytes", _firmwareSize);

    // generate a response
    generateFirmwareStartResponse(interface, header->srcNode, _firmwareStarted ? 1 : 0);

    // alert main
    if (onEvent) onEvent(DRONE_LINK_MANAGER_FIRMWARE_UPDATE_START, 0);
  }
}


void DroneLinkManager::receiveFirmwareWrite(NetworkInterfaceModule *interface, uint8_t *buffer, uint8_t metric) {
  DRONE_MESH_MSG_HEADER *header = (DRONE_MESH_MSG_HEADER*)buffer;

  //Log.noticeln("[DLM.rFW] Firmware Write from %u", header->srcNode);

  // are we primed to receive?
  if (_firmwareStarted && _firmwareSize > 0 && !_firmwareComplete) {
    DRONE_MESH_MSG_FIRMWARE_WRITE* wBuffer = (DRONE_MESH_MSG_FIRMWARE_WRITE*)buffer;

    // get total payload size
    uint8_t payloadSize = getDroneMeshMsgPayloadSize(buffer);

    // calc data size
    uint8_t dataSize = payloadSize - 4;

    // check offset matches write pos
    if (wBuffer->offset == _firmwarePos) {

      Log.noticeln("[DLM.rFW] Firmware Write %u bytes at pos: %u", dataSize, _firmwarePos);

      // write data to firmware
      cli();
      Update.write(&wBuffer->data[0], dataSize);
      sei();

      _firmwarePos += dataSize;
      _firmwareLastRewind = millis();

      // have we reached the end?
      if (_firmwarePos >= _firmwareSize) {
        _firmwareStarted = false;
        _firmwareComplete = true;

        cli();
        if (Update.end()) {
          if (Update.isFinished())
          {
            // alert main
            if (onEvent) onEvent(DRONE_LINK_MANAGER_FIRMWARE_UPDATE_END, 1);

            Log.noticeln("Update successfully completed. Rebooting.");
            ESP.restart();
          }
          else
          {
            Log.errorln("Update not finished? Something went wrong!");
          }
        } else {
          Log.errorln("Update failed");
        }
        sei();
      } else {
        // alert main
        if (onEvent) onEvent(DRONE_LINK_MANAGER_FIRMWARE_UPDATE_PROGRESS, 1.0f * _firmwarePos / _firmwareSize);
      }

    } else {
      // are we behind?
      if (wBuffer->offset > _firmwarePos) {
        Log.noticeln("[DLM.rFW] Behind Firmware Write %u vs %u", wBuffer->offset, _firmwarePos);

        // we've obviously missed a packet... ask for a rewind
        generateFirmwareRewind(interface, header->srcNode, _firmwarePos);
      } else {
        // we're ahead... someone else must have asked for a rewind, we'll wait for them to catchup
        Log.noticeln("[DLM.rFW] Ahead of Firmware Write %u vs %u", wBuffer->offset, _firmwarePos);

      }
    }

  }
}


void DroneLinkManager::receiveFSFileRequest(NetworkInterfaceModule *interface, uint8_t *buffer, uint8_t metric) {
  DRONE_MESH_MSG_HEADER *header = (DRONE_MESH_MSG_HEADER*)buffer;

  Log.noticeln("[DLM.rT] FS File request from %u to %u", header->srcNode, header->destNode);

  // check if we are the nextNode... otherwise ignore it
  if (header->nextNode == _node) {

    // are we the destination?
    if (header->destNode == _node) {

      DRONE_MESH_MSG_FS_FILE_REQUEST* rbuffer = (DRONE_MESH_MSG_FS_FILE_REQUEST*)buffer;

      // ensure path is null terminated
      rbuffer->path[DRONE_MESH_MSG_FS_MAX_PATH_SIZE-1] = 0;

      // handle request type
      DroneFSEntry* entry = NULL;

      if (rbuffer->flags == DRONE_MESH_MSG_FS_FLAG_PATH_INFO) {
         entry = _fs->getEntryByPath((char*)rbuffer->path);

      } else if (rbuffer->flags == DRONE_MESH_MSG_FS_FLAG_INDEX_INFO) {
        entry = _fs->getEntryByIndex((char*)rbuffer->path, rbuffer->id);
      }
      generateFSFileResponse(interface, header->srcNode, header->txNode, entry);

    } else {
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

      // lookup txNode and get avgAttempts value as metric
      uint8_t m = metric;

      DRONE_LINK_NODE_INFO* txInfo = getNodeInfo(header->txNode, false);
      if (txInfo) {
        m = getMetricFromNodeInfo(txInfo);
      }

      // add our info
      buffer[p] = m;
      buffer[p+1] = _node;

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

    // check if we are the nextNode... otherwise ignore it
    if (header->nextNode == _node) {

      // tweak buffer to add our traceroute info
      uint8_t payloadLen = getDroneMeshMsgPayloadSize(buffer);

      if (payloadLen < DRONE_MESH_MSG_MAX_PAYLOAD_SIZE-2) {
        // calc index of insertion point
        uint8_t p = sizeof(DRONE_MESH_MSG_HEADER) + payloadLen;

        // lookup txNode and get avgAttempts value as metric
        uint8_t m = metric;

        DRONE_LINK_NODE_INFO* txInfo = getNodeInfo(header->txNode, false);
        if (txInfo) {
          m = getMetricFromNodeInfo(txInfo);
        }

        // add our info
        buffer[p] = m;
        buffer[p+1] = _node;

        // update payload size
        setDroneMeshMsgPayloadSize(buffer, payloadLen+2);

        // no need to recalc CRC, as that will be dealt with by generateResponse or hopAlong

      }
    }

    // send the packet on its way
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


void DroneLinkManager::receiveRouterResponse(NetworkInterfaceModule *interface, uint8_t *buffer, uint8_t metric) {
  DRONE_MESH_MSG_HEADER *header = (DRONE_MESH_MSG_HEADER*)buffer;

  Log.noticeln("[DLM.rT] Router response from %u to %u", header->srcNode, header->destNode);

  // ignore if we're not the next node
  if (header->nextNode == _node) {
    // check if we are the destination...
    if (header->destNode == _node) {
      // do nothing... this shouldn't ever happen on a node

    } else {
      Log.noticeln("  Response - hopAlong");
      hopAlong(buffer);
    }
  }
}


void DroneLinkManager::receiveLinkCheckRequest(NetworkInterfaceModule *interface, uint8_t *buffer, uint8_t metric) {
  DRONE_MESH_MSG_HEADER *header = (DRONE_MESH_MSG_HEADER*)buffer;

  Log.noticeln("[DLM.rT] Link Check request from %u to %u", header->srcNode, header->destNode);

  // ignore if we're not the next node
  if (header->nextNode == _node) {
    // check if we are the destination...
    if (header->destNode == _node) {
      // do nothing... all we needed todo was Ack the packet

    } else {
      // no need to hop - link checks are only for directly connected nodes
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

  // if this is guaranteed, we need to generate a unique sequence number
  if (isDroneMeshMsgGuaranteed(buffer)) {
    header->seq = _gSeq++;
  }

  // use hopAlong to start the response
  hopAlong(buffer);
}


boolean DroneLinkManager::sendDroneLinkMessage(uint8_t extNode, DroneLinkMsg *msg) {
  // see if we have a valid route to the target node
  DRONE_LINK_NODE_INFO* nodeInfo = getNodeInfo(extNode, false);
  if (nodeInfo && nodeInfo->heard) {
    // generate a DLM on the relevant interface
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


uint8_t DroneLinkManager::getTxQueueActive() {
  uint8_t n = 0;
  DRONE_MESH_MSG_BUFFER* b;
  for (uint8_t i=0; i<_txQueue.size(); i++) {
    b = _txQueue.get(i);
    if (b->state > DRONE_MESH_MSG_BUFFER_STATE_EMPTY) n++;
  }
  return n;
}


DRONE_MESH_MSG_BUFFER* DroneLinkManager::getTransmitBuffer(NetworkInterfaceModule *interface, uint8_t priority) {
  DRONE_MESH_MSG_BUFFER *buffer = NULL;
  boolean isKicked = false;
  boolean isChoked = false;

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
      // if not an Ack and lower priority
      if (getDroneMeshMsgPriority(b->data) < priority &&
          !isDroneMeshMsgAck(b->data)) {
        buffer = b;
        _kicked++;
        isKicked = true;
        break;
      }
    }
  }

  if (buffer) {
    // update state
    buffer->state = DRONE_MESH_MSG_BUFFER_STATE_READY;
    // set interface
    buffer->interface = interface;
    // set creation time
    buffer->created = millis();
    // reset attempts
    buffer->attempts = 0;
  } else {
    _choked++;
    isChoked = true;
  }

  _kickRate = (_kickRate * 99 + (isKicked ? 1 : 0)) / 100;
  _chokeRate = (_chokeRate * 99 + (isChoked ? 1 : 0)) / 100;

  return buffer;
}


void DroneLinkManager::scrubDuplicateTransmitBuffers(DRONE_MESH_MSG_BUFFER *buffer) {
  uint8_t bufferSize = getDroneMeshMsgTotalSize(buffer->data);
  // look through txQueue
  for (uint8_t i=0; i<_txQueue.size(); i++) {
    DRONE_MESH_MSG_BUFFER *b = _txQueue.get(i);
    // check for packets ready to send
    if (b != buffer &&
        b->state > DRONE_MESH_MSG_BUFFER_STATE_EMPTY) {
      // see if b contains duplicate data as buffer
      uint8_t bSize = getDroneMeshMsgTotalSize(b->data);
      if (bSize == bufferSize) {
        // compare memory
        if (memcmp(&b->data[0], &buffer->data[0], bufferSize) == 0) {
          // match, so mark buffer as empty, in case b has already sent and is waiting for Ack
          buffer->state = DRONE_MESH_MSG_BUFFER_STATE_EMPTY;
          Log.noticeln("transmitScrub %u vs %u", buffer, b);
        }
      }
    }
  }
}


void DroneLinkManager::processTransmitQueue() {
  uint32_t loopTime = millis();
  // sort txQueue
  // a negative return value means item a is sorted before b
  _txQueue.sort( [](DRONE_MESH_MSG_BUFFER*& a, DRONE_MESH_MSG_BUFFER*& b) -> int {
    // only need to sort non-empty packets
    if (a->state > DRONE_MESH_MSG_BUFFER_STATE_EMPTY && b->state > DRONE_MESH_MSG_BUFFER_STATE_EMPTY) {
      // send Acks before new packets
      uint8_t a1 = isDroneMeshMsgAck(a->data) ? 0 : 1;
      uint8_t b1 = isDroneMeshMsgAck(b->data) ? 0 : 1;
      if (a1 == 0 && b1 == 1) {
        return -1;
      } else if (a1 == 1 && b1 == 0) {
        return 1;
      } else {
        // send higher priority items first
        a1 = getDroneMeshMsgPriority(a->data);
        b1 = getDroneMeshMsgPriority(b->data);
        if (a1 != b1) {
          return b1 - a1;
        }
        // send older items first (FIFO)
        return a->created - b->created;
      }
    } else {
      if (a->state > DRONE_MESH_MSG_BUFFER_STATE_EMPTY) {
        return -1;
      } else if (b->state > DRONE_MESH_MSG_BUFFER_STATE_EMPTY) {
        return 1;
      }
    }
    return 0;
  } );

  // look through txQueue
  uint8_t sentAPacket = false;

  for (uint8_t i=0; i<_txQueue.size(); i++) {
    DRONE_MESH_MSG_BUFFER *b = _txQueue.get(i);

    // check age of packet
    if (b->state > DRONE_MESH_MSG_BUFFER_STATE_EMPTY &&
        loopTime > b->created + DRONE_LINK_MANAGER_MAX_RETRY_INTERVAL) {

      // update stats on nextNode
      DRONE_LINK_NODE_INFO* nextNodeInfo = getNodeInfo(getDroneMeshMsgNextNode(b->data), false);
      if (nextNodeInfo) {
        nextNodeInfo->avgAttempts = (nextNodeInfo->avgAttempts * (DRONE_LINK_MANAGER_AVG_SAMPLES-1) + b->attempts) / DRONE_LINK_MANAGER_AVG_SAMPLES;
        nextNodeInfo->givenUp++;
      }

      // give up and release the buffer
      b->state = DRONE_MESH_MSG_BUFFER_STATE_EMPTY;
    }

    // check for packets ready to send
    if (b->state == DRONE_MESH_MSG_BUFFER_STATE_READY) {

      if (!sentAPacket &&
          b->interface->sendPacket(b->data)) {
        _packetsSent++;

        // if this is guaranteed, then flag to wait for a reply
        if (!isDroneMeshMsgAck(b->data) && isDroneMeshMsgGuaranteed(b->data)) {
          b->state = DRONE_MESH_MSG_BUFFER_STATE_WAITING;
          b->sent = loopTime;
        } else {
          // otherwise set to empty
          b->state = DRONE_MESH_MSG_BUFFER_STATE_EMPTY;
          // update stats
          DRONE_LINK_NODE_INFO* nextNodeInfo = getNodeInfo(getDroneMeshMsgNextNode(b->data), false);
          if (nextNodeInfo) {
            nextNodeInfo->avgTxTime = (nextNodeInfo->avgTxTime * (DRONE_LINK_MANAGER_AVG_SAMPLES-1) + (loopTime - b->created)) / DRONE_LINK_MANAGER_AVG_SAMPLES;
          }
        }

        // just the one Mrs Wemberley
        sentAPacket = true;
      } else {
        // ??
      }
    } else if (b->state == DRONE_MESH_MSG_BUFFER_STATE_WAITING) {

      // get avgAck time for this link to compare against
      uint32_t t = DRONE_LINK_MANAGER_MAX_ACK_INTERVAL;
      DRONE_LINK_NODE_INFO* nextNodeInfo = getNodeInfo(getDroneMeshMsgNextNode(b->data), false);
      if (nextNodeInfo) {
        // use avgAckTime + 10% to reduce duplicate packet transmission
        t = min(t, (uint32_t)(1.1 * nextNodeInfo->avgAckTime));
      }

      if (loopTime > b->sent + t) {

        //increment the attempts counter
        b->attempts++;

        // reset to ready to trigger retransmission
        b->state = DRONE_MESH_MSG_BUFFER_STATE_READY;

        // TODO - check/update route?

      }
    }
  }
}


boolean DroneLinkManager::generateNextHop(NetworkInterfaceModule *interface, uint8_t *pbuffer, uint8_t nextHop) {
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


boolean DroneLinkManager::generateAck(NetworkInterfaceModule *interface, uint8_t *buffer) {
  // need to make sure we're not in the middle of transmitting an Ack for this same buffer...  do that toward the end.. with a scrub

  // request a new buffer in the transmit queue, treat Acks as critical priority so we can clear them fast
  DRONE_MESH_MSG_BUFFER *abuffer = getTransmitBuffer(interface, DRONE_MESH_MSG_PRIORITY_CRITICAL);

  // if successful
  if (abuffer) {
    // get message size from previous buffer
    uint8_t len = getDroneMeshMsgTotalSize(buffer);

    // copy previous buffer to new
    memcpy(abuffer->data, buffer, len);

    // update next hop based on prev tx node
    DRONE_MESH_MSG_HEADER *header = (DRONE_MESH_MSG_HEADER*)abuffer->data;
    header->nextNode = getDroneMeshMsgTxNode(buffer);

    // flip source and dest
    uint8_t src = header->srcNode;
    header->srcNode = header->destNode;
    header->destNode = src;

    // set tx node to us
    header->txNode = node();

    // set Ack flag
    setDroneMeshMsgPacketType(abuffer->data, DRONE_MESH_MSG_ACK);

    // update CRC
    abuffer->data[len-1] = _CRC8.smbus((uint8_t*)abuffer, len-1);

    // check we've not already got an equivalent packet in the queue
    scrubDuplicateTransmitBuffers(abuffer);

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
    subBuffer->header.seq = _gSeq++;
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
    tBuffer->header.seq = _gSeq++;
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
  DRONE_MESH_MSG_BUFFER *buffer = getTransmitBuffer(interface, DRONE_MESH_MSG_PRIORITY_HIGH);

  // if successful
  if (buffer) {
    DRONE_MESH_MSG_ROUTEENTRY_RESPONSE *rBuffer = (DRONE_MESH_MSG_ROUTEENTRY_RESPONSE*)buffer->data;

    // populate packet
    rBuffer->header.typeGuaranteeSize = DRONE_MESH_MSG_SEND | DRONE_MESH_MSG_GUARANTEED | (sizeof(DRONE_MESH_MSG_ROUTEENTRY_RESPONSE) - sizeof(DRONE_MESH_MSG_HEADER) - 2) ;
    rBuffer->header.txNode = node();;
    rBuffer->header.srcNode = node();
    rBuffer->header.nextNode = nextHop;
    rBuffer->header.destNode = dest;
    rBuffer->header.seq = _gSeq++;
    //rBuffer->header.type = DRONE_MESH_MSG_TYPE_ROUTEENTRY_RESPONSE;
    setDroneMeshMsgPriorityAndPayloadType(buffer->data, DRONE_MESH_MSG_PRIORITY_HIGH, DRONE_MESH_MSG_TYPE_ROUTEENTRY_RESPONSE);

    // populate route entry info
    rBuffer->src = node();
    rBuffer->node = target;
    rBuffer->seq = ni->seq;
    rBuffer->metric = ni->metric;
    rBuffer->interfaceType = ni->interface->getInterfaceType();
    rBuffer->nextHop = ni->nextHop;
    rBuffer->age = millis() - ni->lastHeard;
    rBuffer->uptime = ni->uptime;
    rBuffer->avgAttempts = round(ni->avgAttempts * 10);
    rBuffer->avgAckTime = round(ni->avgAckTime);

    // calc CRC
    rBuffer->crc = _CRC8.smbus((uint8_t*)rBuffer, sizeof(DRONE_MESH_MSG_ROUTEENTRY_RESPONSE)-1);

    return true;
  }

  return false;
}


boolean DroneLinkManager::generateRouterResponse(NetworkInterfaceModule *interface, uint8_t dest, uint8_t nextHop) {
  // request a new buffer in the transmit queue
  DRONE_MESH_MSG_BUFFER *buffer = getTransmitBuffer(interface, DRONE_MESH_MSG_PRIORITY_HIGH);

  // if successful
  if (buffer) {
    DRONE_MESH_MSG_ROUTER_RESPONSE *rBuffer = (DRONE_MESH_MSG_ROUTER_RESPONSE*)buffer->data;

    // populate packet
    rBuffer->header.typeGuaranteeSize = DRONE_MESH_MSG_SEND | DRONE_MESH_MSG_GUARANTEED | (sizeof(DRONE_MESH_MSG_ROUTER_RESPONSE) - sizeof(DRONE_MESH_MSG_HEADER) - 2) ;
    rBuffer->header.txNode = node();;
    rBuffer->header.srcNode = node();
    rBuffer->header.nextNode = nextHop;
    rBuffer->header.destNode = dest;
    rBuffer->header.seq = _gSeq++;
    //rBuffer->header.type = DRONE_MESH_MSG_TYPE_ROUTEENTRY_RESPONSE;
    setDroneMeshMsgPriorityAndPayloadType(buffer->data, DRONE_MESH_MSG_PRIORITY_HIGH, DRONE_MESH_MSG_TYPE_ROUTER_RESPONSE);

    // populate route entry info
    rBuffer->txQueueSize = getTxQueueSize();
    rBuffer->txQueueActive = getTxQueueActive();
    rBuffer->choked = _choked;
    rBuffer->kicked = _kicked;
    rBuffer->chokeRate = round(_chokeRate * 10);
    rBuffer->kickRate = round(_kickRate * 10);
    rBuffer->utilisation = round(_utilisation * 100);

    // calc CRC
    rBuffer->crc = _CRC8.smbus((uint8_t*)rBuffer, sizeof(DRONE_MESH_MSG_ROUTER_RESPONSE)-1);

    return true;
  }

  return false;
}


boolean DroneLinkManager::generateLinkCheckRequest(NetworkInterfaceModule *interface, uint8_t dest, uint8_t nextHop) {
  // request a new buffer in the transmit queue
  DRONE_MESH_MSG_BUFFER *buffer = getTransmitBuffer(interface, DRONE_MESH_MSG_PRIORITY_MEDIUM);

  // if successful
  if (buffer) {
    DRONE_MESH_MSG_LINK_CHECK_REQUEST *rBuffer = (DRONE_MESH_MSG_LINK_CHECK_REQUEST*)buffer->data;

    // populate packet
    rBuffer->header.typeGuaranteeSize = DRONE_MESH_MSG_SEND | DRONE_MESH_MSG_GUARANTEED | (sizeof(DRONE_MESH_MSG_LINK_CHECK_REQUEST) - sizeof(DRONE_MESH_MSG_HEADER) - 2) ;
    rBuffer->header.txNode = node();;
    rBuffer->header.srcNode = node();
    rBuffer->header.nextNode = nextHop;
    rBuffer->header.destNode = dest;
    rBuffer->header.seq = _gSeq++;
    setDroneMeshMsgPriorityAndPayloadType(buffer->data, DRONE_MESH_MSG_PRIORITY_MEDIUM, DRONE_MESH_MSG_TYPE_LINK_CHECK_REQUEST);

    // calc CRC
    rBuffer->crc = _CRC8.smbus((uint8_t*)rBuffer, sizeof(DRONE_MESH_MSG_LINK_CHECK_REQUEST)-1);

    return true;
  }

  return false;
}


boolean DroneLinkManager::sendDroneLinkMessage(NetworkInterfaceModule *interface, uint8_t destNode, uint8_t nextNode, DroneLinkMsg *msg) {
  uint8_t payloadSize = msg->totalSize();
  uint8_t totalSize = payloadSize + sizeof(DRONE_MESH_MSG_HEADER) + 1;

  // ignore anything where the source node is not this node
  if (msg->source() != _node) return true;

  // param filter check only relevant to local address space
  // and can be ignored for name responses
  if (msg->node() == _node && msg->type() < DRONE_LINK_MSG_TYPE_NAME) {
    // calc hashmap index
    int index = (msg->_msg.channel << 8) | getDroneLinkMsgParam(msg->_msg.paramPriority);

    // see if we already have an entry in the hasmap
    struct DRONE_LINK_PARAM_FILTER *pf;
    HASH_FIND_INT(_paramFilter, &index, pf);
    if (pf) {
      // check last tx time
      if (millis() <= pf->lastTxTime + 1000) return true; // abandon packet

      // update lastTxTime
      pf->lastTxTime = millis();
    } else {
      // create a new entry in the hashmap
      pf = (DRONE_LINK_PARAM_FILTER*)malloc(sizeof *pf);
      pf->index = index;
      pf->lastTxTime = millis();
      HASH_ADD_INT(_paramFilter, index, pf);
    }
  }


  // before we allocate a new transmit buffer...  check to see if this is a duplicate msg
  // i.e. matching signature (first 5 bytes)
  // same sig if memcmp(&_msg, &msg->_msg, 5) == 0;
  // scan txQueue and compare to msg signature
  for (uint8_t i=0; i<_txQueue.size(); i++) {
    DRONE_MESH_MSG_BUFFER *b = _txQueue.get(i);
    // check for packets ready to send
    if (b->state > DRONE_MESH_MSG_BUFFER_STATE_EMPTY) {
      if (getDroneMeshMsgPayloadType(b->data) == DRONE_MESH_MSG_TYPE_DRONELINKMSG) {
        // compare signatures and destNodes
        uint8_t offset = sizeof(DRONE_MESH_MSG_HEADER);
        if (memcmp(&b->data[offset], &msg->_msg, 5) == 0 &&
            getDroneMeshMsgDestNode(b->data) == destNode) {
          // signatures and destinations match... overwrite existing transmit buffer with new payload

          // copy msg data
          memcpy(&b->data[sizeof(DRONE_MESH_MSG_HEADER)], (uint8_t*)&msg->_msg, payloadSize);

          // calc CRC
          b->data[totalSize-1] = _CRC8.smbus((uint8_t*)b->data, totalSize-1);

          return true;
        }
      }
    }
  }


  /*
  uint8_t p = DRONE_MESH_MSG_PRIORITY_LOW;
  uint8_t g = DRONE_MESH_MSG_NOT_GUARANTEED;
  if (msg->type() < DRONE_LINK_MSG_TYPE_CHAR) {
    p = DRONE_MESH_MSG_PRIORITY_HIGH;
    g = DRONE_MESH_MSG_GUARANTEED;
  }
  */

  // let's try guaranteeing everything and see if prioritisation works
  uint8_t p = msg->priority();
  uint8_t g = DRONE_MESH_MSG_GUARANTEED;

  // request a new buffer in the transmit queue
  DRONE_MESH_MSG_BUFFER *buffer = getTransmitBuffer(interface, p);

  // if successful
  if (buffer) {

    DRONE_MESH_MSG_HEADER *header = (DRONE_MESH_MSG_HEADER*)buffer->data;

    // populate with a DroneLinkMsg packet
    header->typeGuaranteeSize = DRONE_MESH_MSG_SEND | g | (payloadSize-1) ;
    header->txNode = node();
    header->srcNode = node();
    header->nextNode = nextNode;
    header->destNode = destNode;
    header->seq = _gSeq++;
    //header->type = DRONE_MESH_MSG_TYPE_DRONELINKMSG;

    setDroneMeshMsgPriorityAndPayloadType(buffer->data, p, DRONE_MESH_MSG_TYPE_DRONELINKMSG);

    // copy msg data
    memcpy(&buffer->data[sizeof(DRONE_MESH_MSG_HEADER)], (uint8_t*)&msg->_msg, payloadSize);

    // calc CRC
    buffer->data[totalSize-1] = _CRC8.smbus((uint8_t*)buffer->data, totalSize-1);

    return true;
  }

  return false;
}


boolean DroneLinkManager::generateFSFileResponse(NetworkInterfaceModule *interface, uint8_t dest, uint8_t nextHop, DroneFSEntry* entry) {

  // request a new buffer in the transmit queue
  DRONE_MESH_MSG_BUFFER *buffer = getTransmitBuffer(interface, DRONE_MESH_MSG_PRIORITY_HIGH);

  // if successful
  if (buffer) {
    DRONE_MESH_MSG_FS_FILE_RESPONSE *rBuffer = (DRONE_MESH_MSG_FS_FILE_RESPONSE*)buffer->data;

    // populate packet
    rBuffer->header.typeGuaranteeSize = DRONE_MESH_MSG_SEND | DRONE_MESH_MSG_GUARANTEED | (sizeof(DRONE_MESH_MSG_FS_FILE_RESPONSE) - sizeof(DRONE_MESH_MSG_HEADER) - 2) ;
    rBuffer->header.txNode = node();;
    rBuffer->header.srcNode = node();
    rBuffer->header.nextNode = nextHop;
    rBuffer->header.destNode = dest;
    rBuffer->header.seq = _gSeq++;
    setDroneMeshMsgPriorityAndPayloadType(buffer->data, DRONE_MESH_MSG_PRIORITY_HIGH, DRONE_MESH_MSG_TYPE_FS_FILE_RESPONSE);

    // populate info
    if (entry) {
      if (entry->isDirectory()) {
        rBuffer->flags = DRONE_MESH_MSG_FS_FLAG_DIRECTORY;
      } else {
        rBuffer->flags = DRONE_MESH_MSG_FS_FLAG_FILE;
      }

      rBuffer->id = entry->getId();
      rBuffer->size = entry->getSize();
      entry->getPath((char*)rBuffer->path, DRONE_MESH_MSG_FS_MAX_PATH_SIZE-1);

    } else {
      rBuffer->flags = DRONE_MESH_MSG_FS_FLAG_NOT_FOUND;
      rBuffer->id = 0;
      rBuffer->size = 0;
      rBuffer->path[0] = 0;
    }

    // calc CRC
    rBuffer->crc = _CRC8.smbus((uint8_t*)rBuffer, sizeof(DRONE_MESH_MSG_FS_FILE_RESPONSE)-1);

    return true;
  }

  return false;
}


boolean DroneLinkManager::generateFirmwareStartResponse(NetworkInterfaceModule *interface, uint8_t dest, uint8_t status) {

  // request a new buffer in the transmit queue
  DRONE_MESH_MSG_BUFFER *buffer = getTransmitBuffer(interface, DRONE_MESH_MSG_PRIORITY_CRITICAL);

  // if successful
  if (buffer) {
    DRONE_MESH_MSG_FIRMWARE_START_RESPONSE *rBuffer = (DRONE_MESH_MSG_FIRMWARE_START_RESPONSE*)buffer->data;

    // populate packet
    rBuffer->header.typeGuaranteeSize = DRONE_MESH_MSG_SEND | DRONE_MESH_MSG_NOT_GUARANTEED | (sizeof(DRONE_MESH_MSG_FIRMWARE_START_RESPONSE) - sizeof(DRONE_MESH_MSG_HEADER) - 2) ;
    rBuffer->header.txNode = node();;
    rBuffer->header.srcNode = node();
    rBuffer->header.nextNode = dest;
    rBuffer->header.destNode = dest;
    rBuffer->header.seq = 0;
    setDroneMeshMsgPriorityAndPayloadType(buffer->data, DRONE_MESH_MSG_PRIORITY_CRITICAL, DRONE_MESH_MSG_TYPE_FIRMWARE_START_RESPONSE);

    // populate info
    rBuffer->status = status;

    // calc CRC
    rBuffer->crc = _CRC8.smbus((uint8_t*)rBuffer, sizeof(DRONE_MESH_MSG_FIRMWARE_START_RESPONSE)-1);

    return true;
  }

  return false;
}


boolean DroneLinkManager::generateFirmwareRewind(NetworkInterfaceModule *interface, uint8_t dest, uint32_t offset) {

  // request a new buffer in the transmit queue
  DRONE_MESH_MSG_BUFFER *buffer = getTransmitBuffer(interface, DRONE_MESH_MSG_PRIORITY_CRITICAL);

  // if successful
  if (buffer) {
    DRONE_MESH_MSG_FIRMWARE_REWIND *rBuffer = (DRONE_MESH_MSG_FIRMWARE_REWIND*)buffer->data;

    Log.noticeln("Firmware rewind %u", offset);

    // populate packet
    rBuffer->header.typeGuaranteeSize = DRONE_MESH_MSG_SEND | DRONE_MESH_MSG_NOT_GUARANTEED | (sizeof(DRONE_MESH_MSG_FIRMWARE_REWIND) - sizeof(DRONE_MESH_MSG_HEADER) - 2) ;
    rBuffer->header.txNode = node();;
    rBuffer->header.srcNode = node();
    rBuffer->header.nextNode = dest;
    rBuffer->header.destNode = dest;
    rBuffer->header.seq = 0;
    setDroneMeshMsgPriorityAndPayloadType(buffer->data, DRONE_MESH_MSG_PRIORITY_CRITICAL, DRONE_MESH_MSG_TYPE_FIRMWARE_REWIND);

    // populate info
    rBuffer->offset = offset;

    // calc CRC
    rBuffer->crc = _CRC8.smbus((uint8_t*)rBuffer, sizeof(DRONE_MESH_MSG_FIRMWARE_REWIND)-1);

    // check we've not already got an equivalent packet in the queue
    scrubDuplicateTransmitBuffers(buffer);

    _firmwareLastRewind = millis();

    return true;
  }

  return false;
}


void DroneLinkManager::processFirmwareUpdate() {
  uint32_t loopTime = millis();
  if (_firmwareStarted &&
      _firmwarePos < _firmwareSize &&
      loopTime > _firmwareLastRewind + 250) {

    generateFirmwareRewind(getSourceInterface(_firmwareSrc), _firmwareSrc, _firmwarePos);
  }
}

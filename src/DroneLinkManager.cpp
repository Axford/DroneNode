#include "Arduino.h"
#include "DroneLinkManager.h"

DroneLinkManager::DroneLinkManager():
  _node(0),
  _channels(IvanLinkedList::LinkedList<DroneLinkChannel*>())
{
  _publishedMessages = 0;
  _peerNodes = 0;
  _minPeer = 255;
  _maxPeer = 0;
  for (uint8_t i=0; i<DRONE_LINK_NODE_PAGES; i++) {
    _nodePages[i] = NULL;
  }
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
    c = new DroneLinkChannel(node, channel);
    _channels.add(c);
  }

  // wire up the subscriber to the channel
  c->subscribe(subscriber, param);

  // print new set of subscribers
  //c->printSubscribers();
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

void DroneLinkManager::processChannels() {
  DroneLinkChannel* c;
  for(int i = 0; i < _channels.size(); i++){
    c = _channels.get(i);
    c->processQueue();
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

DRONE_LINK_NODE_INFO* DroneLinkManager::getNodeInfo(uint8_t source) {
  // get page
  uint8_t pageIndex = source >> 4;  // div by 16
  uint8_t nodeIndex = source & 0xF;

  // see if page exists
  DRONE_LINK_NODE_PAGE* page = _nodePages[pageIndex];
  if (page != NULL) {
    if (page->nodeInfo[nodeIndex].heard) {
      return &page->nodeInfo[nodeIndex];
    } else
      return NULL;
  } else
    return NULL;
}

uint8_t DroneLinkManager::getSourceInterface(uint8_t source) {
  DRONE_LINK_NODE_INFO* nodeInfo = getNodeInfo(source);

  if (nodeInfo == NULL) {
    return 0;
  } else
    return nodeInfo->interface;
}


void DroneLinkManager::serveNodeInfo(AsyncWebServerRequest *request) {
  AsyncResponseStream *response = request->beginResponseStream("text/text");
  response->addHeader("Server","ESP Async Web Server");
  response->print(F("Source Node Info: \n"));

  unsigned long loopTime = millis();

  DRONE_LINK_NODE_PAGE *page;
  for (uint8_t i=0; i<DRONE_LINK_NODE_PAGES; i++) {
    page = _nodePages[i];
    if (page != NULL) {
      for (uint8_t j=0; j<DRONE_LINK_NODE_PAGE_SIZE; j++) {
        if (page->nodeInfo[j].heard) {
          uint8_t id = (i << 4) + j;
          unsigned int age = (loopTime - page->nodeInfo[j].lastHeard) / 1000;
          response->printf("%u > ", id);
          if (page->nodeInfo[j].name == NULL) {
            response->print("???");
          } else
            response->printf("%s", page->nodeInfo[j].name);
          response->printf(", RSSI: %u, Age: %u sec, int: %u\n", page->nodeInfo[j].RSSI, age, page->nodeInfo[j].interface);
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
  response->print(F("Channels: \n"));

  DroneLinkChannel* c;
  for(int i = 0; i < _channels.size(); i++){
    c = _channels.get(i);
    response->printf("%u > %u = %u (%u)\n", c->node(), c->id(), c->size(), c->peakSize());
  }


  //send the response last
  request->send(response);
}

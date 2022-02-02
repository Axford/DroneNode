#include "NetworkInterfaceModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"

NetworkInterfaceModule::NetworkInterfaceModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs):
  DroneModule ( id, dmm, dlm, dem, fs ),
  _txQueue(IvanLinkedList::LinkedList<DRONE_MESH_MSG_BUFFER*>())
 {
   _interfaceState = false;  // start inactive
   _helloSeq = 0;
   _helloTimer = 0;
   _seqTimer = 0;
}


void NetworkInterfaceModule::loop() {
  DroneModule::loop();

  uint32_t loopTime = millis();

  if (loopTime > _helloTimer + NETWORK_INTERFACE_HELLO_INTERVAL || _helloTimer == 0) {
    generateHello();
    _helloTimer = loopTime;
  }

  processTransmitQueue();
}


boolean NetworkInterfaceModule::getInterfaceState() {
  return _interfaceState && _enabled;
}


uint8_t NetworkInterfaceModule::getTxQueueSize() {
  return _txQueue.size();
}


DRONE_MESH_MSG_BUFFER* NetworkInterfaceModule::getTransmitBuffer() {
  DRONE_MESH_MSG_BUFFER *buffer = NULL;

  if (!getInterfaceState()) return NULL;

  // see if we have an empty transmit buffer than can be used
  for (uint8_t i=0; i<_txQueue.size(); i++) {
    DRONE_MESH_MSG_BUFFER *b = _txQueue.get(i);
    if (b->state == DRONE_MESH_MSG_BUFFER_STATE_EMPTY) {
      buffer = b;
      break;
    }
  }

  // if none available, see if we have space to create one
  if (!buffer && _txQueue.size() < NETWORK_INTERFACE_MAX_TX_QUEUE) {
    buffer = (DRONE_MESH_MSG_BUFFER*)malloc(sizeof(DRONE_MESH_MSG_BUFFER));
    _txQueue.add(buffer);
  }

  if (buffer) {
    // update state
    buffer->state = DRONE_MESH_MSG_BUFFER_STATE_READY;
  }

  return buffer;
}


void NetworkInterfaceModule::processTransmitQueue() {
  // look through txQueue and see if anything is Ready to send
  for (uint8_t i=0; i<_txQueue.size(); i++) {
    DRONE_MESH_MSG_BUFFER *b = _txQueue.get(i);
    if (b->state == DRONE_MESH_MSG_BUFFER_STATE_READY) {
      if (sendPacket(b->data)) {
        // if this is guaranteed, then wait for a reply
        if (isDroneMeshMsgGuaranteed(b->data)) {
          //b->state = DRONE_MESH_MSG_BUFFER_STATE_WAITING;
          // TODO - actually implement guaranteed delivery
          b->state = DRONE_MESH_MSG_BUFFER_STATE_EMPTY;
        } else {
          // otherwise set to empty
          b->state = DRONE_MESH_MSG_BUFFER_STATE_EMPTY;
        }
      }

      // just the one Mrs Wemberley
      break;
    }
  }
}


boolean NetworkInterfaceModule::generateNextHop(uint8_t *pbuffer, uint8_t nextHop) {
  if (!getInterfaceState()) return false;

  // request a new buffer in the transmit queue
  DRONE_MESH_MSG_BUFFER *buffer = getTransmitBuffer();

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
    header->txNode = _dlm->node();

    // update CRC
    buffer->data[len-1] = _CRC8.smbus((uint8_t*)buffer, len-1);

    return true;
  }

  return false;
}


void NetworkInterfaceModule::generateHello() {
  Serial.println("[NIM.gH]");
  if (getInterfaceState()) {
    if (generateHello(_dlm->node(), _helloSeq, 0)) {
      uint32_t loopTime = millis();
      if (loopTime > _seqTimer + NETWORK_INTERFACE_SEQ_INTERVAL) {
        _helloSeq++;
        _seqTimer = loopTime;
      }
    }
  }
}


boolean NetworkInterfaceModule::generateHello(uint8_t src, uint8_t seq, uint8_t metric) {
  if (!getInterfaceState()) return false;

  // request a new buffer in the transmit queue
  DRONE_MESH_MSG_BUFFER *buffer = getTransmitBuffer();

  // if successful
  if (buffer) {
    DRONE_MESH_MSG_HELLO *helloBuffer = (DRONE_MESH_MSG_HELLO*)buffer->data;

    // populate with a Hello packet
    helloBuffer->header.modeGuaranteeSize = DRONE_MESH_MSG_MODE_MULTICAST | DRONE_MESH_MSG_NOT_GUARANTEED | 0 ;  // payload is 1 byte... sent as n-1
    helloBuffer->header.txNode = _dlm->node();
    helloBuffer->header.srcNode = src;
    helloBuffer->header.nextNode = 0;
    helloBuffer->header.destNode = 0;
    helloBuffer->header.seq = seq;
    helloBuffer->header.typeDir = DRONE_MESH_MSG_TYPE_HELLO | DRONE_MESH_MSG_REQUEST;
    helloBuffer->metric = metric;

    // calc CRC
    helloBuffer->crc = _CRC8.smbus((uint8_t*)helloBuffer, sizeof(DRONE_MESH_MSG_HELLO)-1);

    return true;
  }

  return false;
}


boolean NetworkInterfaceModule::generateSubscriptionRequest(uint8_t src, uint8_t next, uint8_t dest, uint8_t channel, uint8_t param) {
  if (!getInterfaceState()) return false;

  // request a new buffer in the transmit queue
  DRONE_MESH_MSG_BUFFER *buffer = getTransmitBuffer();

  // if successful
  if (buffer) {
    DRONE_MESH_MSG_SUBCSRIPTION *subBuffer = (DRONE_MESH_MSG_SUBCSRIPTION*)buffer->data;

    // populate with a Hello packet
    subBuffer->header.modeGuaranteeSize = DRONE_MESH_MSG_MODE_UNICAST | DRONE_MESH_MSG_GUARANTEED | 1 ;  // payload is 2 byte... sent as n-1
    subBuffer->header.txNode = src;
    subBuffer->header.srcNode = _dlm->node();
    subBuffer->header.nextNode = next;
    subBuffer->header.destNode = dest;
    subBuffer->header.seq = 0;
    subBuffer->header.typeDir = DRONE_MESH_MSG_TYPE_SUBSCRIPTION | DRONE_MESH_MSG_REQUEST;
    subBuffer->channel = channel;
    subBuffer->param = param;

    // calc CRC
    subBuffer->crc = _CRC8.smbus((uint8_t*)subBuffer, sizeof(DRONE_MESH_MSG_SUBCSRIPTION)-1);

    return true;
  }

  return false;
}


boolean NetworkInterfaceModule::generateTraceroute(uint8_t destNode, uint8_t nextNode) {
  if (!getInterfaceState()) return false;

  // request a new buffer in the transmit queue
  DRONE_MESH_MSG_BUFFER *buffer = getTransmitBuffer();

  // if successful
  if (buffer) {
    DRONE_MESH_MSG_TRACEROUTE *tBuffer = (DRONE_MESH_MSG_TRACEROUTE*)buffer->data;

    // populate with a Hello packet
    tBuffer->header.modeGuaranteeSize = DRONE_MESH_MSG_MODE_UNICAST | DRONE_MESH_MSG_GUARANTEED | 1 ;  // payload is 2 byte... sent as n-1
    tBuffer->header.txNode = _dlm->node();;
    tBuffer->header.srcNode = _dlm->node();
    tBuffer->header.nextNode = nextNode;
    tBuffer->header.destNode = destNode;
    tBuffer->header.seq = 0;
    tBuffer->header.typeDir = DRONE_MESH_MSG_TYPE_TRACEROUTE | DRONE_MESH_MSG_REQUEST;
    tBuffer->dummyNode = _dlm->node();
    tBuffer->dummyMetric = 0;

    // calc CRC
    tBuffer->crc = _CRC8.smbus((uint8_t*)tBuffer, sizeof(DRONE_MESH_MSG_TRACEROUTE)-1);

    return true;
  }

  return false;
}


boolean NetworkInterfaceModule::sendDroneLinkMessage(uint8_t destNode, uint8_t nextNode, DroneLinkMsg *msg) {
  if (!getInterfaceState()) return false;

  // request a new buffer in the transmit queue
  DRONE_MESH_MSG_BUFFER *buffer = getTransmitBuffer();

  // if successful
  if (buffer) {
    uint8_t payloadSize = msg->totalSize();
    uint8_t totalSize = payloadSize + sizeof(DRONE_MESH_MSG_HEADER) + 1;

    DRONE_MESH_MSG_HEADER *header = (DRONE_MESH_MSG_HEADER*)buffer->data;

    // populate with a DroneLinkMsg packet
    header->modeGuaranteeSize = DRONE_MESH_MSG_MODE_UNICAST | DRONE_MESH_MSG_GUARANTEED | (payloadSize-1) ;
    header->txNode = _dlm->node();
    header->srcNode = _dlm->node();
    header->nextNode = nextNode;
    header->destNode = destNode;
    header->seq = 0;
    header->typeDir = DRONE_MESH_MSG_TYPE_DRONELINKMSG | DRONE_MESH_MSG_REQUEST;

    // copy msg data
    memcpy(&buffer->data[sizeof(DRONE_MESH_MSG_HEADER)], (uint8_t*)&msg->_msg, payloadSize);

    // calc CRC
    buffer->data[totalSize-1] = _CRC8.smbus((uint8_t*)buffer->data, totalSize-1);

    return true;
  }

  return false;
}


boolean NetworkInterfaceModule::sendPacket(uint8_t *buffer) {
  // to be inherited
  return false;
}


void NetworkInterfaceModule::receivePacket(uint8_t *buffer, uint8_t metric) {
  // validate packet
  uint8_t len = getDroneMeshMsgTotalSize(buffer);
  uint8_t crc = _CRC8.smbus((uint8_t*)buffer, len-1);
  if (buffer[len-1] != crc) {
    Serial.println("[NIM.rP] CRC fail");
    return;
  }

  Log.noticeln("[NIM.rP] Rec %u bytes with metric %u on int: ", len, metric, getName());

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

  // see if this is an acknowledgement of a guaranteed delivery
  // TODO



  // otherwise, pass onto DLM for processing
  _dlm->receivePacket(this, buffer, metric);
}

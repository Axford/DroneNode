#include "NetworkInterfaceModule.h"
#include "../DroneLinkMsg.h"
#include "../DroneLinkManager.h"
#include "strings.h"

NetworkInterfaceModule::NetworkInterfaceModule(uint8_t id, DroneModuleManager* dmm, DroneLinkManager* dlm, DroneExecutionManager* dem, fs::FS &fs):
  DroneModule ( id, dmm, dlm, dem, fs ),
  _txQueue(IvanLinkedList::LinkedList<DRONE_MESH_MSG_BUFFER*>())
 {
   _interfaceState = false;  // start inactive
   _helloSeq = 255; // ready for rollover on first Hello
   _helloTimer = 0;
}


void NetworkInterfaceModule::loop() {
  DroneModule::loop();

  uint32_t loopTime = millis();

  if (loopTime > _helloTimer + NETWORK_INTERFACE_HELLO_INTERVAL) {
    generateHello();
    _helloTimer = loopTime;
  }

  processTransmitQueue();
}


boolean NetworkInterfaceModule::getInterfaceState() {
  return _interfaceState;
}


DRONE_MESH_MSG_BUFFER* NetworkInterfaceModule::getTransmitBuffer() {
  DRONE_MESH_MSG_BUFFER *buffer = NULL;

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
          b->state = DRONE_MESH_MSG_BUFFER_STATE_WAITING;
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


void NetworkInterfaceModule::generateHello() {
  Serial.println("[NIM.gH]");
  if (getInterfaceState()) {
    if (generateHello(_dlm->node(), _helloSeq, 0)) {
      _helloSeq++;
    }
  }
}


boolean NetworkInterfaceModule::generateHello(uint8_t src, uint8_t seq, uint8_t metric) {
  // request a new buffer in the transmit queue
  DRONE_MESH_MSG_BUFFER *buffer = getTransmitBuffer();

  // if successful
  if (buffer) {
    DRONE_MESH_MSG_HELLO *helloBuffer = (DRONE_MESH_MSG_HELLO*)buffer->data;

    // populate with a Hello packet
    helloBuffer->header.modeGuaranteeSize = DRONE_MESH_MSG_MODE_MULTICAST | DRONE_MESH_MSG_NOT_GUARANTEED | 0 ;  // payload is 1 byte... sent as n-1
    helloBuffer->header.txNode = _dlm->node();
    helloBuffer->header.srcNode = src;
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

  Serial.print("[NIM.rP] recv metric: ");
  Serial.println(metric);


  // decode packet header
  Serial.print("Header: ");
  for (uint8_t i=0; i<6; i++) {
    Serial.print(buffer[i], BIN);
    Serial.print(" ");
  }
  Serial.println();

  // see if this is an acknowledgement of a guaranteed delivery
  // TODO



  // otherwise, pass onto DLM for processing
  _dlm->receivePacket(this, buffer, metric);
}

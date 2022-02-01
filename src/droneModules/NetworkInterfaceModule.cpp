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
  // TODO
  if (getInterfaceState()) {
    // request a new buffer in the transmit queue
    DRONE_MESH_MSG_BUFFER *buffer = getTransmitBuffer();

    // if successful
    if (buffer) {
      // increment sequence
      _helloSeq++;

      DRONE_MESH_MSG_HELLO *helloBuffer = (DRONE_MESH_MSG_HELLO*)buffer->data;

      // populate with a Hello packet
      helloBuffer->header.modeGuaranteeSize = DRONE_MESH_MSG_MODE_MULTICAST | DRONE_MESH_MSG_NOT_GUARANTEED | 0 ;  // payload is 1 byte... sent as n-1
      helloBuffer->header.txNode = _dlm->node();
      helloBuffer->header.srcNode = _dlm->node();
      helloBuffer->header.destNode = 0;
      helloBuffer->header.seq = _helloSeq;
      helloBuffer->header.typeDir = DRONE_MESH_MSG_TYPE_HELLO | DRONE_MESH_MSG_REQUEST;
      helloBuffer->metric = 0;

      // calc CRC
      // TODO


    }
  }
}

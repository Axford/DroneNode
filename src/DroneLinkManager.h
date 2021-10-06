#ifndef DRONE_LINK_MANAGER_H
#define DRONE_LINK_MANAGER_H

#include "Arduino.h"
#include "LinkedList.h"

#include "DroneLinkMsg.h"
#include "DroneLinkChannel.h"


class DroneLinkManager
{
protected:
  uint8_t _node;  // local node id
  unsigned long _publishedMessages;
  IvanLinkedList::LinkedList<DroneLinkChannel*> _channels;

public:
    DroneLinkManager();

    void node(uint8_t node);
    uint8_t node();

    uint32_t getChokes();

    void subscribe(DRONE_LINK_ADDR *addr, DroneModule *subscriber);
    void subscribe(uint8_t channel, DroneModule *subscriber, uint8_t param);
    void subscribe(uint8_t node, uint8_t channel, DroneModule *subscriber, uint8_t param);

    bool publish(DroneLinkMsg msg);

    void processChannels();

    DroneLinkChannel* findChannel(uint8_t node, uint8_t chan);

    unsigned long publishedMessages();
    void resetPublishedMessages();
};


#endif

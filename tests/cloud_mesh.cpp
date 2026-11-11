#include <cassert>
#include <cstdint>
#include <cstring>
#include <vector>
#include "CloudCommandWire.h"

constexpr uint8_t DRONE_LINK_MSG_TYPE_CHAR=4, DRONE_LINK_MSG_TYPE_NAME=5,
    DRONE_LINK_MSG_TYPE_NAMEQUERY=6, DRONE_LINK_MSG_TYPE_QUERY=7;
constexpr uint8_t DRONE_MESH_MSG_TYPE_CLOUD_REQUEST=27, DRONE_MESH_MSG_TYPE_CLOUD_REPLY=28;
constexpr unsigned DRONE_LINK_MANAGER_MAX_TX_QUEUE=24;
constexpr uint8_t DRONE_MESH_MSG_BUFFER_STATE_EMPTY=0;
static bool isDroneMeshMsgAck(const uint8_t* data) {return data[0] & 128;}
static uint8_t getDroneMeshMsgPayloadType(const uint8_t* data) {return data[6] & 63;}
static uint8_t getDroneMeshMsgSrcNode(const uint8_t* data) {return data[2];}
struct MeshBuffer {uint8_t data[7];uint8_t state;};
template <typename T> struct Queue : std::vector<T> {
  T get(size_t index) {return (*this)[index];}
  int size() const {return static_cast<int>(std::vector<T>::size());}
};
constexpr size_t DRONE_LINK_CHANNEL_QUEUE_LIMIT=30;
struct DRONE_MESH_MSG_HEADER {uint8_t size,txNode,srcNode,nextNode,destNode,seq,type;};
static bool isDroneMeshMsgGuaranteed(uint8_t*) {return true;}
static size_t getDroneMeshMsgPayloadSize(uint8_t* buffer) {return buffer[0];}
struct DroneLinkMsg {
  struct {uint8_t source,node,channel,param,shape;struct {uint8_t uint8[16];} payload;} _msg{};
  bool tryParse(const uint8_t* p,size_t size) {
    if(size<6 || size>21 || size!=size_t(6+(p[4]&15))) return false;
    memcpy(&_msg,p,size);return true;
  }
  uint8_t source() const {return _msg.source;} void source(uint8_t n) {_msg.source=n;}
  uint8_t node() const {return _msg.node;}
  uint8_t channel() const {return _msg.channel;}
  uint8_t param() const {return _msg.param&63;}
  uint8_t priority() const {return _msg.param>>6;}
  uint8_t type() const {return (_msg.shape>>4)&7;}
  uint8_t length() const {return (_msg.shape&15)+1;}
  bool writable() const {return _msg.shape&128;}
};
struct Channel {
  std::vector<DroneLinkMsg> queue;
  bool hasLocalTarget(){return true;}
  void discardCloudRequests(){}
  size_t size(){return queue.size();}
  bool publish(DroneLinkMsg& message,bool injected){assert(injected);queue.push_back(message);return true;}
};
struct DroneModule;
struct DroneLinkManager {
  uint8_t _node=7;
  DroneLinkMsg* _peerQuery=nullptr;
  void* _publicationObserverContext=nullptr;
  void (*_publicationObserver)(void*,const DroneLinkMsg&,bool)=nullptr;
  struct Route {void* gSequencer;} route{this};
  Route* getNodeInfo(uint8_t,bool){return &route;}
  bool hasCommandRoute(uint8_t){return true;}
  Queue<MeshBuffer*> _txQueue;
  Queue<Channel*> _channels;
  bool cloudTxAvailable();
  void discardCloudRequests();
  Channel channel;
  unsigned hops=0, ordinary=0, replies=0;
  DroneLinkMsg sent;
  Channel* findChannel(uint8_t node,uint8_t id){return node==_node&&id==4 ? &channel : nullptr;}
  void hopAlong(uint8_t*) {++hops;}
  bool publish(DroneLinkMsg&) {++ordinary;return true;}
  bool sendCloudLink(uint8_t peer,DroneLinkMsg& msg,uint8_t kind) {
    assert(peer==8&&kind==DRONE_MESH_MSG_TYPE_CLOUD_REPLY);
    sent=msg;++replies;return true;
  }
  void dispatchCloudRequest(DroneModule*,DroneLinkMsg&);
  void receiveCloudLink(uint8_t*,bool);
  bool publishLocalObservation(DroneLinkMsg&);
};
struct DroneModule {
  DroneLinkManager& manager;
  explicit DroneModule(DroneLinkManager& owner):manager(owner){}
  void handleLinkMessage(DroneLinkMsg&)=delete;
  void handleLinkMessage(DroneLinkMsg* request) {
    DroneLinkMsg result;
    const uint8_t data[]={0,7,4,request->param(),0x80,42};
    assert(result.tryParse(data,sizeof data));
    manager.publishLocalObservation(result);
    manager.publishLocalObservation(result);
  }
};
#include "actual_cloud_mesh.inc"

int main() {
  DroneLinkManager manager;
  uint8_t request[]={6,8,8,7,7,0,27,8,7,4,2,112,0};
  manager.receiveCloudLink(request,false);
  assert(manager.channel.size()==1 && manager.ordinary==0);
  DroneModule target(manager);
  manager.dispatchCloudRequest(&target,manager.channel.queue[0]);
  assert(manager.replies==1 && manager.ordinary==1);
  assert(manager.sent.source()==7 && manager.sent.node()==7 && manager.sent._msg.payload.uint8[0]==42);
  request[4]=9;
  manager.receiveCloudLink(request,false);
  assert(manager.hops==1 && manager.channel.size()==1);
  request[4]=7;request[7]=9;
  manager.receiveCloudLink(request,false);
  assert(manager.channel.size()==1);
  request[7]=8;request[11]=0x80;
  manager.receiveCloudLink(request,false);
  assert(manager.channel.size()==1);
  unsigned captured=0;
  manager._publicationObserverContext=&captured;
  manager._publicationObserver=[](void* state,const DroneLinkMsg& reply,bool directed){
    assert(directed&&reply.node()==8&&reply.source()==8);
    ++*static_cast<unsigned*>(state);
  };
  uint8_t reply[]={6,8,8,7,7,0,28,8,8,4,2,0x80,42};
  manager.receiveCloudLink(reply,true);
  assert(captured==1&&manager.ordinary==1);
  reply[8]=9;
  manager.receiveCloudLink(reply,true);
  assert(captured==1);

  MeshBuffer ownRequest{{0,7,7,8,8,1,27},1};
  MeshBuffer inboundAck{{128,7,7,8,8,2,27},1};
  MeshBuffer transit{{0,7,8,9,9,3,27},1};
  MeshBuffer peerReply{{0,7,7,8,8,4,28},1};
  manager._txQueue.push_back(&ownRequest);
  manager._txQueue.push_back(&inboundAck);
  manager._txQueue.push_back(&transit);
  manager._txQueue.push_back(&peerReply);
  assert(manager.cloudTxAvailable());
  manager.discardCloudRequests();
  assert(ownRequest.state==0 && inboundAck.state==1 && transit.state==1 && peerReply.state==1);
}

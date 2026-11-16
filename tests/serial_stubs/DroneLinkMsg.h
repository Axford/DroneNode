#pragma once
#include "DroneModule.h"
inline uint8_t getDroneMeshMsgTotalSize(uint8_t* p){return ((p[0]&63)>=48?48:(p[0]&63)+1)+8;}

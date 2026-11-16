#pragma once
#define ERR_OK 0
#include <functional>
#include <vector>
extern std::vector<std::function<void()>> dnsJobs;
inline int tcpip_callback(void(*callback)(void*),void* arg){dnsJobs.push_back([=](){callback(arg);});return 0;}

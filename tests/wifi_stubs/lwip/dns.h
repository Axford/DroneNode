#pragma once
#include <stdint.h>
#define DNS_MAX_SERVERS 3
struct ip_addr_t {uint8_t octets[4];};
#define IPADDR4_INIT(n) {{8,8,8,8}}
#define IP_ADDR4(p,a,b,c,d) (*(p)=ip_addr_t{{a,b,c,d}})
extern ip_addr_t resolvers[3];extern unsigned dnsWrites;
inline void dns_setserver(unsigned i,const ip_addr_t* address){resolvers[i]=*address;++dnsWrites;}

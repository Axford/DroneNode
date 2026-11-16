#include "WiFiManager.h"
#include "WiFi.h"
#include "lwip/dns.h"
#include "lwip/tcpip.h"
#include <cassert>
std::vector<std::function<void()>> dnsJobs;
void dispatch(){for(auto &job:dnsJobs)job();dnsJobs.clear();}
Logger Log;WiFiClass WiFi;ip_addr_t resolvers[3]={};unsigned dnsWrites=0;
int main(){
 for(const char* bad:{"","0.0.0.0","127.0.0.1","224.0.0.1","256.1.1.1","1.2.3","1.2.3.4x","01.2.3.4","name.test"}) {DnsOverride value;assert(!value.parse(bad));}
 {
 WiFiManager legacy;fs::FS fs;legacy.loadConfiguration(fs);legacy.start();
 assert(WiFi.registrations==0&&dnsWrites==0);
 legacy.disable();legacy.enable();assert(dnsWrites==1&&resolvers[0].octets[0]==8);dnsWrites=0;
 }
 for(const char* invalid:{"null","12","\"not-an-IP-DO-NOT-LOG\""}) {
 WiFiManager invalidManager;fs::FS fs;fs.text=std::string("{\"dnsServer\":")+invalid+"}";invalidManager.loadConfiguration(fs);invalidManager.start();assert(WiFi.registrations==0);
 }
 WiFiManager configured;fs::FS fs;fs.text="{\"dnsServer\":\"192.168.1.50\",\"networks\":[{\"ssid\":\"fixture\",\"password\":\"DO-NOT-LOG\"}]}";
 configured.loadConfiguration(fs);configured.start();assert(WiFi.registrations==1);
 assert(dnsWrites==0);dispatch();
 for(unsigned reconnect=0;reconnect<3;++reconnect) {
  for(auto &r:resolvers)r={{8,8,8,8}};
  WiFi.callback(ARDUINO_EVENT_WIFI_STA_GOT_IP,0);dispatch();
  for(const auto &r:resolvers)assert(r.octets[0]==192&&r.octets[3]==50);
 }
 assert(dnsWrites==12);assert(Log.text.find("DO-NOT-LOG")==std::string::npos);
}

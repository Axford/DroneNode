
#pragma once
#include <cstdint>
#include <cstring>
#include <cmath>
#include <string>
using boolean = bool;
#define __packed __attribute__((packed))
#define F(x) x
class String {
  std::string value;
public:
  String(std::string v = "") : value(v) {}
  int indexOf(const char* s) const { auto p = value.find(s); return p == std::string::npos ? -1 : int(p); }
  String substring(int first, int last) const { return String(value.substr(first, last-first)); }
  String substring(int first) const { return String(value.substr(first)); }
  long toInt() const { return std::strtol(value.c_str(), nullptr, 10); }
  unsigned length() const { return value.size(); }
};
class HostPrint {
public:
  template<class... Args> void print(Args...) {}
  template<class... Args> void println(Args...) {}
  template<class... Args> void printf(Args...) {}
};
inline HostPrint Serial;
using std::isnan;
using std::isinf;
using std::fabs;

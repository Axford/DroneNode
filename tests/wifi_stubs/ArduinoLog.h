#pragma once
#include <string>
struct Logger {
 std::string text;
 void record(const char* value){text+=value;}
 void record(const String& value){text+=value;}
 template<class T>void record(T){}
 template<class... T>void noticeln(const char* s,T... args){text+=s;int unused[]={0,(record(args),0)...};(void)unused;}
 template<class... T>void errorln(const char* s,T... args){noticeln(s,args...);}
};
extern Logger Log;

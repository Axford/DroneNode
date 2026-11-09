#ifndef CONFIGURATION_LINE_H
#define CONFIGURATION_LINE_H
#include <stddef.h>
#include <string.h>
namespace configuration {
enum class Setting { Unhandled, Handled, Invalid };
// Same whitespace/semicolon/quote rules as config.ini, with opt-in long-string escapes.
struct Line {
  char name[20]={},value[769]={};
  bool section=false,empty=true;
  bool parse(const char* raw,bool escapes) {
    name[0]=value[0]=0; section=false;empty=true;
    bool inValue=false,inString=false,escaped=false;size_t n=0,v=0;
    for(const char* p=raw;*p;++p) {
      char c=*p;
      if(escaped) {
        if(c!='"'&&c!='\\') return false;
        escaped=false;
      } else {
        if(inString && escapes && c=='\\') {escaped=true;continue;}
        if(c=='"') {inString=!inString;continue;}
        if(!inString) {
          if(c==';') break;
          if(c==' '||c=='\t'||c=='\r'||c=='\n') continue;
          if(c=='['&&!n&&!inValue) {section=true;continue;}
          if(c==']'&&section) continue;
          if(c=='='&&!inValue) {inValue=true;continue;}
        }
      }
      empty=false;
      if(inValue) { if(v==sizeof(value)-1) return false;value[v++]=c;value[v]=0; }
      else { if(n==sizeof(name)-1) return false;name[n++]=c;name[n]=0; }
    }
    return empty || (!inString&&!escaped&&inValue&&n&&v);
  }
};
}
#endif

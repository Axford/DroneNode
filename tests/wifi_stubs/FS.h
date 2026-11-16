#pragma once
#include <string>
#include <cstring>
#define FILE_READ 0
struct File {
 std::string text;size_t offset=0;
 File(std::string value):text(value){}
 int read(){return offset<text.size()?unsigned(text[offset++]):-1;}
 size_t readBytes(char* target,size_t size){size_t n=0;for(;n<size&&offset<text.size();++n)target[n]=text[offset++];return n;}
 void close(){}
};
namespace fs {struct FS {std::string text;bool exists(const char*){return !text.empty();}File open(const char*,int){return File(text);}};}

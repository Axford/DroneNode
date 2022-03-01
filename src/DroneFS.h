/*

Manage and abstract local filesystem

Cache directory structure for fast enumeration

*/

#ifndef DRONE_FS_H
#define DRONE_FS_H


#include "Arduino.h"
#include "LinkedList.h"
#include "FS.h"
#include <LITTLEFS.h>

#define DRONE_FS_MAX_NAME_SIZE    13  // 8.3 + null
#define DRONE_FS_MAX_PATH_SIZE    24  // inc null termination

// forward decl
class DroneFS;

//--------------------------------------------------------
// DroneFSEntry
//--------------------------------------------------------
class DroneFSEntry {
protected:
  DroneFS* _fs;
  File _file;  // our own file handle... used in read operations
  DroneFSEntry* _parent;
  char _name[DRONE_FS_MAX_NAME_SIZE];
  uint8_t _id;
  boolean _isDir;
  boolean _enumerated;
  uint32_t _size;
  IvanLinkedList::LinkedList<DroneFSEntry*> _children;
public:
  DroneFSEntry(DroneFS* fs, DroneFSEntry* parent, const char* name, boolean isDir);

  void setName(const char* name);
  char* getName();

  void setId(uint8_t id);
  uint8_t getId();

  uint32_t getSize();
  boolean setSize(uint32_t size);

  void getPath(char * path, uint8_t maxLen);
  boolean matchesPath(char* path);

  void isDirectory(boolean isDir);
  boolean isDirectory();

  boolean enumerate(); // if a directory

  void readProperties(File f);  // read properties from an open file handle, used by enumerate

  boolean readBlock(uint32_t offset, uint8_t* buffer, uint8_t size);
  boolean writeBlock(uint32_t offset, uint8_t* buffer, uint8_t size);

  DroneFSEntry* getEntryByPath(char* path);
  DroneFSEntry* getEntryByIndex(uint8_t index);
  DroneFSEntry* getEntryById(uint8_t id);
};


//--------------------------------------------------------
// DroneFS
//--------------------------------------------------------
class DroneFS {
protected:
  uint8_t _nextId;
  DroneFSEntry* _root;  // root entry "/"


public:
  DroneFS();

  void setup();

  uint8_t getNextId();
  DroneFSEntry* getEntryByPath(char* path);
  DroneFSEntry* getEntryByIndex(char* path, uint8_t index);
  DroneFSEntry* getEntryById(uint8_t id);

};

#endif

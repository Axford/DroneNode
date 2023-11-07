/*

Manage and abstract local filesystem

Cache directory structure for fast enumeration

*/

#ifndef DRONE_FS_H
#define DRONE_FS_H


#include "Arduino.h"
#include "LinkedList.h"
#include "FS.h"
#include <LittleFS.h>
#define LITTLEFS LittleFS

#define DRONE_FS_MAX_NAME_SIZE    14  // forward-slash plus 8.3 + null
#define DRONE_FS_MAX_PATH_SIZE    24  // inc null termination

// forward decl
class DroneFS;
class DroneSystem;

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

  void close();

  DroneFSEntry* getParent();

  void setName(const char* name);
  char* getName();

  void setId(uint8_t id);
  uint8_t getId();

  uint32_t getSize();
  void setSize(uint32_t size);  // set internal size value, doesn't affect disk

  void getPath(char * path, uint8_t maxLen);
  boolean matchesPath(char* path);

  void isDirectory(boolean isDir);
  boolean isDirectory();

  boolean enumerate(); // if a directory

  void readProperties(File f);  // read properties from an open file handle, used by enumerate

  boolean readBlock(uint32_t offset, uint8_t* buffer, uint8_t size);
  //boolean writeBlock(uint32_t offset, uint8_t* buffer, uint8_t size);

  DroneFSEntry* getEntryByPath(char* path);
  DroneFSEntry* getEntryByIndex(uint8_t index);
  DroneFSEntry* getEntryById(uint8_t id);
};


//--------------------------------------------------------
// DroneFS
//--------------------------------------------------------

#define DRONE_FS_UPLOAD_STATE_NONE       0  // no upload initiated, buffer is null
#define DRONE_FS_UPLOAD_STATE_WIP        1  // upload initiated and in progress
#define DRONE_FS_UPLOAD_STATE_COMPLETE   2  // all blocks uploaded, ready to be written to disk


class DroneFS {
protected:
  DroneSystem* _ds;
  uint8_t _nextId;
  DroneFSEntry* _root;  // root entry "/"

  // Upload mgmt
  uint8_t* _uploadBuffer; // malloc space to receive uploaded file blocks ahead of being written to disk
  uint32_t _uploadSize;  // size of upload buffer
  char _uploadPath[DRONE_FS_MAX_PATH_SIZE];  // target path of upload
  uint8_t _uploadState; 

public:
  DroneFS(DroneSystem* ds);

  void setup();

  uint8_t getNextId();
  DroneFSEntry* getEntryByPath(char* path);
  DroneFSEntry* getEntryByIndex(char* path, uint8_t index);
  DroneFSEntry* getEntryById(uint8_t id);

  boolean deleteEntryByPath(char* path);

  boolean startUpload(char* path, uint32_t size);
  uint8_t getUploadState();
  boolean writeUploadBlock(uint32_t offset, uint8_t* data, uint8_t size);
  boolean saveUpload();  // save to disk
  void cancelUpload();

  void enumerate();
};

#endif

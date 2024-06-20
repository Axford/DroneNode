
#include "DroneFS.h"
#include "DroneSystem.h"
#include <ArduinoLog.h>

//--------------------------------------------------------
// DroneFSEntry
//--------------------------------------------------------

DroneFSEntry::DroneFSEntry(DroneFS* fs, DroneFSEntry* parent, const char* name, boolean isDir) {
  _fs = fs;
  _parent = parent;

  // init name
  _name[0] = '/';
  for (uint8_t i=1; i<DRONE_FS_MAX_NAME_SIZE; i++) _name[i] = 0;
  setName(name);

  _size = 0;
  _id = _fs->getNextId();

  isDirectory(isDir);
  _enumerated = false;
}

void DroneFSEntry::close() {
  if (_file) {
    _file.close();
  }
}

DroneFSEntry* DroneFSEntry::getParent() {
  return _parent;
}

void DroneFSEntry::setName(const char* name) {
  strncpy(_name, name, DRONE_FS_MAX_NAME_SIZE-1);
}

char* DroneFSEntry::getName() {
  return _name;
}

void DroneFSEntry::setId(uint8_t id) {
  _id = id;
}

uint8_t DroneFSEntry::getId() {
  return _id;
}

uint32_t DroneFSEntry::getSize() {
  return _isDir ? _children.size() : _size;
}

void DroneFSEntry::setSize(uint32_t size) {
  _size = size;
}

/*
boolean DroneFSEntry::setSize(uint32_t size) {
  if (_isDir) return false;

  if (_file) {
    _file.close();
  }

  char tpath[DRONE_FS_MAX_PATH_SIZE];
  getPath((char*)&tpath, DRONE_FS_MAX_PATH_SIZE);

  // need to delete ready for writeBlock to save new file
  Log.noticeln("Removing prev file");
  LITTLEFS.remove(tpath);

  _size = 0;
  return true;
}
*/

void DroneFSEntry::getPath(char * path, uint8_t maxLen) {
  uint8_t len = 0;

  // get path to this entry (will inc trailing /)
  
  if (_parent) {
    _parent->getPath(path, maxLen);
    len  = strlen(path);
  }

  // add our name to the path
  strncpy(&path[len], _name, maxLen - len);
  len += strlen(path);

  // if we are a directory, but not the root, add a trailing /
  if (_parent &&
      _isDir &&
      len < maxLen) {
    path[len] = '/';
    len++;
  }

  // add null
  path[len] = 0;
}

boolean DroneFSEntry::matchesPath(char* path) {
  char tpath[DRONE_FS_MAX_PATH_SIZE];
  getPath((char*)&tpath, DRONE_FS_MAX_PATH_SIZE);
  if (strlen(path) != strlen(tpath)) return false;
  return strcmp(path, tpath) == 0;
}

void DroneFSEntry::isDirectory(boolean isDir) {
  _isDir = isDir;
}

boolean DroneFSEntry::isDirectory() {
  return _isDir;
}

boolean DroneFSEntry::enumerate() {
  if (!_isDir) return false;

  char path[DRONE_FS_MAX_PATH_SIZE];
  getPath(path, DRONE_FS_MAX_PATH_SIZE-1);

  Log.noticeln(path);

  File dir = LITTLEFS.open(path, FILE_READ);
  if (!dir) {
    Log.errorln(F("  can't open path"));
    return false;
  }

  if (!dir.isDirectory()) {
    dir.close();
    Log.errorln(F("  not a directory"));
    return false;
  }

  uint8_t i = 0;
  File file = dir.openNextFile();
  while(file){
    // do we need to create a new entry object?
    DroneFSEntry* entry = NULL;
    if (i < _children.size()) {
      entry = _children.get(i);
      // sync entry
      entry->setName(file.name());
      entry->isDirectory(file.isDirectory());
      entry->close(); // in case we have an open file handle
    } else {
      // create a new entry
      entry = new DroneFSEntry(_fs, this, file.name(), file.isDirectory());
      // add to children
      _children.add(entry);
    }
    // sync properties
    entry->readProperties(file);

    Log.noticeln(entry->getName());

    file.close();

    file = dir.openNextFile();
    i++;
  }
  if (file) file.close();
  dir.close();

  // remove excess children entries (e.g. after delete), should only ever be max one excess entry
  while (_children.size() > i) {
    DroneFSEntry* entry = _children.pop();
    delete entry;
  }

  _enumerated = true;
  return true;
}

void DroneFSEntry::readProperties(File f) {
  _size = f.size();
}

boolean DroneFSEntry::readBlock(uint32_t offset, uint8_t* buffer, uint8_t size) {
  if (!_file) {
    char tpath[DRONE_FS_MAX_PATH_SIZE];
    getPath((char*)&tpath, DRONE_FS_MAX_PATH_SIZE);

    _file = LITTLEFS.open(tpath, FILE_READ);
  }

  if (!_file) return false;

  if (_file.seek(offset, SeekSet)) {
    _file.read(buffer, size);
    return true;
  } else {
    return false;
  }
}


DroneFSEntry* DroneFSEntry::getEntryByPath(char* path) {
  if (matchesPath(path)) return this;

  DroneFSEntry* entry = NULL;

  if (_isDir) {
    for (uint8_t i=0; i<_children.size(); i++) {
      entry = _children.get(i)->getEntryByPath(path);
      if (entry) break;
    }
  }

  return entry;
}

DroneFSEntry* DroneFSEntry::getEntryByIndex(uint8_t index) {
  if (index < _children.size()) return _children.get(index);
  return NULL;
}

DroneFSEntry* DroneFSEntry::getEntryById(uint8_t id) {
  if (_id == id) return this;

  DroneFSEntry* entry = NULL;

  if (_isDir) {
    for (uint8_t i=0; i<_children.size(); i++) {
      entry = _children.get(i)->getEntryById(id);
      if (entry) break;
    }
  }

  return entry;
}

//--------------------------------------------------------
// DroneFS
//--------------------------------------------------------

DroneFS::DroneFS(DroneSystem* ds): _ds(ds) {
  char rootName[2] = "/";
  _nextId = 0;
  // create root entry
  _root = new DroneFSEntry(this, NULL, rootName, true);

  // detect filesystem
  // TODO


  _uploadBuffer = NULL;
  _uploadSize = 0;
  _uploadPath[0] = 0;
  _uploadState = DRONE_FS_UPLOAD_STATE_NONE;
}


void DroneFS::setup() {
  Log.noticeln(F("DroneFS setup..."));

  Log.noticeln(F("[] Mount filesystem..."));
  // passing true to .begin triggers a format if required
  if(!LITTLEFS.begin(true)){
    Log.errorln("[] LITTLEFS Mount Failed");
    _ds->dled->setState(DRONE_LED_STATE_ERROR);
    delay(3000);
    // Should be formatted and working after a reboot
    ESP.restart();
  }

  _root->enumerate(); // will not recurse

  Log.noticeln(F("  totalBytes %d"), LITTLEFS.totalBytes());
  Log.noticeln(F("  usedBytes %d"), LITTLEFS.usedBytes());
}

uint8_t DroneFS::getNextId() {
  return _nextId++;
}

DroneFSEntry* DroneFS::getEntryByPath(char* path) {
  return _root->getEntryByPath(path);
}

DroneFSEntry* DroneFS::getEntryByIndex(char* path, uint8_t index) {
  // get the specified directory
  DroneFSEntry* dir = _root->getEntryByPath(path);

  if (dir) {
    return dir->getEntryByIndex(index);
  } else{
    return NULL;
  }
}

DroneFSEntry* DroneFS::getEntryById(uint8_t id) {
  return _root->getEntryById(id);
}


boolean DroneFS::deleteEntryByPath(char* path) {
  DroneFSEntry* f = _root->getEntryByPath(path);
  if (f) {
    f->close();

    // delete file on disk
    Log.noticeln("Deleting file");
    LITTLEFS.remove(path);

    // re-enumerate parent directory
    f->getParent()->enumerate();

    return true;
  } else {
    return false;
  }
}


boolean DroneFS::startUpload(char* path, uint32_t size) {
  if (_uploadState > DRONE_FS_UPLOAD_STATE_NONE) return false;

  // alloc memory
  _uploadBuffer = (uint8_t*) malloc(size);

  // check malloc succeeded
  if (_uploadBuffer == NULL) return false;

  _uploadState = DRONE_FS_UPLOAD_STATE_WIP;
  _uploadSize = size;
  strcpy(_uploadPath, path);

  return true;
}


uint8_t DroneFS::getUploadState() {
  return _uploadState;
}


boolean DroneFS::writeUploadBlock(uint32_t offset, uint8_t* data, uint8_t size) {
  if (_uploadState == DRONE_FS_UPLOAD_STATE_NONE) {
    Log.errorln("Upload not initiated");
    return false;
  }

  // check size and offset are within range
  if (offset + size > _uploadSize) {
    Log.errorln("Out of range");
    return false;
  }

  // double check the uploadBuffer is not null
  if (_uploadBuffer == NULL) {
    Log.errorln("Buffer NULL");
    return false;
  }

  memcpy( &_uploadBuffer[offset], data, size );

  return true;
}


boolean DroneFS::saveUpload() {
  // check state
  if (_uploadState == DRONE_FS_UPLOAD_STATE_NONE) return false;

  char backupPath[] = "/backup.txt";

  DroneFSEntry* parent = _root;
  DroneFSEntry* f = getEntryByPath(_uploadPath);
  if (f) {
    parent = f->getParent();
    f->close();

    // rename old file as backup
    Log.noticeln("Renaming old as backup");
    LITTLEFS.rename(_uploadPath, backupPath);
  }

  // write to disk
  Log.noticeln("Writing new file to disk");
  File file = LITTLEFS.open(_uploadPath, FILE_WRITE);

  // return error if unable to allocate file... but retain new info in memory
  if (!file) {
    Log.noticeln("Unable to allocate file");
    return false;  
  }

  file.write(_uploadBuffer, _uploadSize);
  file.flush();
  file.close();

  if (LITTLEFS.exists(backupPath)) {
    Log.noticeln("Deleting backup");
    LITTLEFS.remove(backupPath);
  }

  // free upload buffer
  Log.noticeln("Free buffer");
  free(_uploadBuffer);
  _uploadSize = 0;
  _uploadState = DRONE_FS_UPLOAD_STATE_NONE;

  // enumerate file system from parent
  Log.noticeln("Enumerate");
  parent->enumerate();

  return true;
}


void DroneFS::cancelUpload() {
  if (_uploadState == DRONE_FS_UPLOAD_STATE_NONE) return;

  if (_uploadBuffer && _uploadSize > 0) {
    free(_uploadBuffer);
    _uploadSize = 0;
  }

  _uploadState = DRONE_FS_UPLOAD_STATE_NONE;

  Log.noticeln("Upload cancelled");  
}


void DroneFS::enumerate() {
  _root->enumerate();
}
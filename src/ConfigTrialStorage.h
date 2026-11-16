#pragma once

#include <FS.h>
#include "ConfigTrial.h"

namespace configTrial {

class LittleFsStore {
public:
  explicit LittleFsStore(fs::FS& filesystem) : fs(filesystem) {}

  Read load(Record& record);
  bool save(const Record& record);
  bool digest(const char* path, Digest& result);
  bool stage(const uint8_t* bytes, size_t length, const Digest& expected);
  bool preserve(const Digest& expected);
  bool promote();
  bool publishBackup();

private:
  fs::FS& fs;

  fs::File scratch();
  bool publish(const char* name, const Digest& expected);
};

}

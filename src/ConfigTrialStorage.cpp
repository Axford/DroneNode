#include "ConfigTrialStorage.h"

#include <mbedtls/sha256.h>

namespace configTrial {
namespace {
constexpr const char* Directory = "/.repair";
constexpr const char* ScratchPath = "/.repair/write";
constexpr const char* RecordPath = "/.repair/state";

bool hash(const uint8_t* bytes, size_t length, Digest& result) {
  return mbedtls_sha256_ret(bytes, length, result.bytes, 0) == 0;
}
}

Read LittleFsStore::load(Record& record) {
  if (!fs.exists(RecordPath)) return Read::Missing;
  auto file = fs.open(RecordPath, FILE_READ);
  if (!file || file.size() != sizeof record + sizeof(Digest) ||
      file.read(reinterpret_cast<uint8_t*>(&record), sizeof record) != sizeof record) return Read::Failed;
  Digest expected, actual;
  return file.read(expected.bytes, sizeof expected.bytes) == sizeof expected.bytes &&
         hash(reinterpret_cast<const uint8_t*>(&record), sizeof record, actual) &&
         actual == expected ? Read::Found : Read::Failed;
}

fs::File LittleFsStore::scratch() {
  if (!fs.exists(Directory) && !fs.mkdir(Directory)) return {};
  return fs.open(ScratchPath, FILE_WRITE);
}

bool LittleFsStore::save(const Record& record) {
  const auto* bytes = reinterpret_cast<const uint8_t*>(&record);
  Digest checksum;
  auto file = scratch();
  if (!file || !hash(bytes, sizeof record, checksum) ||
      file.write(bytes, sizeof record) != sizeof record ||
      file.write(checksum.bytes, sizeof checksum.bytes) != sizeof checksum.bytes) return false;
  file.close();
  auto staged = fs.open(ScratchPath, FILE_READ);
  uint8_t readback[sizeof record];
  Digest verify;
  const bool valid = staged && staged.size() == sizeof record + sizeof(Digest) &&
                     staged.read(readback, sizeof readback) == sizeof readback &&
                     staged.read(verify.bytes, sizeof verify.bytes) == sizeof verify.bytes &&
                     memcmp(readback, bytes, sizeof readback) == 0 && verify == checksum;
  staged.close();
  return valid && fs.rename(ScratchPath, RecordPath);
}

bool LittleFsStore::digest(const char* path, Digest& result) {
  auto file = fs.open(path, FILE_READ);
  if (!file || file.isDirectory() || !file.size() || file.size() > MaxConfigBytes) return false;
  mbedtls_sha256_context context;
  mbedtls_sha256_init(&context);
  bool valid = mbedtls_sha256_starts_ret(&context, 0) == 0;
  uint8_t buffer[256];
  size_t remaining = file.size();
  while (valid && remaining) {
    const size_t chunk = remaining < sizeof buffer ? remaining : sizeof buffer;
    valid = file.read(buffer, chunk) == chunk && mbedtls_sha256_update_ret(&context, buffer, chunk) == 0;
    remaining -= chunk;
  }
  valid = valid && mbedtls_sha256_finish_ret(&context, result.bytes) == 0;
  mbedtls_sha256_free(&context);
  return valid;
}

bool LittleFsStore::publish(const char* path, const Digest& expected) {
  Digest actual;
  return digest(ScratchPath, actual) && actual == expected && fs.rename(ScratchPath, path);
}

bool LittleFsStore::stage(const uint8_t* bytes, size_t length, const Digest& expected) {
  auto file = scratch();
  if (!file || file.write(bytes, length) != length) return false;
  file.close();
  return publish(CandidatePath, expected);
}

bool LittleFsStore::preserve(const Digest& expected) {
  auto input = fs.open(InstalledPath, FILE_READ);
  if (!input || input.isDirectory() || !input.size() || input.size() > MaxConfigBytes) return false;
  auto output = scratch();
  if (!output) return false;
  uint8_t buffer[256];
  size_t remaining = input.size();
  while (remaining) {
    const size_t chunk = remaining < sizeof buffer ? remaining : sizeof buffer;
    if (input.read(buffer, chunk) != chunk || output.write(buffer, chunk) != chunk) return false;
    remaining -= chunk;
  }
  output.close();
  input.close();
  return publish(PendingBackupPath, expected);
}

bool LittleFsStore::promote() {
  return fs.rename(CandidatePath, InstalledPath);
}

bool LittleFsStore::publishBackup() {
  return fs.rename(PendingBackupPath, PreviousPath);
}
}

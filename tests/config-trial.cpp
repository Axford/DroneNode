#include "ConfigTrial.h"

#include <cassert>
#include <map>
#include <string>

using namespace configTrial;

Digest digest(uint8_t value) {
  Digest result;
  result.bytes[0] = value;
  return result;
}

const auto installed = digest(1);
const auto candidate = digest(2);
const auto another = digest(3);
const auto precedingBackup = digest(4);
const uint8_t replacement[] = {2};

struct Store {
  std::map<std::string, Digest> files = {{InstalledPath, installed}, {PreviousPath, precedingBackup}};
  Record journal;
  Read state = Read::Missing;
  bool writeFails = false;
  Phase failSave = Phase::Idle;
  bool failStage = false;
  bool failPreserve = false;
  bool failPromote = false;
  bool failBackup = false;
  int promotions = 0;

  Read load(Record& record) {
    if (state == Read::Found) record = journal;
    return state;
  }

  bool save(const Record& record) {
    if (writeFails && record.phase == failSave) return false;
    journal = record;
    state = Read::Found;
    return true;
  }

  bool digest(const char* path, Digest& result) {
    const auto it = files.find(path);
    if (it == files.end()) return false;
    result = it->second;
    return true;
  }

  bool stage(const uint8_t* bytes, size_t, const Digest& expected) {
    if (failStage || !(::digest(bytes[0]) == expected)) return false;
    files[CandidatePath] = expected;
    return true;
  }

  bool preserve(const Digest& expected) {
    if (failPreserve) return false;
    files[PendingBackupPath] = files.at(InstalledPath);
    return files.at(PendingBackupPath) == expected;
  }

  bool promote() {
    if (failPromote) return false;
    files[InstalledPath] = files.at(CandidatePath);
    files.erase(CandidatePath);
    ++promotions;
    return true;
  }

  bool publishBackup() {
    if (failBackup) return false;
    files[PreviousPath] = files.at(PendingBackupPath);
    files.erase(PendingBackupPath);
    return true;
  }
};

void stageAndArm(Store& store) {
  Transaction<Store> repair(store);
  assert(repair.stage(replacement, sizeof replacement, installed, candidate));
  assert(repair.arm(installed, candidate));
}

int main() {
  {
    Store store;
    Transaction<Store> repair(store);
    assert(repair.boot(false) == Boot::Installed);
    assert(!repair.arm(installed, candidate));
    assert(!repair.stage(replacement, sizeof replacement, another, candidate));
    assert(!repair.stage(replacement, sizeof replacement, installed, another));
    assert(!repair.stage(replacement, 0, installed, candidate));
    assert(!repair.stage(replacement, MaxConfigBytes + 1, installed, candidate));
    const uint8_t invalid[] = {2, 0};
    assert(!repair.stage(invalid, sizeof invalid, installed, candidate));
    store.failStage = true;
    assert(!repair.stage(replacement, sizeof replacement, installed, candidate));
    assert(store.files.count(CandidatePath) == 0);
    store.failStage = false;
    assert(repair.stage(replacement, sizeof replacement, installed, candidate));
    assert(repair.boot(false) == Boot::Installed); // Staging alone cannot change the boot.
    assert(store.files.at(InstalledPath) == installed);
    store.files[InstalledPath] = another;
    assert(!repair.arm(installed, candidate));
  }
  {
    Store store;
    stageAndArm(store);
    Transaction<Store> afterRestart(store);
    assert(!afterRestart.arm(installed, candidate));
    assert(afterRestart.boot(true) == Boot::Candidate);
    assert(store.files.at(InstalledPath) == installed);
    assert(!afterRestart.stage(replacement, sizeof replacement, installed, candidate));
    assert(!afterRestart.commit(another));
    assert(afterRestart.commit(candidate));
    assert(store.files.at(InstalledPath) == candidate);
    assert(store.files.at(PreviousPath) == installed);
    assert(store.files.count(CandidatePath) == 0);
    assert(!afterRestart.commit(candidate));
    assert(Transaction<Store>(store).boot(false) == Boot::Installed);
    assert(store.promotions == 1);
  }
  for (bool trialStarted : {false, true}) {
    Store store;
    stageAndArm(store);
    if (trialStarted) assert(Transaction<Store>(store).boot(true) == Boot::Candidate);
    assert(Transaction<Store>(store).boot(false) == Boot::Recovery);
    assert(Transaction<Store>(store).boot(true) == Boot::Recovery);
    assert(store.files.at(InstalledPath) == installed);
    assert(store.promotions == 0);
    // A deliberate new arm can try a different candidate in recovery.
    Transaction<Store> repair(store);
    const uint8_t newConfig[] = {3};
    assert(repair.stage(newConfig, sizeof newConfig, installed, another));
    assert(repair.arm(installed, another));
    assert(repair.boot(true) == Boot::Candidate);
  }
  {
    Store store;
    stageAndArm(store);
    store.files[CandidatePath] = another; // Local edit after arming.
    assert(Transaction<Store>(store).boot(true) == Boot::Recovery);
    assert(store.files.at(InstalledPath) == installed);
  }
  {
    Store store;
    stageAndArm(store);
    store.writeFails = true;
    store.failSave = Phase::Running;
    assert(Transaction<Store>(store).boot(true) == Boot::Recovery);
    assert(store.files.at(InstalledPath) == installed);
  }
  for (const auto failure : {Phase::Running, Phase::Approved, Phase::Committed, Phase::Rejected}) {
    Store store;
    stageAndArm(store);
    Transaction<Store> repair(store);
    assert(repair.boot(true) == Boot::Candidate);
    if (failure == Phase::Running) {
      store.failPreserve = true;
    } else if (failure == Phase::Rejected) {
      store.failPromote = true;
    } else {
      store.writeFails = true;
      store.failSave = failure;
    }
    assert(!repair.commit(candidate));
    store.writeFails = false;
    if (failure == Phase::Committed) {
      assert(Transaction<Store>(store).boot(false) == Boot::Installed);
      assert(store.files.at(InstalledPath) == candidate);
      assert(store.files.at(PreviousPath) == installed);
      assert(store.promotions == 1);
    } else {
      assert(Transaction<Store>(store).boot(false) == Boot::Recovery);
      assert(store.files.at(InstalledPath) == installed);
      assert(store.files.at(PreviousPath) == precedingBackup);
      assert(store.promotions == 0);
    }
  }
  {
    Store store;
    stageAndArm(store);
    Transaction<Store> repair(store);
    assert(repair.boot(true) == Boot::Candidate);
    store.failBackup = true;
    assert(!repair.commit(candidate));
    assert(Transaction<Store>(store).boot(false) == Boot::Recovery);
    assert(store.files.at(PreviousPath) == precedingBackup);
    store.failBackup = false;
    assert(Transaction<Store>(store).boot(false) == Boot::Installed);
    assert(store.files.at(PreviousPath) == installed);
    assert(store.promotions == 1);
  }
  {
    Store store;
    stageAndArm(store);
    Transaction<Store> repair(store);
    assert(repair.boot(true) == Boot::Candidate);
    repair.reject();
    assert(!repair.commit(candidate));
    assert(Transaction<Store>(store).boot(true) == Boot::Recovery);
    assert(store.files.at(InstalledPath) == installed);
    assert(!repair.retryInstalled(candidate));
    assert(repair.retryInstalled(installed));
    assert(Transaction<Store>(store).boot(true) == Boot::Installed);
  }
  {
    Store store;
    store.state = Read::Failed;
    assert(Transaction<Store>(store).boot(true) == Boot::Recovery);
    assert(!Transaction<Store>(store).stage(replacement, sizeof replacement, installed, candidate));
  }
}

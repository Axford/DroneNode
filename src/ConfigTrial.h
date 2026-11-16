#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace configTrial {

constexpr const char* InstalledPath = "/config.ini";
constexpr const char* CandidatePath = "/trial.ini";
constexpr const char* PendingBackupPath = "/.repair/pending-backup.ini";
constexpr const char* PreviousPath = "/.repair/previous.ini";
constexpr size_t MaxConfigBytes = 8192;

struct Digest {
  uint8_t bytes[32] = {};

  bool operator==(const Digest& other) const {
    return memcmp(bytes, other.bytes, sizeof bytes) == 0;
  }
};

enum class Phase : uint8_t { Idle, Armed, Running, Approved, Rejected, Committed };
enum class Boot { Installed, Candidate, Recovery };

struct Record {
  static constexpr uint8_t Version = 1;
  uint8_t version = Version;
  Phase phase = Phase::Idle;
  Digest installed;
  Digest candidate;
};

static_assert(sizeof(Record) == 66, "Repair journal layout must remain stable");

enum class Read { Missing, Found, Failed };

// All mutations run on the foreground. Store replacements must be atomic and
// preserve the destination when they fail. A successful save survives power loss.
template<typename Store>
class Transaction {
public:
  explicit Transaction(Store& store) : storage(store) {}

  bool stage(const uint8_t* bytes, size_t length, const Digest& expected, const Digest& candidate) {
    Record current;
    return bytes && length && length <= MaxConfigBytes && !memchr(bytes, 0, length) &&
           read(current) && canArm(current.phase) && matches(InstalledPath, expected) &&
           storage.stage(bytes, length, candidate) && matches(CandidatePath, candidate);
  }

  bool arm(const Digest& installed, const Digest& candidate) {
    Record current;
    if (!read(current) || !canArm(current.phase) || installed == candidate ||
        !matches(InstalledPath, installed) || !matches(CandidatePath, candidate)) return false;
    current.installed = installed;
    current.candidate = candidate;
    current.phase = Phase::Armed;
    return storage.save(current);
  }

  // Pass true only for the retained explicit activation restart.
  Boot boot(bool activationRestart) {
    Record current;
    if (!read(current)) return Boot::Recovery;
    switch (current.phase) {
      case Phase::Idle:
      case Phase::Committed:
        return Boot::Installed;
      case Phase::Armed:
        if (activationRestart && matches(InstalledPath, current.installed) &&
            matches(CandidatePath, current.candidate) && advance(current, Phase::Running)) {
          return Boot::Candidate;
        }
        break;
      case Phase::Approved:
        // No boot may promote trial.ini. If the rename already succeeded,
        // reconcile the backup and journal for the approved configuration.
        if (matches(InstalledPath, current.candidate)) {
          return finalize(current) ? Boot::Installed : Boot::Recovery;
        }
        break;
      case Phase::Running:
        break;
      case Phase::Rejected:
        return Boot::Recovery;
    }
    advance(current, Phase::Rejected);
    return Boot::Recovery;
  }

  // The caller has verified local probation and the authenticated acknowledgement.
  bool commit(const Digest& acknowledged) {
    Record current;
    if (!read(current) || current.phase != Phase::Running || !(current.candidate == acknowledged) ||
        !matches(InstalledPath, current.installed) || !matches(CandidatePath, current.candidate) ||
        !storage.preserve(current.installed) || !matches(PendingBackupPath, current.installed) ||
        !advance(current, Phase::Approved)) return false;

    // Recovery can finish the journal if power is lost after promotion. Until
    // promotion succeeds, config.ini still contains the previous exact bytes.
    return storage.promote() && matches(InstalledPath, current.candidate) && finalize(current);
  }

  // Recovery persists across a cold boot until another explicit retry is accepted.
  bool retryInstalled(const Digest& expected) {
    Record current;
    return read(current) && current.phase == Phase::Rejected && matches(InstalledPath, expected) &&
           advance(current, Phase::Idle);
  }

  // Called for a setup/probation error or timed-out acknowledgement in this boot.
  // A failed write leaves Running in the journal, which also forces recovery.
  void reject() {
    Record current;
    if (read(current) && current.phase == Phase::Running) advance(current, Phase::Rejected);
  }

private:
  Store& storage;

  static bool canArm(Phase phase) {
    return phase == Phase::Idle || phase == Phase::Rejected || phase == Phase::Committed;
  }

  bool read(Record& current) {
    const auto result = storage.load(current);
    return result == Read::Missing || (result == Read::Found && current.version == Record::Version &&
           static_cast<uint8_t>(current.phase) <= static_cast<uint8_t>(Phase::Committed));
  }

  bool matches(const char* path, const Digest& expected) {
    Digest actual;
    return storage.digest(path, actual) && actual == expected;
  }

  bool finalize(Record& current) {
    if (!matches(PreviousPath, current.installed) &&
        !(matches(PendingBackupPath, current.installed) && storage.publishBackup() &&
          matches(PreviousPath, current.installed))) return false;
    return advance(current, Phase::Committed);
  }

  bool advance(Record& current, Phase next) {
    current.phase = next;
    return storage.save(current);
  }
};

}

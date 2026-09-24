#pragma once
#include <string>
namespace opennav::integration {
struct RecoveryRecord {
  unsigned failed_starts = 0;
  bool pending = false;
};
RecoveryRecord DecodeRecovery(const std::string &record);
std::string EncodeRecovery(const RecoveryRecord &record);
// Called once by the next process. An uncompleted XNav startup is one failure.
RecoveryRecord ObservePreviousStart(RecoveryRecord previous);
bool RecoveryRequired(const RecoveryRecord &record);
} // namespace opennav::integration

#include "integration/StartupRecovery.h"
#include <algorithm>
#include <sstream>
#include <stdexcept>
namespace opennav::integration {
RecoveryRecord DecodeRecovery(const std::string &text) {
  if (text.size() > 128)
    throw std::invalid_argument("Oversized startup record");
  std::istringstream in(text);
  std::string header, failed, pending;
  int count = -1, open = -1;
  std::getline(in, header);
  if (header != "OpenNavXRecovery 1" ||
      !(in >> failed >> count >> pending >> open) || !(in >> std::ws).eof() ||
      failed != "failures" || pending != "pending" || count < 0 || count > 2 ||
      (open != 0 && open != 1))
    throw std::invalid_argument("Invalid startup recovery record");
  return {static_cast<unsigned>(count), open == 1};
}
std::string EncodeRecovery(const RecoveryRecord &r) {
  if (r.failed_starts > 2)
    throw std::invalid_argument("Invalid failure count");
  return "OpenNavXRecovery 1\nfailures " + std::to_string(r.failed_starts) +
         "\npending " + (r.pending ? "1\n" : "0\n");
}
RecoveryRecord ObservePreviousStart(RecoveryRecord r) {
  if (r.failed_starts > 2)
    throw std::invalid_argument("Invalid failure count");
  if (r.pending)
    r.failed_starts = std::min(2u, r.failed_starts + 1);
  r.pending = false;
  return r;
}
bool RecoveryRequired(const RecoveryRecord &r) { return r.failed_starts >= 2; }
} // namespace opennav::integration

#include "integration/StartupRecovery.h"
#include <iostream>
#include <stdexcept>
using namespace opennav::integration;
void Check(bool ok, const char *why) {
  if (!ok)
    throw std::runtime_error(why);
}
int main() {
  try {
    RecoveryRecord record;
    Check(!RecoveryRequired(record), "Clean state permits XNav");
    record.pending = true;
    record = ObservePreviousStart(DecodeRecovery(EncodeRecovery(record)));
    Check(record.failed_starts == 1 && !record.pending &&
              !RecoveryRequired(record),
          "First failed startup counted once");
    auto observed = ObservePreviousStart(record);
    Check(observed.failed_starts == 1,
          "Reading completed accounting does not count failure again");
    record.pending = true;
    record = ObservePreviousStart(record);
    Check(RecoveryRequired(record) && record.failed_starts == 2,
          "Two unfinished startups require recovery");
    record.pending = true;
    record = ObservePreviousStart(record);
    Check(record.failed_starts == 2, "Counter saturates");
    Check(!RecoveryRequired(DecodeRecovery(EncodeRecovery({}))),
          "Explicit retry/healthy close reset");
    for (const auto &bad :
         {std::string{}, std::string(129, 'x'),
          std::string("OpenNavXRecovery 2\nfailures 0\npending 1\n"),
          std::string("OpenNavXRecovery 1\nfailures -1\npending 0\n"),
          std::string("OpenNavXRecovery 1\nfailures 0\npending 2\n"),
          EncodeRecovery({}) + "extra"}) {
      bool caught = false;
      try {
        DecodeRecovery(bad);
      } catch (const std::invalid_argument &) {
        caught = true;
      }
      Check(caught, "Malformed state rejected");
    }
    std::cout << "Startup recovery contract passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}

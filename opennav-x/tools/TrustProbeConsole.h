#pragma once

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <wx/debug.h>
#include <wx/init.h>
#include <wx/log.h>

namespace opennav {
// Standalone probes have no application log target. Keep wx diagnostics visible
// on redirected stderr and make assertions fail the probe instead of awaiting UI.
class TrustProbeConsole {
 public:
  TrustProbeConsole()
      : previous_assert_(wxSetAssertHandler(&FailAssertion)),
        logger_(new wxLogStderr(stderr)),
        previous_log_(wxLog::SetActiveTarget(logger_.get())),
        initializer_(new wxInitializer) {}
  ~TrustProbeConsole() {
    wxLog::FlushActive();
    wxLog::SetActiveTarget(previous_log_);
    logger_.reset();
    initializer_.reset();
    wxSetAssertHandler(previous_assert_);
  }
  TrustProbeConsole(const TrustProbeConsole&) = delete;
  TrustProbeConsole& operator=(const TrustProbeConsole&) = delete;
  bool IsOk() const { return initializer_->IsOk(); }
  static void Stage(const char* stage) {
    std::fprintf(stderr, "probe_stage=%s\n", stage);
    std::fflush(stderr);
  }

 private:
  static void FailAssertion(const wxString& file, int line,
                            const wxString& function, const wxString& condition,
                            const wxString& message) {
    std::fprintf(stderr, "probe_assertion_failed line=%d\n", line);
    std::fprintf(stderr, "%s | %s | %s | %s\n", file.utf8_str().data(),
                 function.utf8_str().data(), condition.utf8_str().data(),
                 message.utf8_str().data());
    std::fflush(stderr);
    std::_Exit(86);
  }
  wxAssertHandler_t previous_assert_;
  std::unique_ptr<wxLogStderr> logger_;
  wxLog* previous_log_;
  std::unique_ptr<wxInitializer> initializer_;
};
}  // namespace opennav

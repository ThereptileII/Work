#include "integration/UpdateStartupReceipt.h"

#include <iostream>
#include <stdexcept>

using opennav::integration::UpdateStartupReceiptState;
using namespace std::chrono_literals;
namespace {
void Require(bool passed, const char* message) {
  if (!passed) throw std::runtime_error(message);
}
const std::string pipe = "Skager.Update." + std::string(32, 'a');
const std::string generation(32, 'b');
const std::string challenge(64, 'c');
const std::string compiled_commit(40, 'd');
const UpdateStartupReceiptState::Time start{100s};

void Ready(UpdateStartupReceiptState& state) {
  state.ObserveReady(true, start);
  state.ObserveReady(true, start + 30s);
  state.RecoveryCheckpointReached();
}
}  // namespace

int main() {
  try {
    UpdateStartupReceiptState absent;
    Ready(absent);
    Require(!absent.TakeReady(), "No environment request produces no receipt");

    for (int bad = 0; bad < 12; ++bad) {
      auto p = pipe, g = generation, c = challenge, commit = compiled_commit;
      switch (bad) {
        case 0: p.clear(); break;
        case 1: p = "\\\\remote\\pipe\\" + pipe; break;
        case 2: p += "\\other"; break;
        case 3: p[0] = 's'; break;
        case 4: p.back() = 'A'; break;
        case 5: g.pop_back(); break;
        case 6: g.back() = '\n'; break;
        case 7: c.clear(); break;
        case 8: c += '0'; break;
        case 9: c[0] = 'G'; break;
        case 10: commit = "local development"; break;
        case 11: commit[0] = 'D'; break;
      }
      UpdateStartupReceiptState state;
      state.Capture(p, g, c, commit);
      // Even a rejected initial environment cannot be replaced by a later
      // plugin or child injecting a seemingly valid request into this process.
      state.Capture(pipe, generation, challenge, compiled_commit);
      Ready(state);
      Require(!state.TakeReady(), "Invalid request must never become a receipt");
    }

    UpdateStartupReceiptState state;
    state.Capture(pipe, generation, challenge, compiled_commit);
    state.Capture(pipe, generation, challenge, std::string(40, 'e'));
    state.ObserveReady(true, start);
    state.ObserveReady(true, start + 30s);
    Require(!state.TakeReady(), "UI readiness alone cannot replace durable health checkpoint");
    state.ObserveReady(false, start + 31s);
    state.RecoveryCheckpointReached();
    Require(!state.TakeReady(), "Successful journal checkpoint cannot override lost readiness");
    state.ObserveReady(true, start + 32s);
    state.ObserveReady(true, start + 61s);
    Require(!state.TakeReady(), "Receipt needs thirty continuous seconds after interruption");
    state.ObserveReady(true, start + 62s);
    const auto receipt = state.TakeReady();
    Require(receipt && receipt->pipe == pipe &&
                receipt->message == "SKAGER-UPDATE-READY/1 " + generation + " " +
                    compiled_commit + " " + challenge + "\n",
            "Wire receipt binds exact captured generation, compiled commit and challenge");
    Require(receipt->message.size() < 256, "Wire receipt fits receiver's bounded buffer");
    Require(!state.TakeReady(), "Repeated ticks cannot send another receipt");
    state.Capture(pipe, generation, challenge, compiled_commit);
    Ready(state);
    Require(!state.TakeReady(), "Recapture cannot retry a completed send");

    UpdateStartupReceiptState clock;
    clock.Capture(pipe, generation, challenge, compiled_commit);
    clock.RecoveryCheckpointReached();
    clock.ObserveReady(true, start);
    clock.ObserveReady(true, start + 29s);
    clock.ObserveReady(true, start - 1s);
    clock.ObserveReady(true, start + 28s);
    Require(!clock.TakeReady(), "Backward observation restarts continuous readiness clock");
    clock.ObserveReady(true, start + 29s);
    Require(bool(clock.TakeReady()), "Clock recovery still requires full thirty seconds");

    UpdateStartupReceiptState safe;
    safe.Capture(pipe, generation, challenge, compiled_commit);
    safe.RecoveryCheckpointReached();
    safe.ObserveReady(false, start);
    safe.ObserveReady(false, start + 1h);
    Require(!safe.TakeReady(), "Legacy/Safe or unready UI cannot emit health receipt");
    UpdateStartupReceiptState modal;
    modal.Capture(pipe, generation, challenge, compiled_commit);
    Ready(modal);  // A nested event loop may have previous observations.
    auto wait = modal.NavigationWarning(true, false);
    Require(wait && !wait->terminal && wait->message ==
        "SKAGER-UPDATE-WAIT/1 " + generation + " " + compiled_commit + " " + challenge + "\n",
        "Actual modal entry emits authenticated waiting identity");
    Ready(modal);
    Require(!modal.TakeReady(), "Nested modal events cannot acknowledge readiness");
    auto resume = modal.NavigationWarning(false, true);
    Require(resume && !resume->terminal && resume->message.find("SKAGER-UPDATE-CONTINUE/1 ") == 0,
        "Agree resumes the existing authenticated session");
    Require(!modal.TakeReady(), "Agree itself is not startup success");
    modal.ObserveReady(true, start + 2h);
    modal.ObserveReady(true, start + 2h + 30s);
    Require(!modal.TakeReady(), "Modal invalidates a previous durable checkpoint");
    modal.RecoveryCheckpointReached();
    Require(bool(modal.TakeReady()), "Post-modal full health can acknowledge startup");

    UpdateStartupReceiptState cancelled;
    cancelled.Capture(pipe, generation, challenge, compiled_commit);
    cancelled.NavigationWarning(true, false);
    const auto cancel = cancelled.NavigationWarning(false, false);
    Require(cancel && cancel->terminal && cancel->message.find("SKAGER-UPDATE-CANCEL/1 ") == 0,
        "Cancel reports a terminal cancellation");
    Ready(cancelled);
    Require(!cancelled.TakeReady(), "Cancelled startup cannot later become healthy");
    Require(!cancelled.NavigationWarning(false, true), "Cancel cannot be undone by a callback");

    for (int invalid = 0; invalid < 4; ++invalid) {
      UpdateStartupReceiptState rejected;
      rejected.Capture(pipe, generation, challenge, compiled_commit);
      if (invalid == 0) rejected.NavigationWarning(false, true);
      if (invalid == 1) rejected.NavigationWarning(true, true);
      if (invalid == 2) {
        rejected.NavigationWarning(true, false);
        rejected.NavigationWarning(true, false);
      }
      if (invalid == 3) {
        rejected.NavigationWarning(true, false);
        rejected.NavigationWarning(false, true);
        rejected.NavigationWarning(false, true);
      }
      Ready(rejected);
      Require(!rejected.TakeReady(), "Unexpected/repeated warning transition fails closed");
    }

    std::cout << "Startup receipt identity and continuous-health contract passed.\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}

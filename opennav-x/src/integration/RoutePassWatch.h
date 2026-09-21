#pragma once
#include <wx/event.h>
#include <wx/eventfilter.h>

namespace opennav::integration {
// Upstream progress can open dialogs/process nested events. A before/after
// geometry comparison alone cannot detect edit-then-restore (ABA) in a nested
// event. Reject any pass which dispatched an event, even if values match again.
// This filter never consumes or modifies an event.
class RoutePassWatch final : public wxEventFilter {
 public:
  RoutePassWatch() { wxEvtHandler::AddFilter(this); }
  ~RoutePassWatch() override { Finish(); }
  RoutePassWatch(const RoutePassWatch&) = delete;
  RoutePassWatch& operator=(const RoutePassWatch&) = delete;
  int FilterEvent(wxEvent&) override { interrupted_ = true; return Event_Skip; }
  bool Finish() {
    if (attached_) { wxEvtHandler::RemoveFilter(this); attached_ = false; }
    return interrupted_;
  }
 private:
  bool attached_ = true, interrupted_ = false;
};
}  // namespace opennav::integration

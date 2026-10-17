#pragma once

class wxWindow;

namespace opennav::ui {
// Presentation only, on the wx application thread. The caller owns first-run
// and version triggers, persistence, updater receipts and startup cancellation.
// Returns true only after the user explicitly activates Agree.
bool ShowStartupNavigationNotice(wxWindow* parent);
}  // namespace opennav::ui

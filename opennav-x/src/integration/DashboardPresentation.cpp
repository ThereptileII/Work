#include "integration/DashboardPresentation.h"
#include "integration/DashboardPresentationApi.h"
#include "integration/OpenCPNIntegration.h"
#include <algorithm>
#include <memory>

extern wxAuiManager* g_pauimgr;
namespace {
std::unique_ptr<opennav::integration::DashboardPresentation> presentation;
bool closed = false;
opennav::integration::DashboardPresentation* Current() {
  if (!opennav::IsXNav() || closed || !g_pauimgr) return nullptr;
  if (!presentation)
    presentation = std::make_unique<opennav::integration::DashboardPresentation>(*g_pauimgr);
  return presentation.get();
}
}
extern "C" void OpenNavDashboardWindow(wxWindow* window, bool add) {
  if (auto* state = Current()) {
    if (add) state->Register(window);
    else state->Unregister(window);
  }
}
extern "C" void OpenNavDashboardLayout(bool begin) {
  if (auto* state = Current()) {
    if (begin) state->BeginLayout();
    else state->EndLayout();
  }
}

namespace opennav::integration {
void EnableDashboardPresentation() { if (auto* state = Current()) state->Enable(true); }
void FinishDashboardPresentation() {
  closed = true;
  presentation.reset();  // Restore while the real manager and plugin live.
}
DashboardPresentation::~DashboardPresentation() { Enable(false); }
void DashboardPresentation::Prune() {
  panes_.erase(std::remove_if(panes_.begin(), panes_.end(),
      [](const Pane& pane) { return !pane.window; }), panes_.end());
}
void DashboardPresentation::Register(wxWindow* window) {
  Prune();
  if (!window || !manager_.GetPane(window).IsOk()) return;
  for (const auto& pane : panes_) if (pane.window.get() == window) return;
  panes_.emplace_back(window);
  if (enabled_ && depth_ == 0) Suppress();
}
void DashboardPresentation::Unregister(wxWindow* window) {
  panes_.erase(std::remove_if(panes_.begin(), panes_.end(),
      [window](const Pane& pane) { return !pane.window || pane.window.get() == window; }), panes_.end());
}
void DashboardPresentation::Restore() {
  Prune();
  for (auto& entry : panes_) {
    auto& pane = manager_.GetPane(entry.window.get());
    if (pane.IsOk() && entry.original) pane.SafeSet(*entry.original);
    entry.original.reset();
  }
}
void DashboardPresentation::Suppress() {
  Prune();
  bool changed = false;
  for (auto& entry : panes_) {
    auto& pane = manager_.GetPane(entry.window.get());
    if (!pane.IsOk()) continue;
    // Preserve the first unmodified state. Adding another pane must not learn
    // an earlier pane's temporary hidden state as the user's preference.
    if (!entry.original) entry.original = pane;
    changed = changed || pane.IsShown();
    pane.Hide();
  }
  if (changed) manager_.Update();
}
void DashboardPresentation::Enable(bool enabled) {
  if (enabled_ == enabled) return;
  enabled_ = enabled;
  if (enabled_ && depth_ == 0) Suppress();
  else if (!enabled_) { Restore(); manager_.Update(); }
}
void DashboardPresentation::BeginLayout() {
  if (depth_++ == 0 && enabled_) Restore();
}
void DashboardPresentation::EndLayout() {
  wxASSERT(depth_ > 0);
  if (depth_ && --depth_ == 0 && enabled_) Suppress();
}
}  // namespace opennav::integration

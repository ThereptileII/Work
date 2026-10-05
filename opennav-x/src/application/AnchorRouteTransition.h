#pragma once
#include "application/NavigationObjects.h"

namespace opennav::application {
enum class AnchorStartResult { Started, SaveFailed, StopFailed, RollbackFailed };
// Persist the new mark before stopping navigation; never arm a competing
// watch on failure. Ownership and actual database operations stay upstream.
inline AnchorStartResult CommitAnchorWatch(
    const std::function<bool()> &persist,
    const std::function<bool()> &stop_navigation,
    const std::function<bool()> &rollback,
    const std::function<void()> &arm) {
  if (!persist()) return AnchorStartResult::SaveFailed;
  if (!stop_navigation())
    return rollback() ? AnchorStartResult::StopFailed
                      : AnchorStartResult::RollbackFailed;
  arm();
  return AnchorStartResult::Started;
}

// Revalidate the complete watch set, including revisions, after confirmation.
// An added, removed, moved or repurposed mark requires a new human decision.
inline bool SameAnchorWatchSelection(const AnchorWatchSelection &expected,
                                     const AnchorWatchSelection &current) {
  if (!expected.available || !current.available ||
      expected.watches.size() != current.watches.size())
    return false;
  for (std::size_t i = 0; i < expected.watches.size(); ++i) {
    const auto &a = expected.watches[i], &b = current.watches[i];
    if (a.id.empty() || a.revision.empty() || a.id != b.id ||
        a.revision != b.revision)
      return false;
  }
  return true;
}

// The real integration validates route/GPS first, then uses this ordering:
// exact consent, persistence, mutation. Failed persistence cannot stop a watch.
inline CommandResult CommitRouteActivation(
    const AnchorWatchSelection &current, const AnchorWatchSelection *confirmed,
    const std::function<bool()> &persist,
    const std::function<CommandResult()> &activate) {
  if (!current.available)
    return {false, "Anchor watch state unavailable; refresh before activating"};
  if (confirmed) {
    if (!SameAnchorWatchSelection(*confirmed, current))
      return {false, "Anchor watch changed; review and confirm again"};
  } else if (!current.watches.empty())
    return {false, "Stop the anchor watch before activating this route"};
  if (!persist())
    return {false, "Route save failed; navigation and anchor watch unchanged"};
  return activate();
}

// Cancel returns no command result because no mutation is dispatched. The
// integration callback revalidates both owned selections after the modal UI.
inline std::optional<CommandResult> ConfirmRouteActivation(
    const Route &route, const NavigationActions &actions,
    const std::function<bool(bool stops_anchor)> &confirm) {
  if (!actions.anchor_watches)
    return CommandResult{false, "Anchor watch state unavailable; refresh before activating"};
  const auto selected = actions.anchor_watches();
  if (!selected.available)
    return CommandResult{false, selected.reason.empty()
                                   ? "Anchor watch state unavailable"
                                   : selected.reason};
  const bool stopping = !selected.watches.empty();
  if ((stopping && !actions.activate_after_anchor) ||
      (!stopping && !actions.activate))
    return CommandResult{false, "Route activation unavailable"};
  if (!confirm(stopping))
    return std::nullopt;
  return stopping ? actions.activate_after_anchor(route, selected)
                  : actions.activate(route);
}
} // namespace opennav::application

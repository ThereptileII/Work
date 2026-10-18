#pragma once
#include "application/NavigationObjects.h"
#include "ui/Drawer.h"
#include <cstdint>
#include <wx/stattext.h>

namespace opennav::ui {
// Fixed header footprint: the optional GPS warning uses a second text line.
// The inherited button retains touch, keyboard and accessibility behavior.
class XNavTrackRecordButton final : public XNavButton {
public:
  explicit XNavTrackRecordButton(wxWindow *);
  void Update(const application::TrackRecordingState &, LightMode);
private:
  void Paint(wxPaintEvent &);
  LightMode light_ = LightMode::Day;
  bool rendered_ = false, recording_ = false, pending_ = false, gps_warning_ = false;
  std::string reason_;
};

class XNavTrackDrawer final : public XNavDrawer {
public:
  XNavTrackDrawer(wxWindow &, application::NavigationActions);
  void Open(const wxRect &, const application::TrackRecordingState &, bool live, LightMode);
  void Update(const application::TrackRecordingState &, bool live, LightMode);
  std::function<void()> on_saved_tracks, on_changed;

private:
  void ChangeRecording(bool stop, const std::string &expected_id);
  void Reflow();
  application::NavigationActions actions_;
  application::TrackRecordingState state_;
  bool live_ = false;
  std::uint64_t command_generation_ = 0;
  wxString feedback_, status_copy_, detail_copy_, note_copy_;
  wxStaticText *status_ = nullptr, *details_ = nullptr, *notes_ = nullptr;
  XNavButton *record_ = nullptr, *saved_ = nullptr;
};
} // namespace opennav::ui

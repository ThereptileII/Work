#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

// Inert boundary for the actual extracted production event handlers. No chart,
// database, profile, plugin, display server or hardware is opened by this test.
#define WXUNUSED(name) /* unused */
constexpr int wxEVT_LEFT_UP = 1, wxEVT_RIGHT_DOWN = 2, WXK_MENU = 3;
constexpr int wxTIMER_ONE_SHOT = 1;
constexpr int wxID_YES = 10, wxID_NO = 11, wxID_CANCEL = 12;
constexpr int wxYES_NO = 1, wxCANCEL = 2, wxYES_DEFAULT = 4, wxNO_DEFAULT = 8;
constexpr int Undo_AppendWaypoint = 1, Undo_HasParent = 2;
#define _(text) wxString(text)
class wxString : public std::string {
 public:
  using std::string::string;
  wxString Left(int count) const { return wxString(c_str(), count); }
  static wxString Format(const char *, int, const wxString &) { return "M01-fixture"; }
};
struct RoutePoint {
  bool m_bIsInLayer = false, shared = false;
  wxString name = "Retained mark";
  bool IsVisible() const { return true; }
  const wxString &GetName() const { return name; }
  void SetName(const wxString &value) { name = value; }
  void SetShared(bool value) { shared = value; }
};
struct Route {
  int index = 2;
  RoutePoint *last = nullptr;
  int GetnPoints() const { return 3; }
  int GetIndexOf(RoutePoint *) const { return index; }
  RoutePoint *GetLastPoint() const { return last; }
  wxString GetGUID() const { return "fixture-guid"; }
};
struct Undo { void BeforeUndoableAction(int, RoutePoint *, int, void *) {} };
struct RouteManager {
  Route *tail = nullptr;
  Route *FindVisibleRouteContainingWaypoint(RoutePoint *) { return tail; }
};
RouteManager manager;
RouteManager *g_pRouteMan = &manager;
Route *FindRouteContainingWaypoint(RoutePoint *) { return nullptr; }
struct wxPoint { int x = 0, y = 0; };
struct wxRect { wxRect(int, int, int, int) {} };
struct wxFocusEvent {};
struct wxTimerEvent {};
struct wxLongPressEvent {};
struct wxMouseEvent {
  explicit wxMouseEvent(int value = 0) : type(value) {}
  int type, m_x = 0, m_y = 0;
  wxPoint GetPosition() const { return {m_x, m_y}; }
  void GetPosition(int *x, int *y) { *x = m_x; *y = m_y; }
  bool RightDown() const { return type == wxEVT_RIGHT_DOWN; }
};
struct Timer { void Start(int, int) {} };
bool g_btouch = false;
bool g_brouteCreating = false;
constexpr int ID_MENU_ROUTE_NEW = 1;
int pencil_cursor = 7;
int *pCursorPencil = &pencil_cursor;
namespace opennav {
enum class Mode { Legacy, Safe, XNav };
Mode mode = Mode::Legacy;
bool IsXNav() { return mode == Mode::XNav; }
}
struct MainToolbar {
  int calls = 0;
  std::function<void()> on_disable;
  void DisableTooltips() { ++calls; if (on_disable) on_disable(); }
};
MainToolbar *g_MainToolbar = nullptr;
struct Frame {
  int calls = 0;
  bool route_selected = false;
  void SetMasterToolbarItemState(int, bool selected) {
    ++calls;
    route_selected = selected;
  }
};
Frame frame;
Frame *gFrame = &frame;
int hidden_toolbars = 0;
void HideGlobalToolbar() { ++hidden_toolbars; }
class ChartCanvas;
int OCPNMessageBox(ChartCanvas *, const wxString &, const wxString &, long);
class ChartCanvas {
 public:
  bool m_FinishRouteOnKillFocus = false, m_leftdown = false, m_popupWanted = false;
  bool m_bChartDragging = false, m_bMeasure_Active = false;
  bool m_bDrawingRoute = false;
  int cursor = -1;
  int m_routeState = 2, m_focus_indicator_pix = 1, finishes = 0, popups = 0;
  int named_saves = 0;
  bool m_disable_edge_pan = false, all_dialogs_suppressed = true;
  int dialogs = 0, fail_dialog = 0, last_answer = wxID_NO;
  std::vector<int> answers;
  RoutePoint nearby, other_point;
  Route current_route, tail_route;
  Route *m_pMouseRoute = &current_route;
  RoutePoint *m_prev_pMousePoint = nullptr;
  Undo undo_owner;
  Undo *undo = &undo_owner;
  wxPoint last_drag;
  Timer m_routeFinishTimer;
  std::function<void()> popup_action;
  wxPoint GetClientSize() { return {100, 100}; }
  void RefreshRect(wxRect, bool) {}
  void SetFocus() {}
  void SetCursor(int value) { cursor = value; }
  void StartRoute();
  void FinishRoute() { ++finishes; m_routeState = 0; }
  void OnKillFocus(wxFocusEvent &);
  void OnRouteFinishTimerEvent(wxTimerEvent &);
  void OnLongPress(wxLongPressEvent &);
  void OnLeftUp(wxMouseEvent &);
  bool RightClick(wxMouseEvent &);
  void KeyboardMenu();
  void ReuseMouse();
  void ReuseTouch();
  void GreatCircleMouse();
  void GreatCircleTouch();
  void LoseFocus() {
    wxFocusEvent event;
    OnKillFocus(event);
#ifdef __WXOSX__
    wxTimerEvent timer;
    OnRouteFinishTimerEvent(timer);
#endif
  }
  void CallPopupMenu(int, int) {
    ++popups;
    LoseFocus();
    if (popup_action) popup_action();
  }
  void MouseEvent(wxMouseEvent &event) {
    if (event.RightDown()) RightClick(event);
  }
  bool SaveNamed() {
    // The actual finish_route_named entrypoint requires an active draft. A
    // native focus-finish must not consume it before this dispatch can run.
    if (!m_routeState) return false;
    ++named_saves;
    FinishRoute();
    return true;
  }
};
int OCPNMessageBox(ChartCanvas *canvas, const wxString &, const wxString &, long) {
  ++canvas->dialogs;
  canvas->all_dialogs_suppressed &= !canvas->m_FinishRouteOnKillFocus;
  canvas->LoseFocus();
  if (canvas->fail_dialog == canvas->dialogs)
    throw std::runtime_error("inert route dialog failure");
  return canvas->dialogs <= static_cast<int>(canvas->answers.size())
             ? canvas->answers[canvas->dialogs - 1] : wxID_NO;
}
#include "production-route-focus.h"

namespace {
int checks = 0;
void Check(bool value, const char *why) {
  ++checks;
  if (!value) throw std::runtime_error(why);
}
void Popup(ChartCanvas &canvas, int input) {
  if (input == 0) {
    wxMouseEvent event(wxEVT_RIGHT_DOWN);
    canvas.RightClick(event);
  } else if (input == 1) {
    canvas.KeyboardMenu();
  } else {
    g_btouch = true;
    wxLongPressEvent press;
    canvas.OnLongPress(press);
    wxMouseEvent release(wxEVT_LEFT_UP);
    canvas.OnLeftUp(release);
    g_btouch = false;
  }
}
}

int main() {
  try {
    // Exercise the entire production StartRoute, including the native caller
    // path. A second canvas must not change its policy when creation is busy.
    for (auto mode : {opennav::Mode::Legacy, opennav::Mode::Safe, opennav::Mode::XNav}) {
      opennav::mode = mode;
      for (bool prior : {false, true}) {
        for (bool busy : {false, true}) {
          for (bool with_toolbar : {false, true}) {
            ChartCanvas canvas;
            canvas.m_routeState = 0;
            canvas.m_bDrawingRoute = true;
            canvas.m_FinishRouteOnKillFocus = prior;
            g_brouteCreating = busy;
            frame = {};
            hidden_toolbars = 0;
            bool expected_policy = prior;
#ifdef OPENNAV_X
            if (!busy && mode == opennav::Mode::XNav) expected_policy = false;
#endif
            MainToolbar toolbar;
            toolbar.on_disable = [&] {
              Check(canvas.m_FinishRouteOnKillFocus == expected_policy,
                    "StartRoute establishes policy before the first toolbar callback");
            };
            g_MainToolbar = with_toolbar ? &toolbar : nullptr;
            canvas.StartRoute();
            Check(canvas.m_FinishRouteOnKillFocus == expected_policy,
                  "StartRoute preserves Legacy/Safe policy and disables XNav focus completion");
            if (busy) {
              Check(!canvas.m_routeState && canvas.m_bDrawingRoute && canvas.cursor == -1 &&
                        !toolbar.calls && !frame.calls && !hidden_toolbars,
                    "competing route start has no canvas, toolbar or focus-policy side effects");
            } else {
              Check(g_brouteCreating && canvas.m_routeState == 1 && !canvas.m_bDrawingRoute &&
                        canvas.cursor == pencil_cursor && toolbar.calls == int(with_toolbar) &&
                        frame.calls == 1 && frame.route_selected && hidden_toolbars == 1,
                    "accepted native route start retains normal draft and toolbar setup");
              canvas.LoseFocus();
              // A previously queued macOS timer must also respect the policy
              // established by StartRoute, independently of OnKillFocus's gate.
              wxTimerEvent timer;
              canvas.OnRouteFinishTimerEvent(timer);
              Check(expected_policy ? canvas.finishes == 1 && !canvas.m_routeState
                                    : !canvas.finishes && canvas.m_routeState == 1,
                    "new route focus/timer completion follows its actual mode policy");
            }
            g_MainToolbar = nullptr;
          }
        }
      }
    }
    g_brouteCreating = false;
    opennav::mode = opennav::Mode::Legacy;
    for (int input = 0; input != 3; ++input) {
      for (bool legacy : {false, true}) {
        ChartCanvas canvas;
        canvas.m_FinishRouteOnKillFocus = legacy;
        Popup(canvas, input);
        Check(canvas.popups == 1 && canvas.finishes == 0,
              "popup focus transfer must not auto-save the draft");
        Check(canvas.m_FinishRouteOnKillFocus == legacy,
              "native popup must restore the caller's explicit completion policy");
        canvas.LoseFocus();  // Focus transfer to XNav Done or Cancel.
        if (legacy) {
          Check(canvas.finishes == 1 && !canvas.m_routeState,
                "Legacy still finishes on subsequent canvas focus loss");
        } else {
          Check(canvas.finishes == 0 && canvas.m_routeState,
                "XNav Done must still have the editable draft");
          Check(canvas.SaveNamed() && canvas.named_saves == 1,
                "named save dispatch remains available exactly once");
        }
      }
      ChartCanvas canvas;
      canvas.popup_action = [] { throw std::runtime_error("inert popup failure"); };
      try { Popup(canvas, input); }
      catch (const std::runtime_error &) {}
      Check(!canvas.m_FinishRouteOnKillFocus && canvas.m_routeState,
            "exception must preserve XNav explicit completion and draft");
    }
    // Compile and run both actual nearby-waypoint scopes, including their
    // nested insert/append branches. Skipped dialogs must restore Legacy too.
    for (bool touch : {false, true}) {
      for (bool prior : {false, true}) {
        for (int scenario = 0; scenario != 5; ++scenario) {
          for (int answer : {wxID_YES, wxID_NO, wxID_CANCEL}) {
            ChartCanvas canvas;
            canvas.m_FinishRouteOnKillFocus = prior;
            canvas.m_routeState = scenario < 3 ? 1 : 2;
            canvas.tail_route.index = scenario == 2 ? 1 : 2;
            canvas.tail_route.last = scenario == 4 ? &canvas.nearby : &canvas.other_point;
            manager.tail = scenario ? &canvas.tail_route : nullptr;
            canvas.answers = {wxID_YES, answer};
            // No tail: exercise YES/NO/CANCEL on the first dialog itself.
            if (!scenario) canvas.answers = {answer};
            if (touch) canvas.ReuseTouch(); else canvas.ReuseMouse();
            int expected = scenario == 1 || scenario == 3 ? 2 : 1;
#ifdef __WXOSX__
            if (touch) --expected;  // Native macOS reuse skips the first prompt.
#endif
            Check(canvas.dialogs == expected && canvas.all_dialogs_suppressed,
                  "actual reuse/tail dialogs run only in their native branches with focus suppressed");
            Check(canvas.m_FinishRouteOnKillFocus == prior && !canvas.finishes,
                  "reuse/tail YES NO CANCEL and skipped dialogs preserve caller policy and draft");
            canvas.LoseFocus();
            Check(prior ? canvas.finishes == 1 : canvas.SaveNamed(),
                  "reuse/tail preserves Legacy auto-finish and XNav named dispatch");
          }
        }
        // Exceptions from either the first or nested real modal call unwind
        // both guards. The skipped macOS first prompt changes call numbering.
        for (int failure = 1; failure <= 2; ++failure) {
          ChartCanvas canvas;
          canvas.m_FinishRouteOnKillFocus = prior;
          canvas.m_routeState = 1;
          manager.tail = &canvas.tail_route;
          canvas.answers = {wxID_YES, wxID_YES};
          canvas.fail_dialog = failure;
          bool failed = false;
          try { if (touch) canvas.ReuseTouch(); else canvas.ReuseMouse(); }
          catch (const std::runtime_error &) { failed = true; }
          int available = 2;
#ifdef __WXOSX__
          if (touch) --available;
#endif
          Check(failed == (failure <= available), "exception injected at an actual available native modal call");
          Check(canvas.m_FinishRouteOnKillFocus == prior && !canvas.finishes,
                "actual reuse/tail exception restores original caller policy");
        }
        for (int answer : {wxID_YES, wxID_NO}) {
          ChartCanvas canvas;
          canvas.m_FinishRouteOnKillFocus = prior;
          canvas.answers = {answer};
          if (touch) canvas.GreatCircleTouch(); else canvas.GreatCircleMouse();
          int expected = 1;
#ifdef __WXOSX__
          if (touch) expected = 0;
#endif
          Check(canvas.dialogs == expected && canvas.all_dialogs_suppressed,
                "actual great-circle modal and native skipped branch preserve focus");
          Check(canvas.m_FinishRouteOnKillFocus == prior && !canvas.finishes,
                "great-circle YES or NO restores caller policy");
        }
        ChartCanvas canvas;
        canvas.m_FinishRouteOnKillFocus = prior;
        canvas.fail_dialog = 1;
        bool failed = false;
        try { if (touch) canvas.GreatCircleTouch(); else canvas.GreatCircleMouse(); }
        catch (const std::runtime_error &) { failed = true; }
        bool available = true;
#ifdef __WXOSX__
        if (touch) available = false;
#endif
        Check(failed == available && canvas.m_FinishRouteOnKillFocus == prior,
              "actual great-circle exception or skipped branch preserves caller policy");
      }
    }
#ifdef HAS_ROUTE_FOCUS_GUARD
    for (bool prior : {false, true}) {
      bool finish = prior;
      const auto early_return = [&] {
        RouteFinishFocusGuard outer(finish);
        Check(!finish, "modal suppresses native focus completion");
        {
          RouteFinishFocusGuard inner(finish);
          Check(!finish, "nested route-tail dialog remains suppressed");
        }
        Check(!finish, "inner dialog restores the outer suppression");
        return;
      };
      early_return();
      Check(finish == prior, "early return restores original completion policy");
      try {
        RouteFinishFocusGuard guard(finish);
        throw std::runtime_error("inert route dialog failure");
      } catch (const std::runtime_error &) {}
      Check(finish == prior, "dialog exception restores original completion policy");
    }
#endif
    std::cout << checks << " route-focus checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << "route-focus regression: " << error.what() << '\n';
    return 1;
  }
}

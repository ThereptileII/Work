#!/usr/bin/env python3
"""Execute the actual chart input methods with inert cursor/movement sinks.

The source must be prepared by prepare-integration.py. This checks input dispatch,
not wx event delivery, native cursor rendering, or an installed application.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "source_extract", ROOT / "tools/verify-anchor-loader.py")
EXTRACT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(EXTRACT)

FIXTURE = r"""
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

enum Region { CENTER, MID_RIGHT, MID_LEFT, MID_TOP, MID_BOT };
enum class Mode { XNav, Legacy, Safe };
namespace opennav {
Mode mode = Mode::XNav;
bool IsXNav() { return mode == Mode::XNav; }
}
struct wxPoint { int x = 0, y = 0; };
struct wxCursor { int id; };
wxCursor arrow{0}, right_cursor{1}, left_cursor{2}, down_cursor{3},
    up_cursor{4}, pencil{5}, plugin{6};
struct wxMouseEvent {
  enum Kind { Move, Down, Up, Drag } kind = Move;
  int x = 100, y = 120, wheel = 0;
  bool shift = false;
  bool ShiftDown() const { return shift; }
  void GetPosition(int* px, int* py) const { *px = x; *py = y; }
  int GetWheelRotation() const { return wheel; }
  bool LeftDown() const { return kind == Down; }
  bool LeftUp() const { return kind == Up; }
  bool Dragging() const { return kind == Drag; }
  bool LeftIsDown() const { return kind == Down || kind == Drag; }
};
struct Timer {
  int starts = 0;
  int Time() const { return 1000; }
  void Start(int = 0) { ++starts; }
};
bool g_btouch = false, g_bsmoothpanzoom = false, panleftIsDown = false;
double g_mouse_zoom_sensitivity = 2;
int canvas_count = 1;
int GetCanvasCount() { return canvas_count; }
class ChartCanvas;
ChartCanvas* g_focusCanvas = nullptr;
class ChartCanvas {
 public:
  double m_displayScale = 1;
  Region cursor_region = CENTER;
  bool m_bChartDragging = false, m_bMeasure_Active = false;
  int m_routeState = 0, m_nMeasureState = 0;
  int m_wheelzoom_stop_oneshot = 0, m_last_wheel_dir = 0;
  Timer m_wheelstopwatch, double_click_timer;
  Timer* m_DoubleClickTimer = &double_click_timer;
  bool singleClickEventIsValid = true;
  wxPoint last_drag, m_pan_drag;
  std::uint64_t m_last_drag_time = 0;
  double m_chart_drag_total_time = 0;
  int m_chart_drag_total_x = 0, m_chart_drag_total_y = 0;
  int m_inertia_last_drag_x = 0, m_inertia_last_drag_y = 0;
  std::vector<int> m_drag_vec_x, m_drag_vec_y;
  std::vector<double> m_drag_vec_t;
  struct Viewport { int pix_width = 1000, pix_height = 600; } vp;
  Viewport& GetVP() { return vp; }
  std::vector<std::pair<int, int>> pans;
  int starts = 0, stops = 0, cursor = -1;
  std::vector<double> zooms;
  void PanCanvas(int x, int y) { pans.emplace_back(x, y); }
  void StartTimedMovement() { ++starts; }
  void StopMovement() { ++stops; }
  void ZoomCanvas(double factor, bool, bool) { zooms.push_back(factor); }
  wxCursor *pCursorArrow = &arrow, *pCursorRight = &right_cursor,
      *pCursorLeft = &left_cursor, *pCursorDown = &down_cursor,
      *pCursorUp = &up_cursor, *pCursorPencil = &pencil,
      *pPlugIn_Cursor = nullptr;
  void SetCursor(const wxCursor& value) { cursor = value.id; }
  bool MouseEventProcessCanvas(wxMouseEvent&);
  void SetCanvasCursor(wxMouseEvent&);
};

#include "methods.inc"

int checks = 0, failures = 0;
std::string context;
void Check(bool ok, const std::string& name) {
  ++checks;
  if (!ok) { ++failures; std::cout << "FAIL " << context << name << "\n"; }
}
void Click(ChartCanvas& canvas) {
  wxMouseEvent event;
  event.kind = wxMouseEvent::Down;
  canvas.MouseEventProcessCanvas(event);
  event.kind = wxMouseEvent::Up;
  canvas.MouseEventProcessCanvas(event);
}
int main() {
  for (Mode mode : {Mode::XNav, Mode::Legacy, Mode::Safe}) {
    opennav::mode = mode;
    const char* name = mode == Mode::XNav ? "xnav" :
                       mode == Mode::Legacy ? "legacy" : "safe";
    bool native_click = true;
#ifdef OPENNAV_X
    native_click = mode != Mode::XNav;
#endif
    for (Region region : {CENTER, MID_RIGHT, MID_LEFT, MID_TOP, MID_BOT}) {
      context = std::string(name) + "/" + std::to_string(region) + "/";
      ChartCanvas canvas;
      canvas.cursor_region = region;
      wxMouseEvent event;
      canvas.SetCanvasCursor(event);
      const bool stock = native_click || region == CENTER;
      Check(canvas.cursor == (stock ? static_cast<int>(region) : arrow.id),
            "cursor");
      Click(canvas);
      const std::pair<int, int> delta[] = {
          {-400, -180}, {100, 0}, {-100, 0}, {0, 100}, {0, -100}};
      Check(stock ? canvas.pans == std::vector<std::pair<int, int>>{delta[region]}
                  : canvas.pans.empty(), "click");
      Check(!panleftIsDown, "release-clears-press");
      canvas.pans.clear();
      event.kind = wxMouseEvent::Up;
      canvas.MouseEventProcessCanvas(event);
      Check(canvas.pans.empty(), "orphan-release");

      canvas.m_routeState = 2;
      canvas.SetCanvasCursor(event);
      Check(canvas.cursor == pencil.id, "route-pencil");
      canvas.m_routeState = 0;
      canvas.m_bMeasure_Active = true;
      canvas.SetCanvasCursor(event);
      Check(canvas.cursor == pencil.id, "measure-pencil");
      Click(canvas);
      Check(canvas.pans.empty(), "measure-click");
      canvas.pPlugIn_Cursor = &plugin;
      canvas.SetCanvasCursor(event);
      Check(canvas.cursor == plugin.id, "plugin-tool-cursor");
      canvas.m_bMeasure_Active = false;
      canvas.SetCanvasCursor(event);
      Check(canvas.cursor == plugin.id, "plugin-cursor");

      for (bool touch : {false, true}) {
        g_btouch = touch;
        ChartCanvas dragged;
        dragged.cursor_region = region;
        event = {};
        event.kind = wxMouseEvent::Down;
        dragged.MouseEventProcessCanvas(event);
        event.kind = wxMouseEvent::Drag;
        event.x += 10; event.y += 20;
        dragged.MouseEventProcessCanvas(event);
        // Touch records its first motion as the start of the gesture.
        if (touch) {
          event.x += 10; event.y += 20;
          dragged.MouseEventProcessCanvas(event);
        }
        Check(dragged.starts == 1 && dragged.m_pan_drag.x == -10 &&
              dragged.m_pan_drag.y == -20, touch ? "touch-drag" : "mouse-drag");
        event.kind = wxMouseEvent::Up;
        dragged.MouseEventProcessCanvas(event);
        Check(dragged.pans.empty() && !panleftIsDown &&
              (touch || !dragged.m_bChartDragging),
              touch ? "touch-drag-release" : "mouse-drag-release");
        ChartCanvas tapped;
        tapped.cursor_region = region;
        if (touch) {
          Click(tapped);
          Check(tapped.pans.empty(), "touch-tap");
        }
      }
      g_btouch = false;
    }
    ChartCanvas canvas;
    context = std::string(name) + "/";
    wxMouseEvent event;
    event.kind = wxMouseEvent::Down;
    event.shift = true;
    Check(!canvas.MouseEventProcessCanvas(event) && !panleftIsDown, "shift");
    event.shift = false;
    canvas_count = 2;
    Check(!canvas.MouseEventProcessCanvas(event) && !panleftIsDown, "focus");
    canvas_count = 1;
    event.kind = wxMouseEvent::Move;
    event.wheel = 120;
    canvas.MouseEventProcessCanvas(event);
    event.wheel = -120;
    canvas.MouseEventProcessCanvas(event);
    Check(canvas.zooms == std::vector<double>{2.0, 0.5}, "wheel");
  }
  std::cout << checks << " checks, " << failures << " failures\n";
  return failures ? 1 : 0;
}
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--upstream", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--cxx", default="c++")
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    current_path = args.source / "gui/src/chcanv.cpp"
    stock_path = args.upstream / "gui/src/chcanv.cpp"
    current, stock = current_path.read_text(), stock_path.read_text()
    # The independent editing/measurement edge-scroll policy is untouched.
    edge = "bool ChartCanvas::CheckEdgePan("
    assert EXTRACT.function(current, edge) == EXTRACT.function(stock, edge)
    start = "  //  Establish the event region\n"
    end = "  if (!g_btouch) SetCanvasCursor(event);"
    region = lambda text: text[text.index(start):text.index(end, text.index(start))]
    assert region(current) == region(stock)
    identities = {
        "CheckEdgePan": hashlib.sha256(EXTRACT.function(current, edge).encode()).hexdigest(),
        "cursorRegion": hashlib.sha256(region(current).encode()).hexdigest(),
    }
    signatures = ("bool ChartCanvas::MouseEventProcessCanvas(",
                  "void ChartCanvas::SetCanvasCursor(")
    results = []
    for label, source, integrated in (
            ("patched-xnav", current, True),
            ("patched-stock-build", current, False),
            ("original-regression", stock, True)):
        directory = output / label
        directory.mkdir(exist_ok=True)
        methods = "".join(EXTRACT.function(source, sig) for sig in signatures)
        (directory / "methods.inc").write_text(methods)
        (directory / "fixture.cpp").write_text(FIXTURE)
        # The stock cursor method does not use its wx event parameter.
        command = [args.cxx, "-std=c++17", "-Wall", "-Wextra", "-Werror",
                   "-Wno-unused-parameter",
                   str(directory / "fixture.cpp"), "-o", str(directory / "test")]
        if integrated:
            command.insert(1, "-DOPENNAV_X")
        compiled = subprocess.run(command, capture_output=True, text=True)
        (directory / "compile.log").write_text(compiled.stdout + compiled.stderr)
        compiled.check_returncode()
        run = subprocess.run([str(directory / "test")], capture_output=True,
                             text=True, timeout=10)
        (directory / "run.log").write_text(run.stdout + run.stderr)
        if label == "original-regression":
            expected = {f"FAIL xnav/{n}/{kind}"
                        for n in range(1, 5) for kind in ("cursor", "click")}
            actual = {line for line in run.stdout.splitlines() if line.startswith("FAIL ")}
            assert run.returncode == 1 and actual == expected, run.stdout + run.stderr
        else:
            run.check_returncode()
        print(f"{label}: {run.stdout.splitlines()[-1]}")
        results.append({"variant": label, "command": command, "exit": run.returncode,
                        "methodsSha256": hashlib.sha256(methods.encode()).hexdigest(),
                        "output": run.stdout})
    report = {"scope": __doc__, "unchangedSource": identities, "runs": results,
              "inputs": {str(path.resolve()): hashlib.sha256(path.read_bytes()).hexdigest()
                         for path in (current_path, stock_path, Path(__file__),
                                      ROOT / "patches/opencpn-5.12.4-chart-tool-input.patch")}}
    (output / "receipt.json").write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()

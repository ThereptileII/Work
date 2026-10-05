# SCRUM-14 — saved navigation-object Search slice

The top-bar Search button opens the native Search drawer with the immutable
prototype heading, search field and list-card hierarchy. The available scope is
explicitly **saved OpenCPN routes and waypoints**. No general chart place-name
index is exposed by the existing navigation integration. No illustrative places,
distances, search modes, coordinates or separate navigation database are added.

## Source and interaction boundary

- `src/integration/NavigationObjects.cpp::CopyNavigationCatalog` copies the
  pinned OpenCPN `pRouteList` and waypoint-manager list into owned values, bounded
  at 1,000 routes and 10,000 waypoints. Search filters names case-insensitively
  using wx Unicode strings and renders at most 100 matches. Truncation is shown.
- `src/ui/SearchDrawer.cpp` retains an owned catalog snapshot and identity/name/type presentation
  values. Empty and duplicated identities are omitted. A fresh catalog is read
  on opening and selecting; selection rechecks uniqueness. Typing filters the
  snapshot, avoiding repeated full OpenCPN catalog traversal for each keystroke.
  Dismissal, query changes and destruction invalidate pending selections.
- Waypoint selection delegates to the existing `NavigationActions::view_waypoint`
  callback, which resolves a unique native identity and checks geometry before
  centering, then to the existing Shell waypoint context. Routes open existing
  route details, whose model refresh resolves the native route again. Search
  deliberately does not call `view_route`: that existing action also changes
  and persists route visibility.
- Search does not activate, create, edit, delete or reverse navigation objects.
  Existing context/detail actions retain their existing guards and confirmations.
- `XNavDrawer` supplies close/Escape/Alt-Left and responsive placement. Result
  rows inherit `XNavButton` keyboard activation, touch capture and pan behavior.
  Opening focuses the native borderless text editor. Tab reaches results.

## Focused development evidence

The immutable HTML was opened without edits at 1280×800, DPR 1, with the existing
Search button. Computed reference: drawer `(682,80,398,674)`; search field 44px
high with 4px top / 15px bottom margin; list-card rows 66px; title 12px/550;
secondary copy 9px; icons 22px; 12px icon-to-copy gap. The drawer shares the
existing native header geometry and theme controls; no global drawer change is
included in this increment.

`search_drawer_test` is an offline component executable with clearly named test
objects, no OpenCPN profile or equipment. It checks case/Unicode filtering,
owned-copy lifetime, deleted/duplicate identities, bounded results, immediate
query updates, stale deferred events after query/dismissal/destruction, native
Return activation, exact drawer bounds and Escape. It captures Day populated
and Night empty surfaces and an offline Shell at 1280×800 and 853×600. All test
data is confined to the test executable.

Local result: **62 focused checks passed**. Inspected the fresh Day/Night and
normal/compact Shell captures: Search is visible, remains a 44px target, does
not overlap top-bar neighbors and opens a drawer contained by the compact
viewport. The GTK capture uses fresh root-window pixels (the wx screen DC
returned a stale image during harness development). Final local artifacts are
`evidence/local/search/interaction-final.log` and
`evidence/local/search/component/*.png`. The original-file manifest still
passes. No full OpenCPN build or CI was run for this increment.

Local commands (with the optional project-local dependencies available):

```sh
source tools/local-env.sh
cmake -S . -B build/search-component -G Ninja \
  -DOPENNAV_BUILD_TESTS=OFF -DOCPN_BUILD_TEST=ON \
  -DOPENNAV_BUILD_UI_COMPONENTS=ON \
  -DwxWidgets_CONFIG_EXECUTABLE="$PWD/tools/wx-config-local" \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build/search-component --target search_drawer_test -j2
# Run on an isolated Xvfb desktop; pass a fresh capture directory.
GDK_BACKEND=x11 DISPLAY=:219 build/search-component/search_drawer_test \
  evidence/local/search/component
```

The combined follow-up preserves those five capture names and adds 20 Shell
checks (**82 total**): Layers opens the observed chart drawer; a failed action
retains actual readback; Escape and Close return to the chart; Settings →
Navigation opens fresh chart state; a 250px chart keeps the bottom tool strip
visible while hiding the overlapping Layers action. These checks register a
real offline chart pane, so floating controls are exercised without an OpenCPN
profile or device. The combined local run passed; the short native proof now
requires all 82 checks. New artifacts remain under
`evidence/local/prototype-combined/` until exact-source evidence is recorded.

Native Windows typography/input/DPI and integrated top-bar layout, physical boat
review and full SCRUM-14 visual acceptance remain pending. The Linux component
checks and captures are development evidence only. No release/boat claim or
qualification transfer is made.

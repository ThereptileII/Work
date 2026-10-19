#!/usr/bin/env python3
"""Run actual pinned canvas focus/popup handlers against inert event stubs.

This extracts production C++, not wx event delivery or a native UI acceptance
test. --unpatched retains the original bug for a failing regression control.
"""
import argparse
import ast
import os
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]
PIN = '37fd0cddb7334fe489e9f18aa163977a9c5c84f7'
PATCH = 'patches/opencpn-5.12.4-route-focus.patch'


def function(source, signature):
    start = source.index(signature)
    opening = source.index('{', start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--upstream', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--unpatched', action='store_true')
    args = parser.parse_args()
    upstream = args.upstream.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    actual = subprocess.check_output(
        ['git', '-C', str(upstream), 'rev-parse', 'HEAD'], text=True).strip()
    if actual != PIN:
        raise SystemExit('Exact pinned OpenCPN source required')
    # Read the committed baseline without modifying or trusting a patched tree.
    source = subprocess.check_output(
        ['git', '-C', str(upstream), 'show', PIN + ':gui/src/chcanv.cpp'])
    fixture = output / 'source'
    (fixture / 'gui/src').mkdir(parents=True, exist_ok=True)
    (fixture / 'gui/src/chcanv.cpp').write_bytes(source)
    subprocess.run(['git', '-C', str(fixture), 'init', '-q'], check=True)
    tree = ast.parse((ROOT / 'tools/prepare-integration.py').read_text())
    assignment = next(node for node in tree.body if isinstance(node, ast.Assign)
                      and any(isinstance(t, ast.Name) and t.id == 'patches'
                              for t in node.targets))
    patches = [node.right.value for node in assignment.value.elts]
    if patches.count(PATCH) != 1:
        raise SystemExit('Route focus patch missing or duplicated in actual build')
    for patch in patches:
        if args.unpatched and patch == PATCH:
            continue
        subprocess.run(['git', '-C', str(fixture), 'apply',
                        '--include=gui/src/chcanv.cpp', '-'],
                       input=(ROOT / patch).read_bytes().replace(b'\r\n', b'\n'),
                       check=True)
    source = (fixture / 'gui/src/chcanv.cpp').read_text()
    start_route = function(source, 'void ChartCanvas::StartRoute(')
    guard = ''
    if not args.unpatched:
        guard = function(source, 'class RouteFinishFocusGuard') + ';\n'
        # Every temporary assignment, in both route input paths and both menu
        # paths, must be replaced. Only constructor initialization and the
        # deliberate XNav StartRoute policy may assign directly. StartRoute's
        # actual mode/early-return behavior is exercised by the C++ fixture.
        if source.count('RouteFinishFocusGuard routeFocusGuard(') != 8:
            raise SystemExit('Not all eight native modal scopes are protected')
        assignment = r'\bm_FinishRouteOnKillFocus\s*=(?!=)\s*([^;]+);'
        constructor = function(source, 'ChartCanvas::ChartCanvas(')
        if re.findall(assignment, constructor) != ['true']:
            raise SystemExit('Native constructor focus policy changed')
        if re.findall(assignment, start_route) != ['false']:
            raise SystemExit('Native StartRoute explicit completion policy changed')
        other_scopes = source.replace(constructor, '', 1).replace(start_route, '', 1)
        if re.search(assignment, other_scopes):
            raise SystemExit('Native handler still overwrites caller focus policy')
        guard = '#define HAS_ROUTE_FOCUS_GUARD 1\n' + guard
    handlers = '\n'.join(function(source, signature) for signature in (
        'void ChartCanvas::OnKillFocus(',
        'void ChartCanvas::OnRouteFinishTimerEvent(',
        'void ChartCanvas::OnLongPress(',
        'void ChartCanvas::OnLeftUp('))
    objects = function(source, 'bool ChartCanvas::MouseEventProcessObjects(')
    right = function(objects[objects.rindex('if (event.RightDown())'):],
                     'if (event.RightDown())')
    keyboard = source[source.index('    case WXK_MENU:'):
                      source.index('    case WXK_ALT:', source.index('    case WXK_MENU:'))]
    handlers += '\nbool ChartCanvas::RightClick(wxMouseEvent &event) {\n' \
        'int mx=event.m_x,my=event.m_y; bool ret=false;\n' + right + '\nreturn ret;\n}\n'
    handlers += '\nvoid ChartCanvas::KeyboardMenu() {\n' \
        'wxMouseEvent event; switch(WXK_MENU) {\n' + keyboard + '\n}\n}\n'
    # The second nearby-waypoint condition is the separate native touch path.
    nearby = list(re.finditer(r'if \(pNearbyPoint && \(pNearbyPoint != m_prev_pMousePoint\)', objects))
    if len(nearby) != 2:
        raise SystemExit('Expected exact mouse and touch reuse scopes')
    reuse_mouse = function(objects[nearby[0].start():], 'if (brp_viz)')
    reuse_touch = function(objects[nearby[1].start():], 'if (pNearbyPoint')
    for name, body in [('ReuseMouse', reuse_mouse), ('ReuseTouch', reuse_touch)]:
        handlers += '\nvoid ChartCanvas::' + name + '() {\n' \
            'bool brp_viz=true, inserting=false, appending=false;\n' \
            'RoutePoint *pNearbyPoint=&nearby, *pMousePoint=nullptr; Route *tail=nullptr;\n' \
            + body + '\n(void)brp_viz; (void)inserting; (void)appending;\n}\n'
    # Compile the exact modal portions; subsequent great-circle geometry is
    # unchanged and deliberately outside this inert focus-policy fixture.
    edge = objects.index('m_disable_edge_pan = true;  // This helps on OS X')
    start = max(objects.rfind('RouteFinishFocusGuard', 0, edge),
                objects.rfind('m_FinishRouteOnKillFocus = false;', 0, edge))
    gc_mouse = objects[start:objects.index('if (answer == wxID_YES)', edge)]
    mouse_answer = objects.index('int answer = OCPNMessageBox', edge)
    touch_answer = objects.index('int answer = OCPNMessageBox', mouse_answer + 1)
    touch = objects.rindex('#ifndef __WXOSX__', 0, touch_answer)
    gc_touch = objects[touch:objects.index('#endif', touch) + len('#endif')]
    for name, body in [('GreatCircleMouse', gc_mouse), ('GreatCircleTouch', gc_touch)]:
        handlers += '\nvoid ChartCanvas::' + name + '() {\n' \
            'wxString msg="inert great-circle question";\n' + body + \
            '\nlast_answer=answer;\n}\n'
    (output / 'production-route-focus.h').write_text(guard + start_route + '\n' + handlers)
    failures = []
    for label, defines in [('desktop', ['-DOPENNAV_X']),
                           ('mac-timer', ['-DOPENNAV_X', '-D__WXOSX__']),
                           ('stock-desktop', []), ('stock-mac-timer', ['-D__WXOSX__'])]:
        executable = output / ('route-focus-' + label)
        subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-O2',
                        '-Wall', '-Wextra', '-Werror', '-Wno-unused-parameter', *defines,
                        '-I' + str(output), str(ROOT / 'tests/route_focus_tests.cpp'),
                        '-o', str(executable)], check=True)
        print('Route-focus variant: ' + label, flush=True)
        if subprocess.run([str(executable)]).returncode:
            failures.append(label)
    if failures:
        raise SystemExit('Route-focus regression in: ' + ', '.join(failures))


if __name__ == '__main__':
    main()

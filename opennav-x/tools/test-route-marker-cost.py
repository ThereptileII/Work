#!/usr/bin/env python3
"""Headless production marker resolution: safety semantics and bounded walk cost.

Optional --reference compares every result with an earlier production source.
No painter, GUI, timing threshold, Windows or boat acceptance is exercised.
"""
import argparse
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--source', type=Path, default=ROOT / 'src/integration/ChartRouteWaypoint.cpp')
p.add_argument('--reference', type=Path)
a = p.parse_args()

def marker(source):
    return source[source.index('int ChartWaypointMarker('):
                  source.index('\nbool ChartWaypointMarkerOwnsName(')]

header = (ROOT / 'src/integration/ChartRouteWaypoint.h').read_text()
codes = header[header.index('enum class WaypointMarkerKind'):
               header.index('\nint ChartWaypointMarker(')]
source = a.source.read_text()
generic = source[source.index('bool GenericWaypointIcon('):
                 source.index('\ndouble MarkerScale(')]
presentation = (ROOT / 'src/integration/ChartPresentation.cpp').read_text()
route_style = presentation[presentation.index('bool DefaultChartRouteStyle('):
                           presentation.index('\nbool DrawChartRouteSegment(')]
harness = (ROOT / 'tests/route_marker_cost_test.cpp').read_text()
harness = harness.replace('PRODUCTION_CODES', codes).replace('PRODUCTION_GENERIC', generic)
harness = harness.replace('PRODUCTION_ROUTE_STYLE', route_style)
harness = harness.replace('PRODUCTION_MARKER', marker(source))
reference = marker(a.reference.read_text()).replace('int ChartWaypointMarker(', 'int ReferenceMarker(') if a.reference else ''
harness = harness.replace('REFERENCE_MARKER', reference)
with tempfile.TemporaryDirectory(prefix='skager-route-marker-cost-') as temporary:
    directory = Path(temporary)
    test = directory / 'test.cpp'
    test.write_text(harness)
    exe = directory / 'test'
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-O2',
                    '-DHAS_REFERENCE=' + str(int(bool(a.reference))),
                    str(test), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)

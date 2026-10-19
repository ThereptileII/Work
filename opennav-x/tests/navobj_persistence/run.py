#!/usr/bin/env python3
"""Exercise prepared native persistence methods against an in-memory SQLite DB.

Only value objects and notification/UI boundaries are inert fixtures. Database
SQL, transactions, route/track/waypoint save methods and FinishRoute are extracted
unchanged from the specified prepared OpenCPN source. No profile or hardware opens.
"""
import argparse
from pathlib import Path
import subprocess

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--source', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
p.add_argument('--cxx', default='g++')
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=True)

def definition(source, signature):
    start = source.index(signature)
    brace = source.index('{', start)
    end, depth = brace + 1, 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]

source = (a.source / 'model/src/navobj_db.cpp').read_text()
functions = ['CreateTables', 'TrackExists', 'RouteExistsDB', 'RoutePointExists',
             'DeleteAllCommentsForTrack', 'DeleteAllCommentsForRoute',
             'DeleteAllCommentsForRoutePoint', 'InsertTrackHTML', 'InsertRouteHTML',
             'InsertRoutePointHTML', 'InsertRoutePointDB', 'InsertRoutePointLink',
             'InsertTrackPoint', 'NavObj_dB::UpdateDBTrackAttributes',
             'NavObj_dB::UpdateDBRouteAttributes', 'NavObj_dB::UpdateDBRoutePointAttributes',
             'NavObj_dB::UpdateRoute', 'NavObj_dB::UpdateRoutePoint',
             'NavObj_dB::InsertRoutePoint', 'NavObj_dB::UpdateTrack']
native = definition(source, 'class NavobjWriteScope') + ';\n'
native += '\n'.join(definition(source, 'bool ' + name + '(') for name in functions)
(a.output / 'native.inc').write_text(native)
canvas = (a.source / 'gui/src/chcanv.cpp').read_text()
(a.output / 'finish.inc').write_text(definition(canvas, 'void ChartCanvas::FinishRoute('))
here = Path(__file__).resolve().parent
for name in ('persistence', 'finish'):
    command = [a.cxx, '-std=c++17', '-Wall', '-Wextra', '-Werror',
               '-I', str(a.output), str(here / (name + '.cpp')), '-o', str(a.output / name)]
    if name == 'persistence':
        command += ['-lsqlite3']
    subprocess.run(command, check=True)
    subprocess.run([str(a.output / name)], check=True)

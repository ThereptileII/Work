#!/usr/bin/env python3
"""Extract unchanged persistence and FinishRoute definitions from prepared source."""
import argparse
import hashlib
import json
from pathlib import Path

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--source', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
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


source = (a.source / 'model/src/navobj_db.cpp').read_text(encoding='utf-8')
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
(a.output / 'native.inc').write_text(native, encoding='utf-8')
canvas = (a.source / 'gui/src/chcanv.cpp').read_text(encoding='utf-8')
(a.output / 'finish.inc').write_text(
    definition(canvas, 'void ChartCanvas::FinishRoute('), encoding='utf-8')
inputs = ['model/src/navobj_db.cpp', 'gui/src/chcanv.cpp',
          'libs/sqlite/src/sqlite3.c', 'libs/sqlite/include/sqlite3.h']
(a.output / 'inputs.json').write_text(json.dumps({
    'sha256': {name: hashlib.sha256((a.source / name).read_bytes()).hexdigest()
               for name in inputs},
    'scope': 'Native extracted save methods and bundled SQLite; inert model/UI objects, no hardware'
}, indent=2) + '\n', encoding='utf-8')

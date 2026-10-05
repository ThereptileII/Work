"""Read the two retained public cells; this is not a rendering acceptance test."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import pyogrio
from pyogrio import raw
from shapely import from_wkb

p = argparse.ArgumentParser()
p.add_argument('--noaa', type=Path, required=True)
p.add_argument('--iho', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
expected = {
    'US5SEAFL.000': '7e474bea96e7a84c6f3ee6b44ff7db4288029fa47ac2a285594e18469dbf518c',
    'US5SEAFL.001': '504440d201453548aae160c4c5340d55d480bd39cd57061b7fc03262968e062d',
    'GB4X0000.000': 'c70d9e0f53e149270f85900f8576082db86781d64fbb71aa7d5ceb4af4aa22e3',
}


def clean(value):
    if hasattr(value, 'tolist'):
        value = value.tolist()
    if isinstance(value, list):
        return [clean(x) for x in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def identity(path):
    paths = sorted(x for x in path.parent.glob(path.stem + '.*')
                   if x.suffix[1:].isdigit())
    result = {x.name: {'bytes': x.stat().st_size,
                      'sha256': hashlib.sha256(x.read_bytes()).hexdigest()}
              for x in paths}
    wanted = {k: v for k, v in expected.items() if k.startswith(path.stem + '.')}
    if {k: v['sha256'] for k, v in result.items()} != wanted:
        raise ValueError('Original cell/update identity differs: ' + path.name)
    return result


report = {'purpose': 'Source geometry/attributes only; no application or chart-render proof',
          'decoder': {'pyogrio': pyogrio.__version__, 'gdal': list(pyogrio.__gdal_version__)},
          'updates': 'APPLY', 'coordinateOrder': 'longitude,latitude', 'cells': []}
for path, kind in [(a.noaa, 'NOAA real-world ENC'),
                   (a.iho, 'Official IHO S-64 presentation-test geography, not operational ENC')]:
    before = identity(path)
    layers = {str(x[0]) for x in pyogrio.list_layers(path)}
    cell = {'name': path.stem, 'kind': kind, 'files': before, 'features': {}}
    for layer in ('SMCFAC', 'HRBFAC', 'UWTROC', 'WRECKS', 'FSHFAC', 'CBLSUB'):
        rows = []
        if layer in layers:
            meta, fids, geoms, fields = raw.read(path, layer=layer, return_fids=True,
                                                UPDATES='APPLY')
            for i, wkb in enumerate(geoms):
                values = {str(k): clean(v[i]) for k, v in zip(meta['fields'], fields)}
                values = {k: v for k, v in values.items() if v is not None and v != [] and v != ''}
                geometry = from_wkb(wkb) if wkb is not None else None
                rows.append({'ogrFid': int(fids[i]), 'attributes': values,
                             'geometryType': geometry.geom_type if geometry is not None else None,
                             'bounds': list(geometry.bounds) if geometry is not None else None})
        cell['features'][layer] = rows
    if before != identity(path):
        raise ValueError('Chart bytes changed during read')
    cell['counts'] = {k: len(v) for k, v in cell['features'].items()}
    report['cells'].append(cell)
a.output.parent.mkdir(parents=True, exist_ok=True)
with a.output.open('x', encoding='utf-8') as out:
    json.dump(report, out, indent=2, allow_nan=False)
    out.write('\n')
for cell in report['cells']:
    print(cell['name'], cell['counts'])

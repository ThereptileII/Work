#!/usr/bin/env python3
"""Record reviewed native HPGL geometry using the immutable prototype importer."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from chart_symbol_svg import ROOT, ASSETS, NATIVE_NAMES


def derive(source):
    lock = json.loads((ASSETS / 'source-lock.json').read_text())
    data = source.read_bytes().replace(b'\r\n', b'\n')
    assert hashlib.sha256(data).hexdigest() == lock['files']['chartsymbols.xml']['sha256']
    tree = ET.fromstring(data)
    parser = ROOT / 'docs/design/prototype/symbol-library.mjs'
    symbols = {}
    for name in NATIVE_NAMES:
        node = tree.findall("symbols/symbol[name='" + name + "']")[-1]
        vector, bitmap = node.find('vector'), node.find('bitmap')
        assert vector is not None and bitmap is not None
        result = subprocess.run(
            ['node', '--input-type=module', '-e',
             "import {hpglGeometry} from " + json.dumps(parser.as_uri()) + ";"
             "process.stdout.write(JSON.stringify(hpglGeometry(process.argv[1],process.argv[2])));",
             vector.findtext('HPGL'), node.findtext('color-ref')],
            check=True, capture_output=True, text=True)
        symbols[name] = {
            'description': node.findtext('description'),
            'effectiveRcid': node.get('RCID'),
            'sourceNodeSha256': hashlib.sha256(ET.tostring(node)).hexdigest(),
            'colorRoles': [node.findtext('color-ref')[i+1:i+6]
                           for i in range(0, len(node.findtext('color-ref')), 6)],
            'bitmap': {**{key: int(bitmap.get(key)) for key in ('width', 'height')},
                       'pivot': {key: int(bitmap.find('pivot').get(key)) for key in ('x', 'y')}},
            'nativePivot': {key: int(vector.find('pivot').get(key)) for key in ('x', 'y')},
            'geometry': json.loads(result.stdout)}
    return {'issue': 'SCRUM-378', 'upstreamCommit': lock['upstreamCommit'],
            'sourceXmlSha256': hashlib.sha256(data).hexdigest(),
            'parser': str(parser.relative_to(ROOT)),
            'parserSha256': hashlib.sha256(parser.read_bytes().replace(b'\r\n', b'\n')).hexdigest(),
            'note': 'Native geometry and categorical roles; original logical bitmap size and pivot. '
                    'No selector edits. Stock atlas remains the SVG-unavailable fallback.',
            'symbols': symbols}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    args = parser.parse_args()
    (ASSETS / 'native-points.json').write_text(json.dumps(derive(args.source), indent=2) + '\n')

#!/usr/bin/env python3
"""Extract final HTML cascade variables from verified canonical browser renders.

The supplied HTML includes later style blocks absent from src/style.css.
Canonical Windows and Linux measurements must agree at 1280x800 DPR1.
No original file is written and no CSS cascade is approximated with regexes.
"""
import argparse
import hashlib
import json
from pathlib import Path
from render import ORIGINAL, ROOT, verify_original


def extract():
    identity = verify_original()['htmlSha256']
    themes = {}
    evidence = {}
    for platform in ('windows', 'linux'):
        path = ROOT / 'docs/design/prototype/reference' / platform / 'capture.json'
        report = json.loads(path.read_text())
        assert report['htmlSha256'] == identity and report['deviceScaleFactor'] == 1
        assert report['viewport'] == {'width':1280, 'height':800}
        evidence[platform] = hashlib.sha256(path.read_bytes()).hexdigest()
        for theme in ('day','dusk','night'):
            variables = report['states']['navigation-'+theme]['variables']
            assert all(state['variables'] == variables for state in report['states'].values() if state['theme'] == theme)
            if theme in themes: assert themes[theme] == variables
            themes[theme] = variables
    return {'htmlSha256':identity,
            'source':'Final computed #app CSS cascade in immutable HTML; canonical Chromium 1280x800 DPR1',
            'referenceCaptureSha256':evidence, 'themes':themes}


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--check', action='store_true')
    a = p.parse_args()
    path = ROOT / 'docs/design/prototype-tokens.json'
    result = extract()
    if a.check:
        assert json.loads(path.read_text()) == result
    else:
        path.write_text(json.dumps(result, indent=2)+'\n')
    print('Final HTML tokens verified against both canonical platform measurements')

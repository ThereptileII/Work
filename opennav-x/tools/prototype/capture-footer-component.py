#!/usr/bin/env python3
"""Capture isolated footer test; retain exact reference/current/diff, never fixtures aboard."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from PIL import Image, ImageChops

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--client', type=Path, required=True)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output = args.output.resolve()
    if args.output.exists():
        raise SystemExit('Use a new evidence directory')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    windows = sys.platform == 'win32'
    if windows and os.environ.get('GITHUB_ACTIONS') != 'true':
        raise SystemExit('Native fixtures require disposable CI, never the boat')
    reference = json.loads((args.reference / 'capture.json').read_text())
    original = hashlib.sha256((ROOT / 'docs/design/prototype/index.html').read_bytes()).hexdigest()
    assert reference['htmlSha256'] == original
    assert reference['platform'] == ('Windows' if windows else 'Linux')
    assert reference['viewport'] == dict(width=1280, height=800)
    assert reference['deviceScaleFactor'] == 1
    env = dict(os.environ)
    xserver = None
    if windows:
        spec = importlib.util.spec_from_file_location('windows_ui', ROOT / 'tools/windows-ui.py')
        ui = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(ui)
        ui.ensure_desktop(1440, 900)
    else:
        env['GDK_BACKEND'] = 'x11'
        env.pop('WAYLAND_DISPLAY', None)
        number = next(n for n in range(177, 240) if not Path(f'/tmp/.X{n}-lock').exists())
        env['DISPLAY'] = f':{number}'
        xserver = subprocess.Popen(['Xvfb', env['DISPLAY'], '-screen', '0', '1280x800x24', '-nolisten', 'tcp'],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(.4)
        assert xserver.poll() is None
    try:
        result = subprocess.run([str(args.client.resolve()), str(args.output)], env=env,
                                capture_output=True, timeout=45)
        args.output.mkdir(exist_ok=True)
        (args.output / 'interaction.log').write_bytes(result.stdout + result.stderr)
        if result.returncode:
            raise RuntimeError('Footer input/geometry failed; inspect interaction.log')
        record = json.loads((args.output / 'result.json').read_text())
        assert record['passed'] and record['checks'] >= 153
        assert len(record['captures']) == 10
        record.update(source_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                      source_dirty=bool(subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT)),
                      executable_sha256=hashlib.sha256(args.client.read_bytes()).hexdigest(),
                      html_sha256=original, platform=sys.platform,
                      conformance='PENDING: review exact crops/diffs and native/boat evidence; no similarity waiver')
        record['screenshots'] = {}
        record['comparison'] = {}
        comparison = args.output / 'comparison'
        comparison.mkdir()
        for name in record['captures']:
            path = args.output / (name + '.png')
            record['screenshots'][name] = hashlib.sha256(path.read_bytes()).hexdigest()
            with Image.open(path) as src:
                current = src.convert('RGB')
                width, height = current.size
                assert width in (1280, 1024, 853)
                theme = 'night' if name.startswith('responsive') else name.rsplit('-', 1)[-1]
                background, rule = {'day': ((21, 35, 38), (53, 70, 74)),
                                    'dusk': ((29, 40, 46), (64, 80, 89)),
                                    'night': ((12, 17, 21), (41, 53, 59))}[theme]
                assert current.getpixel((2, height-20)) == background, name + ': wrong footer background'
                assert current.getpixel((2, height-34)) == rule, name + ': missing/displaced top rule'
                for bounds in ((20, height-26, 280, height-5), (width-220, height-26, width-20, height-5)):
                    assert len(current.crop(bounds).getcolors(1000000)) > 8, name + ': missing status/health text'
                if not name.startswith('prototype-fixture-'):
                    continue
                original_path = args.reference / ('navigation-' + theme + '.png')
                expected_hash = reference['states']['navigation-' + theme]['screenshotSha256']
                assert hashlib.sha256(original_path.read_bytes()).hexdigest() == expected_hash
                ref = Image.open(original_path).convert('RGB').crop((0, 766, 1280, 800))
                crop = current.crop((0, 766, 1280, 800))
                diff = ImageChops.difference(ref, crop)
                ref.save(comparison / (theme + '-reference.png'))
                crop.save(comparison / (theme + '-current.png'))
                diff.save(comparison / (theme + '-diff.png'))
                components = reference['states']['navigation-' + theme]['components']
                record['comparison'][theme] = {
                    'bounds': [0, 766, 1280, 34],
                    'reference_groups': {key: components[selector][0]['rect'] for key, selector in
                        [('left', '.statusbar>span:first-child'), ('middle', '.footer-middle'), ('health', '.statusbar>button')]},
                    'native_groups': record['canonical_geometry'],
                    'changed_pixels': sum(pixel != (0, 0, 0) for pixel in diff.getdata())}
        (args.output / 'capture.json').write_text(json.dumps(record, indent=2) + '\n')
        print(f"PASS {record['checks']} footer component checks; 10 real captures; exact differences retained for review")
    finally:
        if xserver is not None:
            xserver.terminate()
            xserver.wait(timeout=5)


if __name__ == '__main__':
    main()

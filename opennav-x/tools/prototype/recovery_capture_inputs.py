"""Audited package input and a fixed official test scene; never discovers profiles."""
import configparser
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import shutil
import stat
import zipfile

from hardware_output_policy import require_status_only

IHO_SHA256 = 'c70d9e0f53e149270f85900f8576082db86781d64fbb71aa7d5ceb4af4aa22e3'
IHO_CENTER = (-32.3471615, 61.169588)
IHO_SCALE = 0.5826126536  # Observed upstream scale from requested 0.6, unchanged e1 scene.


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def require(value, message):
    if not value:
        raise ValueError(message)


def plain(path):
    """Reject links/junctions, including any existing ancestor of the input."""
    for part in (path, *path.parents):
        info = part.lstat()
        require(not stat.S_ISLNK(info.st_mode) and
                not getattr(info, 'st_file_attributes', 0) & 0x400,
                'Linked/reparse input refused: ' + str(part))


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, 'Duplicate JSON key: ' + key)
        result[key] = value
    return result


def verify_package(root, expected_commit, expected_exe, expected_manifest, archive, expected_archive):
    """Expected hashes come from the independent artifact audit, not this package."""
    root = root.absolute()
    plain(root)
    require(root.is_dir(), 'Recovery package root is not a directory')
    require(re.fullmatch(r'[0-9a-f]{40}', expected_commit or ''), 'Exact expected application commit required')
    for value in (expected_exe, expected_manifest, expected_archive):
        require(re.fullmatch(r'[0-9a-f]{64}', value or ''), 'Externally audited SHA256 required')
    manifest_path = root / 'FILE_SHA256.json'
    plain(manifest_path)
    require(sha(manifest_path) == expected_manifest, 'Recovery manifest differs from independent audit')
    manifest = json.loads(manifest_path.read_text(encoding='utf-8-sig'), object_pairs_hook=unique_object)
    require(isinstance(manifest, dict) and 1 <= len(manifest) <= 30000, 'Invalid recovery inventory')
    folded = set()
    for name, digest in manifest.items():
        rel = PurePosixPath(name)
        require(name and not rel.is_absolute() and rel.as_posix() == name and
                all(p not in ('.', '..') and ':' not in p and not p.endswith((' ', '.')) for p in rel.parts) and
                '\\' not in name and name != 'FILE_SHA256.json', 'Unsafe inventory path: ' + name)
        require(name.casefold() not in folded, 'Case-ambiguous inventory path: ' + name)
        folded.add(name.casefold())
        require(isinstance(digest, str) and re.fullmatch(r'[0-9a-f]{64}', digest), 'Invalid inventory hash')
        path = root.joinpath(*rel.parts)
        plain(path)
        require(path.is_file() and sha(path) == digest, 'Recovery payload differs: ' + name)
    actual = set()
    for path in root.rglob('*'):
        plain(path)
        if path.is_file():
            actual.add(path.relative_to(root).as_posix())
    require(actual == set(manifest) | {'FILE_SHA256.json'}, 'Unlisted or missing recovery payload')
    for name in ('app/opencpn.exe', 'app/OPENNAV_PORTABLE_PREVIEW', 'docs/PRODUCT_BUILD.json', 'profile/opencpn.conf'):
        require(name in manifest, 'Required packaged input absent: ' + name)
    archive = archive.absolute()
    plain(archive)
    require(sha(archive) == expected_archive, 'Recovery ZIP differs from independent audit')
    with zipfile.ZipFile(archive) as zipped:
        members = zipped.infolist()
        for member in members:
            # ZIP creators may omit a Unix file type. Both that form and an
            # explicit regular-file type are allowed; links/devices/FIFOs and
            # DOS directory/reparse attributes never describe package payloads.
            kind = stat.S_IFMT(member.external_attr >> 16)
            require(not member.is_dir() and kind in (0, stat.S_IFREG) and
                    not (member.external_attr & (0x10 | 0x400)),
                    'Non-regular/link/reparse ZIP entry refused: ' + member.filename)
        require(len(members) == len(actual) and sum(x.file_size for x in members) < 4 * 1024**3, 'Unexpected recovery ZIP inventory')
        prefix = 'SKAGER-Beta2-Portable-Recovery/'
        require({x.filename for x in members} == {prefix+n for n in actual}, 'Recovery ZIP/extracted inventory differs')
        require(zipped.testzip() is None, 'Recovery ZIP CRC failed')
        for name, expected in manifest.items():
            with zipped.open(prefix+name) as stream:
                digest = hashlib.sha256()
                for block in iter(lambda: stream.read(1024 * 1024), b''):
                    digest.update(block)
            require(digest.hexdigest() == expected, 'Recovery ZIP payload differs: ' + name)
        require(zipped.read(prefix+'FILE_SHA256.json') == manifest_path.read_bytes(), 'Recovery ZIP manifest differs')
    require((root / 'app/OPENNAV_PORTABLE_PREVIEW').read_bytes() in
            (b'SKAGER portable Beta 2 recovery\n', b'SKAGER portable Beta 2 recovery\r\n'),
            'Exact packaged portable marker content required')
    product = json.loads((root / 'docs/PRODUCT_BUILD.json').read_text(encoding='utf-8-sig'), object_pairs_hook=unique_object)
    require_status_only(product)
    require(product.get('commit') == expected_commit and product.get('test_fixtures') is False and
            product.get('build_purpose') == 'INSTALLED PRODUCT' and
            product.get('executable_sha256') == expected_exe == manifest['app/opencpn.exe'],
            'Recovery application identity/capability differs')
    config = configparser.ConfigParser(interpolation=None, strict=True)
    config.read_string((root / 'profile/opencpn.conf').read_text(encoding='utf-8-sig'))
    require('ConfigVersionString' in config['Settings'] and
            'configversionstring' not in config.defaults(), 'Exact packaged profile version missing')
    version = config['Settings']['ConfigVersionString']
    require(re.fullmatch(r'Version [^\r\n\x00]+ Build [^\r\n\x00]+', version), 'Malformed packaged profile version')
    return {'root': str(root), 'application_commit': expected_commit, 'executable_sha256': expected_exe,
            'archive_sha256': expected_archive, 'archive_crc': 'all passed',
            'file_manifest_sha256': expected_manifest, 'verified_files': len(manifest),
            'product_build_sha256': manifest['docs/PRODUCT_BUILD.json'],
            'packaged_profile_sha256': manifest['profile/opencpn.conf'], 'config_version_string': version}


def new_profile(profile, version):
    profile.mkdir(parents=True, exist_ok=False)
    (profile / 'OPENNAV_TEST_PROFILE').write_text('Disposable disconnected UI test.\n', encoding='utf-8')
    # Only the version crosses this boundary. No connections, credentials,
    # plugins, routes or user preferences are copied from any existing profile.
    (profile / 'opencpn.conf').write_text(
        '[Settings]\nConfigVersionString=' + version + '\nNavMessageShown=1\nShowStatusBar=1\nShowMenuBar=1\n'
        '[Settings/GlobalState]\nFrameWinX=1280\nFrameWinY=800\nFrameWinPosX=0\nFrameWinPosY=0\nFrameMax=0\n',
        encoding='utf-8')


def stage_iho(source, output):
    source = source.absolute()
    plain(source)
    require(source.name == 'GB4X0000.000' and source.is_file() and sha(source) == IHO_SHA256,
            'Exact retained official IHO cell required')
    output.mkdir(parents=True, exist_ok=False)
    target = output / source.name
    shutil.copyfile(source, target)
    require(sha(target) == IHO_SHA256, 'IHO copy identity differs')
    return {'file': source.name, 'sha256': IHO_SHA256,
            'scope': 'Official IHO S-64 presentation-test geography, not an operational nautical ENC',
            'center': list(IHO_CENTER), 'requested_scale_ppm': .6, 'observed_scale_ppm': IHO_SCALE,
            'source_features': [{'class': 'BOYSPP', 'RCID': 254, 'COLOUR': ['6'], 'BOYSHP': 3},
                                {'class': 'TOPMAR', 'RCID': 257, 'COLOUR': ['6'], 'TOPSHP': 7}],
            'source_audit_sha256': '567c4dea09a1d05a11919174e10268b7e75ea9e5ac85c281a10097bd0360e15b'}


def runtime_identity(snapshot, expected_commit, recovery):
    require(snapshot['build_commit'] == expected_commit, 'Running application commit differs from expected application commit')
    if recovery:
        require_status_only(snapshot)
        require(snapshot.get('test_fixtures') is False and snapshot.get('build_purpose') == 'INSTALLED PRODUCT',
                'Running recovery application is not fixture-free production')
        require(snapshot.get('data_mode') == 'OPENCPN selected navigation', 'Demo/replay data mode refused')
        pilot = snapshot['runtime']['pilot']
        require(pilot['enabled'] is False and pilot['simulated'] is False and
                pilot['control_capability'] is False and pilot['command_state'] == 'None',
                'Pilot/control activation is outside this disconnected capture')


def presentation(snapshot, style, iho=False):
    p = snapshot['runtime']['chart_presentation']
    require(p['requested'] == style, 'Requested chart style was not retained')
    core_status = 'SKAGER presentation v1 / pinned symbols' if style == 'XNav' else 'Standard OpenCPN presentation'
    require(p['status'] == core_status + '; o-charts: no SKAGER adapter loaded',
            'Core presentation or expected absent private adapter differs')
    require(p['core']['available'] is True and p['private_ocharts'] == {'available': False},
            'Core/private presentation observation differs')
    if iho:
        require(p['core'] == {'available': True, 'saved_point_style': 76, 'effective_point_style': 76},
                'IHO Simplified saved/effective table differs')
        c = snapshot['runtime']['chart']
        require(abs(c['latitude']-IHO_CENTER[0]) < 1e-7 and abs(c['longitude']-IHO_CENTER[1]) < 1e-7 and
                abs(c['scale_ppm']-IHO_SCALE) < 1e-7 and c['follow'] is False and c['quilt'] is True,
                'Actual official test viewport differs')
        require(c['canvas_pixels'] == {'width': 1014, 'height': 566} and c['database_entries'] == 1 and
                c['quilt_members'] == [{'type': 5, 'native_scale': 52000, 'file': 'GB4X0000.000', 'index': c['quilt_reference']}],
                'Actual single official test ENC quilt differs')


def iho_pixels(image_path, snapshot, client_origin, style, renderer, root):
    """Fixed source-geometry probes; never search colors for a passing position."""
    from PIL import Image, ImageChops
    reference_file = root / 'tools/prototype/iho-yellow-pixel-reference.json'
    definition = json.loads(reference_file.read_text())
    theme = snapshot['runtime']['display']['light']
    label = 'SKAGER' if style == 'XNav' else 'Standard'
    entry = definition['references'][f'{renderer}/{label}/{theme}']
    reference = root / entry['path']
    require(sha(reference) == entry['sha256'], 'Retained pair pixel reference changed')
    region = snapshot['runtime']['display']['chart_region']
    # The exact chart center equals the real co-located source coordinates,
    # proved by presentation() before this function. No geographic projection
    # or synthetic model object is introduced by the collector.
    x = region['x'] - client_origin[0] + 507
    y = region['y'] - client_origin[1] + 283
    receipt = {'scope': definition['scope'], 'reference': entry,
               'reference_definition_sha256': sha(reference_file),
               'canvas_anchor': [507, 283], 'client_anchor': [x, y], 'probes': {}}
    with Image.open(image_path) as source, Image.open(reference) as previous:
        actual = source.convert('RGB')
        previous = previous.convert('RGB')
        require(actual.size == (1280, 800) and previous.size == (72, 72), 'Unexpected pair image dimensions')
        actual.crop((x-36, y-36, x+36, y+36)).save(image_path.with_name(image_path.stem+'-source-pair.png'))
        for name, bounds in definition['probes'][label].items():
            a,b,c,d = bounds
            current = actual.crop((x+a,y+b,x+c,y+d))
            expected = previous.crop((36+a,36+b,36+c,36+d))
            difference = ImageChops.difference(current, expected)
            equal = difference.getbbox() is None
            receipt['probes'][name] = {'relative_bounds': bounds, 'exact': equal,
                                      'actual_rgb_sha256': hashlib.sha256(current.tobytes()).hexdigest(),
                                      'reference_rgb_sha256': hashlib.sha256(expected.tobytes()).hexdigest()}
            if not equal:
                difference.save(image_path.with_name(image_path.stem+'-'+name+'-difference.png'))
    image_path.with_suffix('.pair.json').write_text(json.dumps(receipt, indent=2)+'\n')
    require(all(p['exact'] for p in receipt['probes'].values()),
            'Actual fixed IHO pair pixels differ; inspect retained exact probes before any retry')
    return receipt


def day_return(image_path, first_path, snapshot, client_origin):
    from PIL import Image, ImageChops
    r = snapshot['runtime']['display']['chart_region']
    x,y = r['x']-client_origin[0], r['y']-client_origin[1]
    bounds = (x,y,x+r['width'],y+r['height'])
    with Image.open(image_path) as current, Image.open(first_path) as first:
        difference = ImageChops.difference(current.convert('RGB').crop(bounds), first.convert('RGB').crop(bounds))
        equal = difference.getbbox() is None
        receipt = {'bounds': bounds, 'mask': None, 'exact': equal, 'difference_bounds': difference.getbbox()}
        image_path.with_suffix('.return.json').write_text(json.dumps(receipt, indent=2)+'\n')
        if not equal:
            difference.save(image_path.with_name(image_path.stem+'-whole-chart-difference.png'))
    require(equal, 'Whole chart Day-return differs; original images and difference retained')
    return receipt

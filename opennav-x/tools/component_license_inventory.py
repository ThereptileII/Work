"""Deterministic evidence inventory; deliberately not a license-compliance verdict.

Called only after package dependency validators and create_source_archive succeed.
The source archive is the same locally generated, exact-commit archive; this
module does not authenticate arbitrary archives supplied by a third party.
"""
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import zipfile

from source_package import PINNED_UPSTREAM

INVENTORY = 'docs/COMPONENT_LICENSE_INVENTORY.json'
NOTICES = 'docs/THIRD_PARTY_NOTICES.md'
GENERATED = {INVENTORY, NOTICES, 'FILE_SHA256.json'}
DEPENDENCIES = {
    'OpenSSL': ('3.5.9', ('libssl-3.dll', 'libcrypto-3.dll')),
    'curl': ('8.22.0', ('libcurl.dll',)),
    'zlib': ('1.3.2', ('zlib1.dll',)),
}


def digest(data):
    return hashlib.sha256(data).hexdigest()


def file_record(path):
    h = hashlib.sha256()
    size = 0
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
            size += len(block)
    return {'sha256': h.hexdigest(), 'bytes': size}


def read_json(data):
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError('Ambiguous duplicate JSON key: ' + key)
            result[key] = value
        return result
    return json.loads(data, object_pairs_hook=unique)


def require_path(name):
    if (not isinstance(name, str) or not name or '\\' in name or ':' in name or
            PurePosixPath(name).is_absolute() or '..' in PurePosixPath(name).parts or
            PurePosixPath(name).as_posix() != name):
        raise ValueError('Unsafe inventory path: ' + str(name))


def generate_inventory(package, source_archive, commit, verified_manifests):
    """Return JSON-compatible inventory, rechecking every declared evidence link.

    verified_manifests is the result of the existing OpenSSL and curl/zlib
    package validators, not untrusted records loaded by this generator.
    """
    package, source_archive = Path(package), Path(source_archive)
    if re.fullmatch('[0-9a-f]{40}', commit) is None:
        raise ValueError('Inventory requires the exact product commit')
    if set(verified_manifests) != set(DEPENDENCIES):
        raise ValueError('Inventory requires all verified dependency manifests')
    files = {}
    folded = set()
    for path in sorted(package.rglob('*')):
        if path.is_symlink():
            raise ValueError('Package contains a symlink: ' + str(path))
        if not path.is_file():
            continue
        name = path.relative_to(package).as_posix()
        require_path(name)
        if name.casefold() in folded:
            raise ValueError('Ambiguous Windows package path: ' + name)
        folded.add(name.casefold())
        if name not in GENERATED:
            files[name] = file_record(path)
    if source_archive.is_symlink() or not source_archive.is_file():
        raise ValueError('Corresponding source archive missing or unsafe')
    source_record = file_record(source_archive)
    components = []
    attributed = {}

    def assign(name, component):
        if name not in files:
            raise ValueError('Declared package evidence missing: ' + name)
        if name in attributed:
            raise ValueError('Ambiguous component attribution: ' + name)
        attributed[name] = component

    with zipfile.ZipFile(source_archive) as archive:
        names = archive.namelist()
        if len(set(names)) != len(names):
            raise ValueError('Ambiguous duplicate corresponding-source member')
        refs = read_json(archive.read('SOURCE_REFERENCE.json'))
        if refs.get('productCommit') != commit or refs.get('upstreamCommit') != PINNED_UPSTREAM:
            raise ValueError('Corresponding-source identity differs from package')
        source_files = refs['files']

        def source_bytes(name):
            require_path(name)
            record = source_files.get(name)
            if not isinstance(record, dict) or record.get('gitMode') not in ('100644', '100755'):
                raise ValueError('Declared source evidence missing or not regular: ' + name)
            data = archive.read(name)
            if not data or digest(data) != record.get('sha256'):
                raise ValueError('Declared source evidence digest differs: ' + name)
            return data

        def notice(package_name, source_name, component):
            assign(package_name, component)
            expected = digest(source_bytes(source_name))
            if files[package_name]['sha256'] != expected:
                raise ValueError('Packaged notice differs from corresponding source: ' + package_name)
            return {'path': package_name, 'sourcePath': source_name, 'sha256': expected}

        product_notices = [notice('docs/licenses/SKAGER-COPYING.txt',
                                  'opennav-x/LICENSE', 'skager')]
        components.append({'id': 'skager', 'source': {'productCommit': commit},
                           'evidenceStatus': 'source-notice-matched', 'notices': product_notices})
        upstream_notices = []
        expected_upstream = {
            'docs/licenses/OpenCPN/' + name.removeprefix('OpenCPN-5.12.4-integrated/')
            for name in source_files
            if name.startswith('OpenCPN-5.12.4-integrated/') and
            PurePosixPath(name).name.lower().startswith(('copying', 'license', 'copyright')) and
            'cache' not in PurePosixPath(name).parts
        }
        actual_upstream = {name for name in files if name.startswith('docs/licenses/OpenCPN/')}
        if not expected_upstream or actual_upstream != expected_upstream:
            raise ValueError('OpenCPN source/license notice inventory differs')
        for name in sorted(files):
            if name.startswith('docs/licenses/OpenCPN/'):
                upstream_notices.append(notice(name, 'OpenCPN-5.12.4-integrated/' +
                                               name.removeprefix('docs/licenses/OpenCPN/'), 'opencpn'))
        if not upstream_notices:
            raise ValueError('OpenCPN source/license notices missing')
        components.append({'id': 'opencpn', 'version': '5.12.4',
                           'source': {'upstreamCommit': PINNED_UPSTREAM},
                           'evidenceStatus': 'source-notice-matched', 'notices': upstream_notices,
                           'limitations': ['Embedded third-party code and its individual terms remain under review.']})
        for library, (version, dlls) in DEPENDENCIES.items():
            prefix = 'docs/licenses/' + library + '-' + version + '/'
            source_prefix = 'opennav-x/docs/third-party/' + library + '-' + version + '/'
            provenance_notice = notice(prefix + 'provenance.json', source_prefix + 'provenance.json', library)
            license_notice = notice(prefix + 'LICENSE.txt', source_prefix + 'LICENSE.txt', library)
            provenance = read_json(source_bytes(source_prefix + 'provenance.json'))
            manifest = verified_manifests[library]
            if (manifest.get('library') != library or manifest.get('version') != version or
                    provenance.get('library') != library + ' ' + version or
                    provenance.get('licenseSha256') != license_notice['sha256']):
                raise ValueError('Dependency license identity differs: ' + library)
            source = manifest['source']
            if (provenance.get('sourceArchiveSha256') != source['sha256'] or
                    provenance.get('sourceArchive') != source['url'] or
                    provenance.get('sourceArchiveBytes') != source['bytes']):
                raise ValueError('Dependency source provenance differs: ' + library)
            matches = [item for item in refs.get('bundledDependencySources', [])
                       if item.get('reference', {}).get('library') == library]
            if len(matches) != 1:
                raise ValueError('Missing or ambiguous bundled dependency source: ' + library)
            bundle = matches[0]
            if (bundle.get('sha256') != source['sha256'] or bundle.get('bytes') != source['bytes'] or
                    bundle.get('reference', {}).get('version') != version or
                    bundle['reference'].get('url') != source['url'] or
                    len(source_bytes(bundle['path'])) != source['bytes'] or
                    source_files[bundle['path']]['sha256'] != source['sha256']):
                raise ValueError('Bundled dependency source differs: ' + library)
            runtime_files = []
            for dll in dlls:
                name = 'app/' + dll
                assign(name, library)
                if files[name] != manifest['outputs']['bin/' + dll]:
                    raise ValueError('Packaged dependency runtime differs: ' + name)
                runtime_files.append(name)
            manifest_path = 'app/' + library.lower() + '-build.json'
            assign(manifest_path, library)
            if read_json((package / manifest_path).read_bytes()) != manifest:
                raise ValueError('Packaged build manifest differs: ' + library)
            components.append({'id': library, 'version': version,
                               'declaredLicense': provenance['license'],
                               'evidenceStatus': 'runtime-source-notice-matched',
                               'source': {'path': bundle['path'], 'sha256': source['sha256'], 'url': source['url']},
                               'buildManifest': manifest_path, 'runtimeFiles': runtime_files,
                               'notices': [license_notice, provenance_notice]})
        wx_prefix = 'docs/licenses/wxWidgets-3.2.8/'
        wx_source = 'opennav-x/docs/third-party/wxWidgets-3.2.8/'
        wx_notices = [notice(wx_prefix + 'provenance.json', wx_source + 'provenance.json', 'wxWidgets')]
        wx = read_json(source_bytes(wx_source + 'provenance.json'))
        if wx.get('version') != '3.2.8' or {item['file'] for item in wx['files']} != {'licence.txt', 'lgpl.txt', 'gpl.txt'}:
            raise ValueError('wxWidgets license provenance incomplete')
        for item in wx['files']:
            record = notice(wx_prefix + item['file'], wx_source + item['file'], 'wxWidgets')
            if record['sha256'] != item['sha256']:
                raise ValueError('wxWidgets notice provenance differs')
            wx_notices.append(record)
        components.append({'id': 'wxWidgets', 'version': wx['version'],
                           'source': {'commit': wx['commit']},
                           'evidenceStatus': 'source-notice-matched', 'notices': wx_notices,
                           'limitations': ['Runtime-to-upstream-source mapping is not established by this inventory.']})
    unresolved = []
    for name, record in files.items():
        record['componentId'] = attributed.get(name)
        if name not in attributed:
            suffix = PurePosixPath(name).suffix.lower()
            category = ('font' if suffix in {'.ttf', '.otf', '.woff', '.woff2', '.ttc'} else
                        'asset' if suffix in {'.png', '.svg', '.jpg', '.jpeg', '.ico', '.bmp', '.gif', '.xpm'} else
                        'runtime' if suffix in {'.dll', '.exe'} else 'other')
            unresolved.append({'path': name, 'category': category,
                               'reason': 'No reviewed component/license attribution in this inventory.'})
    return {'schema': 1, 'productCommit': commit,
            'sourceArchive': {'name': source_archive.name, **source_record},
            'status': 'incomplete-review-required',
            'limitations': [
                'Evidence inventory only; no legal compatibility or redistribution conclusion.',
                'Fonts, assets, embedded dependencies, MSVC, wxWidgets runtime, updater and o-charts terms need separate review.',
                'Absent font files do not establish clearance for system fonts or fonts embedded in assets.',
                'This inventory excludes itself, THIRD_PARTY_NOTICES.md and FILE_SHA256.json; the package file manifest covers generated notices.',
            ],
            'components': sorted(components, key=lambda item: item['id']),
            'files': files, 'unresolvedFiles': unresolved}


def write_inventory(package, source_archive, commit, verified_manifests):
    inventory = generate_inventory(package, source_archive, commit, verified_manifests)
    package = Path(package)
    text = ['# Third-party notice inventory', '',
            'Product commit: `' + commit + '`', '',
            '**Incomplete: source and license review remains required.**',
            'This generated index records evidence; it does not grant rights or determine license compatibility.', '',
            'Machine-readable file hashes, source links and unresolved files: [COMPONENT_LICENSE_INVENTORY.json](COMPONENT_LICENSE_INVENTORY.json).', '']
    for component in inventory['components']:
        text.extend(['## ' + component['id'] + (' ' + component['version'] if 'version' in component else ''), '',
                     'Evidence: ' + component['evidenceStatus'] + '.', ''])
        if 'declaredLicense' in component:
            text.extend(['Recorded license identifier: ' + component['declaredLicense'] + '.', ''])
        for item in component['notices']:
            relative = item['path'].removeprefix('docs/')
            text.append('- [' + relative + '](' + relative + ')')
        text.extend(['', *component.get('limitations', []), ''])
    text.extend(['## Unresolved coverage', '', *inventory['limitations'][:3], '',
                 str(len(inventory['unresolvedFiles'])) + ' packaged files lack reviewed component/license attribution; see the machine-readable inventory.', ''])
    (package / INVENTORY).write_text(json.dumps(inventory, indent=2, sort_keys=True) + '\n', encoding='utf-8', newline='\n')
    (package / NOTICES).write_text('\n'.join(text), encoding='utf-8', newline='\n')
    return inventory

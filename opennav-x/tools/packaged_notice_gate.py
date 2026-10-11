"""Offline notice/source consistency for declared packaged components.

No extraction, network access, license-compatibility decision, or source-completeness
claim. Trusted CI provenance and existing dependency producer gates remain required.
"""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import stat
from urllib.parse import urlsplit
import zipfile
import freefont_payload

PREFIX = 'SKAGER-Beta2-Portable-Recovery/'
UPSTREAM = '37fd0cddb7334fe489e9f18aa163977a9c5c84f7'
LIBRARIES = {'wxWidgets-3.2.8': ('gpl.txt', 'lgpl.txt', 'licence.txt'),
             'OpenSSL-3.5.9': ('LICENSE.txt',), 'curl-8.22.0': ('LICENSE.txt',),
             'zlib-1.3.2': ('LICENSE.txt',)}
MAX_ARCHIVE = 2 << 30
MAX_ENTRIES = 100000
MAX_METADATA = 16 << 20
MAX_NOTICE = 1 << 20
MAX_BUNDLE = 128 << 20
MAX_READ = 256 << 20


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def object_json(data):
    def unique(pairs):
        result = {}
        for key, value in pairs:
            require(key not in result, 'Duplicate notice/source JSON key: ' + key)
            result[key] = value
        return result
    value = json.loads(data, object_pairs_hook=unique)
    require(isinstance(value, dict), 'Notice/source metadata must be an object')
    return value


def safe_path(name):
    require(isinstance(name, str) and 0 < len(name) <= 1024 and
            not any(ord(c) < 32 for c in name) and '\\' not in name and ':' not in name and
            not name.startswith('/') and PurePosixPath(name).as_posix() == name and
            all(part not in ('', '.', '..') and not part.endswith(('.', ' '))
                for part in name.split('/')), 'Unsafe notice/source path')


def path_map(value):
    require(isinstance(value, dict) and len(value) <= MAX_ENTRIES, 'Invalid source/hash inventory')
    folded = set()
    for name in value:
        safe_path(name)
        require(name.casefold() not in folded, 'Ambiguous case-colliding source/hash reference')
        folded.add(name.casefold())
    return value


class Archive:
    def __init__(self, archive):
        self.archive = archive
        self.read_bytes = 0
        entries = archive.infolist()
        require(len(entries) <= MAX_ENTRIES, 'Notice/source archive entry bound exceeded')
        self.files = {}
        folded = set()
        for entry in entries:
            name = entry.filename.rstrip('/') if entry.is_dir() else entry.filename
            safe_path(name)
            require(entry.orig_filename == entry.filename, 'Normalized archive entry refused')
            require(name.casefold() not in folded, 'Duplicate/case-colliding notice/source archive entry')
            folded.add(name.casefold())
            self.files[name] = entry

    def entry(self, name, limit):
        require(name in self.files, 'Missing notice/source file: ' + name)
        entry = self.files[name]
        mode = stat.S_IFMT(entry.external_attr >> 16)
        require(not entry.is_dir() and mode in (0, stat.S_IFREG) and not entry.flag_bits & 1 and
                entry.compress_type in (zipfile.ZIP_STORED, zipfile.ZIP_DEFLATED) and
                0 < entry.file_size <= limit, 'Unsafe or oversized notice/source file: ' + name)
        return entry

    def chunks(self, name, limit):
        entry = self.entry(name, limit)
        require(self.read_bytes + entry.file_size <= MAX_READ, 'Notice/source read budget exceeded')
        self.read_bytes += entry.file_size
        total = 0
        with self.archive.open(entry) as stream:
            while chunk := stream.read(min(65536, limit - total + 1)):
                total += len(chunk)
                require(total <= limit, 'Notice/source read bound exceeded')
                yield chunk
        require(total == entry.file_size, 'Notice/source size differs')

    def read(self, name, limit=MAX_NOTICE):
        return b''.join(self.chunks(name, limit))

    def hash(self, name, limit):
        value = hashlib.sha256()
        for chunk in self.chunks(name, limit):
            value.update(chunk)
        return value.hexdigest()


def validate_archives(portable_zip, source_zip, commit):
    """Validate open archives; called only after their raw sizes have been bounded."""
    require(isinstance(commit, str) and re.fullmatch('[0-9a-f]{40}', commit), 'Exact notice/source commit required')
    portable, source = Archive(portable_zip), Archive(source_zip)
    reference = object_json(source.read('SOURCE_REFERENCE.json', MAX_METADATA))
    product_url = 'https://github.com/ThereptileII/Work/tree/' + commit + '/opennav-x'
    require(type(reference.get('schema')) is int and reference['schema'] == 1 and
            reference.get('productCommit') == commit and reference.get('productSource') == product_url and
            reference.get('upstreamCommit') == UPSTREAM and reference.get('openCpnVersion') == '5.12.4' and
            reference.get('integratedTree') == 'OpenCPN-5.12.4-integrated',
            'Notice/source identity differs')
    files = path_map(reference.get('files'))
    hashes = path_map(object_json(portable.read(PREFIX + 'FILE_SHA256.json', MAX_METADATA)))

    def source_bytes(name, limit=MAX_NOTICE):
        record = files.get(name)
        require(isinstance(record, dict) and record.get('gitMode') in ('100644', '100755'),
                'Missing/nonregular notice source reference: ' + name)
        data = source.read(name, limit)
        require(record.get('sha256') == digest(data), 'Notice source digest differs: ' + name)
        return data

    def packaged_bytes(name):
        data = portable.read(PREFIX + name)
        require(hashes.get(name) == digest(data), 'Packaged notice digest differs: ' + name)
        return data

    notices = {}

    def compare(packaged, original):
        data = packaged_bytes(packaged)
        require(data == source_bytes(original), 'Packaged notice differs from corresponding source: ' + packaged)
        notices[packaged] = digest(data)
        return data

    compare('docs/licenses/SKAGER-COPYING.txt', 'opennav-x/LICENSE')
    # Package assembly copies these names recursively from the pinned upstream.
    upstream_prefix = 'OpenCPN-5.12.4-integrated/'
    upstream_notices = sorted(name for name in files if name.startswith(upstream_prefix) and
        PurePosixPath(name).name.lower().startswith(('copying', 'license', 'copyright')) and
        'cache' not in PurePosixPath(name).parts)
    # This pinned release has four named license texts, not a bare COPYING.
    for name in ('COPYING.gplv2', 'COPYING.gplv3', 'COPYING.lgplv2', 'COPYING.lgplv3'):
        require(upstream_prefix + name in upstream_notices,
                'Missing pinned OpenCPN license reference: ' + name)
    for name in upstream_notices:
        compare('docs/licenses/OpenCPN/' + name[len(upstream_prefix):], name)

    bundles = reference.get('bundledDependencySources')
    require(isinstance(bundles, list) and len(bundles) <= 32, 'Missing/bounded dependency source references required')
    bundle_paths = {}
    bundle_components = set()
    for bundle in bundles:
        require(isinstance(bundle, dict), 'Invalid dependency source reference')
        name = bundle.get('path')
        safe_path(name)
        require(name.startswith('third-party-sources/') and len(PurePosixPath(name).parts) == 2 and
                name.casefold() not in bundle_paths, 'Ambiguous dependency source reference')
        bundle_paths[name.casefold()] = bundle
        declaration = bundle.get('reference')
        require(isinstance(declaration, dict), 'Missing dependency source identity')
        if 'library' in declaration:
            component = declaration['library']
            require(isinstance(component, str) and component and
                    component.casefold() not in bundle_components,
                    'Ambiguous dependency component source reference')
            bundle_components.add(component.casefold())

    for library, names in LIBRARIES.items():
        target = 'docs/licenses/' + library + '/'
        original = 'opennav-x/docs/third-party/' + library + '/'
        provenance = object_json(compare(target + 'provenance.json', original + 'provenance.json'))
        contents = {name: compare(target + name, original + name) for name in names}
        if library.startswith('wxWidgets-'):
            entries = provenance.get('files')
            wx_commit = '8aef5f40b93958719771331ca03866b7b6fff6bf'
            require(provenance.get('version') == '3.2.8' and provenance.get('commit') == wx_commit and
                    isinstance(entries, list) and len(entries) == len(names),
                    'wxWidgets notice provenance differs')
            declared = {}
            for entry in entries:
                require(isinstance(entry, dict) and isinstance(entry.get('file'), str) and
                        entry['file'] not in declared, 'Ambiguous wxWidgets notice reference')
                require(entry.get('source') == 'https://raw.githubusercontent.com/wxWidgets/wxWidgets/' +
                        wx_commit + '/docs/' + entry['file'], 'wxWidgets notice source differs')
                declared[entry['file']] = entry.get('sha256')
            require(declared == {name: digest(data) for name, data in contents.items()},
                    'wxWidgets notice content differs from provenance')
            continue
        component, version = library.rsplit('-', 1)
        require(provenance.get('library') == component + ' ' + version and
                provenance.get('licenseSha256') == digest(contents['LICENSE.txt']),
                'Notice content differs from component provenance: ' + library)
        url = provenance.get('sourceArchive')
        require(isinstance(url, str) and url.startswith('https://'), 'Missing dependency source URL')
        name = 'third-party-sources/' + PurePosixPath(urlsplit(url).path).name
        bundle = bundle_paths.get(name.casefold())
        require(isinstance(bundle, dict) and bundle.get('path') == name and
                bundle.get('sha256') == provenance.get('sourceArchiveSha256') and
                type(bundle.get('bytes')) is int and bundle['bytes'] == provenance.get('sourceArchiveBytes'),
                'Dependency source/provenance differs: ' + library)
        declaration = bundle.get('reference')
        require(isinstance(declaration, dict) and declaration.get('library') == component and
                declaration.get('version') == version and declaration.get('url') == url,
                'Dependency source identity differs: ' + library)
        record = files.get(name)
        require(isinstance(record, dict) and record.get('gitMode') == '100644' and
                record.get('sha256') == bundle['sha256'] and
                source.entry(name, MAX_BUNDLE).file_size == bundle['bytes'] and
                source.hash(name, MAX_BUNDLE) == bundle['sha256'], 'Dependency source bytes differ: ' + library)

    freefont_payload.verify_sources(lambda name: source_bytes('opennav-x/' + name, MAX_BUNDLE))
    for name in freefont_payload.NOTICE_FILES:
        compare(freefont_payload.PACKAGE_ROOT + name.removeprefix(freefont_payload.NOTICE_ROOT),
                'opennav-x/' + name)

    packaged_notices = {name[len(PREFIX):] for name, entry in portable.files.items()
                        if name.startswith(PREFIX + 'docs/licenses/') and not entry.is_dir()}
    require(packaged_notices == set(notices), 'Packaged notice inventory differs from declared source set')
    summary = packaged_bytes('docs/SOURCE_AND_LICENSES.md').decode('utf-8')
    product_refs = re.findall(r'https://github\.com/ThereptileII/Work/tree/[^\s`]+', summary)
    upstream_refs = re.findall(r'https://github\.com/OpenCPN/OpenCPN/tree/[^\s`]+', summary)
    require(product_refs == [product_url] and upstream_refs == ['https://github.com/OpenCPN/OpenCPN/tree/' + UPSTREAM],
            'Missing/ambiguous packaged same-commit source URL')
    return {'schema': 1, 'productCommit': commit, 'notices': dict(sorted(notices.items()))}


def validate(portable, source, commit):
    for path in (portable, source):
        entry = Path(path).lstat()
        require(stat.S_ISREG(entry.st_mode) and 0 < entry.st_size <= MAX_ARCHIVE,
                'Regular bounded notice/source archive required')
    with zipfile.ZipFile(portable) as recovery, zipfile.ZipFile(source) as corresponding:
        return validate_archives(recovery, corresponding, commit)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--portable', required=True, type=Path)
    parser.add_argument('--source', required=True, type=Path)
    parser.add_argument('--commit', required=True)
    args = parser.parse_args()
    print(json.dumps(validate(args.portable, args.source, args.commit), sort_keys=True, indent=2))


if __name__ == '__main__':
    main()

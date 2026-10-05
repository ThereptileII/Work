"""Exact source inventory for the native package (including monorepo CI recipe)."""
import hashlib
import json
from pathlib import Path
from pathlib import PurePosixPath
import re
import stat
import subprocess
import zipfile

PINNED_UPSTREAM = '37fd0cddb7334fe489e9f18aa163977a9c5c84f7'


def git(directory, *args):
    return subprocess.check_output(['git', '-C', str(directory), *args])


def source_inventory(root, commit):
    root = Path(root).resolve()
    upstream = root / 'build/integration-source'
    if git(root, 'rev-parse', 'HEAD').decode().strip() != commit:
        raise ValueError('Source archive commit does not match the tested product commit')
    if git(upstream, 'rev-parse', 'HEAD').decode().strip() != PINNED_UPSTREAM:
        raise ValueError('Source archive requires the exact pinned OpenCPN baseline')
    # The application source itself must be committed. The intentionally patched
    # upstream tree is independently checked by prepare-integration.py at build.
    subprocess.run(['git', '-C', str(root), 'diff', '--exit-code', '--quiet', '--', '.'], check=True)
    subprocess.run(['git', '-C', str(root), 'diff', '--cached', '--exit-code', '--quiet', '--', '.'], check=True)
    entries = {}
    submodules = []
    for directory, prefix in ((root, 'opennav-x'), (upstream, 'OpenCPN-5.12.4-integrated')):
        for item in git(directory, 'ls-files', '--stage', '-z').split(b'\0'):
            if not item:
                continue
            metadata, raw_name = item.split(b'\t', 1)
            mode, object_id, stage = metadata.decode().split()
            if stage != '0':
                raise ValueError('Unmerged source entry')
            name = raw_name.decode('utf-8')
            if directory == root and name.startswith(('upstream/', '.github/')):
                continue
            target = prefix + '/' + name
            if mode == '160000':
                submodules.append({'path': target, 'commit': object_id,
                                   'note': 'Gitlink reference; not an application build input'})
                continue
            path = directory / name
            if mode == '120000':
                # Preserve link text, never dereference a path outside the source.
                contents = git(directory, 'cat-file', 'blob', object_id)
            else:
                contents = path.read_bytes()
            entries[target] = (contents, mode)
    repository = Path(git(root, 'rev-parse', '--show-toplevel').decode().strip())
    workflows = [name for name in git(repository, 'ls-tree', '-r', '--name-only', commit,
                                     '--', '.github/workflows').decode().splitlines()
                 if (name.startswith('.github/workflows/opennav-') or
                     name == '.github/workflows/update-verifier-probe.yml') and name.endswith('.yml')]
    if '.github/workflows/opennav-baseline.yml' not in workflows:
        raise ValueError('Exact root GitHub Actions workflow is missing from source package')
    baseline = git(repository, 'show', commit + ':.github/workflows/opennav-baseline.yml')
    if (b'./.github/workflows/update-verifier-probe.yml' in baseline and
            '.github/workflows/update-verifier-probe.yml' not in workflows):
        raise ValueError('Required reusable updater build gate is missing from source package')
    # Git applies the checkout's text/EOL policy; CRLF on native Windows is not
    # an uncommitted recipe. The archive still records the exact checkout bytes.
    for name in workflows:
        workflow = repository / name
        if not workflow.is_file() or workflow.is_symlink():
            raise ValueError('Exact root CI recipe missing or not a regular file')
        comparison = subprocess.run(['git', '-C', str(repository), 'diff', '--exit-code', '--quiet',
                                     commit, '--', name])
        if comparison.returncode:
            raise ValueError('CI recipe differs from the packaged commit')
        entries[name] = (workflow.read_bytes(), '100644')
    references = {
        'schema': 1, 'productCommit': commit,
        'productSource': 'https://github.com/ThereptileII/Work/tree/' + commit + '/opennav-x',
        'openCpnVersion': '5.12.4', 'upstreamCommit': PINNED_UPSTREAM,
        'integratedTree': 'OpenCPN-5.12.4-integrated',
        'integrationPatches': [
            {'path': 'opennav-x/' + name,
             'sha256': hashlib.sha256((root / name).read_bytes()).hexdigest()}
            for name in ('patches/opencpn-5.12.4-xnav.patch',
                         'patches/opencpn-5.12.4-regression-tests.patch',
                         'patches/opencpn-5.12.4-ais-transport.patch',
                         'patches/opencpn-5.12.4-chart-presentation.patch',
                         'patches/opencpn-5.12.4-maintained-curl.patch',
                         'patches/opencpn-5.12.4-download-trust.patch',
                         'patches/opencpn-5.12.4-wxcurl-trust.patch',
                         'patches/opencpn-5.12.4-peer-response-buffer.patch',
                         'patches/opencpn-5.12.4-peer-unavailable.patch',
                         'patches/opencpn-5.12.4-pilot-serial.patch')],
        'workflow': '.github/workflows/opennav-baseline.yml',
        'workflows': workflows,
        'gitlinkReferences': submodules,
        'files': {name: {'sha256': hashlib.sha256(content).hexdigest(), 'gitMode': mode}
                  for name, (content, mode) in sorted(entries.items())},
    }
    return entries, references


def create_source_archive(root, commit, archive, bundled_sources=()):
    archive = Path(archive)
    if archive.exists():
        raise ValueError('Refusing to overwrite a corresponding-source archive')
    entries, references = source_inventory(root, commit)
    dependency_sources = []
    for item in bundled_sources:
        source = Path(item['archive'])
        target_name = item['path']
        portable = PurePosixPath(target_name)
        if (not source.is_file() or source.is_symlink() or
                '\\' in target_name or ':' in target_name or
                not target_name.startswith('third-party-sources/') or
                portable.as_posix() != target_name or '..' in portable.parts or
                len(portable.parts) != 2 or
                re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9._+-]*', portable.name) is None or
                target_name in entries):
            raise ValueError('Bundled dependency source path is missing or unsafe')
        source = source.resolve()
        content = source.read_bytes()
        digest = hashlib.sha256(content).hexdigest()
        if digest != item['sha256']:
            raise ValueError('Bundled dependency source digest changed before archiving')
        entries[target_name] = (content, '100644')
        dependency_sources.append({
            'path': target_name, 'sha256': digest, 'bytes': len(content),
            'reference': item['reference'],
        })
    if dependency_sources:
        references['bundledDependencySources'] = dependency_sources
        references['files'].update({
            item['path']: {'sha256': item['sha256'], 'gitMode': '100644'}
            for item in dependency_sources
        })
    with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED) as target:
        for name, (content, mode) in sorted(entries.items()):
            info = zipfile.ZipInfo(name)
            info.compress_type = zipfile.ZIP_DEFLATED
            info.create_system = 3
            info.external_attr = ((stat.S_IFLNK | 0o777) if mode == '120000'
                                  else (stat.S_IFREG | (0o755 if mode == '100755' else 0o644))) << 16
            target.writestr(info, content)
        target.writestr('SOURCE_REFERENCE.json', json.dumps(references, indent=2) + '\n')
    with zipfile.ZipFile(archive) as check:
        if check.testzip() is not None:
            raise ValueError('Corresponding-source archive CRC verification failed')
    return references

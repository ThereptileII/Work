"""Exact source inventory for the native package (including monorepo CI recipe)."""
import hashlib
import json
from pathlib import Path
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
    workflow = repository / '.github/workflows/opennav-baseline.yml'
    if not workflow.is_file():
        raise ValueError('Exact root GitHub Actions workflow is missing from source package')
    git(repository, 'cat-file', '-e', commit + ':.github/workflows/opennav-baseline.yml')
    # Git applies the checkout's text/EOL policy; CRLF on native Windows is not
    # an uncommitted recipe. The archive still records the exact checkout bytes.
    comparison = subprocess.run(['git', '-C', str(repository), 'diff', '--exit-code', '--quiet',
                                 commit, '--', '.github/workflows/opennav-baseline.yml'])
    if comparison.returncode:
        raise ValueError('CI recipe differs from the packaged commit')
    entries['.github/workflows/opennav-baseline.yml'] = (workflow.read_bytes(), '100644')
    references = {
        'schema': 1, 'productCommit': commit,
        'productSource': 'https://github.com/ThereptileII/Work/tree/' + commit + '/opennav-x',
        'openCpnVersion': '5.12.4', 'upstreamCommit': PINNED_UPSTREAM,
        'integratedTree': 'OpenCPN-5.12.4-integrated',
        'integrationPatches': [
            {'path': 'opennav-x/' + name,
             'sha256': hashlib.sha256((root / name).read_bytes()).hexdigest()}
            for name in ('patches/opencpn-5.12.4-xnav.patch',
                         'patches/opencpn-5.12.4-regression-tests.patch')],
        'workflow': '.github/workflows/opennav-baseline.yml',
        'gitlinkReferences': submodules,
        'files': {name: {'sha256': hashlib.sha256(content).hexdigest(), 'gitMode': mode}
                  for name, (content, mode) in sorted(entries.items())},
    }
    return entries, references


def create_source_archive(root, commit, archive):
    archive = Path(archive)
    if archive.exists():
        raise ValueError('Refusing to overwrite a corresponding-source archive')
    entries, references = source_inventory(root, commit)
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

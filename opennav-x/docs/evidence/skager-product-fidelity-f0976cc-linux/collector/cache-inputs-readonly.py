"""Private source operations, only through the read-only-root bwrap wrapper."""
import ast
from contextlib import contextmanager
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile

CACHE = Path('/home/standard/Projects/X-nav-worktrees/skager-product-fidelity/.local/integrated-fidelity')
SOURCE = Path('/home/standard/Projects/X-nav-worktrees/skager-product-fidelity')
APP = Path('/home/standard/Projects/X-nav-worktrees/waypoint-touch-regression')
UPSTREAM = Path('/home/standard/Projects/X-nav-worktrees/skager-product-integration/build/integration-source')
PIN = '37fd0cddb7334fe489e9f18aa163977a9c5c84f7'
UP_GIT = CACHE / 'upstream.git'


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def git(root, *args, env=None, data=None):
    return subprocess.check_output(['git', '-C', str(root), *args],
        env=env or dict(os.environ, GIT_OPTIONAL_LOCKS='0'), input=data)


def identity():
    require(os.environ.get('SKAGER_PRIVATE_CACHE') == str(CACHE), 'Use run-isolated.sh')
    require(git(UPSTREAM, 'rev-parse', '--absolute-git-dir').decode().strip() == str(UP_GIT),
            'Refusing non-private upstream metadata')
    require(git(APP, 'rev-parse', '--absolute-git-dir').decode().strip() == str(APP / '.git'),
            'Refusing non-private application metadata')
    require(git(UPSTREAM, 'rev-parse', 'HEAD').decode().strip() == PIN,
            'Pinned upstream HEAD differs')
    require(json.loads((APP / 'upstream.lock.json').read_text())['commit'] == PIN,
            'Application upstream pin differs')
    require(not git(APP, 'status', '--porcelain', '--untracked-files=no').strip(),
            'Private application has tracked local edits')
    return git(APP, 'rev-parse', 'HEAD').decode().strip()


def patches(root=APP):
    tree = ast.parse((root / 'tools/prepare-integration.py').read_text())
    nodes = [n for n in tree.body if isinstance(n, ast.Assign) and
             any(isinstance(t, ast.Name) and t.id == 'patches' for t in n.targets)]
    require(len(nodes) == 1, 'Expected one explicit integration patch list')
    namespace = {'root': root}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), 'private-patch-list', 'exec'), namespace)
    result = namespace['patches']
    require(len(result) == 9 and len(set(result)) == 9 and
            all(p.parent == root / 'patches' and p.is_file() and not p.is_symlink() for p in result),
            'Expected nine distinct current integration patch files')
    return result


@contextmanager
def expected(root=APP):
    with tempfile.TemporaryDirectory(prefix='patch-index-', dir=Path(os.environ['SKAGER_CAPTURE_SCRATCH'])) as temporary:
        tmp = Path(temporary)
        (tmp / 'objects').mkdir()
        env = dict(os.environ, GIT_OPTIONAL_LOCKS='0', GIT_INDEX_FILE=str(tmp / 'index'),
                   GIT_OBJECT_DIRECTORY=str(tmp / 'objects'),
                   GIT_ALTERNATE_OBJECT_DIRECTORIES=str(UP_GIT / 'objects'))
        git(UPSTREAM, 'read-tree', PIN, env=env)
        inputs = patches(root)
        for patch in inputs:
            git(UPSTREAM, 'apply', '--cached', '-', env=env,
                data=patch.read_bytes().replace(b'\r\n', b'\n'))
        records = {}
        for item in git(UPSTREAM, 'ls-files', '--stage', '-z', env=env).split(b'\0'):
            if not item:
                continue
            meta, raw_path = item.split(b'\t', 1)
            mode, oid, stage = meta.decode().split()
            require(stage == '0', 'Unmerged expected source')
            records[raw_path.decode()] = (mode, oid)
        yield env, records, inputs


def verify_expected(env):
    result = subprocess.run(['git', '-C', str(UPSTREAM), 'diff', '--exit-code',
                             '--ignore-submodules=all'], env=env, capture_output=True)
    if result.returncode:
        raise RuntimeError('Private upstream differs from all current reviewed patches:\n' +
                           result.stdout.decode(errors='replace')[:12000])


def input_record(commit, inputs):
    return {'schema': 1, 'commit': commit, 'upstreamCommit': PIN,
            'patches': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}}


def verify(frozen=True):
    commit = identity()
    with expected() as (env, records, inputs):
        verify_expected(env)
        record = input_record(commit, inputs)
    if frozen:
        required = json.loads((CACHE / 'frozen-inputs.json').read_text())
        require(all(required[key] == value for key, value in record.items()),
                'Frozen application/patch identity changed')
    print('Verified private clean application', commit, 'and exact upstream result of nine patches')
    return record

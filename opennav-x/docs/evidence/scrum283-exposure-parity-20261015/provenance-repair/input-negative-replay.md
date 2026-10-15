# Linux fixture-input negative replay

Run this Python block from the monorepo checkout. It changes disposable copies
only and uses the already retained local inputs. Windows junction/reparse
execution is separate and is not claimed by these POSIX symlink checks.

```python
from pathlib import Path
import importlib.util, json, shutil, subprocess, tempfile

root = Path('opennav-x').resolve()
spec = importlib.util.spec_from_file_location('hazard', root/'tools/verify-private-hazard-association.py')
helper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helper)
core = Path('/home/standard/Projects/X-nav/upstream/OpenCPN')
original = Path('/home/standard/Projects/X-nav-worktrees/scrum259-adapter-preparation/.local/pinned-source')
patched = Path('/home/standard/Work/skager-scrum283-exposure-20261015/resumed-corrected-source')
results = []

def refuse(name, action):
    try:
        action()
    except ValueError as error:
        results.append(dict(case=name, status='passed', rejection=str(error)))
    else:
        raise AssertionError('Accepted ' + name)

with tempfile.TemporaryDirectory(prefix='skager-hazard-negative-') as raw:
    temp = Path(raw)
    altered = temp/'patched'
    shutil.copytree(patched, altered)
    added = altered/'libs/s52plib/src/bbox.h'
    added.write_text('#error unexpected shadow header\n')
    refuse('added quoted-include shadow header', lambda: helper.bind_sources(core, original, altered))
    added.unlink()
    target = altered/'libs/geoprim/src/bbox.h'
    target.unlink()
    target.symlink_to(patched/'libs/geoprim/src/bbox.h')
    refuse('same-byte header symlink', lambda: helper.bind_sources(core, original, altered))
    target.unlink()
    shutil.copyfile(patched/'libs/geoprim/src/bbox.h', target)
    directory = altered/'libs/geoprim'
    shutil.rmtree(directory)
    directory.symlink_to(patched/'libs/geoprim', target_is_directory=True)
    refuse('same-byte ancestor directory symlink', lambda: helper.bind_sources(core, original, altered))
    directory.unlink()
    shutil.copytree(patched/'libs/geoprim', directory)
    redirect = temp/'redirected-root'
    redirect.symlink_to(altered, target_is_directory=True)
    refuse('redirected source root', lambda: helper.bind_sources(core, original, redirect))
    before = helper.inventory(altered)
    added.write_text('// added during fixture\n')
    refuse('late added source path', lambda: helper.verify_identity(altered, before, exact=True))
    added.unlink()
    local = temp/'local'
    for name in helper.LOCAL_HEADERS:
        path = local/name
        path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(root/name, path)
    identities = {name: helper.record(local/name) for name in helper.LOCAL_HEADERS}
    for name in helper.LOCAL_HEADERS:
        path = local/name
        data = path.read_bytes()
        path.write_bytes(data + b'\n// changed owned header\n')
        refuse('changed ' + name, lambda: helper.verify_identity(local, identities))
        path.write_bytes(data)
    copied_core = temp/'core'
    subprocess.run(['git', 'clone', '--quiet', '--shared', '--no-checkout', str(core), str(copied_core)], check=True)
    names = ['libs/s52plib/src/s52cnsy.cpp', *helper.SOURCE_HEADERS,
             'gui/src/s57obj.cpp', 'data/s57data/s57expectedinput.csv']
    pin = json.loads((root/'upstream.lock.json').read_text())['commit']
    subprocess.run(['git', '-C', str(copied_core), 'checkout', pin, '--', *names], check=True)
    copied_original = temp/'original'
    shutil.copytree(original, copied_original)
    for label, copy in [('core', copied_core), ('original', copied_original), ('patched', altered)]:
        path = copy/'libs/s52plib/src/s52cnsy.cpp'
        path.write_bytes(path.read_bytes() + b'\n// altered body source\n')
        refuse(label + ' source tamper', lambda: helper.bind_sources(
            copy if label == 'core' else core,
            copy if label == 'original' else original,
            copy if label == 'patched' else patched))
disabled = subprocess.run(['python', '-O', str(root/'tools/verify-private-hazard-association.py'), '--help'], text=True, capture_output=True)
assert disabled.returncode == 1 and 'Run without -O' in disabled.stderr
results.append(dict(case='disabled assertions', status='passed', rejection=disabled.stderr.strip()))
print(json.dumps(dict(helperCommit=subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip(),
    helperIdentity=helper.record(root/'tools/verify-private-hazard-association.py'), cases=results,
    scope='Linux input guards only; Windows reparse execution remains unrun'), indent=2))
```

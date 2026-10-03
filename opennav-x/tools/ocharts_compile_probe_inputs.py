"""Stage locked real inputs for the private adapter's object-only native probe.

This deliberately does not call production prepare(): no dependency-producer
manifest, linked adapter, runtime qualification or placeholder library is made.
"""
import importlib.util
import json
from pathlib import Path
import shutil
import sys
import tarfile


def stage(root: Path, output: Path, cache: Path, evidence: Path, api):
    root, output, cache, evidence = (Path(p).resolve() for p in
                                     (root, output, cache, evidence))
    output.mkdir(parents=True, exist_ok=True)
    cache.mkdir(parents=True, exist_ok=True)
    evidence.mkdir(parents=True, exist_ok=True)
    owned = ('source', 'local', 'sdk', 'chart-source', 'resources')
    if any((output / name).exists() for name in owned):
        raise ValueError('Refusing to overwrite earlier probe inputs')
    receipt = {'schema': 1, 'scope': 'Locked inputs for object compilation only; '
               'not dependency producer, link, package or runtime acceptance',
               'status': 'staging', 'locks': {}, 'archives': {}, 'files': {}}
    receipt_path = evidence / 'compile-probe-inputs.json'

    def save():
        receipt_path.write_text(json.dumps(receipt, indent=2) + '\n', encoding='utf-8')

    def inventory(directory):
        result = {}
        for path in sorted(directory.rglob('*')):
            if path.is_symlink():
                raise ValueError('Linked probe input: ' + str(path))
            if path.is_file():
                result[path.relative_to(directory).as_posix()] = api.record(path)
        return result

    def lock(name):
        path = root / name
        receipt['locks'][name] = api.record(path)
        return json.loads(path.read_text(encoding='utf-8'))

    def fetch(item, path):
        api.fetch(item, path)
        actual = api.record(path)
        if actual != {key: item[key] for key in ('sha256', 'bytes')}:
            raise ValueError('Locked input differs after fetch: ' + str(path))
        return actual

    def archive_headers(archive, prefix, accept):
        count = 0
        with tarfile.open(archive, 'r:*') as stream:
            for member in stream.getmembers():
                if not member.name.startswith(prefix):
                    continue
                relative = member.name[len(prefix):]
                target = accept(relative)
                if target is None:
                    continue
                if (not member.isfile() or '..' in Path(relative).parts or
                        Path(relative).is_absolute() or '\\' in relative):
                    raise ValueError('Unsafe header archive member: ' + member.name)
                target.parent.mkdir(parents=True, exist_ok=True)
                with stream.extractfile(member) as source:
                    target.write_bytes(source.read())
                count += 1
        if not count:
            raise ValueError('No real headers in locked archive: ' + str(archive))
        return count

    save()
    try:
        spec = importlib.util.spec_from_file_location(
            'ocharts_probe_preparation', root / 'tools/prepare-ocharts-adapter.py')
        prepare = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(prepare)
        prepare.ROOT = root
        adapter = lock(prepare.LOCK)
        receipt['production_inputs'] = {name: api.record(root / name)
                                         for name in prepare.INPUTS}
        prepare.fetch_sources(output / 'source', cache / 'source-blobs')
        receipt['original_source'] = inventory(output / 'source')
        save()
        prepare.apply_patches(output / 'source', root)
        prepare.copy_local_inputs(output)
        receipt['files']['source'] = inventory(output / 'source')
        receipt['files']['local'] = inventory(output / 'local')
        save()

        for item in lock('tools/windows-wx.lock.json')['archives']:
            archive = cache / item['file']
            receipt['archives'][item['file']] = fetch(item, archive)
            save()
            api.run(['7z', 'x', '-y', '-o' + str(output / 'sdk/wx'), str(archive)],
                    evidence / (item['file'] + '.extract.log'), cwd=root, timeout=180)
        for name, item in adapter['glew']['files'].items():
            prepare.safe_path(name)
            cached = cache / ('glew-' + item['sha256'])
            receipt['archives']['glew/' + name] = fetch(item, cached)
            target = output / 'sdk/glew' / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(cached, target)
        for library in ('curl', 'zlib'):
            item = lock('tools/windows-' + library + '.lock.json')
            archive = cache / item['archive']
            receipt['archives'][item['archive']] = fetch(item, archive)
            save()
            if library == 'curl':
                archive_headers(archive, 'curl-' + item['version'] + '/include/curl/',
                                lambda name: output / 'sdk/include/curl' / name
                                if name.endswith('.h') else None)
            else:
                archive_headers(archive, 'zlib-' + item['version'] + '/',
                                lambda name: output / 'sdk/include' / name
                                if name in ('zlib.h', 'zconf.h') else None)
        for name in ('curl/curl.h', 'curl/curlver.h', 'zlib.h', 'zconf.h'):
            if not (output / 'sdk/include' / name).is_file():
                raise ValueError('Missing real dependency header: ' + name)
        receipt['files']['sdk'] = inventory(output / 'sdk')
        save()

        resources = lock('resources/chart-style/v1/source-lock.json')
        upstream = lock('upstream.lock.json')
        if resources['upstreamCommit'] != upstream['commit']:
            raise ValueError('Chart resource and upstream commit differ')
        for name, expected in resources['files'].items():
            if len(prepare.safe_path(name).parts) != 1:
                raise ValueError('Non-flat locked chart input')
            item = dict(expected, url='https://raw.githubusercontent.com/OpenCPN/OpenCPN/'
                        + upstream['commit'] + '/data/s57data/' + name)
            cached = cache / ('chart-' + expected['sha256'])
            receipt['archives']['chart/' + name] = fetch(item, cached)
            target = output / 'chart-source/data/s57data' / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(cached, target)
        receipt['files']['chart-source'] = inventory(output / 'chart-source')
        save()
        api.run([sys.executable, str(root / 'tools/generate-xnav-chart-style.py'),
                 '--source', str(output / 'chart-source/data/s57data'),
                 '--output', str(output / 'resources')],
                evidence / 'chart-resource-generation.log', cwd=root, timeout=300)
        manifest = json.loads((output / 'resources/manifest.json').read_text())
        if manifest['upstreamCommit'] != upstream['commit']:
            raise ValueError('Generated chart identity differs')
        for name, expected in manifest['files'].items():
            prepare.safe_path(name)
            if api.record(output / 'resources' / name) != expected:
                raise ValueError('Generated chart resource differs: ' + name)
        if not (output / 'resources/XNavChartResources.h').is_file():
            raise ValueError('Actual generated resource header missing')
        receipt['files']['resources'] = inventory(output / 'resources')
        receipt['status'] = 'staged'
        return receipt
    except Exception as error:
        receipt['status'] = 'failed'
        receipt['error'] = type(error).__name__ + ': ' + str(error)
        raise
    finally:
        # Partial files remain diagnostic evidence, never a successful producer.
        for name in owned:
            if (output / name).is_dir():
                try:
                    receipt['files'][name] = inventory(output / name)
                except Exception as error:
                    receipt.setdefault('inventory_errors', {})[name] = str(error)
        if receipt.get('inventory_errors') and receipt['status'] == 'staged':
            receipt['status'] = 'failed'
            receipt['error'] = 'Final input inventory failed'
            save()
            raise ValueError(receipt['error'])
        save()

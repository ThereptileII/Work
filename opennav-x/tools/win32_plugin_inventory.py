#!/usr/bin/env python3
"""Read-only declared-root PE dependency inventory; never Windows loader proof."""
import argparse
from collections import deque
from contextlib import contextmanager
import hashlib
import json
import os
from pathlib import Path
import stat
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
MAX_FILE = 128 * 1024 * 1024
LIMITS = [
    'Candidate lookup uses importer directory plus explicit search directories; it does not emulate Windows loader order.',
    'No PATH, registry, system directories, API-set map, already-loaded modules, manifests or SxS state is inferred.',
    'Dynamic LoadLibrary/GetProcAddress calls, child processes, export forwarders and import thunks are not inventoried.',
    'This is a filesystem observation, not an atomic snapshot, execution lock, signature/trust decision or runtime enforcement.',
    'Only supplied roots and recursively named candidate files are inspected; unused sibling files are not swept.',
]


def plain_path(value, directory=False):
    if str(value).startswith(('\\\\', '//')):
        raise ValueError('Network/device path refused: ' + str(value))
    path = Path(os.path.abspath(value))
    for node in (*reversed(path.parents), path):
        info = node.lstat()
        if stat.S_ISLNK(info.st_mode) or getattr(info, 'st_file_attributes', 0) & 0x400:
            raise ValueError('Symlink/reparse path refused: ' + str(node))
    info = path.stat()
    if directory and not stat.S_ISDIR(info.st_mode):
        raise ValueError('Not a directory: ' + str(path))
    if not directory and not stat.S_ISREG(info.st_mode):
        raise ValueError('Not a regular file: ' + str(path))
    return path


def stamp(info):
    return (info.st_dev, info.st_ino, info.st_mode, info.st_nlink,
            info.st_size, info.st_mtime_ns, info.st_ctime_ns)


def common_stamp(info):
    # On Windows, path and descriptor stat can disagree on ctime semantics and
    # suffix-derived execute bits. Compare those only within each stat source.
    return (info.st_dev, info.st_ino, stat.S_IFMT(info.st_mode), info.st_nlink,
            info.st_size, info.st_mtime_ns)


def read_file(value, byte_budget=MAX_FILE):
    path = plain_path(value)
    before = path.stat()
    if before.st_size > min(MAX_FILE, byte_budget):
        raise ValueError('Unsupported PE byte size: ' + str(path))
    descriptor = os.open(path, os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0) | getattr(os, 'O_NONBLOCK', 0))
    with os.fdopen(descriptor, 'rb') as stream:
        opened = os.fstat(stream.fileno())
        if not stat.S_ISREG(opened.st_mode) or common_stamp(opened) != common_stamp(before):
            raise ValueError('File changed before read: ' + str(path))
        data = stream.read(MAX_FILE + 1)
        after = os.fstat(stream.fileno())
    if len(data) != before.st_size or stamp(after) != stamp(opened) or stamp(plain_path(path).stat()) != stamp(before):
        raise ValueError('File changed during read: ' + str(path))
    return path, data, stamp(before)


@contextmanager
def compiled_reader(cxx=None):
    """Compile only our byte parser into owned temporary output, never a target."""
    cxx = cxx or os.environ.get('CXX', 'cl' if sys.platform == 'win32' else 'c++')
    with tempfile.TemporaryDirectory(prefix='skager-inventory-reader-') as directory:
        exe = Path(directory) / ('reader.exe' if sys.platform == 'win32' else 'reader')
        source = ROOT / 'tools/win32-pe-import-probe.cpp'
        if Path(cxx).name.lower() in ('cl', 'cl.exe'):
            command = [cxx, '/nologo', '/std:c++17', '/EHsc', '/W4', '/WX',
                       '/I' + str(ROOT / 'src'), str(source), '/Fe:' + str(exe)]
        else:
            command = [cxx, '-std=c++17', '-Wall', '-Wextra', '-Werror', '-pedantic',
                       '-I' + str(ROOT / 'src'), str(source), '-o', str(exe)]
        subprocess.run(command, cwd=directory, check=True, timeout=60, stdout=sys.stderr)

        def parse(data):
            done = subprocess.run([str(exe)], input=data, stdout=subprocess.PIPE,
                                  stderr=subprocess.PIPE, check=True, timeout=15)
            if len(done.stdout) > 8 * 1024 * 1024 or done.stderr:
                raise ValueError('Unexpected parser output')
            result = json.loads(done.stdout)
            if (set(result) != {'parsed', 'error', 'modules'} or
                    type(result['parsed']) is not bool or not isinstance(result['error'], str) or
                    not isinstance(result['modules'], list) or len(result['modules']) > 4096):
                raise ValueError('Malformed parser response')
            for module in result['modules']:
                if (set(module) != {'name', 'delayed'} or not isinstance(module['name'], str) or
                        not module['name'] or type(module['delayed']) is not bool):
                    raise ValueError('Malformed parser module')
            if (result['parsed'] and result['error']) or (not result['parsed'] and (not result['error'] or result['modules'])):
                raise ValueError('Inconsistent parser response')
            return result

        parse.identity = {
            'executableSha256': hashlib.sha256(exe.read_bytes()).hexdigest(),
            'sources': {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                        for path in (source, ROOT / 'src/integration/Win32PeImports.h', Path(__file__).resolve())},
        }
        yield parse


def inventory(roots, search_dirs, parse, *, max_nodes=256, max_edges=16384,
              max_bytes=256 * 1024 * 1024, max_entries=20000):
    if not roots or any(type(limit) is not int or limit < 1 for limit in (max_nodes, max_edges, max_bytes, max_entries)):
        raise ValueError('Nonempty roots and positive integer budgets required')
    roots = sorted(set(str(Path(os.path.abspath(path))) for path in roots))
    if len(roots) > max_nodes:
        raise ValueError('Root count exceeds node budget')
    directories = sorted(set(str(Path(os.path.abspath(path))) for path in search_dirs))
    report = {'schema': 'SKAGER.PluginDependencyInventory.1', 'roots': roots,
              'searchDirectories': directories, 'scope': 'Declared-root static import candidates only',
              'runtimeEnforcement': False, 'loaderClosureAccepted': False,
              'readerIdentity': getattr(parse, 'identity', None),
              'limits': LIMITS[:], 'nodes': [], 'edges': [], 'errors': [],
              'budgets': dict(nodes=max_nodes, edges=max_edges, bytes=max_bytes, entriesPerDirectory=max_entries)}
    listings = {}
    observed = {}
    total_bytes = 0
    queued = set(roots)
    queue = deque(roots)

    def listing(value):
        if value not in listings:
            try:
                directory = plain_path(value, directory=True)
                before = stamp(directory.stat())
                names = []
                with os.scandir(directory) as entries:
                    for entry in entries:
                        if len(names) >= max_entries:
                            raise ValueError('Directory entry budget exceeded: ' + value)
                        names.append(entry.name)
                if stamp(directory.stat()) != before:
                    raise ValueError('Directory changed during listing: ' + value)
                listings[value] = (sorted(names), before, None)
            except (OSError, ValueError) as error:
                listings[value] = ([], None, str(error))
        return listings[value]

    while queue:
        if len(report['nodes']) >= max_nodes:
            report['errors'].append('Node budget exceeded; pending files were not inspected')
            break
        value = queue.popleft()
        node = {'path': value}
        report['nodes'].append(node)
        try:
            path = plain_path(value)
            if path.stat().st_size > max_bytes - total_bytes:
                raise ValueError('Aggregate byte budget exceeded')
            path, data, file_stamp = read_file(path, max_bytes - total_bytes)
            total_bytes += len(data)
            node.update(bytes=len(data), sha256=hashlib.sha256(data).hexdigest())
            observed[value] = file_stamp
            parsed = parse(data)  # Hash and parser receive the SAME captured bytes.
            node['parsed'] = parsed['parsed']
            if not parsed['parsed']:
                node['error'] = parsed['error']
                continue
            for module in parsed['modules']:
                if len(report['edges']) >= max_edges:
                    raise ValueError('Edge budget exceeded; remaining imports were not inventoried')
                edge = {'from': value, 'name': module['name'], 'kind': 'delay' if module['delayed'] else 'normal', 'candidates': []}
                report['edges'].append(edge)
                name = module['name']
                if (name in ('.', '..') or name.endswith((' ', '.')) or
                        any(c in name for c in '/\\:<>"|?*') or any(ord(c) < 32 for c in name)):
                    edge['status'] = 'unsupported-name'
                    continue
                lookup = sorted(set([str(path.parent), *directories]))
                edge['searchedDirectories'] = lookup
                failures = []
                candidates = set()
                for directory in lookup:
                    names, _, error = listing(directory)
                    if error:
                        failures.append(error)
                    else:
                        candidates.update(str(Path(directory) / item) for item in names if item.casefold() == name.casefold())
                edge['candidates'] = sorted(candidates)
                if failures:
                    edge.update(status='unavailable-search', errors=failures)
                elif not candidates:
                    edge['status'] = 'unresolved'
                elif len(candidates) > 1:
                    edge['status'] = 'ambiguous'
                else:
                    edge['status'] = 'unique-candidate'
                # Inventory ALL candidates, including ambiguity; never choose one.
                for candidate in sorted(candidates):
                    if candidate not in queued:
                        if len(queued) >= max_nodes:
                            edge.setdefault('errors', []).append('Node budget prevents candidate inspection')
                            continue
                        queued.add(candidate)
                        queue.append(candidate)
        except (OSError, ValueError, subprocess.SubprocessError) as error:
            node.setdefault('parsed', False)
            node['error'] = str(error)
    for value, before in observed.items():
        try:
            if stamp(plain_path(value).stat()) != before:
                report['errors'].append('File changed after observation: ' + value)
        except (OSError, ValueError) as error:
            report['errors'].append(str(error))
    for value, (_, before, error) in listings.items():
        if error:
            continue
        try:
            if stamp(plain_path(value, directory=True).stat()) != before:
                report['errors'].append('Directory changed after observation: ' + value)
        except (OSError, ValueError) as error:
            report['errors'].append(str(error))
    report['observedBytes'] = total_bytes
    report['pendingPaths'] = list(queue)
    report['status'] = 'inventory-observed' if (not report['errors'] and not queue and
        all(node.get('parsed') and not node.get('error') for node in report['nodes']) and
        all(edge['status'] == 'unique-candidate' and not edge.get('errors') for edge in report['edges'])) else 'inventory-incomplete'
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, action='append', required=True)
    parser.add_argument('--search-dir', type=Path, action='append', default=[])
    parser.add_argument('--cxx')
    args = parser.parse_args()
    with compiled_reader(args.cxx) as reader:
        report = inventory(args.root, args.search_dir, reader)
    json.dump(report, sys.stdout, indent=2, ensure_ascii=True)
    print()
    return 0 if report['status'] == 'inventory-observed' else 1


if __name__ == '__main__':
    raise SystemExit(main())

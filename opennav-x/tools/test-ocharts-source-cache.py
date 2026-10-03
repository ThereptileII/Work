#!/usr/bin/env python3
"""Deterministic duplicate-identity publication and trust checks; no network."""
from contextlib import contextmanager
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('cache_preparation', ROOT / 'tools/prepare-ocharts-adapter.py')
prep = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prep)


def identity(data):
    return {'bytes': len(data),
            'gitBlob': hashlib.sha1(b'blob ' + str(len(data)).encode() + b'\0' + data).hexdigest()}


@contextmanager
def fixture(files, gitlink=None):
    with tempfile.TemporaryDirectory() as raw:
        root = Path(raw)
        lock = {'source': {'repository': 'example/source', 'commit': 'a' * 40, 'files': files},
                'gitlink': {'repository': 'example/link', 'commit': 'b' * 40, 'files': gitlink or {}}}
        (root / 'lock.json').write_text(json.dumps(lock))
        cache = root / 'cache'
        cache.mkdir()
        with patch.object(prep, 'ROOT', root), patch.object(prep, 'LOCK', 'lock.json'):
            yield root / 'output', cache


class CacheTests(unittest.TestCase):
    def test_duplicate_paths_have_one_cache_publication(self):
        data = b'one locked blob\n'
        item = identity(data)
        with fixture({'COPYING': item, 'COPYING.gplv2': item}, {'license.txt': item}) as (output, cache):
            original_exists, original_replace = Path.exists, Path.replace
            publications = []
            guard = threading.Lock()
            def missing(path):
                # Freeze the cache-miss window: every contender sees absence.
                # On the old per-path workers this deterministically attempts a
                # second publication, simulating Windows denial while a reader
                # owns the first published identity. No sleep or scheduler luck.
                return False if path == cache / item['gitBlob'] else original_exists(path)
            def publish(path, target):
                with guard:
                    if target in publications:
                        raise PermissionError('duplicate cache publication during reader ownership')
                    publications.append(target)
                    return original_replace(path, target)
            with patch.object(Path, 'exists', missing), patch.object(Path, 'replace', publish), \
                    patch.object(prep.urllib.request, 'urlopen', side_effect=lambda *a, **k: io.BytesIO(data)) as download:
                prep.fetch_sources(output, cache)
            self.assertEqual(download.call_count, 1)
            self.assertEqual(publications, [cache / item['gitBlob']])
            for path in ('COPYING', 'COPYING.gplv2', 'opencpn-libs/license.txt'):
                self.assertEqual((output / path).read_bytes(), data)
            self.assertEqual((cache / item['gitBlob']).read_bytes(), data)
            self.assertEqual(len(list(cache.iterdir())), 1)

    def test_distinct_blobs_remain_parallel(self):
        data = {'first.txt': b'first', 'second.txt': b'second'}
        barrier = threading.Barrier(2)
        def download(url, **kwargs):
            barrier.wait(timeout=5)
            return io.BytesIO(data[url.rsplit('/', 1)[1]])
        with fixture({name: identity(value) for name, value in data.items()}) as (output, cache):
            with patch.object(prep.urllib.request, 'urlopen', side_effect=download):
                prep.fetch_sources(output, cache)
            for name, value in data.items():
                self.assertEqual((output / name).read_bytes(), value)

    def test_tampered_cache_is_rejected_without_download_or_overwrite(self):
        good, bad = b'valid', b'other'
        item = identity(good)
        with fixture({'COPYING': item, 'COPYING.gplv2': item}) as (output, cache):
            cached = cache / item['gitBlob']
            cached.write_bytes(bad)
            with patch.object(prep.urllib.request, 'urlopen') as download:
                with self.assertRaisesRegex(ValueError, 'Source blob differs'):
                    prep.fetch_sources(output, cache)
            download.assert_not_called()
            self.assertFalse(output.exists())
            self.assertEqual(cached.read_bytes(), bad)

    def test_network_tampering_is_rejected_before_publication(self):
        with fixture({'file.txt': identity(b'valid')}) as (output, cache):
            with patch.object(prep.urllib.request, 'urlopen', return_value=io.BytesIO(b'other')):
                with self.assertRaisesRegex(ValueError, 'Source blob differs'):
                    prep.fetch_sources(output, cache)
            self.assertFalse(output.exists())
            self.assertEqual(list(cache.iterdir()), [])

    def test_every_duplicate_path_expectation_is_checked(self):
        data = b'valid'
        item = identity(data)
        incompatible = dict(item, bytes=len(data) + 1)
        with fixture({'COPYING': item}, {'different.txt': incompatible}) as (output, cache):
            with patch.object(prep.urllib.request, 'urlopen', return_value=io.BytesIO(data)):
                with self.assertRaisesRegex(ValueError, 'Source blob differs: different.txt'):
                    prep.fetch_sources(output, cache)
            self.assertFalse(output.exists())
            self.assertEqual(list(cache.iterdir()), [])

    def test_publication_permission_error_is_not_hidden(self):
        with fixture({'file.txt': identity(b'valid')}) as (output, cache):
            with patch.object(prep.urllib.request, 'urlopen', return_value=io.BytesIO(b'valid')), \
                    patch.object(Path, 'replace', side_effect=PermissionError('external reader')):
                with self.assertRaisesRegex(PermissionError, 'external reader'):
                    prep.fetch_sources(output, cache)
            self.assertFalse(output.exists())


if __name__ == '__main__':
    unittest.main()

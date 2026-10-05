"""Focused staging and browser ownership checks; never launches a browser."""
from contextlib import contextmanager
import importlib.util
import json
import os
from pathlib import Path
import stat
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def module(name, filename):
    spec = importlib.util.spec_from_file_location(name, ROOT/'tools/prototype'/filename)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


render = module('render', 'render.py')
stage = module('stage', 'boat-reference.py')


class BoatReferenceBoundary(unittest.TestCase):
    def test_owned_profile_environment_and_cleanup_on_capture_failure(self):
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp)
            before = {k: os.environ.get(k) for k in ('TEMP','TMP','TMPDIR')}
            events = []
            @contextmanager
            def driver():
                owned = Path(os.environ['TEMP'])
                self.assertEqual(owned.parent, output)
                self.assertTrue(owned.is_dir())
                events.append(owned)
                def launch(**options):
                    self.assertEqual(options, {'headless':True})
                    return SimpleNamespace(close=lambda: events.append('closed'))
                yield SimpleNamespace(chromium=SimpleNamespace(launch=launch))
            args = SimpleNamespace(output=output,browser_channel=None,browser_executable=None,
                                   expected_browser_sha256=None,runtime_receipt=None)
            fake = {'playwright':SimpleNamespace(), 'playwright.sync_api':SimpleNamespace(sync_playwright=driver)}
            with patch.dict(sys.modules,fake), self.assertRaisesRegex(RuntimeError,'capture failed'):
                with render.owned_browser(args, {}):
                    raise RuntimeError('capture failed')
            self.assertEqual(events[1], 'closed')
            self.assertFalse(events[0].exists())
            self.assertEqual({k:os.environ.get(k) for k in before}, before)

    def test_installed_browser_hash_refusal_before_driver_or_profile(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp); executable=root/'msedge.exe';executable.write_bytes(b'never executed')
            output=root/'output';output.mkdir()
            args=SimpleNamespace(output=output,browser_channel='msedge',browser_executable=executable,
                                 expected_browser_sha256='0'*64,runtime_receipt=None)
            def forbidden():
                raise AssertionError('driver must not start')
            fake={'playwright':SimpleNamespace(),'playwright.sync_api':SimpleNamespace(sync_playwright=forbidden)}
            with patch.dict(sys.modules,fake), patch.object(render.platform,'system',return_value='Windows'):
                with self.assertRaisesRegex(ValueError,'identity differs'):
                    with render.owned_browser(args, {}): pass
            self.assertEqual(list(output.iterdir()), [])

    def test_bundle_hash_inventory_and_type_refusals(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp); bundle=root/'source.zip'
            content=b'owned input'
            manifest={'schema':1,'files':{'source/input':stage.digest(content)}}
            def write(kind=stat.S_IFREG, extra=False):
                with zipfile.ZipFile(bundle,'w') as z:
                    for name,data in [('source/input',content),('bundle-manifest.json',json.dumps(manifest).encode())]:
                        info=zipfile.ZipInfo(name);info.external_attr=(kind|0o644)<<16;z.writestr(info,data)
                    if extra:
                        info=zipfile.ZipInfo('source/extra');info.external_attr=(stat.S_IFREG|0o644)<<16;z.writestr(info,b'extra')
            write()
            with self.assertRaisesRegex(ValueError,'SHA256'):
                stage.unpack(bundle,'0'*64,root/'bad-hash')
            self.assertFalse((root/'bad-hash').exists())
            write(stat.S_IFLNK)
            with self.assertRaisesRegex(ValueError,'nonregular'):
                stage.unpack(bundle,stage.digest(bundle.read_bytes()),root/'linked')
            write(extra=True)
            with self.assertRaisesRegex(ValueError,'inventory'):
                stage.unpack(bundle,stage.digest(bundle.read_bytes()),root/'extra')
            write(); expected=stage.digest(bundle.read_bytes())
            self.assertEqual(stage.unpack(bundle,expected,root/'owned'),manifest)
            self.assertEqual((root/'owned/source/input').read_bytes(),content)
            with self.assertRaisesRegex(ValueError,'already exists'):
                stage.unpack(bundle,expected,root/'owned')


if __name__ == '__main__': unittest.main()

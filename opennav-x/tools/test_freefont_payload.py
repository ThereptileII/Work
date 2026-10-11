#!/usr/bin/env python3
"""Check the real vendored evidence, without network access or font rendering."""
import hashlib
import io
from pathlib import Path
import tarfile
import unittest
import freefont_payload as font

ROOT = Path(__file__).resolve().parents[1]


class FreeFontPayloadTests(unittest.TestCase):
    def test_actual_checked_in_payload_and_editable_source(self):
        font.verify_sources(lambda name: (ROOT / name).read_bytes())
        data = (ROOT / 'vendor/freefont-20120503/freefont-src-20120503.tar.gz').read_bytes()
        with tarfile.open(fileobj=io.BytesIO(data), mode='r:gz') as archive:
            source = archive.extractfile('freefont-20120503/sfd/FreeSans.sfd').read()
            self.assertEqual(hashlib.sha256(source).hexdigest(),
                             '9dfad4bb43eb8c9cd366f33336844f1b6ace870ee7b652af7147bf3cc8a3df1e')
            for name in ('COPYING', 'README', 'AUTHORS', 'CREDITS'):
                self.assertEqual(archive.extractfile('freefont-20120503/' + name).read(),
                                 (ROOT / font.NOTICE_ROOT / name).read_bytes())
            for name in ('Makefile', 'sfd/Makefile', 'tools/generate/OpenType', 'BUILDING'):
                self.assertTrue(archive.getmember('freefont-20120503/' + name).isfile())

    def test_every_missing_or_modified_input_fails(self):
        for target in font.SOURCE_FILES:
            def read(name):
                if name == target:
                    raise FileNotFoundError(name)
                return (ROOT / name).read_bytes()
            with self.subTest(path=target, failure='missing'):
                with self.assertRaisesRegex(ValueError, 'Missing GNU FreeFont'):
                    font.verify_sources(read)
            with self.subTest(path=target, failure='modified'):
                with self.assertRaisesRegex(ValueError, 'Pinned GNU FreeFont'):
                    font.verify_sources(lambda name: b'changed' if name == target else (ROOT / name).read_bytes())


if __name__ == '__main__':
    unittest.main()

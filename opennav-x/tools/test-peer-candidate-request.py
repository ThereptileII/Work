#!/usr/bin/env python3
"""Inert strict request schema tests; no API, compilation or candidate execution."""
import importlib.util
import json
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('request', Path(__file__).with_name('read-peer-candidate-request.py'))
request = importlib.util.module_from_spec(spec)
spec.loader.exec_module(request)
VALID = {'run': '123', 'commit': 'a' * 40, 'artifact': '456', 'digest': 'b' * 64}


class RequestTests(unittest.TestCase):
    def test_exact_four_fields(self):
        self.assertEqual(request.parse_request(json.dumps(VALID).encode()), VALID)

    def test_invalid_schema_and_values(self):
        cases = [b'[]', b'null', b'{}', b'\xff', b' ' * 2049,
                 json.dumps(VALID).encode() + b'{}',
                 (json.dumps(VALID)[:-1] + ',"run":"123"}').encode(),
                 (json.dumps(VALID)[:-1] + ',"r\\u0075n":"123"}').encode(),
                 json.dumps({**VALID, 'unknown': 'x'}).encode(),
                 json.dumps({**VALID, 'Run': '123'}).encode()]
        for key in VALID:
            for value in (None, True, 123, {}, [], '', 'x\nrun=evil'):
                cases.append(json.dumps({**VALID, key: value}).encode())
        for key in ('run', 'artifact'):
            for value in ('0', '01', '-1', '1.0', '１', '1' * 21):
                cases.append(json.dumps({**VALID, key: value}).encode())
        for key in ('commit', 'digest'):
            cases.append(json.dumps({**VALID, key: VALID[key].upper()}).encode())
        for data in cases:
            with self.subTest(data=data[:100]), self.assertRaises((ValueError, UnicodeError)):
                request.parse_request(data)


if __name__ == '__main__':
    unittest.main()

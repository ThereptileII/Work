#!/usr/bin/env python3
"""Inert receipt regressions; never launches OpenCPN or accesses Windows profiles."""
import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location('receipt', Path(__file__).with_name('peer-cli-receipt.py'))
RECEIPT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RECEIPT)
EXPECTED = {'schema': 1, 'owner': 'OpenNavX.CI.PeerCli.1', 'status': 'success',
            'commit': 'a' * 40, 'runId': '123', 'runAttempt': '1',
            'job': 'windows-integration', 'sourcesSha256': {'tools/test-peer-cli.py': 'b' * 64},
            'executableSha256': 'c' * 64}


class ReceiptTests(unittest.TestCase):
    def test_exact_positive_receipt(self):
        RECEIPT.verify(copy.deepcopy(EXPECTED), EXPECTED)

    def test_missing_stale_failed_or_changed_receipt(self):
        candidates = [None, {}, {**EXPECTED, 'status': 'failure'}]
        for key in ('commit', 'runId', 'runAttempt', 'job', 'executableSha256', 'sourcesSha256'):
            candidates.append({**EXPECTED, key: 'changed'})
            missing = copy.deepcopy(EXPECTED)
            missing.pop(key)
            candidates.append(missing)
        for receipt in candidates:
            with self.subTest(receipt=receipt), self.assertRaises(ValueError):
                RECEIPT.verify(receipt, EXPECTED)

    def test_capture_publishes_only_after_success(self):
        with tempfile.TemporaryDirectory() as tmp:
            destination = Path(tmp) / 'receipt.json'
            with patch.object(RECEIPT.subprocess, 'run') as run, patch.object(RECEIPT, 'identity', return_value=EXPECTED):
                RECEIPT.capture(Path(tmp) / 'opencpn-cmd.exe', destination, EXPECTED)
                self.assertTrue(run.call_args.kwargs['check'])
                self.assertIn('test-peer-cli.py', str(run.call_args.args[0][1]))
            self.assertEqual(json.loads(destination.read_text()), EXPECTED)
            with patch.object(RECEIPT.subprocess, 'run') as run, self.assertRaises(ValueError):
                RECEIPT.capture(Path(tmp) / 'opencpn-cmd.exe', destination, EXPECTED)
            run.assert_not_called()

    def test_failed_cli_or_post_test_identity_change_leaves_no_receipt(self):
        for failed_cli in (True, False):
            with self.subTest(failed_cli=failed_cli), tempfile.TemporaryDirectory() as tmp:
                destination = Path(tmp) / 'receipt.json'
                error = subprocess.CalledProcessError(1, 'inert-test') if failed_cli else None
                with patch.object(RECEIPT.subprocess, 'run', side_effect=error), patch.object(RECEIPT, 'identity', return_value={}):
                    with self.assertRaises((subprocess.CalledProcessError, ValueError)):
                        RECEIPT.capture(Path(tmp) / 'opencpn-cmd.exe', destination, EXPECTED)
                self.assertFalse(destination.exists())


if __name__ == '__main__':
    unittest.main()

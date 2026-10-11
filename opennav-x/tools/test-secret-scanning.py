#!/usr/bin/env python3
"""Real pinned-scanner regressions; no live secret material or remote services."""
import contextlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import ci_changes
import secret_scanning as scan


class SecretScanTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.runtime = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.runtime.cleanup)
        cls.scanner = scan.load_scanner(Path(cls.runtime.name))

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def scanner_findings(self, files):
        return scan.scan_files(self.scanner, files, self.root / 'scan')

    def test_live_scanner_seeded_self_check(self):
        scan.self_check(self.scanner, self.root / 'seed')

    def test_path_ignore_and_inline_allow_cannot_hide_fixture(self):
        fixture = ('ghp_' + 'Ab3dE7gH9jK2mN4pQ6rS8tU0vW1xY5zC0D2F').encode()
        findings = self.scanner_findings({
            'vendor/node_modules/test.svg': b'token="' + fixture + b'" # gitleaks:allow',
            '.gitleaksignore': b'*',
            '.gitleaks.toml': b'[allowlist]\npaths=[".*"]',
            '.gitignore': b'*',
        })
        self.assertIn({'path': 'vendor/node_modules/test.svg', 'rule': 'github-pat'}, findings)
        # No matching text/value is in the API result or temporary scanner report.
        self.assertNotIn(fixture.decode(), json.dumps(findings))
        self.assertNotIn(fixture, (self.root / 'scan/report.json').read_bytes())
        self.assertTrue(all(set(f) == {'path', 'rule'} for f in findings))

    def test_generic_release_secret_rejected(self):
        secret = ''.join(('9gT4kL8vN2qS6bX0', 'mC5zH7dF1pR3wY0j'))
        findings = self.scanner_findings({'release.env': ('release_api_key="' + secret + '"').encode()})
        self.assertIn({'path': 'release.env', 'rule': 'generic-api-key'}, findings)
        self.assertNotIn(secret, json.dumps(findings))

    def test_binary_headers_cannot_silently_qualify(self):
        fixture = ('ghp_' + 'Ab3dE7gH9jK2mN4pQ6rS8tU0vW1xY5zC0D2F').encode()
        files = {name: header + b'\ntoken="' + fixture + b'"\n'
                 for name, header in (
                     ('public.pdf', b'%PDF-1.7'),
                     ('program.elf', b'\x7fELF' + b'\0' * 128),
                     ('program.exe', b'MZ' + b'\0' * 128),
                 )}
        findings = self.scanner_findings(files)
        for name in files:
            self.assertIn({'path': name, 'rule': 'unsupported-binary-input'}, findings)
        self.assertNotIn(fixture.decode(), json.dumps(findings))
        self.assertNotIn(fixture, (self.root / 'scan/report.json').read_bytes())

        # Exercise the actual qualification decision, not just the scan result.
        self.git_fixture()
        for name, data in files.items():
            (self.root / name).write_bytes(data)
        subprocess.run(['git', '-C', str(self.root), 'add', *files],
                       check=True, capture_output=True)
        with patch.object(scan, 'load_scanner', return_value=self.scanner):
            report = scan.audit_repository(self.root)
        self.assertFalse(report['qualificationPassed'])
        self.assertEqual(report['status'], 'unreviewed-findings')

    def test_unsupported_text_encoding_requires_review(self):
        fixture = 'token="ghp_' + 'Ab3dE7gH9jK2mN4pQ6rS8tU0vW1xY5zC0D2F' + '"'
        # Gitleaks does not normalize these encodings. They must not silently
        # qualify even when its regex rules return no credential finding.
        files = {encoding: fixture.encode(encoding)
                 for encoding in ('utf-16', 'utf-16-le', 'utf-16-be', 'utf-32')}
        files['non-utf8'] = b'public \xff fixture'
        findings = self.scanner_findings(files)
        for name in files:
            self.assertIn({'path': name, 'rule': 'unsupported-text-encoding'}, findings)
        self.assertNotIn(fixture, json.dumps(findings))

    def test_supported_public_checkout_still_qualifies(self):
        self.git_fixture()
        with patch.object(scan, 'load_scanner', return_value=self.scanner):
            report = scan.audit_repository(self.root)
        self.assertTrue(report['qualificationPassed'])
        self.assertEqual(report['status'], 'clean-checkout')

    def test_scanner_archive_tamper_rejected_before_execution(self):
        bad = self.root / 'bad.tar.gz'
        bad.write_bytes(b'not the pinned scanner')
        with self.assertRaisesRegex(scan.ScanError, 'checksum or size'):
            scan.load_scanner(self.root, bad)

    def git_fixture(self):
        subprocess.run(['git', 'init', '-q', str(self.root)], check=True, capture_output=True)
        (self.root / 'tracked.txt').write_text('public fixture')
        (self.root / 'ignored.txt').write_text('untracked fixture')
        subprocess.run(['git', '-C', str(self.root), 'add', 'tracked.txt'], check=True, capture_output=True)

    def test_snapshot_only_tracked_checkout_bytes(self):
        self.git_fixture()
        (self.root / 'tracked.txt').write_text('changed tracked fixture')
        files, links = scan.snapshot(self.root)
        self.assertEqual(files, {'tracked.txt': b'changed tracked fixture'})
        self.assertEqual(links, [])

    def test_missing_tracked_file_fails_closed(self):
        self.git_fixture()
        (self.root / 'tracked.txt').unlink()
        with self.assertRaises(scan.ScanError):
            scan.snapshot(self.root)

    def test_scanner_raw_errors_never_published(self):
        sentinel = 'sensitive sentinel must never escape'
        with patch.object(scan, 'load_scanner', side_effect=RuntimeError(sentinel)):
            with self.assertRaises(scan.ScanError) as caught:
                scan.audit_repository(self.root)
        self.assertNotIn(sentinel, str(caught.exception))

    def test_cli_qualification_fails_on_findings_report_only_is_explicit(self):
        report = {'qualificationPassed': False, 'status': 'unreviewed-findings',
                  'findings': [{'path': 'release.env', 'rule': 'generic-api-key'}]}
        for extra, expected in (([], 1), (['--report-only'], 0)):
            with patch.object(sys, 'argv', ['secret_scanning.py', *extra]), \
                    patch.object(scan, 'audit_repository', return_value=report), \
                    contextlib.redirect_stdout(io.StringIO()) as output:
                self.assertEqual(scan.main(), expected)
            self.assertIn('unreviewed-findings', output.getvalue())

    def test_ci_entrypoint_audits_without_claiming_clean(self):
        report = {'qualificationPassed': False, 'status': 'unreviewed-findings', 'findings': []}
        output = self.root / 'outputs'
        with patch.object(sys, 'argv', ['ci_changes.py', '--github-output', str(output)]), \
                patch.object(scan, 'audit_repository', return_value=report) as audit, \
                patch.object(ci_changes, 'select', return_value=dict.fromkeys(ci_changes.FLAGS, False)), \
                contextlib.redirect_stdout(io.StringIO()) as logs:
            self.assertEqual(ci_changes.main(), 0)
            audit.assert_called_once()
        self.assertIn('unreviewed-findings', logs.getvalue())
        self.assertTrue(output.is_file())

    def test_ci_entrypoint_refuses_outputs_if_scanner_fails(self):
        output = self.root / 'outputs'
        with patch.object(sys, 'argv', ['ci_changes.py', '--github-output', str(output)]), \
                patch.object(scan, 'audit_repository', side_effect=scan.ScanError('Scanner failed.')), \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(ci_changes.main(), 2)
        self.assertFalse(output.exists())


if __name__ == '__main__':
    unittest.main()

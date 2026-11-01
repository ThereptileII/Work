"""Refusals and outputs of skager_release.py on a synthetic (never product) build."""
import hashlib, json, subprocess, sys, tempfile, unittest, zipfile
from pathlib import Path

TOOL = Path(__file__).with_name('skager_release.py')
COMMIT = 'a' * 40


def build(root: Path, version='0.4.0-beta2.10', gates=None, tamper=None):
    root.mkdir(parents=True)
    files = {'SKAGER-Beta2-Setup.exe': b'setup', 'SKAGER-Beta2-source.zip': b'source',
             'SKAGER-Beta2-Release-Notes.md': b'# Notes\n'}
    with zipfile.ZipFile(root / 'SKAGER-Beta2-Portable-Recovery.zip', 'w') as z:
        z.writestr('SKAGER/docs/SOURCE_AND_LICENSES.md', 'notices')
    for name, data in files.items():
        (root / name).write_bytes(data)
    q = {'schemaVersion': 1, 'channel': 'staging', 'commit': COMMIT,
         'gates': gates or {g: 'passed' for g in ('linux', 'windows', 'installer', 'package', 'restart')}}
    (root / 'QUALIFICATION.json').write_text(json.dumps(q))
    names = list(files) + ['SKAGER-Beta2-Portable-Recovery.zip', 'QUALIFICATION.json']
    sha = lambda n: hashlib.sha256((root / n).read_bytes()).hexdigest()
    (root / 'SHA256SUMS.txt').write_text(''.join(f'{sha(n)}  {n}\n' for n in names if n.startswith('SKAGER')))
    names.append('SHA256SUMS.txt')
    manifest = {'commit': COMMIT, 'version': version,
                'files': [{'name': n, 'sha256': sha(n), 'size': (root / n).stat().st_size} for n in names]}
    (root / 'RELEASE.json').write_text(json.dumps(manifest))
    if tamper:
        (root / tamper).write_bytes(b'changed')
    compat = root.parent / 'compat.json'
    compat.write_text(json.dumps({'openNavVersion': version, 'supportedOpenCpn': [
        {'version': '5.12.4', 'arch': 'x86', 'executableSha256': 'b' * 64, 'upstreamCommit': 'c' * 40, 'extra': 1}]}))
    return compat


def prepare(root: Path, compat: Path):
    return subprocess.run([sys.executable, TOOL, 'prepare', '--staging', root, '--out', root.parent / 'out',
                           '--run-id', '7', '--compatibility', compat], capture_output=True, text=True)


class Release(unittest.TestCase):
    def test_a_passed_build_yields_policy_selection_portal_entry_and_public_notes(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d) / 'staging'
            run = prepare(root, build(root))
            self.assertEqual(run.returncode, 0, run.stderr)
            out = Path(d) / 'out'
            policy = json.loads((out / 'policy.json').read_text())
            self.assertEqual(policy['installer']['url'], f'https://beta.skager.app/artifacts/{COMMIT}/SKAGER-Beta2-Setup.exe')
            self.assertNotIn('extra', policy['supportedOpenCpn'][0])
            entry = json.loads((out / 'portal-entry.json').read_text())
            self.assertEqual((entry['id'], entry['location']), ('0-4-0-beta2-10', 'feed'))
            self.assertEqual([a['id'] for a in entry['artifacts']], ['installer', 'recovery', 'checksums', 'source', 'release-notes'])
            notes = (out / 'public-notes.md').read_text()
            self.assertIn('SKAGER-Beta2-Setup.exe', notes)  # its checksum is published, not the file
            self.assertNotIn('actions/runs', notes)

    def test_failed_gate_changed_file_or_version_mismatch_is_refused(self):
        for case in ({'gates': {'linux': 'passed', 'windows': 'failed', 'installer': 'passed', 'package': 'passed', 'restart': 'passed'}},
                     {'tamper': 'SKAGER-Beta2-Setup.exe'}):
            with tempfile.TemporaryDirectory() as d:
                root = Path(d) / 'staging'
                run = prepare(root, build(root, **case))
                self.assertNotEqual(run.returncode, 0)
                self.assertIn('release refused', run.stderr)
        with tempfile.TemporaryDirectory() as d:
            root = Path(d) / 'staging'
            compat = build(root)
            compat.write_text(compat.read_text().replace('beta2.10', 'beta2.9'))
            self.assertIn('compatibility.json', prepare(root, compat).stderr)

    def test_signing_needs_all_three_keys(self):
        run = subprocess.run([sys.executable, TOOL, 'publish', '--repository', 'x', '--assets', 'x', '--out', 'x'],
                             capture_output=True, text=True, env={'SKAGER_UPDATE_TARGETS_KEY': 'k'})
        self.assertIn('signing keys are not configured', run.stderr)

    def test_portal_list_is_newest_first_without_duplicates_and_bounded(self):
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            (d / 'cur.json').write_text(json.dumps([{'id': f'v{i}', 'version': f'0.{i}'} for i in range(20)]))
            (d / 'entry.json').write_text(json.dumps({'id': 'v3', 'version': '0.3-new'}))
            subprocess.run([sys.executable, TOOL, 'portal', '--current', d / 'cur.json', '--entry', d / 'entry.json',
                            '--out', d / 'out.json'], check=True, capture_output=True)
            out = json.loads((d / 'out.json').read_text())
            self.assertEqual(len(out), 16)
            self.assertEqual(out[0]['version'], '0.3-new')
            self.assertEqual([r['id'] for r in out].count('v3'), 1)

    def test_portal_list_keeps_the_apps_and_trims_older_windows_releases(self):
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            apps = [{'id': f'a{i}', 'version': '0.1.0', 'app': app} for i, app in enumerate(['companion-android', 'helm-android', 'raspberry-pi'])]
            (d / 'cur.json').write_text(json.dumps([{'id': f'v{i}', 'version': f'0.{i}'} for i in range(15)] + apps))
            (d / 'entry.json').write_text(json.dumps({'id': 'new', 'version': '1.0'}))
            subprocess.run([sys.executable, TOOL, 'portal', '--current', d / 'cur.json', '--entry', d / 'entry.json',
                            '--out', d / 'out.json'], check=True, capture_output=True)
            out = json.loads((d / 'out.json').read_text())
            self.assertEqual(len(out), 16)
            self.assertEqual(out[0]['id'], 'new')
            self.assertEqual(out[-3:], apps)


if __name__ == '__main__':
    unittest.main()

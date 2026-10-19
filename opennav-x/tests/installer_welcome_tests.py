"""Expected actual version notices do not authorize arbitrary dialog dismissal."""
import importlib.util
import ast
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import types
import unittest
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
spec = importlib.util.spec_from_file_location('installer_welcome', ROOT / 'tools/installer-welcome.py')
policy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(policy)
BETA1, CANDIDATE, EXE = 'a' * 40, 'b' * 40, 'c' * 64


def ownership(version='0.3.0-beta1', commit=BETA1):
    return {'owner': policy.OWNER, 'version': version, 'commit': commit,
            'managedFiles': [{'path': 'app/opencpn.exe', 'sha256': EXE}]}


class ExpectedVersionNotice(unittest.TestCase):
    def test_successor_requires_exact_selected_version(self):
        version = '0.4.0-beta2.1'
        new = policy.version_transition('beta1-to-candidate', EXE,
                ownership(version, CANDIDATE), BETA1, CANDIDATE, version)
        self.assertEqual(new['to'], version)
        with self.assertRaises(ValueError):
            policy.version_transition('beta1-to-candidate', EXE,
                ownership('0.4.0-beta2', CANDIDATE), BETA1, CANDIDATE, version)

    def test_real_sequence_requires_both_version_transitions(self):
        old = policy.version_transition('candidate-to-beta1', EXE, ownership(), BETA1, CANDIDATE)
        new = policy.version_transition('beta1-to-candidate', EXE, ownership('0.4.0-beta2', CANDIDATE), BETA1, CANDIDATE)
        stock = policy.version_transition('candidate-to-stock', policy.STOCK_SHA256, None, BETA1, CANDIDATE)
        self.assertEqual((old['from'], old['to']), ('0.4.0-beta2', '0.3.0-beta1'))
        self.assertEqual((new['from'], new['to']), ('0.3.0-beta1', '0.4.0-beta2'))
        self.assertEqual(stock['to'], 'stock 5.12.4')

    def test_unknown_transition_or_changed_owned_binary_refused(self):
        for transition in ('', 'upgrade', 'candidate-to-unknown', 'Candidate-to-beta1'):
            with self.subTest(transition=transition), self.assertRaises(ValueError):
                policy.version_transition(transition, EXE, ownership(), BETA1, CANDIDATE)
        for field in ('owner', 'version', 'commit'):
            changed = ownership(); changed[field] = 'unexpected'
            with self.subTest(field=field), self.assertRaises(ValueError):
                policy.version_transition('candidate-to-beta1', EXE, changed, BETA1, CANDIDATE)
        for files in ([], [{'path': 'app/opencpn.exe', 'sha256': 'd' * 64}], ownership()['managedFiles'] * 2):
            changed = ownership(); changed['managedFiles'] = files
            with self.subTest(files=files), self.assertRaises(ValueError):
                policy.version_transition('candidate-to-beta1', EXE, changed, BETA1, CANDIDATE)
        with self.assertRaises(ValueError):
            policy.version_transition('beta1-to-candidate', EXE, ownership(), BETA1, CANDIDATE)
        with self.assertRaises(ValueError):
            policy.version_transition('candidate-to-stock', EXE, None, BETA1, CANDIDATE)
        with self.assertRaises(ValueError):
            policy.version_transition('candidate-to-stock', policy.STOCK_SHA256, ownership(), BETA1, CANDIDATE)

    def test_actual_60cd_prior_warning_shape_is_accepted(self):
        # Observed failure: the accepted Beta1 executable presented this warning
        # before its XNav attachment changed the main frame title.
        policy.validate_notice(4872, 3211842,
            [(3211842, 4872, 'Welcome to OpenCPN'), (3539518, 4872, 'OpenCPN 5.12.4')],
            ['Agree', 'Cancel'])

    def test_unrelated_or_ambiguous_modal_refused(self):
        good = [(1, 42, 'Welcome to OpenCPN'), (2, 42, 'OpenCPN 5.12.4')]
        cases = [([], ['Agree', 'Cancel']),
                 ([(1, 41, 'Welcome to OpenCPN'), good[1]], ['Agree', 'Cancel']),
                 ([(1, 42, 'Configure autopilot'), good[1]], ['Agree', 'Cancel']),
                 (good + [(3, 42, 'Unexpected modal')], ['Agree', 'Cancel']),
                 ([good[0], (2, 42, 'Welcome to OpenCPN')], ['Agree', 'Cancel']),
                 (good, ['Agree']), (good, ['Agree', 'Cancel', 'Agree']),
                 (good, ['AUTO', 'Cancel']), (good, ['OK', 'Cancel'])]
        for windows, buttons in cases:
            with self.subTest(windows=windows, buttons=buttons), self.assertRaises(ValueError):
                policy.validate_notice(42, 1, windows, buttons)
        with self.assertRaises(ValueError):
            policy.validate_notice(42, 99, good, ['Agree', 'Cancel'])

    def test_native_harness_retains_three_explicit_version_notices(self):
        source = (ROOT / 'tools/smoke-installer-windows.py').read_text()
        for transition in ('candidate-to-beta1', 'beta1-to-candidate', 'candidate-to-stock'):
            self.assertIn("welcome_transition='" + transition + "'", source)
        self.assertIn("ui.dismiss_native_dialog(dialog,'Agree')", source)
        self.assertIn('QueryFullProcessImageNameW', source)
        self.assertIn('startup.initialized_since(before,log.read_bytes())', source)


class CurrentNavigationNotice(unittest.TestCase):
    def setUp(self):
        self.windows = [(1, 42, 'Welcome to SKAGER'), (2, 42, 'SKAGER / OpenCPN')]
        def control(handle, label, kind, identifier, style=0):
            return dict(handle=handle, label=label, id=identifier, pid=42,
                        parent=1, enabled=True, visible=True, style=style,
                        **{'class': kind})
        self.controls = [control(3, 'Agree', 'wxWindowNR', 5100),
                         control(4, 'Cancel', 'wxWindowNR', 5101),
                         control(5, policy.NAVIGATION_NOTICE, 'Edit', -1, 0x0804)]

    def test_presentation_requires_transition_mode_and_reviewed_source(self):
        self.assertEqual(policy.notice_presentation('beta1-to-candidate', ['--xnav'],
                         policy.SKAGER_SOURCES), 'skager')
        for kind, mode, sources in (
                ('candidate-to-stock', [], None),
                ('candidate-to-beta1', ['--xnav'], None),
                ('beta1-to-candidate', ['--legacy'], policy.SKAGER_SOURCES),
                ('beta1-to-candidate', ['--safe-mode'], policy.SKAGER_SOURCES),
                ('beta1-to-candidate', ['--xnav'], {})):
            self.assertEqual(policy.notice_presentation(kind, mode, sources), 'opencpn')
        for sources in (None, {'unexpected': 'a' * 64},
                        dict(list(policy.SKAGER_SOURCES.items())[:1]),
                        dict.fromkeys(policy.SKAGER_SOURCES, 'a' * 64)):
            with self.subTest(sources=sources), self.assertRaises(ValueError):
                policy.notice_presentation('beta1-to-candidate', ['--xnav'], sources)
        for kind, mode in (('candidate-to-stock', ['--xnav']),
                           ('candidate-to-beta1', []),
                           ('beta1-to-candidate', []),
                           ('beta1-to-candidate', ['--xnav', '--safe-mode']),
                           ('unknown', ['--xnav'])):
            with self.subTest(kind=kind, mode=mode), self.assertRaises(ValueError):
                policy.notice_presentation(kind, mode, policy.SKAGER_SOURCES)

    def test_current_notice_has_exact_frame_actions_and_full_text(self):
        self.assertEqual(policy.notice_title('skager'), 'Welcome to SKAGER')
        self.assertEqual(policy.validate_notice(42, 1, self.windows,
                         ['Agree', 'Cancel'], 'skager'), 2)
        self.assertEqual(policy.skager_actions(42, 1, self.controls), {'Agree': 3, 'Cancel': 4})
        self.controls[-1]['label'] = policy.NAVIGATION_NOTICE.replace('\n', '\r\n')
        self.assertEqual(policy.skager_actions(42, 1, self.controls)['Agree'], 3)

    def test_current_and_historical_notices_cannot_be_interchanged(self):
        old = [(1, 42, 'Welcome to OpenCPN'), (2, 42, 'OpenCPN 5.12.4')]
        for windows, presentation in ((old, 'skager'), (self.windows, 'opencpn'),
                ([self.windows[0], old[1]], 'skager'),
                ([old[0], self.windows[1]], 'opencpn')):
            with self.subTest(windows=windows, presentation=presentation), self.assertRaises(ValueError):
                policy.validate_notice(42, 1, windows, ['Agree', 'Cancel'], presentation)
        with self.assertRaises(ValueError):
            policy.validate_notice(42, 1, self.windows, ['Agree', 'Cancel'], 'unknown')

    def test_wrong_or_missing_cancel_and_action_identity_are_refused(self):
        for index, key, value in ((0, 'id', 5101), (1, 'id', 5100),
                (0, 'class', 'Button'), (1, 'class', 'Button'),
                (0, 'pid', 43), (1, 'parent', 99),
                (0, 'enabled', False), (1, 'visible', False),
                (0, 'handle', 4), (0, 'handle', 0),
                (0, 'label', 'OK'), (1, 'label', 'Close')):
            controls = copy.deepcopy(self.controls); controls[index][key] = value
            with self.subTest(index=index, key=key, value=value), self.assertRaises(ValueError):
                policy.skager_actions(42, 1, controls)
        for controls in (self.controls[:1] + self.controls[2:],
                         self.controls + [self.controls[0]]):
            with self.assertRaises(ValueError): policy.skager_actions(42, 1, controls)

    def test_incomplete_warranty_navigation_or_action_instruction_is_refused(self):
        for text in ('', policy.NAVIGATION_NOTICE.replace('WITHOUT ANY WARRANTY', 'with a warranty'),
                     policy.NAVIGATION_NOTICE.replace('DO NOT rely upon OpenCPN for safety of life or property.', ''),
                     policy.NAVIGATION_NOTICE.replace('"Agree"', '"OK"'),
                     policy.NAVIGATION_NOTICE.replace('or "Cancel" to quit.', '')):
            controls = copy.deepcopy(self.controls); controls[-1]['label'] = text
            with self.subTest(text=text), self.assertRaises(ValueError):
                policy.skager_actions(42, 1, controls)
        for key, value in (('style', 0x0004), ('style', 0x0800),
                           ('pid', 43), ('parent', 99), ('enabled', False), ('visible', False)):
            controls = copy.deepcopy(self.controls); controls[-1][key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                policy.skager_actions(42, 1, controls)
        for controls in (self.controls[:-1], self.controls + [self.controls[-1]]):
            with self.assertRaises(ValueError): policy.skager_actions(42, 1, controls)


class CorrespondingSourceNotice(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.archive = self.root / 'build/developer-preview/SKAGER-Beta2-source.zip'
        self.archive.parent.mkdir(parents=True)
        tree = ast.parse((ROOT / 'tools/smoke-installer-windows.py').read_text())
        function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and
                        n.name == 'candidate_notice_sources')
        namespace = dict(ROOT=self.root, zipfile=zipfile, hashlib=hashlib, welcome=policy,
                         staging_inputs=types.SimpleNamespace(strict_json=json.loads))
        exec(compile(ast.Module(body=[function], type_ignores=[]), '<notice-source>', 'exec'), namespace)
        self.read_sources = namespace['candidate_notice_sources']

    def write_archive(self, files, reference=None, extra=None):
        if reference is None:
            reference = {'productCommit': CANDIDATE, 'files':
                         {p: {'sha256': hashlib.sha256(data).hexdigest()} for p, data in files.items()}}
        with zipfile.ZipFile(self.archive, 'w') as archive:
            archive.writestr('SOURCE_REFERENCE.json', json.dumps(reference))
            for path, data in files.items(): archive.writestr(path, data)
            if extra: archive.writestr(*extra)

    def test_current_exact_source_and_older_source_select_distinct_contracts(self):
        files = {path: (ROOT.parent / path).read_bytes() for path in policy.SKAGER_SOURCES}
        cpp = 'opennav-x/src/ui/StartupNavigationNotice.cpp'
        lf = files[cpp].replace(b'\r\n', b'\n')
        for body, expected in ((lf, policy.SKAGER_SOURCES),
                               (lf.replace(b'\n', b'\r\n'), policy.SKAGER_CRLF_SOURCES)):
            with self.subTest(newlines='CRLF' if b'\r\n' in body else 'LF'):
                self.write_archive(dict(files, **{cpp: body}))
                observed = self.read_sources(CANDIDATE)
                self.assertEqual(observed, expected)
                self.assertEqual(policy.notice_presentation('beta1-to-candidate', ['--xnav'], observed), 'skager')
        self.write_archive({})
        self.assertEqual(policy.notice_presentation('beta1-to-candidate', ['--xnav'],
                         self.read_sources(CANDIDATE)), 'opencpn')

    def test_wrong_candidate_changed_missing_or_ambiguous_source_is_refused(self):
        path = next(iter(policy.SKAGER_SOURCES))
        self.write_archive({path: b'changed'})
        with self.assertRaises(AssertionError): self.read_sources('d' * 40)
        with self.assertRaises(ValueError):
            policy.notice_presentation('beta1-to-candidate', ['--xnav'], self.read_sources(CANDIDATE))
        self.write_archive({path: b'changed'}, {'productCommit': CANDIDATE,
                           'files': {path: {'sha256': 'e' * 64}}})
        with self.assertRaises(AssertionError): self.read_sources(CANDIDATE)
        self.write_archive({}, {'productCommit': CANDIDATE,
                           'files': {path: {'sha256': 'e' * 64}}})
        with self.assertRaises(AssertionError): self.read_sources(CANDIDATE)
        self.write_archive({path: b'changed'}, extra=(path.upper(), b'changed'))
        with self.assertRaises(AssertionError): self.read_sources(CANDIDATE)


if __name__ == '__main__':
    unittest.main()

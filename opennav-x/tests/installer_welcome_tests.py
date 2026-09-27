"""Expected actual version notices do not authorize arbitrary dialog dismissal."""
import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('installer_welcome', ROOT / 'tools/installer-welcome.py')
policy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(policy)
BETA1, CANDIDATE, EXE = 'a' * 40, 'b' * 40, 'c' * 64


def ownership(version='0.3.0-beta1', commit=BETA1):
    return {'owner': policy.OWNER, 'version': version, 'commit': commit,
            'managedFiles': [{'path': 'app/opencpn.exe', 'sha256': EXE}]}


class ExpectedVersionNotice(unittest.TestCase):
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


if __name__ == '__main__':
    unittest.main()

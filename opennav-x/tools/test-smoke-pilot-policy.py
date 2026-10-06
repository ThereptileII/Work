#!/usr/bin/env python3
"""Exercise the actual smoke guard without importing its app/socket entry point."""
import ast
import copy
from pathlib import Path
import unittest
from hardware_output_policy import require_product_output_policy

source = Path(__file__).with_name('smoke-pilot.py')
tree = ast.parse(source.read_text(encoding='utf-8'))
function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and
                n.name == 'require_product_tcp_refusal')
namespace = {'require_product_output_policy': require_product_output_policy}
exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec'), namespace)
guard = namespace['require_product_tcp_refusal']


class ProductTcpPolicy(unittest.TestCase):
    def setUp(self):
        self.report = dict(test_fixtures=False, build_purpose='INSTALLED PRODUCT',
            xnav_hardware_output_policy='manual-commissioning', xnav_manual_control_contract=1,
            runtime=dict(test_fixtures=False, pilot=dict(output_unavailable=False,
                configured_permission=True, enabled=False, serial_session_enabled=False,
                control_capability=False, simulated=False, track_capability=False,
                wind_capability=False)))

    def test_manual_serial_product_still_refuses_tcp(self):
        self.assertEqual(guard(self.report), 'Enable control')

    def test_historical_status_only_keeps_disabled_label(self):
        self.report.update(xnav_hardware_output_policy='status-only', xnav_manual_control_contract=0)
        self.report['runtime']['pilot']['output_unavailable'] = True
        self.assertEqual(guard(self.report), 'Control unavailable')

    def test_each_enable_or_unsupported_capability_is_refused(self):
        for field in ('enabled', 'serial_session_enabled', 'control_capability',
                      'simulated', 'track_capability', 'wind_capability'):
            for value in (True, 0, None):
                with self.subTest(field=field, value=value):
                    report = copy.deepcopy(self.report)
                    report['runtime']['pilot'][field] = value
                    with self.assertRaises(AssertionError): guard(report)

    def test_saved_permission_is_required_to_exercise_refusal(self):
        self.report['runtime']['pilot']['configured_permission'] = False
        with self.assertRaises(AssertionError): guard(self.report)

    def test_fixture_flags_are_never_product_evidence(self):
        for nested in (False, True):
            report = copy.deepcopy(self.report)
            (report['runtime'] if nested else report)['test_fixtures'] = True
            with self.assertRaises(AssertionError): guard(report)

    def test_global_declaration_must_match_versioned_policy(self):
        self.report['runtime']['pilot']['output_unavailable'] = True
        with self.assertRaises(AssertionError): guard(self.report)

    def test_unversioned_or_test_output_policy_is_refused(self):
        for policy, contract in (('manual-commissioning', 0), ('manual-commissioning', True),
                                 ('test-loopback-only', 1), ('unknown', 1)):
            report = copy.deepcopy(self.report)
            report.update(xnav_hardware_output_policy=policy, xnav_manual_control_contract=contract)
            with self.assertRaises(ValueError): guard(report)


if __name__ == '__main__':
    unittest.main()

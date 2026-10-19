#!/usr/bin/env python3
"""Exercise the actual smoke guard without importing its app/socket entry point."""
import ast
import copy
from pathlib import Path
import unittest
from hardware_output_policy import require_product_output_policy

source = Path(__file__).with_name('smoke-pilot.py')
tree = ast.parse(source.read_text(encoding='utf-8'))
functions = [next(n for n in tree.body if isinstance(n, ast.FunctionDef) and
                 n.name == name) for name in
             ('require_product_tcp_refusal', 'require_tcp_mode_capabilities')]
namespace = {'require_product_output_policy': require_product_output_policy}
exec(compile(ast.Module(body=functions, type_ignores=[]), str(source), 'exec'), namespace)
guard = namespace['require_product_tcp_refusal']
mode_guard = namespace['require_tcp_mode_capabilities']


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


class TcpModeCapabilities(unittest.TestCase):
    def setUp(self):
        self.state = dict(simulated=False, control_capability=True,
            track_capability=False, wind_capability=True,
            track_blocker='Fixture has no qualified native route output')

    def test_manual_and_wind_support_do_not_qualify_track(self):
        mode_guard(self.state)

    def test_manual_support_does_not_require_wind_support(self):
        self.state['wind_capability'] = False
        mode_guard(self.state)

    def test_refused_link_offers_neither_route_nor_wind_mode(self):
        self.state.update(control_capability=False, wind_capability=False)
        mode_guard(self.state)

    def test_track_cannot_appear_on_either_control_state(self):
        for control in (False, True):
            with self.subTest(control=control):
                self.state.update(control_capability=control,
                                  wind_capability=control, track_capability=True)
                with self.assertRaises(AssertionError): mode_guard(self.state)

    def test_wind_cannot_appear_on_refused_control_link(self):
        self.state['control_capability'] = False
        with self.assertRaises(AssertionError): mode_guard(self.state)

    def test_capabilities_must_be_actual_booleans(self):
        for field in ('control_capability', 'track_capability', 'wind_capability'):
            for value in (0, 1, None, 'false'):
                with self.subTest(field=field, value=value):
                    state = dict(self.state, **{field: value})
                    with self.assertRaises(AssertionError): mode_guard(state)

    def test_demo_and_missing_track_reason_are_refused(self):
        with self.assertRaises(AssertionError): mode_guard(dict(self.state, simulated=True))
        for blocker in ('', ' ', None):
            with self.subTest(blocker=blocker):
                with self.assertRaises(AssertionError):
                    mode_guard(dict(self.state, track_blocker=blocker))


if __name__ == '__main__':
    unittest.main()

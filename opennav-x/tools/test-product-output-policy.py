#!/usr/bin/env python3
"""Distribution admission is separate from tested physical-command authority."""
import unittest
from hardware_output_policy import require_product_output_policy, require_status_only


class ProductOutputPolicyTests(unittest.TestCase):
    def test_previous_status_only_packages_remain_usable(self):
        require_product_output_policy({'xnav_hardware_output_policy': 'status-only'})
        require_product_output_policy({'xnav_hardware_output_policy': 'status-only',
                                       'xnav_manual_control_contract': 0})

    def test_exact_manual_contract_preserves_truthful_identity(self):
        value = {'xnav_hardware_output_policy': 'manual-commissioning',
                 'xnav_manual_control_contract': 1}
        require_product_output_policy(value)
        with self.assertRaises(ValueError):
            require_status_only(value)

    def test_missing_coerced_unknown_and_contradictory_contracts_refused(self):
        for value in (None, {}, [], True,
                      {'xnav_hardware_output_policy': 'test-loopback-only'},
                      {'xnav_hardware_output_policy': 'physical-output'},
                      {'xnav_hardware_output_policy': 'manual-commissioning'}):
            with self.subTest(value=value), self.assertRaises(ValueError):
                require_product_output_policy(value)
        for version in (None, True, False, '1', 1.0, 0, 2, -1):
            with self.subTest(version=version), self.assertRaises(ValueError):
                require_product_output_policy({'xnav_hardware_output_policy': 'manual-commissioning',
                                               'xnav_manual_control_contract': version})
        for version in (True, False, '0', 0.0, 1):
            with self.subTest(version=version), self.assertRaises(ValueError):
                require_product_output_policy({'xnav_hardware_output_policy': 'status-only',
                                               'xnav_manual_control_contract': version})


if __name__ == '__main__':
    unittest.main()

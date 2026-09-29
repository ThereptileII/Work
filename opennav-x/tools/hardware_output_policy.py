"""Reject unqualified XNav output capability at distribution boundaries."""


def require_status_only(report):
    if (not isinstance(report, dict) or
            report.get('xnav_hardware_output_policy') != 'status-only'):
        raise ValueError('Product requires explicit status-only XNav equipment output policy')

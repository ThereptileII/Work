"""Reject unqualified XNav output capability at distribution boundaries."""


def require_status_only(report):
    if (not isinstance(report, dict) or
            report.get('xnav_hardware_output_policy') != 'status-only'):
        raise ValueError('Product requires explicit status-only XNav equipment output policy')


def require_product_output_policy(report):
    """Admit only named, versioned product capabilities; never enable control.

    Manual commissioning still requires selected serial transport, exact device
    identity, explicit permission and a new disabled-by-default process session.
    Those actual sink gates are qualified independently of this declaration.
    """
    if isinstance(report, dict):
        if (report.get('xnav_hardware_output_policy') == 'status-only' and
                ('xnav_manual_control_contract' not in report or
                 (type(report['xnav_manual_control_contract']) is int and
                  report['xnav_manual_control_contract'] == 0))):
            return
        if (report.get('xnav_hardware_output_policy') == 'manual-commissioning' and
                type(report.get('xnav_manual_control_contract')) is int and
                report['xnav_manual_control_contract'] == 1):
            return
    raise ValueError('Product requires status-only or versioned manual-commissioning output policy')

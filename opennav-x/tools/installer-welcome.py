"""Closed policy for expected version-change notices in disposable installer CI.

No UI, process or profile operation occurs here. Actual notice capture and the
single visible Agree action stay in the native lifecycle harness.
"""
import re

OWNER = 'OpenNavX.Alpha1.SideBySide.1'
STOCK_SHA256 = '7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c'


def version_transition(kind, executable_sha256, ownership, beta1_commit, candidate_commit):
    """Require the exact owned version/executable before expecting a notice."""
    if not re.fullmatch('[a-f0-9]{64}', executable_sha256):
        raise ValueError('Exact executable digest required')
    if kind == 'candidate-to-stock':
        if ownership is not None or executable_sha256 != STOCK_SHA256:
            raise ValueError('Exact restored stock executable required')
        return {'from': '0.4.0-beta2', 'to': 'stock 5.12.4', 'executableSha256': executable_sha256}
    if kind == 'candidate-to-beta1':
        version, commit, previous = '0.3.0-beta1', beta1_commit, '0.4.0-beta2'
    elif kind == 'beta1-to-candidate':
        version, commit, previous = '0.4.0-beta2', candidate_commit, '0.3.0-beta1'
    else:
        raise ValueError('Unreviewed welcome transition')
    if not re.fullmatch('[a-f0-9]{40}', commit):
        raise ValueError('Exact accepted/candidate source commit required')
    if not isinstance(ownership, dict) or ownership.get('owner') != OWNER or ownership.get('version') != version or ownership.get('commit') != commit:
        raise ValueError('Welcome transition does not match owned generation')
    files = [f for f in ownership.get('managedFiles', []) if f.get('path') == 'app/opencpn.exe']
    if len(files) != 1 or files[0].get('sha256') != executable_sha256:
        raise ValueError('Owned executable hash differs or is ambiguous')
    return {'from': previous, 'to': version, 'commit': commit, 'executableSha256': executable_sha256}


def validate_notice(pid, dialog, windows, buttons):
    """Refuse unrelated/ambiguous dialogs rather than dismissing by label."""
    if type(pid) is not int or pid <= 0:
        raise ValueError('Exact launched process required')
    if len(windows) != 2 or any(item[1] != pid for item in windows):
        raise ValueError('Unexpected visible windows in launched process')
    modals = [item for item in windows if item[2] == 'Welcome to OpenCPN']
    frames = [item for item in windows if item[2].startswith('OpenCPN ')]
    if len(modals) != 1 or modals[0][0] != dialog or len(frames) != 1:
        raise ValueError('Expected version-change navigation caution missing')
    if sorted(buttons) != ['Agree', 'Cancel']:
        raise ValueError('Unexpected navigation caution actions')

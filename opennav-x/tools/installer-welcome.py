"""Closed policy for expected version-change notices in disposable installer CI.

No UI, process or profile operation occurs here. Actual notice capture and the
single visible Agree action stay in the native lifecycle harness.
"""
import re
from product_version import validate_product_version

OWNER = 'OpenNavX.Alpha1.SideBySide.1'
STOCK_SHA256 = '7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c'
SKAGER_SOURCES = {
    'opennav-x/src/ui/StartupNavigationNotice.cpp':
        '5609b1e94671cb475abc46709c54d00ccbee40afcf0a876d77def6cbc4a40815',
    'opennav-x/patches/opencpn-5.12.4-startup-presentation.patch':
        'dfa5863b67701dcf4e9e25e59619e96ad2216ed30065e6183762225d41bbb502',
}
# Corresponding source preserves checkout bytes. Native Windows may use CRLF
# for C++, while .gitattributes keeps patches LF. Verify raw manifest bytes
# before accepting either of these exact reviewed representations.
SKAGER_CRLF_SOURCES = dict(SKAGER_SOURCES, **{
    'opennav-x/src/ui/StartupNavigationNotice.cpp':
        '0750845170b2fe1b2f042ba52041abaeebc61d8cde0aae06b775c39c216c9a7a',
})
NAVIGATION_NOTICE = (
    'OpenCPN is distributed in the hope that it will be useful, '
    'but WITHOUT ANY WARRANTY; without even the implied '
    'warranty of MERCHANTABILITY or FITNESS FOR A '
    'PARTICULAR PURPOSE.\n\n'
    'See the GNU General Public License for more details.\n\n'
    'OpenCPN must only be used in conjunction with approved '
    'paper charts and traditional methods of navigation.\n\n'
    'DO NOT rely upon OpenCPN for safety of life or property.\n\n'
    'Please click "Agree" and proceed, or "Cancel" to quit.\n')


def notice_presentation(kind, mode, candidate_sources=None):
    """Choose one exact notice, never accept either caption opportunistically.

    Candidate source bytes come from its checked corresponding-source archive.
    Both absent preserves retained pre-presentation candidates; partial or changed
    modern source requires another explicit helper review.
    """
    if kind == 'candidate-to-stock' and mode == []:
        return 'opencpn'
    if kind == 'candidate-to-beta1' and mode == ['--xnav']:
        return 'opencpn'
    if kind != 'beta1-to-candidate' or mode not in (['--xnav'], ['--legacy'], ['--safe-mode']):
        raise ValueError('Unreviewed notice launch mode or transition')
    if mode != ['--xnav']:
        return 'opencpn'
    if candidate_sources == {}:
        return 'opencpn'
    if candidate_sources not in (SKAGER_SOURCES, SKAGER_CRLF_SOURCES):
        raise ValueError('Candidate navigation notice source differs')
    return 'skager'


def notice_title(presentation):
    if presentation not in ('opencpn', 'skager'):
        raise ValueError('Unknown navigation notice presentation')
    return 'Welcome to SKAGER' if presentation == 'skager' else 'Welcome to OpenCPN'


def version_transition(kind, executable_sha256, ownership, beta1_commit, candidate_commit,
                       candidate_version='0.4.0-beta2'):
    """Require the exact owned version/executable before expecting a notice."""
    validate_product_version(candidate_version)
    if not re.fullmatch('[a-f0-9]{64}', executable_sha256):
        raise ValueError('Exact executable digest required')
    if kind == 'candidate-to-stock':
        if ownership is not None or executable_sha256 != STOCK_SHA256:
            raise ValueError('Exact restored stock executable required')
        return {'from': candidate_version, 'to': 'stock 5.12.4', 'executableSha256': executable_sha256}
    if kind == 'candidate-to-beta1':
        version, commit, previous = '0.3.0-beta1', beta1_commit, candidate_version
    elif kind == 'beta1-to-candidate':
        version, commit, previous = candidate_version, candidate_commit, '0.3.0-beta1'
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


def validate_notice(pid, dialog, windows, buttons, presentation='opencpn'):
    """Refuse unrelated/ambiguous dialogs rather than dismissing by label."""
    if type(pid) is not int or pid <= 0:
        raise ValueError('Exact launched process required')
    if len(windows) != 2 or any(item[1] != pid for item in windows):
        raise ValueError('Unexpected visible windows in launched process')
    title = notice_title(presentation)
    modals = [item for item in windows if item[2] == title]
    frames = [item for item in windows if
              (item[2] == 'SKAGER / OpenCPN' if presentation == 'skager'
               else item[2].startswith('OpenCPN '))]
    if len(modals) != 1 or modals[0][0] != dialog or len(frames) != 1:
        raise ValueError('Expected version-change navigation caution missing')
    if sorted(buttons) != ['Agree', 'Cancel']:
        raise ValueError('Unexpected navigation caution actions')
    return frames[0][0]


def skager_actions(pid, dialog, controls):
    """Validate actual observed custom actions and the complete readonly notice."""
    actions = [c for c in controls if c['class'] in ('Button', 'wxWindowNR')]
    if len(actions) != 2 or {c['label'] for c in actions} != {'Agree', 'Cancel'}:
        raise ValueError('Expected exactly two custom navigation actions')
    expected = {'Agree': 5100, 'Cancel': 5101}
    for c in actions:
        if (c['class'] != 'wxWindowNR' or c['id'] != expected[c['label']] or
                c['pid'] != pid or c['parent'] != dialog or
                c['enabled'] is not True or c['visible'] is not True):
            raise ValueError('Navigation action identity or availability differs')
    if len({c['handle'] for c in actions}) != 2 or any(not c['handle'] for c in actions):
        raise ValueError('Ambiguous navigation action handles')
    bodies = [c for c in controls if c['class'] == 'Edit']
    if len(bodies) != 1:
        raise ValueError('Expected one complete navigation notice text surface')
    body = bodies[0]
    if (body['pid'] != pid or body['parent'] != dialog or
            body['visible'] is not True or body['enabled'] is not True or
            body['style'] & 0x0804 != 0x0804 or  # ES_READONLY | ES_MULTILINE
            body['label'].replace('\r\n', '\n') != NAVIGATION_NOTICE):
        raise ValueError('Complete readonly navigation safety instruction differs')
    return {c['label']: c['handle'] for c in actions}

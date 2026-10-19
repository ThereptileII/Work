#!/usr/bin/env python3
"""Supply the pinned Poedit prerequisite on disposable Windows CI, without Chocolatey.

The existing SDK/gettext identity inputs and native verification stay unchanged.
This preparation is neither an application build nor product/boat acceptance.
"""
import argparse
import json
import os
from pathlib import Path
import sys
import tempfile

import windows_gettext as gettext

# Exact package source for Poedit 3.9.1, including installer arguments/exit code:
# https://github.com/nykula/chocolatey-packages/blob/17164f32f21671a6d9c17bebc183eb1c15913859/automatic/poedit/tools/chocolateyInstall.ps1
SOURCE_COMMIT = '17164f32f21671a6d9c17bebc183eb1c15913859'
URL = 'https://download.poedit.com/Poedit-3.9.1-setup.exe'
INSTALLER = {'bytes': 20038048,
             'sha256': 'a6cb88246834e6e54224b923a839c9566580cb5d4be95d4eba1d18bd34e8ecc9'}
ARGUMENTS = ('/verysilent', '/norestart')


def require_ci():
    if (sys.platform != 'win32' or os.environ.get('GITHUB_ACTIONS') != 'true' or
            os.environ.get('RUNNER_OS') != 'Windows' or os.environ.get('RUNNER_ARCH') != 'X64'):
        raise RuntimeError('Poedit preparation requires disposable Windows x64 CI; never the boat')


def verify_installer(path):
    gettext.plain_file(path)
    if path.stat().st_size != INSTALLER['bytes'] or gettext.identity(path) != INSTALLER:
        raise RuntimeError('Pinned Poedit installer size/checksum mismatch')


def prepare(evidence):
    require_ci()
    evidence = evidence.absolute()
    # No old receipt, partial download or failed installation is accepted.
    evidence.mkdir(parents=True, exist_ok=False)
    state = {'schemaVersion': 1, 'status': 'failed', 'phase': 'initial-probe',
             'url': URL, 'packageSourceCommit': SOURCE_COMMIT,
             'expectedInstaller': INSTALLER, 'arguments': list(ARGUMENTS),
             'nativeProductAcceptance': False}
    try:
        selected, failures = gettext.discover(evidence / 'initial')
        state['initialProbeFailures'] = failures
        if selected:
            state['origin'] = 'preinstalled-known-Poedit-path'
        else:
            state['phase'] = 'download'
            curl = Path(os.environ['SystemRoot']) / 'System32/curl.exe'
            gettext.plain_file(curl)
            state['downloadTool'] = {'path': str(curl), **gettext.identity(curl)}
            # Keep logs/identities, not a copy of the vendor installer in the
            # uploaded evidence. This directory owns only this invocation's file.
            with tempfile.TemporaryDirectory(prefix='skager-poedit-') as temporary:
                installer = Path(temporary).absolute() / 'Poedit-3.9.1-setup.exe'
                result, _ = gettext.native([
                    curl, '--fail', '--location', '--silent', '--show-error',
                    '--proto', '=https', '--proto-redir', '=https',
                    '--connect-timeout', '20', '--max-time', '120',
                    '--max-filesize', str(INSTALLER['bytes']),
                    '--output', installer, URL], 150, evidence / 'download')
                state['download'] = result
                if result['exitCode'] != 0:
                    raise RuntimeError('Pinned Poedit download failed')
                verify_installer(installer)
                state['installer'] = {'path': str(installer), **gettext.identity(installer)}
                state['phase'] = 'install'
                result, _ = gettext.native([installer, *ARGUMENTS], 180, evidence / 'install')
                state['install'] = result
                # Do not probe or accept leftover tools from a failed/timed-out run.
                if result['exitCode'] != 0:
                    raise RuntimeError('Pinned Poedit installer returned a nonzero exit code')
                verify_installer(installer)
            state['origin'] = 'verified-official-Poedit-install'
        state['phase'] = 'native-verification'
        # Never fall back to Chocolatey; the unchanged helper must find both
        # actual known-path tools and record fresh native version/hash probes.
        gettext.ensure(evidence / 'gettext.json', allow_install=False)
        state['gettextReceipt'] = gettext.identity(evidence / 'gettext.json')
        state.update(status='passed', phase='complete')
        return state
    except Exception as error:
        state['error'] = str(error)
        raise
    finally:
        (evidence / 'receipt.json').write_text(json.dumps(state, indent=2) + '\n', encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(prepare(args.evidence), indent=2))


if __name__ == '__main__':
    main()

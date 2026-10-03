#!/usr/bin/env python3
"""Early Poedit gettext prerequisite; compilation/runtime acceptance is separate.

Only explicit --allow-install permits acquisition, through the existing reviewed
Chocolatey provider and observed Poedit version. PATH is never a tool source.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import stat
import sys
import time

VERSION = '3.9.1'
PROVIDER = 'https://community.chocolatey.org/api/v2/'
TOOLS = ('msgfmt.exe', 'msgmerge.exe')


class PrerequisiteDeadline(RuntimeError):
    pass


def identity(path):
    data = path.read_bytes()
    return {'bytes':len(data), 'sha256':hashlib.sha256(data).hexdigest()}


def native(command, timeout, log):
    """Bound runtime; retain streams and reject oversized output after capture.

    communicate() buffers output: the 1 MiB check is a post-capture rejection,
    not a streaming memory bound. These are selected trusted host-tool probes.
    A timeout targets only this invocation's owned process tree.
    """
    process = subprocess.Popen([str(v) for v in command], stdout=subprocess.PIPE,
        stderr=subprocess.PIPE, stdin=subprocess.DEVNULL,
        creationflags=subprocess.CREATE_NEW_PROCESS_GROUP if sys.platform == 'win32' else 0)
    timed_out = False
    cleanup = None
    try:
        stdout, stderr = process.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        timed_out = True
        try:
            if sys.platform == 'win32' and process.poll() is None:
                killer = Path(os.environ['SystemRoot'])/'System32/taskkill.exe'
                killed = subprocess.run([str(killer), '/PID', str(process.pid), '/T', '/F'],
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=15, check=False)
                cleanup = {'treeExitCode':killed.returncode}
        except (OSError, subprocess.TimeoutExpired) as error:
            cleanup = {'treeError':str(error)}
        finally:
            if process.poll() is None:
                process.kill()
        try:
            stdout, stderr = process.communicate(timeout=10)
        except subprocess.TimeoutExpired as error:
            # A descendant may still hold a pipe if OS tree cleanup failed.
            # Close our handles and fail; never start another installer then.
            stdout, stderr = error.output or b'', error.stderr or b''
            process.stdout.close(); process.stderr.close()
            process.wait(timeout=5)
            cleanup = {**(cleanup or {}), 'pipesClosedAfterCleanupTimeout':True}
    log.parent.mkdir(parents=True, exist_ok=True)
    # Exact separate streams are retained, including nonzero/timeout attempts.
    log.with_suffix('.stdout.log').write_bytes(stdout)
    log.with_suffix('.stderr.log').write_bytes(stderr)
    result = {'command':[str(v) for v in command], 'exitCode':process.returncode,
              'timedOut':timed_out, 'timeoutSeconds':timeout, 'ownedProcessCleanup':cleanup,
              'stdout':identity(log.with_suffix('.stdout.log')),
              'stderr':identity(log.with_suffix('.stderr.log'))}
    log.with_suffix('.json').write_text(json.dumps(result, indent=2)+'\n')
    if timed_out:
        raise PrerequisiteDeadline('Gettext prerequisite subprocess timed out; owned-process cleanup recorded, no retry')
    if len(stdout)+len(stderr) > 1024*1024:
        raise RuntimeError('Gettext prerequisite output exceeded 1 MiB')
    return result, stdout.decode('utf-8',errors='replace')


def directories():
    values = []
    for key in ('ProgramFiles', 'ProgramFiles(x86)'):
        value = os.environ.get(key)
        if not value or not Path(value).is_absolute():
            continue
        path = Path(value)/'Poedit/Gettexttools/bin'
        if path not in values:
            values.append(path)
    return values


def plain_file(path):
    """Reject actual links/reparse components, not Windows 8.3 spelling aliases."""
    if not path.is_absolute() or '..' in path.parts:
        raise RuntimeError('Required tool path is not absolute and plain: '+str(path))
    for component in (path, *path.parents):
        info = component.lstat()
        if stat.S_ISLNK(info.st_mode) or getattr(info,'st_file_attributes',0) & stat.FILE_ATTRIBUTE_REPARSE_POINT:
            raise RuntimeError('Required tool path contains a link/reparse point: '+str(component))
        if component == path and not stat.S_ISREG(info.st_mode):
            raise RuntimeError('Required tool is not a regular file: '+str(path))


def tool_fact(path, logs):
    # Windows resolves legitimate 8.3 aliases to long names. Inspect actual
    # filesystem components instead of treating canonical spelling as a link.
    plain_file(path)
    before = identity(path)
    result, output = native([path, '--version'], 30, logs/path.stem)
    lines = [line.strip() for line in output.splitlines() if line.strip()]
    line = lines[0] if lines else ''
    if result['exitCode'] != 0 or not re.fullmatch(re.escape(path.stem)+r' \(GNU gettext-tools\) [0-9]+(?:\.[0-9]+)+(?:[-.][A-Za-z0-9]+)*', line):
        raise RuntimeError('Poedit '+path.name+' version probe failed: '+line)
    if identity(path) != before:
        raise RuntimeError('Poedit tool changed during its version probe')
    return {'path':str(path.resolve()), **before, 'versionLine':line}


def discover(logs):
    failures = []
    for index, folder in enumerate(directories()):
        try:
            facts = {name:tool_fact(folder/name, logs/str(index)) for name in TOOLS}
            return {'directory':str(folder.resolve()), 'tools':facts}, failures
        except PrerequisiteDeadline:
            raise  # Never start an installer after a timed-out tool probe.
        except (RuntimeError, OSError) as error:
            failures.append(str(error))
    return None, failures


def chocolatey():
    # Match the existing host package manager, never a PATH lookalike.
    root = os.environ.get('ChocolateyInstall')
    if not root:
        root = str(Path(os.environ.get('ProgramData', r'C:\ProgramData'))/'chocolatey')
    path = Path(root)/'bin/choco.exe'
    plain_file(path)
    return path


def ensure(receipt, allow_install=False):
    logs = receipt.parent/(receipt.stem+'-logs')
    state = {'schemaVersion':1, 'status':'failed', 'provider':PROVIDER,
             'requestedPoeditVersion':VERSION, 'allowInstall':allow_install,
             'attempts':[], 'nativeProductAcceptance':False}
    def save():
        receipt.parent.mkdir(parents=True, exist_ok=True)
        receipt.write_text(json.dumps(state, indent=2)+'\n')
    try:
        selected, failures = discover(logs/'initial')
        state['initialProbeFailures'] = failures
        if selected:
            state.update(selected);state['origin']='preinstalled-known-Poedit-path'
        else:
            if not allow_install:
                raise RuntimeError('Usable Poedit msgfmt and msgmerge absent; installation was not authorized')
            choco = chocolatey();state['packageManager']={'path':str(choco),**identity(choco)}
            for attempt in range(1,4):
                command = [choco, 'install', 'poedit', '--yes', '--no-progress', '--limit-output',
                           '--version='+VERSION, '--source='+PROVIDER, '--execution-timeout=120']
                result, _ = native(command, 180, logs/('install-'+str(attempt)))
                state['attempts'].append(result);save()
                # A failed package operation can leave files behind. Never
                # accept them from that attempt just because a later probe works.
                if result['exitCode'] == 0:
                    selected, failures = discover(logs/('after-install-'+str(attempt)))
                    state['attempts'][-1]['probeFailures'] = failures
                    if selected:
                        state.update(selected);state['origin']='verified-after-Chocolatey-install';break
                if attempt < 3:time.sleep(attempt*5)
            else:
                raise RuntimeError('Poedit acquisition failed after three bounded attempts; stopping before dependency builds')
        state['status']='passed';save();return state
    except Exception as error:
        state['error']=str(error);save();raise


def verify(receipt):
    state=json.loads(receipt.read_text())
    if state.get('schemaVersion') != 1 or state.get('status') != 'passed' or set(state.get('tools',{})) != set(TOOLS):
        raise RuntimeError('Gettext prerequisite receipt is incomplete')
    known=[str(p.resolve()) for p in directories()]
    if state['directory'] not in known:
        raise RuntimeError('Gettext receipt selects an untrusted directory')
    for name in TOOLS:
        path=Path(state['directory'])/name
        actual=tool_fact(path,receipt.parent/(receipt.stem+'-logs')/'verify')
        if actual != state['tools'][name]:
            raise RuntimeError('Poedit tool identity changed after prerequisite validation')
    return state


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=('ensure','verify'))
    parser.add_argument('--receipt',type=Path,required=True)
    parser.add_argument('--allow-install',action='store_true')
    args=parser.parse_args()
    if sys.platform != 'win32':raise SystemExit('Poedit prerequisite entry point requires Windows')
    if args.mode=='verify' and args.allow_install:raise SystemExit('Verification never installs packages')
    result=ensure(args.receipt,args.allow_install) if args.mode=='ensure' else verify(args.receipt)
    print('Verified usable Poedit msgfmt/msgmerge: '+result['directory'])

if __name__=='__main__':main()

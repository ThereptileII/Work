#!/usr/bin/env python3
"""Explicit first protected B0 enrollment and qualification on a disposable host.

Approval, protected operator inputs, stock OpenCPN and the user profile exist
before this program starts. A refused or uncertain operation is never retried.
"""
import argparse
from ctypes import wintypes
import importlib.util
import json
import os
from pathlib import Path
import re
import secrets
import subprocess
import sys

import updater_package as updater


def phase_driver():
    spec = importlib.util.spec_from_file_location(
        '_installed_protected_phase', Path(__file__).with_name('test-installed-protected-updater-windows.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


phase = phase_driver()
FAILURE = 'Initial protected B0 admission refused; preserve installed state and evidence. No automatic retry or cleanup.'
OPT_IN = 'SCRUM-311-disposable-initial-protected-b0'
STOCK = ('5.12.4', 'x86', '7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c',
         '37fd0cddb7334fe489e9f18aa163977a9c5c84f7')


def require(condition):
    if not condition:
        raise ValueError(FAILURE)


def approved_directory(value):
    require(isinstance(value, str))
    path = Path(value)
    require(path.is_absolute() and os.path.normpath(str(path)) == str(path))
    updater._plain(path)
    before = path.stat()
    require(path.is_dir() and not path.is_symlink())
    return path, (before.st_dev, before.st_ino)


def prepare(path, sha, approved_request, approved_policy):
    phase.pin(sha)
    phase.pin(approved_request)
    phase.pin(approved_policy)
    path, identity = phase.approved_file({'path': str(path), 'sha256': sha}, 64 << 10)
    retained = [(path, identity, sha)]
    declaration = phase.strict(path.read_bytes())
    phase.keys(declaration, ('schema', 'b0', 'operatorKit', 'clientRequest', 'brokerBootstrapPolicy', 'approvedBaseline',
                             'trustConfig', 'bootstrapCache', 'artifactDirectory', 'preserveSettings'))
    require(type(declaration['schema']) is int and declaration['schema'] == 1)
    b0 = phase.fixture(declaration['b0'], retained)
    request_record = declaration['clientRequest']
    phase.keys(request_record, ('path', 'sha256'))
    require(request_record['sha256'] == approved_request)
    request_path, identity = phase.approved_file(request_record, 1 << 20)
    retained.append((request_path, identity, approved_request))
    request = phase.strict(request_path.read_bytes())
    phase.keys(request, ('schema', 'purpose', 'clientSid', 'installed', 'stock', 'trustConfiguration', 'trustConfigSha256'))
    phase.keys(request['installed'], ('version', 'commit', 'generation', 'packageSha256', 'productBuildSha256',
                                       'executableSha256', 'inventorySha256'))
    phase.keys(request['stock'], ('path', 'version', 'arch', 'executableSha256', 'upstreamCommit'))
    installed, stock = request['installed'], request['stock']
    require(type(request['schema']) is int and request['schema'] == 1 and request['purpose'] == 'client-registration' and
            request['trustConfigSha256'] == b0['trustSha256'] and request['trustConfiguration'] == b0['trust'] and
            installed['version'] == b0['version'] and installed['commit'] == b0['commit'] and
            installed['packageSha256'] == b0['packageSha256'] and installed['productBuildSha256'] == b0['productSha256'] and
            installed['executableSha256'] == b0['product']['executable_sha256'] and
            installed['inventorySha256'] == b0['inventorySha256'] and
            (stock['version'], stock['arch'], stock['executableSha256'], stock['upstreamCommit']) == STOCK)
    phase.pin(installed['generation'], 32)
    require(re.fullmatch(r'S-1-5-21-[1-9][0-9]*-[1-9][0-9]*-[1-9][0-9]*-[1-9][0-9]*', request['clientSid']))
    stock_path, identity = phase.approved_file({'path': stock['path'], 'sha256': stock['executableSha256']}, 1 << 30)
    retained.append((stock_path, identity, stock['executableSha256']))
    artifact, artifact_id = approved_directory(declaration['artifactDirectory'])
    cache, cache_id = approved_directory(declaration['bootstrapCache'])
    require(cache != artifact and cache not in artifact.parents and artifact not in cache.parents)
    baseline_record = declaration['approvedBaseline']
    baseline_path, identity = phase.approved_file(baseline_record, 4 << 30)
    require(baseline_path == artifact / 'approved-installed-baseline.zip' and
            baseline_record['sha256'] == declaration['b0']['recovery']['sha256'])
    retained.append((baseline_path, identity, baseline_record['sha256']))
    trust_path, identity = phase.approved_file(declaration['trustConfig'], 576 << 10)
    require(declaration['trustConfig']['sha256'] == b0['trustSha256'] and phase.strict(trust_path.read_bytes()) == b0['trust'])
    retained.append((trust_path, identity, b0['trustSha256']))
    kit_path, identity = phase.approved_file(declaration['operatorKit'], 768 << 20)
    retained.append((kit_path, identity, declaration['operatorKit']['sha256']))
    policy_record = declaration['brokerBootstrapPolicy']
    phase.keys(policy_record, ('path', 'sha256'))
    require(policy_record['sha256'] == approved_policy)
    policy_path, identity = phase.approved_file(policy_record, 32 << 10)
    retained.append((policy_path, identity, approved_policy))
    policy = phase.strict(policy_path.read_bytes())
    phase.keys(policy, ('schema', 'product', 'purpose', 'channel', 'version', 'commit', 'architecture',
                        'serviceName', 'serviceAccount', 'codeLocation', 'dataLocation', 'trustConfigSha256',
                        'broker', 'provisioner', 'correspondingSource', 'notices'))
    for name in ('broker', 'provisioner', 'correspondingSource', 'notices'):
        phase.keys(policy[name], ('path', 'url', 'sha256', 'bytes'))
    require(type(policy['schema']) is int and policy['schema'] == 1 and policy['product'] == 'skager' and
            policy['purpose'] == 'machine-update-broker-enrollment' and policy['channel'] == 'beta' and
            policy['version'] == b0['version'] and policy['commit'] == b0['commit'] and
            policy['trustConfigSha256'] == b0['trustSha256'])
    settings = declaration['preserveSettings']
    require(isinstance(settings, list) and 1 <= len(settings) <= 128)
    seen = set()
    for item in settings:
        phase.keys(item, ('section', 'key'))
        require(all(isinstance(value, str) and 0 < len(value) <= 256 and not any(c in value for c in '\r\n\x00')
                    for value in item.values()))
        pair = item['section'], item['key']
        require(pair not in seen)
        seen.add(pair)
    return dict(b0=b0, request=request, kit=kit_path, brokerPolicy=policy, retained=retained, settings=settings,
                requestPath=request_path, trustPath=trust_path, cache=cache, artifact=artifact,
                directories=((cache, cache_id), (artifact, artifact_id)))


def unchanged(prepared):
    phase.unchanged(prepared)
    for path, identity in prepared['directories']:
        updater._plain(path)
        current = path.stat()
        require(path.is_dir() and (current.st_dev, current.st_ino) == identity)


class BeforeEnrollment(phase.Native):
    """Reuse the existing actual-token and profile observer before a broker exists."""
    def __init__(self, prepared):
        self.w, self.p, self.ui, self.handles = wintypes, prepared, phase.module('windows-ui'), []
        self.profile_root = self.folder('3eb685db-65f9-4cf6-a03a-e3ef65729f3d') / 'opencpn'
        self.sid = self.actual_sid()
        require(self.sid == prepared['request']['clientSid'])


def enrollment_command(provisioner, prepared, policy_sha, provisioner_sha, request_sha):
    return [str(provisioner), '--trust-config=' + str(prepared['trustPath']),
            '--bootstrap-cache=' + str(prepared['cache']), '--artifact-directory=' + str(prepared['artifact']),
            '--policy-sha256=' + policy_sha, '--provisioner-sha256=' + provisioner_sha,
            '--trust-config-sha256=' + prepared['b0']['trustSha256'],
            '--client-registration-request=' + str(prepared['requestPath']),
            '--client-registration-request-sha256=' + request_sha, '--activate-startup=true']


def enroll_once(command, popen=subprocess.Popen):
    # wait(timeout) leaves the original process alone. Never use subprocess.run:
    # it kills the child on TimeoutExpired, obscuring a possible native effect.
    process = popen(command, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL)
    try:
        result = process.wait(timeout=1800)
    except subprocess.TimeoutExpired as error:
        raise ValueError(FAILURE) from error
    require(result == 0)


def qualified_b0(history, prepared):
    installed = prepared['request']['installed']
    require(history['state'] == 'selected-history' and history['generation'] == installed['generation'] and
            history['version'] == installed['version'] and history['commit'] == installed['commit'] and
            history['ancestors'] == [] and history.get('nextPage') is None)
    for field in ('completionSha256', 'selectedHeadSha256', 'qualificationSha256',
                  'qualificationClosedSha256', 'inventorySha256'):
        phase.pin(history[field])
    return {field: history[field] for field in ('generation', 'version', 'commit', 'completionSha256',
                                                'selectedHeadSha256', 'qualificationSha256',
                                                'qualificationClosedSha256', 'inventorySha256')}


def run(prepared, pins, output, native_factory=phase.Native, preflight_factory=BeforeEnrollment,
        stage=phase.stage_operator, publish=phase.publication, enroll=enroll_once):
    native, before = None, None
    try:
        unchanged(prepared)
        before = preflight_factory(prepared)
        profile = before.profile()
        require(before.sid == prepared['request']['clientSid'])
        publish(prepared, 'no-update', pins['timestamp'])
        unchanged(prepared)
        # The acquisition parent is a separately provisioned protected operator
        # input. Retain the new exact stage on every failure; do not stage an
        # elevated executable in the ordinary evidence directory.
        operator = prepared['artifact'] / ('b0-verified-operator-' + secrets.token_hex(16))
        operator.mkdir()
        provisioner = stage(prepared, operator)
        require(prepared['provisionerSha256'] == pins['provisioner'] and
                prepared['brokerSha256'] == pins['broker'] and
                prepared['brokerPolicy']['provisioner']['sha256'] == pins['provisioner'] and
                prepared['brokerPolicy']['broker']['sha256'] == pins['broker'])
        before.pin_image(provisioner, pins['provisioner'])
        unchanged(prepared)
        enroll(enrollment_command(provisioner, prepared, pins['policy'], pins['provisioner'], pins['request']))
        unchanged(prepared)
        native = native_factory(prepared, provisioner)
        require(native.profile() == profile)
        native.start_and_close('no-update', None)
        selected = qualified_b0(native.history(), prepared)
        require(native.profile() == profile)
        publish(prepared, 'no-update', pins['timestamp'])
        unchanged(prepared)
        native.close()
        native = None
        return {'schema': 1, 'state': 'first-b0-qualified', 'nativeInitialB0Acceptance': True,
                'nativeProductAcceptance': False, 'selected': selected,
                'preservation': profile}
    finally:
        try:
            if native is not None:
                native.close()
        finally:
            if before is not None:
                before.close()


class Once(argparse.Action):
    def __call__(self, parser, namespace, values, option_string=None):
        seen = getattr(namespace, '_seen', set())
        require(self.dest not in seen)
        namespace._seen = seen | {self.dest}
        setattr(namespace, self.dest, values)


def main(argv=None):
    class Parser(argparse.ArgumentParser):
        def error(self, message):
            raise ValueError(FAILURE)
    parser = Parser(description=__doc__, allow_abbrev=False)
    for name in ('declaration', 'declaration-sha256', 'approved-client-request-sha256',
                 'approved-bootstrap-policy-sha256', 'approved-provisioner-sha256',
                 'approved-broker-sha256', 'timestamp-sha256', 'evidence', 'disposable-opt-in'):
        parser.add_argument('--' + name, required=True, action=Once)
    report = {'schema': 1, 'state': 'refused', 'nativeInitialB0Acceptance': False,
              'nativeProductAcceptance': False}
    output = None
    result_code = 1
    try:
        args = parser.parse_args(argv)
        require(sys.platform == 'win32' and args.disposable_opt_in == OPT_IN and
                os.environ.get('GITHUB_ACTIONS') == 'true' and os.environ.get('RUNNER_OS') == 'Windows' and
                os.environ.get('RUNNER_ENVIRONMENT') == 'github-hosted' and
                os.environ.get('GITHUB_REPOSITORY') == 'ThereptileII/Work')
        pins = dict(request=phase.pin(args.approved_client_request_sha256),
                    policy=phase.pin(args.approved_bootstrap_policy_sha256),
                    provisioner=phase.pin(args.approved_provisioner_sha256),
                    broker=phase.pin(args.approved_broker_sha256),
                    timestamp=phase.pin(args.timestamp_sha256))
        prepared = prepare(Path(args.declaration), args.declaration_sha256, pins['request'], pins['policy'])
        evidence = Path(args.evidence)
        require(evidence.is_absolute() and not evidence.exists() and not evidence.is_symlink())
        updater._plain(evidence.parent)
        require(all(evidence != p and p not in evidence.parents for p, _, _ in prepared['retained']) and
                all(evidence != p and p not in evidence.parents and evidence not in p.parents for p, _ in prepared['directories']))
        require('skager' not in {part.casefold() for part in evidence.parts} and
                'opencpn' not in {part.casefold() for part in evidence.parts})
        evidence.mkdir()
        output = evidence / 'result.json'
        report = run(prepared, pins, output)
        report['declarationSha256'] = args.declaration_sha256
        report['harnessSha256'] = updater._hash(Path(__file__))
        result_code = 0
    except (Exception, KeyboardInterrupt):
        print(FAILURE, file=sys.stderr)
    finally:
        if output is not None:
            try:
                with output.open('x', encoding='utf-8') as stream:
                    json.dump(report, stream, sort_keys=True, indent=2)
                    stream.write('\n')
            except (Exception, KeyboardInterrupt):
                print(FAILURE, file=sys.stderr)
                result_code = 1
    return result_code


if __name__ == '__main__':
    raise SystemExit(main())

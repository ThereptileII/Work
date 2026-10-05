#!/usr/bin/env python3
"""Explicit, reviewed composition of a failed Staging run and a narrow retest.

The original runtime job remains failed. Only the fixed installer/chart suffix
may be replaced. This module never builds, runs applications, or publishes.
"""
import argparse
import base64
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import stat
import subprocess
import sys
import zipfile

import release_manifest as manifest
import staging_build_inputs as inputs

REPOSITORY = 'ThereptileII/Work'
REQUEST_PATH = 'opennav-x/tools/staging-composition-request.json'
WORKFLOW = '.github/workflows/skager-staging-compose.yml'
BASELINE = '.github/workflows/opennav-baseline.yml'
RETEST = '.github/workflows/skager-staging-retest.yml'
ASSEMBLE_JOB = 'Compose retained Staging evidence'
RUNTIME_JOB = 'Qualify retained native Staging inputs'
REQUIRED_JOBS = (
    'contracts (ubuntu-24.04)', 'contracts (windows-2022)',
    'Linux integrated regression gate', 'Official OpenCPN installation prerequisite',
    'Native private chart loader refusal and fallback gate',
    'Approved OpenCPN ABI on Windows x64',
    'Native boat recovery and commissioning contracts',
    'Native Win32 commissioning restart process boundary',
    'updater-contracts / verify (ubuntu-24.04, amd64)',
    'updater-contracts / verify (windows-2022, 386)',
    'updater-contracts / Native startup update interactions',
    'Actual official OpenCPN portable upgrade caution (sv)',
    'Actual official OpenCPN portable upgrade caution (en_US)',
    'Fixed guarded mode UI actions on disposable native windows',
    'Actual restart broker with disposable marker-only installation', 'windows-integration',
)
NAV_REPORTS = ('navigation', 'route', 'route-standard', 'instruments', 'n2k', 'boat', 'objects')
PREFIX_REPORTS = frozenset({
    'staging-inputs.json', 'boat-feedback-windows/result.json',
    'installer-loader-selftest.json', 'mode-cycle-results.json',
    'signalk-results.json', 'recording-results.json', 'pilot-results.json',
    'recovery-results.json', 'user-flows-results.json',
    'pilot-status-only/pilot-results.json', 'pilot-status-only/pilot-opennav-diagnostics.json',
    'production-recovery-results.json', 'installer-staging.json', 'packaged-updater.json',
    *(name + '-input-results.json' for name in NAV_REPORTS),
})
RETEST_REPORTS = frozenset({'staging-retest.json', 'staging-inputs.json',
                          'installer-staging.json', 'packaged-updater.json', 'charts-results.json'})
UPDATER_CHECKS = [
    'Actual installed launcher bootstraps real application and exact authenticated health receipt',
    'Exact Setup supervised update publishes pending candidate; real startup authenticates and finalizes one attempt',
    'Fault-injected candidate is refused before launch; actual guarded rollback restores verified previous without profile changes',
    'Restored actual generation launches and closes cleanly with retained authenticated receipt',
]
COMMON = {'schemaVersion', 'channel', 'commit', 'runId', 'runAttempt', 'gates',
          'designReview', 'endurance', 'publicAccess'}
GATES = dict(linux='passed', windows='passed', installer='passed', package='passed', restart='passed')
MAX_ARTIFACT = 8 * 1024**3
MAX_REPORT = 32 * 1024**2
FAILURE = "RuntimeError('Native process did not exit within the deadline')"
require = manifest.require


def digest(data):
    return hashlib.sha256(data).hexdigest()


def _fields(value, names):
    require(isinstance(value, dict) and set(value) == set(names), 'Unexpected composition fields')


def _match(pattern, value):
    require(isinstance(value, str) and re.fullmatch(pattern, value), 'Invalid composition identity')


def _identity(value):
    _fields(value, ('commit', 'runId', 'runAttempt'))
    _match('[0-9a-f]{40}', value['commit'])
    for key in ('runId', 'runAttempt'):
        _match('[1-9][0-9]{0,19}', value[key])


def _pins(items, required):
    require(isinstance(items, list) and len(items) == len(required), 'Incomplete reviewed report set')
    names = []
    for item in items:
        _fields(item, ('path', 'sha256'))
        inputs.safe_name(item['path'])
        _match('[a-f0-9]{64}', item['sha256'])
        names.append(item['path'])
    require(len(set(names)) == len(names) and set(names) == required, 'Unexpected or duplicate report substitution')


def validate_request(request):
    _fields(request, ('schema', 'repository', 'producer', 'build', 'original', 'retest'))
    require(type(request['schema']) is int and request['schema'] == 1 and
            request['repository'] == REPOSITORY, 'Unsupported composition request')
    p = request['producer']; _identity(p)
    artifact_keys = {'artifactId', 'artifactName', 'artifactDigest'}
    _fields(request['build'], artifact_keys | {'archiveSha256'})
    _fields(request['original'], artifact_keys | {'jobId', 'prefixReports'})
    _fields(request['retest'], artifact_keys | {'commit', 'runId', 'runAttempt', 'jobId', 'reports'})
    r = request['retest']; _identity({k: r[k] for k in p})
    require(r['runId'] != p['runId'] and r['commit'] != p['commit'], 'Retest must retain its separate harness/run identity')
    for item in (request['build'], request['original'], r):
        _match('[1-9][0-9]{0,19}', item['artifactId'])
        _match('sha256:[a-f0-9]{64}', item['artifactDigest'])
        require(isinstance(item['artifactName'], str), 'Artifact name required')
    for item in (request['original'], r):
        _match('[1-9][0-9]{0,19}', item['jobId'])
    _match('[a-f0-9]{64}', request['build']['archiveSha256'])
    require(request['build']['artifactName'] == f"staging-build-{p['commit']}-run{p['runId']}-attempt{p['runAttempt']}" and
            request['original']['artifactName'] == f"windows-qualification-{p['commit']}-attempt{p['runAttempt']}" and
            r['artifactName'] == f"staging-retest-evidence-{p['runId']}-attempt{p['runAttempt']}-harness{r['commit']}-run{r['runId']}-attempt{r['runAttempt']}",
            'Artifact name does not bind exact producer/retest identity')
    _pins(request['original']['prefixReports'], PREFIX_REPORTS)
    _pins(r['reports'], RETEST_REPORTS)


def request_from_qualification(q):
    c = q['composition']
    return {key: c[key] for key in ('schema', 'repository', 'producer', 'build', 'original', 'retest')}


def qualification(request, request_sha, execution):
    validate_request(request); _identity(execution); _match('[a-f0-9]{64}', request_sha)
    return dict(schemaVersion=2, channel='staging', **request['producer'], gates=GATES.copy(),
                designReview='not-requested', endurance='skipped', publicAccess=False,
                composition=dict(request, requestPath=REQUEST_PATH, requestSha256=request_sha,
                                 execution=execution))


def validate_qualification(q, record, support):
    """Strict offline v2 binding. The unchanged caller owns the v1 branch."""
    _fields(q, COMMON | {'composition'})
    require(type(q['schemaVersion']) is int and q['schemaVersion'] == 2 and
            q['channel'] == 'staging' and q['gates'] == GATES and q['publicAccess'] is False and
            q['designReview'] == 'not-requested' and q['endurance'] == 'skipped', 'Invalid composed qualification policy')
    _fields(q['composition'], {'schema', 'repository', 'producer', 'build', 'original', 'retest',
                              'requestPath', 'requestSha256', 'execution'})
    c = q['composition']; request = request_from_qualification(q); validate_request(request)
    _identity(c['execution']); _match('[a-f0-9]{64}', c['requestSha256'])
    require(c['requestPath'] == REQUEST_PATH, 'Only the committed composition request is supported')
    require(all(q[k] == record[k] == c['producer'][k] for k in ('commit', 'runId', 'runAttempt')),
            'Product origin must remain the original producer')
    e = c['execution']
    require(e['runId'] not in (c['producer']['runId'], c['retest']['runId']), 'Composition run must be separate')
    _fields(support, ('artifactName', 'runId', 'runAttempt', 'commit', 'archiveName', 'sha256', 'size'))
    require(support['commit'] == q['commit'] and support['runId'] == e['runId'] and
            support['runAttempt'] == e['runAttempt'] and
            support['artifactName'] == f"staging-retest-{q['commit']}-attempt{e['runAttempt']}" and
            support['archiveName'] == 'SKAGER-Beta2-Retest-Support.zip' and
            type(support['size']) is int and support['size'] > 0, 'Composed support origin differs')
    _match('[a-f0-9]{64}', support['sha256'])


def _run(gh, identity, workflow, *, conclusion=None):
    run = gh.api(f"{gh.base}/actions/runs/{identity['runId']}/attempts/{identity['runAttempt']}")
    require(str(run.get('id')) == identity['runId'] and str(run.get('run_attempt')) == identity['runAttempt'] and
            run.get('head_sha') == identity['commit'] and run.get('path') == workflow and
            run.get('head_repository', {}).get('full_name') == REPOSITORY and
            run.get('event') in ('push', 'workflow_dispatch'), 'Workflow identity differs')
    if conclusion is not None:
        require(run.get('status') == 'completed' and run.get('conclusion') == conclusion, 'Workflow conclusion differs')
    if workflow == WORKFLOW:
        require(run.get('head_branch') == 'skager-staging-compose', 'Unapproved composition branch')
    return run


def _job(jobs, identity, name, conclusion, job_id=None):
    matches = [j for j in jobs if j.get('name') == name]
    require(len(matches) == 1, 'Required job missing or ambiguous: ' + name)
    j = matches[0]
    require(j.get('status') == 'completed' and j.get('conclusion') == conclusion and
            str(j.get('run_id')) == identity['runId'] and str(j.get('run_attempt')) == identity['runAttempt'] and
            j.get('head_sha') == identity['commit'] and (job_id is None or str(j.get('id')) == job_id),
            'Required job identity/conclusion differs: ' + name)


def _artifact(gh, pin, identity, *, downloadable=False):
    a = gh.api(f"{gh.base}/actions/artifacts/{pin['artifactId']}")
    require(str(a.get('id')) == pin['artifactId'] and a.get('name') == pin['artifactName'] and
            a.get('digest') == pin['artifactDigest'] and
            str(a.get('workflow_run', {}).get('id')) == identity['runId'] and
            a.get('workflow_run', {}).get('head_sha') == identity['commit'] and
            type(a.get('size_in_bytes')) is int and 0 < a['size_in_bytes'] <= MAX_ARTIFACT,
            'Artifact metadata differs')
    if downloadable:
        require(a.get('expired') is False, 'Composition requires retained downloadable evidence')
    return a


def _verify_inputs(gh, q):
    c = q['composition']; p = c['producer']; r = c['retest']
    require(gh.repo == REPOSITORY, 'Composition repository differs')
    _run(gh, p, BASELINE, conclusion='failure')
    jobs = gh.pages(f"{gh.base}/actions/runs/{p['runId']}/attempts/{p['runAttempt']}/jobs")
    for name in REQUIRED_JOBS:
        _job(jobs, p, name, 'success')
    _job(jobs, p, RUNTIME_JOB, 'failure', c['original']['jobId'])
    _run(gh, r, RETEST, conclusion='success')
    jobs = gh.pages(f"{gh.base}/actions/runs/{r['runId']}/attempts/{r['runAttempt']}/jobs")
    _job(jobs, r, 'retest', 'success', r['jobId'])
    for pin, identity in ((c['build'], p), (c['original'], p), (r, r)):
        _artifact(gh, pin, identity)


def _verify_request(gh, q):
    c = q['composition']
    response = gh.api(f"{gh.base}/contents/{REQUEST_PATH}?ref={c['execution']['commit']}")
    require(response.get('type') == 'file' and response.get('encoding') == 'base64' and
            type(response.get('size')) is int and 0 < response['size'] <= 1024 * 1024 and
            isinstance(response.get('content'), str) and len(response['content']) <= 2 * 1024**2,
            'Committed composition request unavailable')
    raw = base64.b64decode(''.join(response['content'].split()), validate=True)
    require(len(raw) == response['size'] and digest(raw) == c['requestSha256'] and
            inputs.strict_json(raw) == request_from_qualification(q), 'Qualification differs from committed request')


def verify_provenance(gh, q, complete=False):
    """API-authenticate v2 composition; never convert a failed run to a pass.

    Archived input metadata remains usable after expiry. Missing metadata fails
    closed. Assembly separately demands downloadable archives and their bytes.
    """
    c = q['composition']; e = c['execution']
    _verify_request(gh, q); _verify_inputs(gh, q)
    run = _run(gh, e, WORKFLOW, conclusion='success' if complete else None)
    if not complete:
        require(run.get('status') in ('in_progress', 'completed') and
                run.get('conclusion') in (None, 'success'), 'Composition has already failed')
    jobs = gh.pages(f"{gh.base}/actions/runs/{e['runId']}/attempts/{e['runAttempt']}/jobs")
    _job(jobs, e, ASSEMBLE_JOB, 'success')
    if not complete:
        require(os.environ.get('GITHUB_REPOSITORY') == REPOSITORY and
                all(os.environ.get(env) == e[key] for env, key in
                    (('GITHUB_SHA', 'commit'), ('GITHUB_RUN_ID', 'runId'), ('GITHUB_RUN_ATTEMPT', 'runAttempt'))) and
                os.environ.get('GITHUB_REF') == 'refs/heads/skager-staging-compose',
                'Publisher must retain actual composition identity')


def selected_zip(archive, wanted, *, max_file=MAX_REPORT):
    """Validate every member before reading only a fixed selected set; no extractall."""
    with zipfile.ZipFile(archive) as z:
        entries = z.infolist(); require(0 < len(entries) <= inputs.MAX_FILES, 'ZIP member count exceeds bound')
        seen = set(); files = {}; total = 0
        for item in entries:
            require(item.orig_filename == item.filename and not item.flag_bits & 1, 'Unsafe ZIP member')
            name = item.filename[:-1] if item.is_dir() else item.filename
            inputs.safe_name(name)
            require(name.casefold() not in seen, 'Aliased ZIP member')
            seen.add(name.casefold()); total += item.file_size
            kind = stat.S_IFMT(item.external_attr >> 16)
            require(kind in ((0, stat.S_IFDIR) if item.is_dir() else (0, stat.S_IFREG)) and
                    0 <= item.file_size <= inputs.MAX_FILE and total <= inputs.MAX_TOTAL,
                    'Linked, special or oversized ZIP member')
            if not item.is_dir(): files[name] = item
        require(set(wanted) <= set(files), 'Missing frozen source/evidence input')
        for name in files:
            require(not any('/'.join(name.split('/')[:i]) in files for i in range(1, len(name.split('/')))),
                    'ZIP file/directory collision')
        result = {}
        for name in wanted:
            require(files[name].file_size <= max_file, 'Selected ZIP member exceeds bound')
            result[name] = z.read(files[name])
        return result


def _download(gh, pin, identity, path):
    a = _artifact(gh, pin, identity, downloadable=True)
    with path.open('xb') as output:
        gh._run(['api', f"{gh.base}/actions/artifacts/{pin['artifactId']}/zip"], output=output)
    require(path.stat().st_size == a['size_in_bytes'] and 'sha256:' + inputs.sha(path) == pin['artifactDigest'],
            'Downloaded artifact digest/size differs')


def _reports(archive, pins):
    raw = selected_zip(archive, [p['path'] for p in pins])
    for pin in pins:
        require(digest(raw[pin['path']]) == pin['sha256'], 'Reviewed report hash differs: ' + pin['path'])
    return raw, {name: inputs.strict_json(data.decode('utf-8-sig')) for name, data in raw.items()}


def _eq(value, expected, message):
    require(value == expected and type(value) is type(expected), message)


def validate_reports(original, retest, q, frozen):
    """Fixed semantic checks in addition to the request's exact reviewed hashes."""
    require(set(original) == PREFIX_REPORTS and set(retest) == RETEST_REPORTS, 'Incomplete report coverage')
    p = q['composition']['producer']; commit = p['commit']; c = q['composition']
    for reports, harness in ((original, commit), (retest, c['retest']['commit'])):
        r = reports['staging-inputs.json']
        require(r['status'] == 'restored' and r['qualification'] == 'not-run' and
                r['producer'] == inputs.producer(**dict(commit=commit, run_id=p['runId'], run_attempt=p['runAttempt'])) and
                r['harnessCommit'] == harness and r['archiveSha256'] == c['build']['archiveSha256'] and
                r['sourceArchiveSha256'] == frozen['sourceArchiveSha256'] and
                r['manifestSha256'] == frozen['manifestSha256'], 'Restored input identity differs')
    feedback = original['boat-feedback-windows/result.json']
    require(feedback['passed'] is True and feedback['platform'] == 'win32' and
            feedback['source_commit'] == feedback['harness_commit'] == commit, 'Original component gate failed')
    tests = feedback['tests']
    require(len(tests) == len(inputs.FEEDBACK_TESTS) and {t['name'] for t in tests} == inputs.FEEDBACK_TESTS and
            all(t['passed'] is True and type(t['exit_code']) is int and t['exit_code'] == 0 and
                t['timed_out'] is False for t in tests), 'Component checks incomplete')
    binaries = {b['name']: b['sha256'] for b in frozen['boatFeedback']['binaries']}
    require(feedback['manifest_sha256'] == frozen['boatFeedback']['manifestSha256'] and
            all(t['binary_sha256'] == binaries[t['name']] for t in tests), 'Component gate tested different binaries')
    loader = original['installer-loader-selftest.json']; _eq(loader['checks'], 5, 'Loader checks incomplete')
    require(loader['status'] == 'passed' and loader['loader']['passed'] is True and
            loader['loader']['profile_initialized'] is False and loader['loader']['plugins_loaded'] is False and
            loader['loader']['commit'] == commit and loader['loader']['test_fixtures'] is True and
            loader['loader']['build_purpose'] == 'DEVELOPER TEST BUILD' and
            loader['loader']['upstream'] == manifest.UPSTREAM and
            loader['loader']['xnav_hardware_output_policy'] == 'test-loopback-only', 'Loader gate differs')
    mode = original['mode-cycle-results.json']
    require(mode['result'] == 'interaction and fixture persistence passed; visual review required' and
            len(mode['steps']) == 6 and len(mode['peer_boundary']) == 5 and
            all(s.get('interaction', s.get('persistence')) == 'pass' for s in mode['steps']) and
            all(s['profile_preserved'] is True and s['owned_tcp_listener_ports'] == [] for s in mode['peer_boundary']) and
            len(mode['chart_rendering']) == 6 and all(s['coastline_visible'] is True for s in mode['chart_rendering']),
            'Mode gate incomplete')
    for name in NAV_REPORTS:
        r = original[name + '-input-results.json']
        require(r['result'] == 'loopback transport and lifecycle passed; numeric and stale screenshot review required' and
                r['transport_errors'] == [] and r['visual_review'] == 'not requested', 'Navigation gate differs')
    for name in ('route', 'route-standard'):
        r = original[name + '-input-results.json']['route_contract']
        require(r['result'] == 'passed' and r['phase'] == 'done' and len(r['checks']) == 26, 'Route contract incomplete')
    r = original['objects-input-results.json']['object_contract']
    require(r['result'] == 'passed' and r['phase'] == 'done' and len(r['checks']) == 28 and
            r['late_connection_added_after_deferred'] is True, 'Object contract incomplete')
    expected_results = {'recording-results.json': ('passed; screenshot review required', 6),
                        'pilot-results.json': ('passed; screenshot review required', 6),
                        'recovery-results.json': ('passed; native screenshot review required', 5),
                        'user-flows-results.json': ('passed; native screenshots require review', 8),
                        'pilot-status-only/pilot-results.json': ('passed; screenshot review required', 5)}
    for name, (result, count) in expected_results.items():
        require(original[name]['result'] == result and len(original[name]['checks']) == count, 'Runtime gate incomplete')
    signal = original['signalk-results.json']
    require(signal['status'] == 'passed' and signal['build_commit'] == commit and
            signal['transport_errors'] == [] and len(signal['checks']) == 5, 'Signal K gate differs')
    require(original['pilot-results.json']['transport_failures'] == [] and
            original['user-flows-results.json']['transport_errors'] == [], 'Original transport failure')
    pilot = original['pilot-status-only/pilot-results.json']
    require(pilot['sent'] == pilot['wire_output'] == pilot['transport_failures'] == [] and
            pilot['received_bytes'] == 0, 'Installed product emitted control bytes')
    diag = original['pilot-status-only/pilot-opennav-diagnostics.json']
    require(diag['build_commit'] == commit and diag['test_fixtures'] is False and
            diag['build_purpose'] == 'INSTALLED PRODUCT' and diag['xnav_hardware_output_policy'] == 'status-only',
            'Installed product identity/output policy differs')
    passive = diag['runtime']['pilot']
    require(passive['output_unavailable'] is True and passive['control_capability'] is False and
            passive['enabled'] is False and passive['command_id'] == '0' and passive['command_state'] == 'None' and
            passive['fresh'] is True and passive['mode'] == 'STANDBY', 'Installed pilot gate was not passive/fresh')
    recovery = original['production-recovery-results.json']
    require(recovery['status'] == 'passed' and len(recovery['checks']) == 7 and
            recovery['build']['commit'] == commit and recovery['build']['test_fixtures'] is False and
            recovery['build']['build_purpose'] == 'INSTALLED PRODUCT' and
            recovery['build']['xnav_hardware_output_policy'] == 'status-only' and
            recovery['design_validation'] == 'not requested' and recovery['files_verified'] == 1019 and
            len(recovery['chart_rendering']) == 10 and
            all(s['coastline_visible'] is True for s in recovery['chart_rendering']), 'Original recovery gate differs')
    for name in ('installer-staging.json', 'packaged-updater.json'):
        require(original[name]['status'] == 'failed' and original[name]['error'] == FAILURE,
                'Original failure must remain the reviewed early-close failure')
    require(original['packaged-updater.json']['checks'] == UPDATER_CHECKS[:3] and
            original['packaged-updater.json']['commit'] == commit and
            original['packaged-updater.json']['faultInjection']['restored'] is True,
            'Original updater failure progressed outside the reviewed boundary')
    result = retest['staging-retest.json']
    require(result['status'] == 'passed' and result['scope'] == 'installer-charts' and
            result['productCommit'] == commit and result['producer'] == original['staging-inputs.json']['producer'] and
            result['harnessCommit'] == c['retest']['commit'] and result['archiveSha256'] == c['build']['archiveSha256'] and
            result['publicAccess'] is False and result['releaseQualification'] is False and
            result['checks'] == [dict(name='installer', status='passed'), dict(name='charts', status='passed')],
            'Narrow retest does not prove both replacement gates')
    require(retest['installer-staging.json']['status'] == 'passed' and
            retest['installer-staging.json']['mode'] == 'staging' and
            retest['installer-staging.json']['product_commit'] == commit and
            retest['installer-staging.json']['harness_commit'] == c['retest']['commit'] and
            retest['charts-results.json']['result'] == 'passed; native screenshot review and physical GPU gate remain separate',
            'Replacement native reports failed or differ')
    updater = retest['packaged-updater.json']
    require(updater['status'] == 'passed' and updater['commit'] == commit and updater['checks'] == UPDATER_CHECKS and
            updater['setupSha256'] == retest['installer-staging.json']['setup_sha256'] and
            updater['executableSha256'] == recovery['build']['executable_sha256'] and
            updater['faultInjection']['restored'] is True, 'Fresh packaged updater did not complete all four mechanics')


def assemble_frozen_payloads(tree, old_raw, new_raw, product_commit):
    """Run only the fixed assembler from the authenticated original source ZIP."""
    source_names = ['tools/alpha-artifacts.py', 'tools/hardware_output_policy.py', 'release/qualification.json',
                    *('docs/beta2/' + name for name in manifest.PRODUCT_FILES if name.endswith('.md'))]
    source = selected_zip(tree / 'build/developer-preview/SKAGER-Beta2-source.zip',
                          ['SOURCE_REFERENCE.json', *('opennav-x/' + n for n in source_names)])
    reference = inputs.strict_json(source['SOURCE_REFERENCE.json'])
    require(reference['productCommit'] == product_commit, 'Frozen source identity differs')
    require(inputs.strict_json(source['opennav-x/release/qualification.json'])['enduranceEnabled'] is False,
            'Frozen product requires different endurance qualification')
    for name in source_names:
        member = 'opennav-x/' + name; data = source[member]
        require(reference['files'][member]['sha256'] == digest(data), 'Frozen source member differs')
        target = tree / name; target.parent.mkdir(parents=True, exist_ok=True); target.write_bytes(data)
    (tree / 'evidence/local').mkdir(parents=True, exist_ok=True)
    for name, raw_report in (('production-recovery-results.json', old_raw['production-recovery-results.json']),
                             ('installer-staging.json', new_raw['installer-staging.json'])):
        (tree / 'evidence/local' / name).write_bytes(raw_report)
    env = {k: v for k, v in os.environ.items() if k not in ('GH_TOKEN', 'GITHUB_TOKEN')}
    env['GITHUB_SHA'] = product_commit
    subprocess.run([sys.executable, str((tree / 'tools/alpha-artifacts.py').resolve()), '--channel', 'staging'],
                   cwd=tree, env=env, check=True)


def assemble(gh, request_path, output):
    """Authenticated downloads and fixed frozen-source assembly; no app execution."""
    root = Path(__file__).resolve().parents[1]; request_path = Path(request_path)
    require(request_path.resolve() == root / 'tools/staging-composition-request.json' and
            not request_path.is_symlink(), 'Only the fixed committed request is supported')
    raw = request_path.read_bytes(); require(len(raw) <= 1024 * 1024, 'Oversized request')
    request = inputs.strict_json(raw); validate_request(request)
    execution = dict(commit=os.environ['GITHUB_SHA'], runId=os.environ['GITHUB_RUN_ID'],
                     runAttempt=os.environ['GITHUB_RUN_ATTEMPT'])
    _identity(execution)
    require(os.environ.get('GITHUB_REPOSITORY') == REPOSITORY and
            os.environ.get('GITHUB_REF') == 'refs/heads/skager-staging-compose', 'Unapproved composition execution')
    repo = subprocess.check_output(['git', '-C', str(root), 'rev-parse', '--show-toplevel'], text=True).strip()
    require(subprocess.check_output(['git', '-C', repo, 'rev-parse', 'HEAD'], text=True).strip() == execution['commit'] and
            subprocess.check_output(['git', '-C', repo, 'show', 'HEAD:' + REQUEST_PATH]) == raw,
            'Composition request is not the exact committed selection')
    q = qualification(request, digest(raw), execution)
    _verify_request(gh, q); _verify_inputs(gh, q); _run(gh, execution, WORKFLOW)
    output = Path(output); require(not output.exists() and not output.is_symlink(), 'Fresh composition output required')
    output.mkdir(parents=True); evidence = output / 'evidence'; evidence.mkdir()
    downloads = output / 'inputs'; downloads.mkdir()
    p = request['producer']; r = request['retest']
    for key, identity in (('build', p), ('original', p), ('retest', r)):
        _download(gh, request[key], identity, downloads / (key + '.zip'))
    from fetch_ci_inputs import unpack
    unpack(downloads / 'build.zip', downloads / 'build', 'staging')
    tree = output / 'assembly'; tree.mkdir()
    receipt = inputs.restore(tree, downloads / 'build/STAGING_BUILD_INPUTS.zip', request['build']['archiveSha256'],
                             inputs.producer(p['commit'], p['runId'], p['runAttempt']), execution['commit'],
                             evidence / 'restored.json')
    old_raw, original = _reports(downloads / 'original.zip', request['original']['prefixReports'])
    new_raw, retest = _reports(downloads / 'retest.zip', r['reports'])
    for group, reports in (('original', old_raw), ('retest', new_raw)):
        for name, data in reports.items():
            target = evidence / group / name; target.parent.mkdir(parents=True, exist_ok=True); target.write_bytes(data)
    validate_reports(original, retest, q, receipt)
    require(digest(old_raw['staging-inputs.json']) == original['boat-feedback-windows/result.json']['restore_receipt_sha256'],
            'Original component receipt does not bind original restoration')
    require(original['installer-loader-selftest.json']['app_sha256'] == inputs.sha(tree / 'build/xnav-install/opencpn.exe') and
            original['production-recovery-results.json']['build']['executable_sha256'] == inputs.sha(tree / 'build/production-install/opencpn.exe') and
            original['packaged-updater.json']['executableSha256'] == inputs.sha(tree / 'build/production-install/opencpn.exe') and
            original['packaged-updater.json']['setupSha256'] == inputs.sha(tree / 'build/beta-installer/SKAGER-Beta2-Setup.exe'),
            'Original runtime report tested different retained executable/Setup bytes')
    for reports, name, filename in ((original, 'production-recovery-results.json',
                                     'build/developer-preview/SKAGER-Beta2-Portable-Recovery.zip'),
                                    (retest, 'installer-staging.json', 'build/beta-installer/SKAGER-Beta2-Setup.exe')):
        key = 'package_sha256' if name.startswith('production') else 'setup_sha256'
        require(reports[name][key] == inputs.sha(tree / filename), 'Native gate tested another payload')
    assemble_frozen_payloads(tree, old_raw, new_raw, p['commit'])
    spec = importlib.util.spec_from_file_location('composition_support', root / 'tools/retain-release-inputs.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    support = module.retain(tree, p['commit'], execution['runId'], execution['runAttempt'])
    release = tree / 'build/beta-artifacts'
    (release / 'QUALIFICATION.json').write_text(json.dumps(q, indent=2) + '\n')
    product = inputs.strict_json((tree / (inputs.PACKAGE_ROOT + '/docs/PRODUCT_BUILD.json')).read_bytes())
    record = manifest.create(release, p['commit'], p['runId'], p['runAttempt'], product['version'])
    validate_qualification(q, record, support); manifest.verify(release)
    shutil.copytree(release, output / 'release')
    (output / 'support').mkdir()
    shutil.copy2(tree / 'build/release-retest' / support['archiveName'], output / 'support' / support['archiveName'])
    proof = dict(status='assembled', product=p, execution=execution, originalRuntimeConclusion='failure',
                 qualificationSha256=inputs.sha(release / 'QUALIFICATION.json'),
                 manifestSha256=inputs.sha(release / 'RELEASE.json'), requestSha256=digest(raw),
                 productRebuilt=False, applicationsExecuted=False, publicAccess=False)
    (evidence / 'assembly.json').write_text(json.dumps(proof, indent=2) + '\n')
    if os.environ.get('GITHUB_OUTPUT'):
        with open(os.environ['GITHUB_OUTPUT'], 'a') as stream:
            stream.write('support_artifact=' + support['artifactName'] + '\n')
            stream.write('product_commit=' + p['commit'] + '\n')
    return proof


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--request', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    from github_release_delivery import GitHub
    try:
        print(json.dumps(assemble(GitHub(REPOSITORY), args.request, args.output), sort_keys=True))
    except (ValueError, KeyError, OSError, zipfile.BadZipFile, subprocess.CalledProcessError) as error:
        parser.exit(1, 'Composition refused: ' + str(error) + '\n')

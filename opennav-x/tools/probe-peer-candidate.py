#!/usr/bin/env python3
"""Read-only package extraction, then the pinned disposable native peer test.

Executes authenticated Setup only for unsupported-build rejection. Never deletes
a portable marker or builds application code.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import stat
import subprocess
import sys
import urllib.error
import urllib.request
import zipfile

ROOT = Path(__file__).resolve().parents[1]
REPO = 'ThereptileII/Work'
SETUP = 'SKAGER-Beta2-Setup.exe'
RECOVERY = 'SKAGER-Beta2-Portable-Recovery.zip'
PREFIX = 'SKAGER-Beta2-Portable-Recovery/'
LIMIT = 2 * 1024**3
HARNESS_SHA = '7cfe340b3fc2949773a72d8e79017315e237f21b0aca6c80c8cd09699c45da0b'


def require(ok, message):
    if not ok:
        raise ValueError(message)


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def safe_name(name):
    p = PurePosixPath(name)
    require(not p.is_absolute() and p.parts and str(p) == name and
            all(part not in ('.', '..') and not part.endswith((' ', '.')) and
                not re.search(r'[\\:\x00-\x1f<>"|?*]', part) and
                not re.fullmatch(r'(?i)(CON|PRN|AUX|NUL|COM[1-9]|LPT[1-9])(?:\..*)?', part)
                for part in p.parts), 'unsafe archive path')
    return name


def members(archive):
    result = {}
    folded = set()
    total = 0
    for item in archive.infolist():
        name = safe_name(item.filename.rstrip('/') if item.is_dir() else item.filename)
        require(name.casefold() not in folded, 'duplicate/case-aliased archive path')
        folded.add(name.casefold())
        require(not stat.S_ISLNK(item.external_attr >> 16), 'archive symlink rejected')
        total += item.file_size
        require(total <= LIMIT and item.file_size <= LIMIT, 'archive size limit exceeded')
        if not item.is_dir():
            result[name] = item
    require(archive.testzip() is None, 'archive CRC failure')
    return result


def api(path):
    request = urllib.request.Request('https://api.github.com/repos/' + REPO + path,
        headers={'Authorization': 'Bearer ' + os.environ['GH_TOKEN'],
                 'Accept': 'application/vnd.github+json', 'User-Agent': 'skager-peer-candidate-probe'})
    with urllib.request.urlopen(request, timeout=60) as response:
        return json.load(response)


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


def download(artifact_id, destination, expected_size, expected_sha):
    # Do not forward the API bearer token to the signed storage redirect.
    request = urllib.request.Request(
        f'https://api.github.com/repos/{REPO}/actions/artifacts/{artifact_id}/zip',
        headers={'Authorization': 'Bearer ' + os.environ['GH_TOKEN'],
                 'User-Agent': 'skager-peer-candidate-probe'})
    try:
        urllib.request.build_opener(NoRedirect).open(request, timeout=60)
    except urllib.error.HTTPError as error:
        require(error.code == 302, 'artifact download did not return a storage redirect')
        url = error.headers['Location']
    else:
        raise ValueError('artifact download missing redirect')
    require(url.startswith('https://'), 'artifact storage must use HTTPS')
    with urllib.request.urlopen(urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'}), timeout=60) as source, destination.open('xb') as target:
        count = 0
        while chunk := source.read(1024**2):
            count += len(chunk)
            require(count <= expected_size <= LIMIT, 'artifact exceeds declared size')
            target.write(chunk)
    require(count == expected_size and sha(destination) == expected_sha, 'artifact length/digest mismatch')


def embedded(setup, output, seven):
    listing = subprocess.check_output([str(seven), 'l', '-slt', str(setup)], timeout=60).decode('utf-8', 'strict')
    require('Type = Nsis' in listing, 'expected NSIS archive')
    (output / 'nsis-listing.txt').write_text(listing, encoding='utf-8')
    for name in ('package.json', 'payload.zip'):
        member = '$PLUGINSDIR/' + name
        paths = [line[7:].replace('\\', '/') for line in listing.splitlines() if line.startswith('Path = ')]
        require(paths.count(member) == 1, 'ambiguous or missing embedded package member')
        with (output / name).open('xb') as stream:
            subprocess.run([str(seven), 'e', '-so', str(setup), member], stdout=stream,
                           stderr=subprocess.PIPE, check=True, timeout=120)
        require((output / name).stat().st_size <= LIMIT, 'embedded member exceeds limit')


def prepare(package_dir, recovery, output, commit):
    package = json.loads((package_dir / 'package.json').read_text(encoding='utf-8-sig'))
    require(package.get('schema') == 1 and package.get('commit') == commit, 'package candidate mismatch')
    require(sha(package_dir / 'payload.zip') == package['payloadSha256'], 'payload digest mismatch')
    records = {}
    for item in package['files']:
        name = safe_name(item['path'])
        require(name not in records and name.startswith(('app/', 'docs/')), 'unexpected package manifest path')
        records[name] = item['sha256']
    with zipfile.ZipFile(package_dir / 'payload.zip') as payload, zipfile.ZipFile(recovery) as portable:
        files, portable_files = members(payload), members(portable)
        require(set(files) == set(records), 'payload and manifest file sets differ')
        require(not any(PurePosixPath(n).name.casefold() == 'opennav_portable_preview' for n in files), 'installer payload unexpectedly portable')
        require(PREFIX + 'app/OPENNAV_PORTABLE_PREVIEW' in portable_files, 'recovery marker missing')
        for name, info in files.items():
            data = payload.read(info)
            require(hashlib.sha256(data).hexdigest() == records[name], 'package file digest mismatch')
            require(PREFIX + name in portable_files and portable.read(PREFIX + name) == data, 'installer/recovery runtime mismatch')
            target = output / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(data)
        product = json.loads((output / 'docs/PRODUCT_BUILD.json').read_text(encoding='utf-8-sig'))
        require(product.get('commit') == commit and product.get('test_fixtures') is False and
                product.get('build_purpose') == 'INSTALLED PRODUCT' and
                product.get('xnav_hardware_output_policy') == 'status-only', 'product policy/identity mismatch')
        exe = output / 'app/opencpn.exe'
        require(sha(exe) == product['executable_sha256'], 'product executable digest mismatch')
        require(any(n.startswith('app/plugins/') for n in files), 'packaged plugin runtime absent')
        config_bytes = portable.read(PREFIX + 'profile/opencpn.conf')
        versions = re.findall(r'^ConfigVersionString=Version ([^\r\n]+) Build ([^\r\n]+)\r?$', config_bytes.decode('utf-8-sig'), re.M)
        require(len(versions) == 1, 'ambiguous retained profile build identity')
        version, date = versions[0]
        require(re.fullmatch(r'[0-9A-Za-z.+_-]{1,100}', version) and re.fullmatch(r'\d{4}-\d{2}-\d{2}', date), 'unsafe retained profile build identity')
        build = output / 'profile-adapter/include'
        build.mkdir(parents=True)
        (build / 'config.h').write_text(f'#define VERSION_FULL "{version}"\n#define VERSION_DATE "{date}"\n', encoding='utf-8')
        return {'executable_sha256': sha(exe), 'payload_sha256': package['payloadSha256'],
                'package_json_sha256': sha(package_dir / 'package.json'), 'verified_runtime_files': len(files),
                'packaged_files_sha256': records,
                'config_source': PREFIX + 'profile/opencpn.conf',
                'config_source_sha256': hashlib.sha256(config_bytes).hexdigest(),
                'version_full': version, 'version_date': date,
                'adapter_sha256': sha(build / 'config.h')}


PREREQUISITES = (
    'Build and exercise integrated modes',
    'Verify packaged source reproduces the curl certificate-tool patch',
    # This step verifies the positive early CLI receipt against the same source,
    # installed executable and run attempt. Keep it mandatory: integration-step
    # success alone must not replace explicit peer CLI evidence.
    'Installed native peer CLI refuses key changes on the disposable runner',
    'Staged native loader check without profile initialization',
    'Native pointer chart gestures, waypoint Go To and route creation',
    'Repeat native crash-recovery returns with separate evidence',
    'Retain complete fixture UI and scenario regression suite',
    'Capture successful same-job Windows dependency closure',
    'Build native installed product without synthetic data',
    'Native public Downloader Windows trust and rejection gate',
    'Exact peer response buffer on native MSVC Win32',
    'Package and test the fixture-free recovery distribution',
    'Build and exercise candidate Beta installer',
    'Native 100 / 125 / 150 percent DPI and touch interactions',
    'Public ENC chart and plugin rendering gate',
    'Require same-run restart qualification before delivering any product artifact',
    'Verify exact restart qualification identity',
    'Prepare development boat review after native functional gates',
    'Upload development review with endurance qualification pending',
)


def paged(path, key):
    result = []
    for page in range(1, 21):
        batch = api(path + f'?per_page=100&page={page}')[key]
        result.extend(batch)
        if len(batch) < 100:
            return result
    raise ValueError('API pagination bound exceeded')


def eligibility(run, artifact, jobs, receipt, commit, run_id):
    require(run['head_sha'] == commit and run['path'] == '.github/workflows/opennav-baseline.yml' and
            run['repository']['full_name'] == REPO and run['status'] in ('in_progress', 'completed') and
            run['conclusion'] in (None, 'success'), 'candidate run identity/status rejected')
    require(not any(j.get('conclusion') in ('failure', 'cancelled', 'timed_out', 'action_required', 'stale') for j in jobs),
            'candidate already has a failed/cancelled job')
    require(artifact['workflow_run']['id'] == int(run_id) and artifact['workflow_run']['head_sha'] == commit and
            artifact['expired'] is False and artifact['created_at'] >= run['run_started_at'],
            'artifact run/attempt binding mismatch')
    require(receipt.get('schema') == 1 and receipt.get('owner') == 'OpenNavX.CI.RestartGates.1' and
            receipt.get('commit') == commit and receipt.get('runId') == run_id and
            receipt.get('runAttempt') == str(run['run_attempt']) and receipt.get('actualBoat') is False and
            all(receipt.get(k) == 'success' for k in ('transport', 'maintenance', 'broker', 'prepareArm')),
            'same-run restart receipt rejected')
    native = [j for j in jobs if j['name'] == 'Native MSVC XNav / Legacy / Safe slice']
    require(len(native) == 1, 'native candidate job missing/ambiguous')
    for name in PREREQUISITES:
        steps = [step for step in native[0]['steps'] if step['name'] == name]
        require(len(steps) == 1 and steps[0]['status'] == 'completed' and steps[0]['conclusion'] == 'success',
                'required native prerequisite not successful: ' + name)
    if artifact['name'] == 'beta-candidate-' + commit:
        require(run['status'] == 'completed' and run['conclusion'] == 'success', 'final candidate requires complete successful baseline')
        return 'candidate-complete-ci'
    require(artifact['name'] == 'beta2-boat-review-pending-endurance-' + commit, 'unexpected candidate artifact name')
    return 'pending-endurance-development-only'


def main(args):
    require(os.name == 'nt' and os.environ.get('GITHUB_ACTIONS') == 'true' and
            os.environ.get('RUNNER_ENVIRONMENT') == 'github-hosted', 'requires disposable hosted Windows')
    require(re.fullmatch(r'[0-9a-f]{40}', args.commit) and re.fullmatch(r'[0-9a-f]{64}', args.digest), 'invalid candidate identity')
    require(args.run.isdecimal() and args.artifact.isdecimal(), 'invalid run/artifact identity')
    args.evidence.mkdir(parents=True, exist_ok=False)
    report = {'result': 'FAILED', 'candidate_commit': args.commit, 'candidate_run': args.run,
              'artifact_id': args.artifact, 'artifact_sha256': args.digest,
              'test_source_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
              'scope': 'Peer/plugin security and exact Setup unsupported-build preservation. No supported installation, application rebuild, packet capture, or release acceptance.'}
    try:
        run = api('/actions/runs/' + args.run)
        artifact = api('/actions/artifacts/' + args.artifact)
        require(artifact['digest'] == 'sha256:' + args.digest, 'artifact API digest mismatch')
        jobs = paged(f"/actions/runs/{args.run}/attempts/{run['run_attempt']}/jobs", 'jobs')
        receipts = [a for a in paged('/actions/runs/' + args.run + '/artifacts', 'artifacts')
                    if a['name'] == 'commissioning-restart-qualified-' + args.commit and not a['expired']]
        require(len(receipts) == 1, 'same-run restart receipt artifact missing/ambiguous')
        receipt_meta = receipts[0]
        require(receipt_meta.get('digest', '').startswith('sha256:') and
                receipt_meta['created_at'] >= run['run_started_at'], 'receipt artifact digest/attempt missing')
        workspace = Path(os.environ['RUNNER_TEMP']) / ('peer-candidate-' + args.artifact)
        workspace.mkdir(exist_ok=False)
        receipt_zip = workspace / 'restart-receipt.zip'
        download(str(receipt_meta['id']), receipt_zip, receipt_meta['size_in_bytes'], receipt_meta['digest'][7:])
        with zipfile.ZipFile(receipt_zip) as receipt_archive:
            require(set(members(receipt_archive)) == {'qualified.json'}, 'restart receipt layout mismatch')
            receipt = json.loads(receipt_archive.read('qualified.json').decode('utf-8-sig'))
        report['qualification'] = eligibility(run, artifact, jobs, receipt, args.commit, args.run)
        report['native_prerequisites'] = {name: 'success' for name in PREREQUISITES}
        report['restart_receipt'] = {'artifact_id': receipt_meta['id'], 'sha256': receipt_meta['digest'][7:], **receipt}
        report['candidate_url'] = run['html_url']
        report['candidate_run_attempt'] = run['run_attempt']
        report['artifact_size'] = artifact['size_in_bytes']
        archive = workspace / 'artifact.zip'
        download(args.artifact, archive, artifact['size_in_bytes'], args.digest)
        with zipfile.ZipFile(archive) as bundle:
            entries = members(bundle)
            require(all(n in entries for n in (SETUP, RECOVERY, 'SHA256SUMS.txt')), 'candidate artifact layout mismatch')
            checks = {}
            for line in bundle.read('SHA256SUMS.txt').decode('ascii').splitlines():
                match = re.fullmatch(r'([a-f0-9]{64})  (.+)', line)
                require(match is not None and match[2] not in checks, 'invalid package checksum record')
                checks[match[2]] = match[1]
            for name in (SETUP, RECOVERY):
                with bundle.open(name) as source, (workspace / name).open('xb') as target:
                    shutil.copyfileobj(source, target)
                require(sha(workspace / name) == checks[name], 'candidate package checksum mismatch')
                report[name + '_sha256'] = checks[name]
        embedded(workspace / SETUP, workspace, args.seven)
        shutil.copyfile(workspace / 'nsis-listing.txt', args.evidence / 'nsis-listing.txt')
        report.update(prepare(workspace, workspace / RECOVERY, workspace / 'runtime', args.commit))
        harness = ROOT / 'tools/smoke-peer-two-instances.py'
        require(sha(harness) == HARNESS_SHA, 'pinned two-instance harness changed')
        report['test_files_sha256'] = {name: sha(ROOT / name) for name in (
            'tools/probe-peer-candidate.py', 'tools/smoke-peer-two-instances.py',
            'tools/diagnostic_snapshot.py', 'tools/peer_boundary.py', 'tools/profile-fixtures.py',
            'tools/prepare-test-profile.py', 'tools/windows-ui.py', 'tests/fixtures/mode-persistence.gpx')}
        test_env = os.environ.copy()
        test_env.pop('GH_TOKEN', None)
        test_env.pop('GITHUB_TOKEN', None)
        result = subprocess.run([sys.executable, str(harness), '--exe', str(workspace / 'runtime/app/opencpn.exe'),
            '--build', str(workspace / 'runtime/profile-adapter'), '--evidence', str(args.evidence)], timeout=300, env=test_env)
        observed = json.loads((args.evidence / 'peer-two-instances-results.json').read_text())
        require(result.returncode == 0 and observed['result'] == 'PASS' and
                observed['test_source_commit'] == report['test_source_commit'] and
                observed['executable_sha256'] == report['executable_sha256'] and len(observed['profiles']) == 2 and
                all(p['build_commit'] == args.commit for p in observed['profiles']), 'running candidate identity/result mismatch')
        plugin_helper = ROOT / 'tools/test-plugin-download-guard-windows.py'
        plugin_evidence = args.evidence / 'plugin-download-guard'
        plugin_manifest = workspace / 'package.json'
        report['plugin_probe_files_sha256'] = {name: sha(ROOT / name) for name in (
            'tools/test-plugin-download-guard-windows.py', 'tools/test-plugin-download-guard.py',
            'tools/plugin-download-guard-probe.cpp', 'tools/plugin-download-guard-server.py',
            'tools/plugin-probe-owned-trust.ps1', 'tools/windows-plugin-archive-sdk.lock.json',
            'tests/plugin_download_guard/CMakeLists.txt',
            'tests/plugin_download_guard/InputPaths.cmake')}
        plugin_result = subprocess.run([sys.executable, str(plugin_helper),
            '--runtime-dir', str(workspace / 'runtime'), '--package-manifest-path', str(plugin_manifest),
            '--expected-manifest-sha256', sha(plugin_manifest), '--expected-commit', args.commit,
            '--evidence', str(plugin_evidence)], timeout=600, env=test_env)
        plugin_observed = json.loads((plugin_evidence / 'native-results.json').read_text())
        require(plugin_result.returncode == 0 and plugin_observed['status'] == 'passed' and
                plugin_observed['commit'] == args.commit and
                plugin_observed['packageManifestSha256'] == sha(plugin_manifest) and
                plugin_observed['ownedCaCleanup'] == 'verified absent',
                'native plugin download guard failed or changed candidate identity')
        report['plugin_download_guard'] = {'status': 'passed',
            'results_sha256': sha(plugin_evidence / 'native-results.json')}
        unsupported_helper = ROOT / 'tools/test-installer-unsupported-candidate.py'
        unsupported_evidence = args.evidence / 'installer-unsupported'
        report['unsupported_probe_source_sha256'] = sha(unsupported_helper)
        # The helper bounds its own child processes and owns their tree cleanup
        # and profile-restoration finally block. An outer subprocess timeout
        # would kill that owner before it could restore or report uncertainty.
        unsupported_result = subprocess.run([sys.executable, str(unsupported_helper),
            '--setup', str(workspace / SETUP), '--setup-sha256', report[SETUP + '_sha256'],
            '--runtime', str(workspace / 'runtime'), '--manifest', str(plugin_manifest),
            '--manifest-sha256', sha(plugin_manifest), '--adapter-sha256', report['adapter_sha256'],
            '--commit', args.commit, '--seven', str(args.seven),
            '--evidence', str(unsupported_evidence)], env=test_env)
        unsupported = json.loads((unsupported_evidence / 'unsupported-results.json').read_text())
        require(unsupported_result.returncode == 0 and unsupported['status'] == 'passed' and
                unsupported['commit'] == args.commit and unsupported['setupSha256'] == report[SETUP + '_sha256'] and
                unsupported['testSourceSha256'] == report['unsupported_probe_source_sha256'] and
                unsupported['packageManifestSha256'] == sha(plugin_manifest) and
                unsupported['profileRestoration'] == 'verified' and unsupported['installAndShortcutRootsAbsent'] is True,
                'Unsupported-build preservation failed or changed candidate identity')
        report['unsupported_build_preservation'] = {'status': 'passed',
            'results_sha256': sha(unsupported_evidence / 'unsupported-results.json')}
        package = json.loads((workspace / 'package.json').read_text(encoding='utf-8-sig'))
        require(all(sha(workspace / 'runtime' / item['path']) == item['sha256'] for item in package['files']),
                'packaged runtime changed during test')
        final_run = api('/actions/runs/' + args.run)
        final_jobs = paged(f"/actions/runs/{args.run}/attempts/{run['run_attempt']}/jobs", 'jobs')
        require(final_run['run_attempt'] == run['run_attempt'], 'upstream attempt changed during probe')
        eligibility(final_run, artifact, final_jobs, receipt, args.commit, args.run)
        report['upstream_status_at_probe_end'] = final_run['status']
        report['runtime_unchanged_after_test'] = True
        report['result'] = 'PASS'
    except Exception as error:
        # Storage errors can contain signed URLs: retain only their type.
        report['error'] = str(error) if isinstance(error, ValueError) else type(error).__name__
        raise RuntimeError(report['error']) from None
    finally:
        (args.evidence / 'candidate-binding.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for field in ('run', 'commit', 'artifact', 'digest'):
        parser.add_argument('--' + field, required=True)
    parser.add_argument('--seven', type=Path, required=True)
    parser.add_argument('--evidence', type=Path, required=True)
    main(parser.parse_args())

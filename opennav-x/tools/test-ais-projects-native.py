#!/usr/bin/env python3
"""Authenticated retained MSBuild project proof; no dependency reuse or AIS run."""
import hashlib
import json
import os
from pathlib import Path
import platform
import stat
import subprocess
import sys
import urllib.error
import urllib.parse
import urllib.request
import zipfile

ROOT = Path(__file__).resolve().parents[1]
REPOSITORY = 'ThereptileII/Work'
FIXTURE = ROOT / 'tests/fixtures/ais-projects-4dd'


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


def validate_original_job(run, job, manifest):
    """Bind the terminal native failure without waiting for unrelated run jobs."""
    if (run['id'] != manifest['originalRunId'] or
            run['head_sha'] != manifest['originalCommit'] or
            run['run_attempt'] != manifest['originalRunAttempt']):
        raise ValueError('Original run identity differs')
    if (job['id'] != manifest['originalNativeJobId'] or
            job['run_id'] != manifest['originalRunId'] or
            job['head_sha'] != manifest['originalCommit'] or
            ('run_attempt' in job and job['run_attempt'] != manifest['originalRunAttempt']) or
            job['status'] != 'completed' or job['conclusion'] != 'failure'):
        raise ValueError('Original native job identity/status differs')
    expected = {
        17: ('Capture successful same-job Windows dependency closure', 'success'),
        18: ('Native AIS observation and transport with same-job maintained TLS', 'failure'),
    }
    for number, (name, conclusion) in expected.items():
        matches = [step for step in job['steps'] if step['number'] == number]
        if (len(matches) != 1 or matches[0]['name'] != name or
                matches[0]['status'] != 'completed' or matches[0]['conclusion'] != conclusion):
            raise ValueError('Original native gate step differs: ' + str(number))


def main():
    if sys.platform != 'win32' or os.environ.get('GITHUB_ACTIONS') != 'true':
        raise ValueError('Requires disposable native Windows Actions')
    if os.environ.get('GITHUB_REPOSITORY') != REPOSITORY:
        raise ValueError('Unexpected proof repository')
    output = ROOT / 'evidence/local/ais-project-closure'
    output.mkdir(parents=True, exist_ok=False)
    manifest = json.loads((FIXTURE / 'manifest.json').read_text())
    source_commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    if source_commit != os.environ.get('GITHUB_SHA'):
        raise ValueError('Proof checkout differs from Actions commit')
    names = ['tools/test-ais-runtime-windows.py', 'tools/test-ais-project-closure.py',
             'tools/test-ais-projects-native.py', 'tools/windows_dependency_receipt.py',
             'tools/windows_dependency_reuse.py', 'tools/windows_dependency_stage.py',
             'tools/windows_dependency_evidence.py', 'tests/fixtures/ais-projects-4dd/manifest.json',
             'tests/fixtures/ais-projects-4dd/projects.zip']
    def fact(path):
        data = path.read_bytes()
        return {'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}
    report = {'passed': False, 'toolingCommit': source_commit,
              'proofRunId': os.environ.get('GITHUB_RUN_ID'),
              'proofRunAttempt': os.environ.get('GITHUB_RUN_ATTEMPT'),
              'originalCommit': manifest['originalCommit'], 'originalRunId': manifest['originalRunId'],
              'originalRunAttempt': manifest['originalRunAttempt'],
              'originalNativeJobId': manifest['originalNativeJobId'],
              'nativePlatform': platform.platform(), 'pythonVersion': platform.python_version(),
              'pythonExecutable': fact(Path(sys.executable)),
              'inputs': {name: fact(ROOT / name) for name in names},
              'actualAISRuntime': False, 'dependencyReuse': False, 'packageAcceptance': False}
    workflow = '.github/workflows/skager-ais-project-closure.yml'
    workflow_path = ROOT / workflow if (ROOT / workflow).is_file() else ROOT.parent / workflow
    report['inputs'][workflow] = fact(workflow_path)
    def save():
        (output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    def api(path):
        token = os.environ.get('GH_TOKEN')
        if not token:
            raise ValueError('Missing read-only Actions authentication')
        request = urllib.request.Request('https://api.github.com/repos/' + REPOSITORY + path,
            headers={'Authorization': 'Bearer ' + token, 'Accept': 'application/vnd.github+json',
                     'X-GitHub-Api-Version': '2022-11-28', 'User-Agent': 'SKAGER-285-project-proof'})
        return request
    def read_json(path):
        with urllib.request.urlopen(api(path), timeout=60) as response:
            data = response.read(4 * 1024 * 1024 + 1)
        if len(data) > 4 * 1024 * 1024:
            raise ValueError('API response exceeds bound')
        return json.loads(data)
    try:
        save()
        run = read_json('/actions/runs/' + str(manifest['originalRunId']))
        job = read_json('/actions/jobs/' + str(manifest['originalNativeJobId']))
        validate_original_job(run, job, manifest)
        artifact = read_json('/actions/artifacts/' + str(manifest['artifactId']))
        if (artifact['id'] != manifest['artifactId'] or artifact['name'] != manifest['artifactName'] or
                artifact['size_in_bytes'] != manifest['artifactBytes'] or artifact['expired'] or
                artifact['digest'] != 'sha256:' + manifest['artifactSha256'] or
                artifact['workflow_run']['id'] != manifest['originalRunId'] or
                artifact['workflow_run']['head_sha'] != manifest['originalCommit']):
            raise ValueError('Original artifact identity differs')
        report['authenticatedRun'] = {k: run[k] for k in ('id', 'head_sha', 'run_attempt', 'status', 'conclusion')}
        report['authenticatedNativeJob'] = {k: job[k] for k in
            ('id', 'run_id', 'head_sha', 'status', 'conclusion', 'run_attempt') if k in job}
        report['authenticatedNativeJob']['gateSteps'] = [
            step for step in job['steps'] if step['number'] in (17, 18)]
        report['authenticatedArtifact'] = {k: artifact[k] for k in ('id', 'name', 'size_in_bytes', 'digest', 'expired')}
        save()
        # Never forward the GitHub bearer token to the signed storage redirect.
        opener = urllib.request.build_opener(NoRedirect)
        try:
            opener.open(api('/actions/artifacts/' + str(manifest['artifactId']) + '/zip'), timeout=60)
        except urllib.error.HTTPError as response:
            if response.code != 302:
                raise ValueError('Artifact download did not return expected redirect') from None
            location = response.headers['Location']
            response.close()
        else:
            raise ValueError('Artifact download unexpectedly lacked redirect')
        url = urllib.parse.urlsplit(location)
        if url.scheme != 'https' or not url.hostname or url.username or url.password:
            raise ValueError('Unsafe artifact redirect')
        archive = output / 'original-artifact.zip'
        count = 0
        digest = hashlib.sha256()
        with urllib.request.urlopen(location, timeout=90) as response, archive.open('xb') as target:
            while data := response.read(1024 * 1024):
                count += len(data)
                if count > manifest['artifactBytes']:
                    raise ValueError('Artifact exceeds exact byte limit')
                digest.update(data)
                target.write(data)
        if count != manifest['artifactBytes'] or digest.hexdigest() != manifest['artifactSha256']:
            raise ValueError('Downloaded original ZIP differs')
        projects = output / 'projects'
        projects.mkdir()
        with zipfile.ZipFile(archive) as z:
            if z.testzip() is not None:
                raise ValueError('Original artifact CRC failure')
            if len(z.namelist()) != len(set(z.namelist())):
                raise ValueError('Duplicate ZIP entries')
            report['zip'] = {'bytes': count, 'sha256': digest.hexdigest(), 'entries': len(z.infolist()), 'allCrcPassed': True}
            for name, expected in manifest['members'].items():
                entry = z.getinfo(name)
                kind = stat.S_IFMT(entry.external_attr >> 16)
                if (entry.is_dir() or kind not in (0, stat.S_IFREG) or entry.external_attr & 0x400 or
                        entry.file_size != expected['bytes']):
                    raise ValueError('Unexpected project entry metadata')
                data = z.read(entry)
                if hashlib.sha256(data).hexdigest() != expected['sha256']:
                    raise ValueError('Original generated project hash differs')
                (projects / Path(name).name).write_bytes(data)
        save()
        command = [sys.executable, str(ROOT / 'tools/test-ais-project-closure.py'),
                   '--projects', str(projects), '--output', str(output / 'focused-result.json')]
        with (output / 'focused.log').open('wb') as log:
            subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, timeout=60, check=True)
        result = json.loads((output / 'focused-result.json').read_text())
        if result['passed'] is not True or result['tests'] != 8 or result['skipped'] != 0:
            raise ValueError('Incomplete project parser proof')
        for name, expected in report['inputs'].items():
            path = workflow_path if name == workflow else ROOT / name
            if fact(path) != expected:
                raise ValueError('Proof source changed: ' + name)
        report['passed'] = True
    except Exception as error:
        # Retain bounded error type/message; omit request URLs and authentication.
        report['error'] = type(error).__name__ + ': ' + (str(error) if not isinstance(error, urllib.error.URLError) else 'HTTPS request failed')
        raise
    finally:
        save()


if __name__ == '__main__':
    main()

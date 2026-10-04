#!/usr/bin/env python3
"""SCRUM-287: bounded read-only rendezvous for this run's fixture runtime.

This is artifact readiness, never producer, endurance or release acceptance.
Only sanitized metadata is retained; gh output/errors and download URLs are not.
"""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import subprocess
import time

WORKFLOW = '.github/workflows/opennav-baseline.yml'
PRODUCER = 'Native MSVC XNav / Legacy / Safe slice'
REQUIRED_STEPS = (
    'Build and exercise integrated modes',
    'Retain complete fixture UI and scenario regression suite',
    'Capture successful same-job Windows dependency closure',
    'Native AIS observation and transport with same-job maintained TLS',
    'Prepare immutable fixture runtime for concurrent endurance',
    'Upload immutable fixture runtime for concurrent endurance',
)
CONCLUSIONS = (None, 'success', 'failure', 'neutral', 'cancelled', 'skipped',
               'timed_out', 'action_required', 'stale', 'startup_failure')


class Refusal(ValueError):
    """A non-retryable, safe-to-log rejection."""


class TransientError(Exception):
    """A metadata read may be retried within the original deadline."""


def require(condition, message):
    if not condition:
        raise Refusal(message)


def integer(value):
    return type(value) is int and value > 0


def stamp(value):
    try:
        require(isinstance(value, str), 'Missing metadata timestamp')
        parsed = datetime.fromisoformat(value.replace('Z', '+00:00'))
        require(parsed.tzinfo is not None, 'Unqualified metadata timestamp')
        return parsed.astimezone(timezone.utc)
    except (ValueError, TypeError):
        raise Refusal('Invalid metadata timestamp') from None


def context(env):
    result = {key: env.get(name, '') for key, name in (
        ('repository', 'GITHUB_REPOSITORY'), ('runId', 'GITHUB_RUN_ID'),
        ('runAttempt', 'GITHUB_RUN_ATTEMPT'), ('commit', 'GITHUB_SHA'),
        ('event', 'GITHUB_EVENT_NAME'))}
    require(re.fullmatch(r'[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+', result['repository']),
            'Exact repository required')
    require(all(re.fullmatch(r'[1-9][0-9]*', result[k]) for k in ('runId', 'runAttempt')),
            'Exact run and attempt required')
    require(re.fullmatch(r'[a-f0-9]{40}', result['commit']), 'Exact commit required')
    require(result['event'] in ('push', 'workflow_dispatch'), 'Unsupported workflow event')
    result['runId'], result['runAttempt'] = int(result['runId']), int(result['runAttempt'])
    return result


def gh_api(endpoint, timeout):
    try:
        reply = subprocess.run(['gh', 'api', '--method', 'GET', '-H',
            'Accept: application/vnd.github+json', endpoint], capture_output=True,
            text=True, timeout=timeout, check=False)
    except subprocess.TimeoutExpired:
        raise TransientError('Metadata request timed out') from None
    except OSError:
        raise Refusal('Cannot start read-only gh client') from None
    if reply.returncode:
        # Never echo stderr: it can contain credentials or signed redirect URLs.
        if re.search(r'HTTP (?:401|403|404)\b', reply.stderr):
            raise Refusal('Metadata access refused')
        raise TransientError('Metadata request failed')
    try:
        value = json.loads(reply.stdout)
    except (ValueError, TypeError):
        raise Refusal('Malformed metadata response') from None
    require(isinstance(value, dict), 'Malformed metadata response')
    return value


def check_run(run, expected):
    require(integer(run.get('id')) and integer(run.get('run_attempt'))
            and run['id'] == expected['runId'] and run['run_attempt'] == expected['runAttempt'],
            'Run or attempt differs')
    require(run.get('head_sha') == expected['commit'] and run.get('path') == WORKFLOW
            and run.get('event') == expected['event'], 'Run source, workflow or event differs')
    repository, head = run.get('repository', {}), run.get('head_repository', {})
    require(isinstance(repository, dict) and isinstance(head, dict)
            and repository.get('full_name') == expected['repository']
            and head.get('full_name') == expected['repository']
            and integer(repository.get('id')) and integer(head.get('id'))
            and head['id'] == repository['id'],
            'Run repository differs')
    require(run.get('status') in ('queued', 'in_progress', 'completed', 'waiting', 'pending', 'requested'),
            'Unknown run status')
    require(run.get('conclusion') in CONCLUSIONS, 'Unknown run conclusion')
    return stamp(run.get('run_started_at'))


def collection(read, endpoint, key):
    """Consume every page; a concurrently changing inventory restarts the poll."""
    items, seen, total = [], set(), None
    for page in range(1, 1001):
        document = read(f'{endpoint}?per_page=100&page={page}')
        count, batch = document.get('total_count'), document.get(key)
        require(type(count) is int and count >= 0 and isinstance(batch, list)
                and len(batch) <= 100, 'Malformed paginated metadata')
        if total is None:
            total = count
        elif count != total:
            raise TransientError('Metadata inventory changed during pagination')
        for item in batch:
            require(isinstance(item, dict) and integer(item.get('id')), 'Malformed metadata item')
            require(item['id'] not in seen, 'Duplicate metadata identity')
            seen.add(item['id'])
            items.append(item)
        require(len(items) <= total, 'Metadata inventory exceeds count')
        if len(items) == total:
            return items
        require(len(batch) == 100, 'Incomplete metadata pagination')
    raise Refusal('Metadata pagination limit exceeded')


def producer_steps(job, expected):
    require(integer(job.get('run_id')) and job['run_id'] == expected['runId'], 'Producer run differs')
    if 'run_attempt' in job:
        require(integer(job['run_attempt']) and job['run_attempt'] == expected['runAttempt'],
                'Producer attempt differs')
    if 'head_sha' in job:
        require(job['head_sha'] == expected['commit'], 'Producer source differs')
    require(job.get('status') in ('queued', 'in_progress', 'completed', 'waiting', 'pending'),
            'Unknown producer status')
    require(job.get('conclusion') in CONCLUSIONS, 'Unknown producer conclusion')
    steps = job.get('steps')
    require(isinstance(steps, list), 'Producer step metadata missing')
    selected, previous, ready = [], 0, True
    for name in REQUIRED_STEPS:
        matches = [s for s in steps if isinstance(s, dict) and s.get('name') == name]
        require(len(matches) <= 1, 'Duplicate required producer step')
        if not matches:
            ready = False
            continue
        step = matches[0]
        require(integer(step.get('number')) and step['number'] > previous,
                'Required producer step order differs')
        previous = step['number']
        if step.get('status') == 'completed':
            require(step.get('conclusion') == 'success', 'Required producer step did not succeed')
        else:
            ready = False
        selected.append({'name': name, 'number': previous,
                         'status': step.get('status'), 'conclusion': step.get('conclusion')})
    return ready, selected


def check_artifact(artifact, expected, run, started, now):
    name = f"native-endurance-runtime-{expected['commit']}-{expected['runAttempt']}"
    require(artifact.get('name') == name and integer(artifact.get('id')),
            'Artifact identity differs')
    owner = artifact.get('workflow_run', {})
    require(isinstance(owner, dict) and integer(owner.get('id'))
            and integer(owner.get('repository_id')) and integer(owner.get('head_repository_id'))
            and owner['id'] == expected['runId']
            and owner.get('head_sha') == expected['commit']
            and owner.get('repository_id') == run['repository']['id']
            and owner.get('head_repository_id') == run['head_repository']['id'],
            'Artifact ownership differs')
    digest = artifact.get('digest')
    require(isinstance(digest, str) and re.fullmatch(r'sha256:[a-f0-9]{64}', digest),
            'Artifact SHA256 digest missing or malformed')
    require(artifact.get('expired') is False and integer(artifact.get('size_in_bytes')),
            'Artifact expired or empty')
    created, expires = stamp(artifact.get('created_at')), stamp(artifact.get('expires_at'))
    require(started <= created <= now and expires > now and expires > created,
            'Artifact is stale or expired')
    return {'id': artifact['id'], 'name': name, 'digest': digest,
            'bytes': artifact['size_in_bytes'], 'createdAt': created.isoformat(),
            'expiresAt': expires.isoformat()}


def wait_ready(expected, *, api=gh_api, timeout=10800, interval=30,
               clock=time.monotonic, sleep=time.sleep,
               utcnow=lambda: datetime.now(timezone.utc), progress=lambda _: None):
    require(type(timeout) is int and 1 <= timeout <= 10800, 'Wait timeout must be 1..10800 seconds')
    require(type(interval) is int and 1 <= interval <= 30, 'Poll interval must be 1..30 seconds')
    begin, failures, polls = clock(), 0, 0
    deadline = begin + timeout
    prefix = f"repos/{expected['repository']}/actions"
    run_endpoint = f"{prefix}/runs/{expected['runId']}"
    def read(endpoint):
        remaining = deadline - clock()
        require(remaining > 0, 'Artifact readiness timed out')
        return api(endpoint, min(30, remaining))
    while clock() < deadline:
        polls += 1
        try:
            run = read(run_endpoint)
            started = check_run(run, expected)
            jobs = collection(read, f"{run_endpoint}/attempts/{expected['runAttempt']}/jobs", 'jobs')
            matches = [j for j in jobs if j.get('name') == PRODUCER]
            require(len(matches) <= 1, 'Duplicate producer job')
            job = matches[0] if matches else None
            ready, steps = producer_steps(job, expected) if job else (False, [])
            artifacts = collection(read, f'{run_endpoint}/artifacts', 'artifacts')
            name = f"native-endurance-runtime-{expected['commit']}-{expected['runAttempt']}"
            matches = [a for a in artifacts if a.get('name') == name]
            require(len(matches) <= 1, 'Duplicate runtime artifact')
            artifact = check_artifact(matches[0], expected, run, started, utcnow()) if matches else None
            if ready and artifact:
                detail = check_artifact(read(f"{prefix}/artifacts/{artifact['id']}"),
                                        expected, run, started, utcnow())
                require(detail == artifact, 'Artifact metadata changed')
                # A rerun must not inherit a handoff observed during the old attempt.
                check_run(read(run_endpoint), expected)
                require(clock() <= deadline, 'Artifact readiness timed out')
                return {'schema': 1, 'owner': 'SKAGER.NativeEnduranceReadiness.1',
                        'status': 'ready', **expected, 'workflow': WORKFLOW,
                        'producer': {'id': job['id'], 'name': PRODUCER,
                                     'status': job['status'], 'conclusion': job.get('conclusion')},
                        'requiredSteps': steps, 'artifact': artifact, 'polls': polls,
                        'elapsedSeconds': round(clock() - begin, 3),
                        'qualification': 'Runtime handoff only; producer, soak and release gates remain required'}
            require(not job or job.get('status') != 'completed',
                    'Producer completed without eligible runtime handoff')
            require(run.get('status') != 'completed', 'Run completed without eligible runtime handoff')
            failures = 0
            progress({'status': 'waiting', 'polls': polls,
                      'artifactVisible': artifact is not None, 'requiredStepsPassed': ready})
        except TransientError:
            failures += 1
            require(failures <= 3, 'Repeated metadata read failures')
            progress({'status': 'waiting', 'polls': polls, 'transientFailures': failures})
        remaining = deadline - clock()
        if remaining > 0:
            sleep(min(interval, remaining))
    raise Refusal('Artifact readiness timed out')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence', type=Path, required=True)
    parser.add_argument('--timeout-seconds', type=int, default=10800)
    parser.add_argument('--poll-seconds', type=int, choices=(30,), default=30)
    args = parser.parse_args()
    args.evidence.mkdir(parents=True, exist_ok=True)
    receipt = args.evidence / 'readiness.json'
    require(not receipt.exists(), 'Readiness receipt already exists')
    def save(document):
        staging = receipt.with_suffix('.tmp')
        staging.write_text(json.dumps(document, indent=2) + '\n')
        staging.replace(receipt)
    expected = {}
    try:
        expected = context(os.environ)
        require(bool(os.environ.get('GH_TOKEN')), 'GH_TOKEN required for metadata reads')
        result = wait_ready(expected, timeout=args.timeout_seconds, interval=args.poll_seconds,
                            progress=lambda state: save({**expected, **state}))
        save(result)
        if os.environ.get('GITHUB_OUTPUT'):
            with open(os.environ['GITHUB_OUTPUT'], 'a') as output:
                output.write(f"artifact-id={result['artifact']['id']}\n")
                output.write(f"artifact-digest={result['artifact']['digest']}\n")
        print('Verified same-run fixture runtime handoff; no qualification implied')
        return 0
    except Refusal as error:
        save({'schema': 1, 'status': 'failed', **expected, 'reason': str(error)})
        print('Runtime handoff refused: ' + str(error))
        return 1


if __name__ == '__main__':
    raise SystemExit(main())

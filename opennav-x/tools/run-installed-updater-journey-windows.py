#!/usr/bin/env python3
"""One explicitly disposable hosted Windows journey; never authorize or publish."""
import argparse
import base64
from datetime import datetime, timezone
import hashlib
import json
import os
import re
from pathlib import Path
import sys
import time

import installed_updater_coordinator as coordinator
import installed_updater_approval_rendezvous as rendezvous
import staging_composition as provenance
import installed_updater_host as host_admission

binding, journey = coordinator.binding, coordinator.journey
FAILURE = 'Installed journey refused or uncertain; preserve evidence and state. No automatic retry.'
OPT_IN = journey.admission.OPT_IN
CONFIG_KEYS = ('schema', 'purpose', 'request', 'runner', 'challenge', 'pendingWorkflow',
    'admission', 'phases', 'artifacts', 'source', 'consumer', 'qualificationParent',
    'consumptionReceipt', 'output', 'phasePlan')


def require(value):
    if not value:
        raise ValueError(FAILURE)


def host(environment, platform, mode='github-hosted'):
    host_admission.environment(environment, platform, mode)


def original(record, limit):
    path, identity = journey.phase.approved_file(record, limit)
    with path.open('rb') as stream:
        raw = stream.read(limit + 1)
    require(len(raw) <= limit and journey.phase.stamp(path) == identity and
            binding.sha256(raw) == record['sha256'])
    return raw


def validate_profile_host(config, environment):
    """Read-only host binding for separately authorized disconnected fixture creation.

    A proposed request/configuration never grants installation or consumes an
    approval. Those remain required later through the original joined coordinator.
    """
    binding._keys(config, CONFIG_KEYS)
    require(type(config['schema']) is int and config['schema'] == 1 and
            config.get('purpose') == 'installed-updater-journey-cli')
    runner = binding._runner(config['runner'])
    request = binding._request(original(config['request'], binding.MAX_RECORD_BYTES))
    require(request.runner == runner and request.challenge == binding._hex(config['challenge']))
    require(request.created_at <= binding._clock(datetime.now(timezone.utc)) < request.expires_at)
    pins = dict(request.pins)
    require(config['admission']['sha256'] == pins['declarationSha256'])
    declared = binding._json(original(config['admission'], 64 << 10), 64 << 10)
    admitted = host_admission.declaration(declared)
    require(admitted is not None and declared['clientRequest']['sha256'] == pins['clientRequestSha256'])
    client = binding._json(original(declared['clientRequest'], 1 << 20), 1 << 20)
    require(client.get('purpose') == 'client-registration' and client.get('clientSid') == admitted['clientSid'])
    reads = coordinator.http.GitHubReads(environment['SKAGER_APPROVAL_READ_TOKEN'])
    host_admission.validate_live(admitted, config['runner'], config['pendingWorkflow'], environment,
                                 sys.platform, reads.read_api)


class Reads:
    repo = binding.REPOSITORY
    base = 'repos/' + repo

    def __init__(self, reads):
        self.reads = reads

    def api(self, endpoint):
        return self.reads.read_api(endpoint, timeout_seconds=30, max_bytes=coordinator.transport.MAX_API_BYTES)


def producer(record, reads):
    """Corroborate API provenance only; prepared archive bytes need prior approval."""
    binding._keys(record, ('artifact', 'workflow', 'jobName'))
    artifact = binding._artifact(record['artifact'], 16 << 30)
    require(type(record['workflow']) is str and re.fullmatch(r'\.github/workflows/[A-Za-z0-9_.-]+\.yml', record['workflow']))
    binding._name(record['jobName'])
    identity = dict(commit=artifact.head_sha, runId=artifact.run_id, runAttempt=artifact.run_attempt)
    api = Reads(reads)
    provenance._run(api, identity, record['workflow'], conclusion='success')
    jobs = api.api(f'{api.base}/actions/runs/{artifact.run_id}/attempts/{artifact.run_attempt}/jobs?per_page=100&page=1')
    require(type(jobs.get('total_count')) is int and type(jobs.get('jobs')) is list and
            jobs['total_count'] == len(jobs['jobs']) <= 100)
    provenance._job(jobs['jobs'], identity, record['jobName'], 'success', artifact.job_id)
    actual = provenance._artifact(api, dict(artifactId=artifact.artifact_id,
        artifactName=artifact.artifact_name, artifactDigest=artifact.artifact_digest), identity, downloadable=True)
    require(actual['size_in_bytes'] == artifact.artifact_bytes)
    return record['artifact']


def phase_locator(runner, challenge, phase, head, plan_sha256):
    require(phase in journey.STEPS)
    binding._hex(head)
    data = dict(schema=1, purpose='installed-updater-phase-ready', runner=runner,
                challenge=challenge, phase=phase, expectedSelectedHeadSha256=head, planSha256=plan_sha256)
    raw = binding._encode(data)
    context = 'installed-b0-phase/' + hashlib.sha256(raw).hexdigest()
    encoded = base64.urlsafe_b64encode(raw).decode('ascii').rstrip('=')
    url = f'https://github.com/{binding.REPOSITORY}/actions/runs/{runner["runId"]}#installed-b0-phase={encoded}'
    require(len(context) <= 100 and len(url) <= 2048)
    return context, url


def phase_handoff(plan, plan_sha256, runner, challenge, creator, reads, *,
                  utcnow=lambda: datetime.now(timezone.utc), monotonic=time.monotonic,
                  sleep=time.sleep, emit=print):
    binding._keys(plan, journey.STEPS)
    for name, value in plan.items():
        if name in ('offline', 'manual-rollback'):
            require(value is None)
        else:
            binding._hex(value)
    binding._decimal(creator)

    def handoff(name, head):
        context, url = phase_locator(runner, challenge, name, head, plan_sha256)
        began, deadline = binding._clock(utcnow()), monotonic() + 900
        emit(json.dumps(dict(phase=name, expectedSelectedHeadSha256=head,
                             context=context, target_url=url), sort_keys=True), flush=True)
        while True:
            matches = []
            for page in range(1, rendezvous.MAX_PAGES + 1):
                remaining = deadline - monotonic()
                require(remaining > 0)
                endpoint = f'repos/{binding.REPOSITORY}/commits/{runner["headSha"]}/statuses?per_page=100&page={page}'
                rows = reads.read_api(endpoint, timeout_seconds=min(30, remaining),
                                      max_bytes=coordinator.transport.MAX_API_BYTES)
                require(monotonic() < deadline and type(rows) is list and len(rows) <= 100)
                for status in rows:
                    require(type(status) is dict and type(status.get('context')) is str)
                    if status['context'].casefold() != context:
                        continue
                    require(status['context'] == context and type(status.get('id')) is int and status['id'] > 0 and
                            status.get('state') == 'success' and type(status.get('creator')) is dict and
                            type(status['creator'].get('id')) is int and str(status['creator']['id']) == creator and
                            status.get('target_url') == url and status.get('updated_at') == status.get('created_at'))
                    require(began < binding._time(status.get('created_at')) <= binding._clock(utcnow()))
                    matches.append(status['id'])
                    require(len(matches) == 1)
                if len(rows) < 100:
                    break
            else:
                raise ValueError(FAILURE)
            if matches:
                return plan[name]  # Existing journey observes actual origin/offline condition.
            require(monotonic() < deadline)
            sleep(min(5, deadline - monotonic()))
    return handoff


def execute(config, environment, host_mode='github-hosted'):
    binding._keys(config, CONFIG_KEYS)
    require(type(config['schema']) is int and config['schema'] == 1 and
            config['purpose'] == 'installed-updater-journey-cli')
    runner = binding._runner(config['runner'])
    require((runner.repository, runner.head_sha, runner.run_id, runner.run_attempt) ==
            (environment['GITHUB_REPOSITORY'], environment['GITHUB_SHA'], environment['GITHUB_RUN_ID'], environment['GITHUB_RUN_ATTEMPT']))
    challenge = binding._hex(config['challenge'])
    request_raw = original(config['request'], binding.MAX_RECORD_BYTES)
    request = binding._request(request_raw)
    require(request.runner == runner and request.challenge == challenge)
    binding._keys(config['source'], ('status_creator_id', 'producer_commit', 'producer_actor_id', 'producer_triggering_actor_id'))
    source = rendezvous.TrustedStatusLocator(**config['source'])
    reads = coordinator.http.GitHubReads(environment['SKAGER_APPROVAL_READ_TOKEN'])
    api = Reads(reads)
    identity = dict(commit=runner.head_sha, runId=runner.run_id, runAttempt=runner.run_attempt)
    pending = provenance._run(api, identity, config['pendingWorkflow'])
    require(pending.get('status') == 'in_progress' and pending.get('event') == 'workflow_dispatch')
    jobs = api.api(f'{api.base}/actions/runs/{runner.run_id}/attempts/{runner.run_attempt}/jobs?per_page=100&page=1')
    require(type(jobs.get('total_count')) is int and type(jobs.get('jobs')) is list and jobs['total_count'] == len(jobs['jobs']) <= 100)
    matching = [job for job in jobs['jobs'] if str(job.get('id')) == runner.job_id]
    require(len(matching) == 1 and matching[0].get('name') == runner.job_name and matching[0].get('status') == 'in_progress' and
            str(matching[0].get('run_id')) == runner.run_id and str(matching[0].get('run_attempt')) == runner.run_attempt and
            matching[0].get('head_sha') == runner.head_sha)
    if host_mode == 'github-hosted':
        require(type(matching[0].get('labels')) is list and 'self-hosted' not in matching[0]['labels'])
    binding._keys(config['artifacts'], ('candidate', 'operatorKit', 'successor'))
    approved = {key: producer(value, reads) for key, value in config['artifacts'].items()}
    require(binding._artifact(approved['candidate'], 16 << 30) == request.candidate and
            binding._artifact(approved['operatorKit'], 768 << 20) == request.operator_kit)
    pins = dict(request.pins)
    admission = config['admission']
    binding._keys(admission, ('path', 'sha256'))
    binding._keys(config['phases'], ('path', 'sha256'))
    initial = journey.admission.prepare(Path(admission['path']), admission['sha256'],
                                       pins['clientRequestSha256'], pins['bootstrapPolicySha256'])
    admitted = initial.get('hostAdmission')
    require((admitted is not None) == (host_mode == host_admission.MODE))
    if admitted is not None:
        host_admission.validate_live(admitted, config['runner'], config['pendingWorkflow'], environment,
                                     sys.platform, reads.read_api)
    phase_record = config['phases']
    phases = journey.phase.prepare(Path(phase_record['path']), phase_record['sha256'])
    journey._same_inputs(initial, phases)
    require(approved['successor']['headSha'] == phases['b1']['commit'])
    plan_raw = original(config['phasePlan'], 4096)
    plan = binding._json(plan_raw, 4096)
    require(plan['no-update'] == pins['timestampSha256'])
    binding._keys(config['consumer'], ('path', 'sha256'))
    binding._hex(config['consumer']['sha256'])
    arguments = dict(request_bytes=request_raw, client_request=original({'path': str(initial['requestPath']), 'sha256': pins['clientRequestSha256']}, 1 << 20),
        declaration=original(admission, 64 << 10), release_policy=original(initial['b0']['policy'], 32 << 10),
        expected_setup=dict(path=str(initial['setup']), sha256=initial['setupSha256'], bytes=initial['setup'].stat().st_size),
        expected_runner=config['runner'], expected_challenge=challenge,
        expected_candidate_artifact=approved['candidate'], expected_operator_kit_artifact=approved['operatorKit'])
    host_options = {'pending_workflow': config['pendingWorkflow']} if admitted is not None else {}
    return coordinator.run_live(source=source, bind_arguments=arguments, initial=initial, phases=phases,
        output=Path(config['output']), handoff=phase_handoff(plan, config['phasePlan']['sha256'], config['runner'], challenge,
            source.status_creator_id, reads), read_only_token=environment['SKAGER_APPROVAL_READ_TOKEN'],
        consumer_executable=config['consumer']['path'], consumer_sha256=config['consumer']['sha256'],
        qualification_parent=config['qualificationParent'], consumption_receipt=config['consumptionReceipt'], **host_options)


def main(arguments=None):
    try:
        class Parser(argparse.ArgumentParser):
            def error(self, message):
                raise ValueError(FAILURE)
        parser = Parser(allow_abbrev=False)
        for name in ('config', 'config-sha256', 'disposable-opt-in'):
            parser.add_argument('--' + name, required=True, action=journey.admission.Once)
        parser.add_argument('--host-mode', default='github-hosted', action=journey.admission.Once)
        args = parser.parse_args(arguments)
        host(os.environ, sys.platform, args.host_mode)  # Before input reads or native effects.
        require(args.disposable_opt_in == OPT_IN)
        config = binding._json(original(dict(path=args.config, sha256=args.config_sha256), 64 << 10), 64 << 10)
        if args.host_mode == 'github-hosted':
            execute(config, os.environ)
        else:
            execute(config, os.environ, args.host_mode)
        print('Installed joined journey completed; customer UAC and product acceptance remain separately qualified.')
        return 0
    except (Exception, KeyboardInterrupt, SystemExit):
        print(FAILURE, file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())

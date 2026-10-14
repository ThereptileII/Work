"""Receive one independently authorized GitHub approval; never grant or enroll.

ApprovalSource must come from the operator's authenticated channel, separately
from the pending runner's proposed request. Merely finding an artifact by name
does not approve it. The reviewed producer is a single-job workflow_dispatch.

read_api(endpoint, timeout_seconds=..., max_bytes=...) returns strictly decoded
GitHub JSON. download_archive(endpoint, timeout_seconds=..., max_bytes=...) returns
original ZIP bytes. Both callbacks MUST enforce their deadline and byte cap
during I/O, not truncate after unbounded reads. They may compose the existing
GitHub client; this module deliberately supplies no second network transport.
Callbacks must not write API resources, follow arbitrary supplied URLs, or retry
failed operations. This module polls only the exact authorized pending run.
"""
from copy import deepcopy
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import io
import json
import stat
import time
import zipfile
import zlib

import installed_updater_approval as binding
import staging_composition as provenance

WORKFLOW = '.github/workflows/skager-installed-approval.yml'
JOB = 'Approve installed B0 originals'
PINS_PURPOSE = 'installed-updater-b0-transport-pins'
MEMBERS = frozenset(('request.json', 'approval.json', 'approval-pins.json'))
MAX_ARCHIVE_BYTES = 64 << 10
MAX_API_BYTES = 256 << 10
FAILURE = 'Installed updater approval transport refused'


def require(condition):
    if not condition:
        raise ValueError(FAILURE)


@dataclass(frozen=True, slots=True)
class ApprovalSource:
    repository: str
    commit: str
    run_id: str
    run_attempt: str
    actor_id: str
    triggering_actor_id: str
    job_id: str
    artifact_id: str
    artifact_name: str
    artifact_digest: str
    artifact_bytes: int


def artifact_name(run_id, run_attempt, request_sha256):
    binding._decimal(run_id)
    binding._decimal(run_attempt)
    binding._hex(request_sha256)
    return f'installed-b0-approval-{run_id}-attempt{run_attempt}-{request_sha256}'


def _source(source, pending):
    require(type(source) is ApprovalSource and source.repository == binding.REPOSITORY and
            source.run_id != pending.runner.run_id)
    binding._hex(source.commit, 40)
    for value in (source.run_id, source.run_attempt, source.actor_id, source.triggering_actor_id,
                  source.job_id, source.artifact_id):
        binding._decimal(value)
    require(source.artifact_name == artifact_name(source.run_id, source.run_attempt, pending.request_sha256) and
            type(source.artifact_digest) is str and source.artifact_digest.startswith('sha256:'))
    binding._hex(source.artifact_digest[7:])
    require(type(source.artifact_bytes) is int and 0 < source.artifact_bytes <= MAX_ARCHIVE_BYTES)
    return dict(commit=source.commit, runId=source.run_id, runAttempt=source.run_attempt)


class _Reads:
    """Only adapt bounded read callbacks to the existing provenance verifiers."""
    repo = binding.REPOSITORY
    base = 'repos/' + repo

    def __init__(self, read_api, remaining):
        self.read_api, self.remaining = read_api, remaining

    def api(self, endpoint):
        require(endpoint.startswith(self.base + '/actions/'))
        value = self.read_api(endpoint, timeout_seconds=min(30, self.remaining()), max_bytes=MAX_API_BYTES)
        self.remaining()
        require(type(value) is dict and len(json.dumps(value, allow_nan=False).encode('utf-8')) <= MAX_API_BYTES)
        return value


def _run(client, source, identity, pending):
    run = provenance._run(client, identity, WORKFLOW)
    require(run.get('event') == 'workflow_dispatch' and
            run.get('repository', {}).get('full_name') == binding.REPOSITORY and
            str(run.get('actor', {}).get('id')) == source.actor_id and
            str(run.get('triggering_actor', {}).get('id')) == source.triggering_actor_id and
            binding._time(run.get('created_at')) > pending.created_at)
    return run


def _unpack(raw, source):
    require(type(raw) is bytes and len(raw) == source.artifact_bytes and
            'sha256:' + hashlib.sha256(raw).hexdigest() == source.artifact_digest)
    result = {}
    with zipfile.ZipFile(io.BytesIO(raw)) as archive:
        entries = archive.infolist()
        require(len(entries) == len(MEMBERS) and {item.filename for item in entries} == MEMBERS)
        for item in entries:
            require(item.orig_filename == item.filename and not item.is_dir() and
                    not item.flag_bits & 1 and
                    stat.S_IFMT(item.external_attr >> 16) in (0, stat.S_IFREG) and
                    item.compress_type in (zipfile.ZIP_STORED, zipfile.ZIP_DEFLATED) and
                    0 < item.file_size <= binding.MAX_RECORD_BYTES)
            # Every member is read to EOF to validate its original size and CRC.
            with archive.open(item) as stream:
                data = stream.read(binding.MAX_RECORD_BYTES + 1)
                require(len(data) == item.file_size and not stream.read(1))
            result[item.filename] = data
    return result


def receive_approval(*, source, read_api, download_archive, bind_arguments,
                     timeout_seconds=3600, poll_seconds=5, cancelled=lambda: False,
                     monotonic=time.monotonic, sleep=time.sleep,
                     utcnow=lambda: datetime.now(timezone.utc)):
    """Return the existing immutable BoundApproval after one authenticated fetch.

bind_arguments contains exactly request_bytes, client_request, declaration,
release_policy, expected_setup, expected_runner, expected_challenge,
expected_candidate_artifact and expected_operator_kit_artifact. Its expected
context and artifacts must already be independently authenticated. Approval pins
come only from the separately authorized producer's approval-pins.json.

Timeout/cancellation/API failure refuses without retrying a download, granting
approval, consuming a challenge, writing files, or invoking enrollment. The
caller owns one-use challenge consumption and immediate pre-execution rechecks.
"""
    require(type(timeout_seconds) in (int, float) and 0 < timeout_seconds <= 3600 and
            type(poll_seconds) in (int, float) and 0 < poll_seconds <= 30)
    binding._keys(bind_arguments, ('request_bytes', 'client_request', 'declaration', 'release_policy',
                                  'expected_setup', 'expected_runner', 'expected_challenge',
                                  'expected_candidate_artifact', 'expected_operator_kit_artifact'))
    arguments = deepcopy(bind_arguments)
    pending = binding.prepare_request(binding._canonical(arguments['request_bytes']),
                                      **{key: arguments[key] for key in
                                         ('client_request', 'declaration', 'release_policy', 'expected_setup')})
    require(pending.runner == binding._runner(arguments['expected_runner']) and
            pending.challenge == binding._hex(arguments['expected_challenge']) and
            pending.candidate == binding._artifact(arguments['expected_candidate_artifact'], 16 << 30) and
            pending.operator_kit == binding._artifact(arguments['expected_operator_kit_artifact'], 768 << 20))
    identity = _source(source, pending)
    deadline = monotonic() + timeout_seconds

    def remaining():
        now = binding._clock(utcnow())
        seconds = min(deadline - monotonic(), (pending.expires_at - now).total_seconds())
        require(not cancelled() and pending.created_at <= now and seconds > 0)
        return seconds

    client = _Reads(read_api, remaining)
    while True:
        run = _run(client, source, identity, pending)
        if run.get('status') == 'completed':
            require(run.get('conclusion') == 'success')
            break
        require(run.get('status') in ('queued', 'requested', 'waiting', 'pending', 'in_progress') and
                run.get('conclusion') is None)
        sleep(min(poll_seconds, remaining()))

    jobs = client.api(f'{client.base}/actions/runs/{source.run_id}/attempts/{source.run_attempt}/jobs?per_page=100&page=1')
    require(type(jobs.get('total_count')) is int and jobs['total_count'] == 1 and
            type(jobs.get('jobs')) is list and len(jobs['jobs']) == 1)
    provenance._job(jobs['jobs'], identity, JOB, 'success', source.job_id)
    job = jobs['jobs'][0]
    pin = dict(artifactId=source.artifact_id, artifactName=source.artifact_name, artifactDigest=source.artifact_digest)
    artifact = provenance._artifact(client, pin, identity, downloadable=True)
    require(artifact['size_in_bytes'] == source.artifact_bytes and
            binding._time(job.get('started_at')) <= binding._time(artifact.get('created_at')) <=
            binding._time(job.get('completed_at')))
    raw = download_archive(f'{client.base}/actions/artifacts/{source.artifact_id}/zip',
                           timeout_seconds=min(30, remaining()), max_bytes=source.artifact_bytes)
    remaining()
    try:
        originals = _unpack(raw, source)
    except (zipfile.BadZipFile, zlib.error, EOFError, RuntimeError, NotImplementedError):
        raise ValueError(FAILURE) from None
    require(originals['request.json'] == arguments['request_bytes'])
    pins = binding._canonical(originals['approval-pins.json'])
    binding._keys(pins, ('schema', 'purpose', 'requestSha256', 'approvalSha256', 'pins'))
    require(type(pins['schema']) is int and pins['schema'] == 1 and pins['purpose'] == PINS_PURPOSE)
    # Recheck the exact producer after download, including root actors. No
    # approval is returned if the observed attempt changed or stopped succeeding.
    run = _run(client, source, identity, pending)
    require(run.get('status') == 'completed' and run.get('conclusion') == 'success')
    result = binding.bind_approval(**arguments, approval_bytes=originals['approval.json'],
                                   approved_request_sha256=pins['requestSha256'],
                                   approved_approval_sha256=pins['approvalSha256'], approved_pins=pins['pins'],
                                   now=utcnow())
    remaining()
    require(binding._clock(utcnow()) < result.expires_at)
    return result

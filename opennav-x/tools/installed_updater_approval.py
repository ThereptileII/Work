"""Byte-only binding for independently approved, postproduction B0 inputs.

This module does not establish who approved anything. Its caller must authenticate
the operator's approval and artifact provenance separately, then supply those
original pins and the live runner context. No I/O, polling, execution or grant
operation is provided here.
"""
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
import hashlib
import json
import re

from product_version import validate_product_version

REPOSITORY = 'ThereptileII/Work'
REQUEST_PURPOSE = 'installed-updater-b0-approval-request'
APPROVAL_PURPOSE = 'installed-updater-b0-postproduction-approval'
MAX_RECORD_BYTES = 16 << 10
MAX_REQUEST_LIFETIME = timedelta(hours=2)
MAX_APPROVAL_LIFETIME = timedelta(minutes=30)
FAILURE = 'Installed updater approval binding refused'
PIN_FIELDS = ('clientRequestSha256', 'declarationSha256', 'bootstrapPolicySha256',
              'provisionerSha256', 'brokerSha256', 'timestampSha256')


def require(condition):
    if not condition:
        raise ValueError(FAILURE)


def sha256(raw):
    require(type(raw) is bytes)
    return hashlib.sha256(raw).hexdigest()


def _keys(value, expected):
    require(type(value) is dict and set(value) == set(expected))


def _hex(value, size=64):
    require(type(value) is str and re.fullmatch('[a-f0-9]{' + str(size) + '}', value) is not None)
    return value


def _decimal(value):
    require(type(value) is str and re.fullmatch('[1-9][0-9]{0,19}', value) is not None)
    return value


def _name(value):
    require(type(value) is str and re.fullmatch('[A-Za-z0-9][A-Za-z0-9_.() -]{0,199}', value) is not None and
            value == value.strip() and not value.endswith('.'))
    return value


def _pairs(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result)
        result[key] = value
    return result


def _integer(value):
    require(len(value) <= 20)
    return int(value)


def _no_number(_):
    raise ValueError(FAILURE)


def _json(raw, limit):
    require(type(raw) is bytes and 0 < len(raw) <= limit)
    try:
        return json.loads(raw.decode('utf-8'), object_pairs_hook=_pairs,
                          parse_int=_integer, parse_float=_no_number, parse_constant=_no_number)
    except (UnicodeError, ValueError, RecursionError):
        raise ValueError(FAILURE) from None


def _encode(value):
    return (json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=True,
                       allow_nan=False) + '\n').encode('ascii')


def _canonical(raw):
    value = _json(raw, MAX_RECORD_BYTES)
    require(_encode(value) == raw)
    return value


def _time(value):
    require(type(value) is str and re.fullmatch(r'[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}Z', value) is not None)
    try:
        return datetime.strptime(value, '%Y-%m-%dT%H:%M:%SZ').replace(tzinfo=timezone.utc)
    except ValueError:
        raise ValueError(FAILURE) from None


def _clock(now):
    require(type(now) is datetime and now.tzinfo is not None and now.utcoffset() == timedelta(0))
    return now


@dataclass(frozen=True, slots=True)
class Runner:
    repository: str
    head_sha: str
    run_id: str
    run_attempt: str
    job_id: str
    job_name: str


def _runner(value):
    _keys(value, ('repository', 'headSha', 'runId', 'runAttempt', 'jobId', 'jobName'))
    require(value['repository'] == REPOSITORY)
    return Runner(REPOSITORY, _hex(value['headSha'], 40), _decimal(value['runId']),
                  _decimal(value['runAttempt']), _decimal(value['jobId']), _name(value['jobName']))


@dataclass(frozen=True, slots=True)
class Artifact:
    repository: str
    head_sha: str
    run_id: str
    run_attempt: str
    job_id: str
    artifact_id: str
    artifact_name: str
    artifact_digest: str
    artifact_bytes: int


def _artifact(value, limit):
    _keys(value, ('repository', 'headSha', 'runId', 'runAttempt', 'jobId', 'artifactId',
                  'artifactName', 'artifactDigest', 'artifactBytes'))
    require(value['repository'] == REPOSITORY and type(value['artifactDigest']) is str and
            value['artifactDigest'].startswith('sha256:'))
    _hex(value['artifactDigest'][7:])
    require(type(value['artifactBytes']) is int and 0 < value['artifactBytes'] <= limit)
    return Artifact(REPOSITORY, _hex(value['headSha'], 40), _decimal(value['runId']),
                    _decimal(value['runAttempt']), _decimal(value['jobId']), _decimal(value['artifactId']),
                    _name(value['artifactName']), value['artifactDigest'], value['artifactBytes'])


def _pins(value):
    _keys(value, PIN_FIELDS)
    return tuple((name, _hex(value[name])) for name in PIN_FIELDS)


@dataclass(frozen=True, slots=True)
class Request:
    original_bytes: bytes
    request_sha256: str  # Observation only; never independent approval.
    runner: Runner
    challenge: str
    created_at: datetime
    expires_at: datetime
    candidate: Artifact
    operator_kit: Artifact
    version: str
    commit: str
    recovery_sha256: str
    source_sha256: str
    policy_sha256: str
    trust_sha256: str
    operator_kit_sha256: str
    release_manifest_sha256: str
    qualification_sha256: str
    pins: tuple[tuple[str, str], ...]


def _request(raw):
    value = _canonical(raw)
    _keys(value, ('schema', 'purpose', 'runner', 'challenge', 'createdAt', 'expiresAt',
                  'candidate', 'operatorKit', 'b0', 'pins'))
    require(type(value['schema']) is int and value['schema'] == 1 and value['purpose'] == REQUEST_PURPOSE)
    runner = _runner(value['runner'])
    created, expires = _time(value['createdAt']), _time(value['expiresAt'])
    require(timedelta(0) < expires - created <= MAX_REQUEST_LIFETIME)
    _keys(value['candidate'], ('artifact', 'releaseManifestSha256', 'qualificationSha256'))
    _keys(value['operatorKit'], ('artifact', 'sha256'))
    candidate = _artifact(value['candidate']['artifact'], 16 << 30)
    kit = _artifact(value['operatorKit']['artifact'], 768 << 20)
    b0 = value['b0']
    _keys(b0, ('version', 'commit', 'recoverySha256', 'sourceSha256', 'policySha256', 'trustSha256'))
    try:
        version = validate_product_version(b0['version'])
    except ValueError:
        raise ValueError(FAILURE) from None
    commit = _hex(b0['commit'], 40)
    require(candidate.head_sha == kit.head_sha == commit and candidate.artifact_id != kit.artifact_id)
    return Request(raw, sha256(raw), runner, _hex(value['challenge']), created, expires, candidate, kit,
                   version, commit, *(_hex(b0[name]) for name in
                   ('recoverySha256', 'sourceSha256', 'policySha256', 'trustSha256')),
                   _hex(value['operatorKit']['sha256']), _hex(value['candidate']['releaseManifestSha256']),
                   _hex(value['candidate']['qualificationSha256']), _pins(value['pins']))


def _originals(request, client_request, declaration):
    require(type(client_request) is bytes and 0 < len(client_request) <= 1 << 20 and
            type(declaration) is bytes and 0 < len(declaration) <= 64 << 10)
    pins = dict(request.pins)
    require(sha256(client_request) == pins['clientRequestSha256'] and
            sha256(declaration) == pins['declarationSha256'])
    client = _json(client_request, 1 << 20)
    declared = _json(declaration, 64 << 10)
    _keys(client, ('schema', 'purpose', 'clientSid', 'installed', 'stock', 'trustConfiguration', 'trustConfigSha256'))
    require(type(client['schema']) is int and client['schema'] == 1 and client['purpose'] == 'client-registration' and
            type(client['clientSid']) is str and re.fullmatch(r'S-1-5-21-[1-9][0-9]*-[1-9][0-9]*-[1-9][0-9]*-[1-9][0-9]*', client['clientSid']) is not None)
    _keys(client['installed'], ('version', 'commit', 'generation', 'packageSha256', 'productBuildSha256',
                                'executableSha256', 'inventorySha256'))
    _hex(client['installed']['generation'], 32)
    for name in ('packageSha256', 'productBuildSha256', 'executableSha256', 'inventorySha256'):
        _hex(client['installed'][name])
    _keys(client['stock'], ('path', 'version', 'arch', 'executableSha256', 'upstreamCommit'))
    _keys(declared, ('schema', 'b0', 'operatorKit', 'clientRequest', 'brokerBootstrapPolicy', 'approvedBaseline',
                     'trustConfig', 'bootstrapCache', 'artifactDirectory', 'preserveSettings'))
    require(type(declared['schema']) is int and declared['schema'] == 1)
    _keys(declared['b0'], ('version', 'commit', 'recovery', 'source', 'policy'))
    require(client['installed']['version'] == declared['b0']['version'] == request.version and
            client['installed']['commit'] == declared['b0']['commit'] == request.commit and
            client['trustConfigSha256'] == request.trust_sha256)
    for record, digest in ((declared['b0']['recovery'], request.recovery_sha256),
                           (declared['b0']['source'], request.source_sha256),
                           (declared['b0']['policy'], request.policy_sha256),
                           (declared['operatorKit'], request.operator_kit_sha256),
                           (declared['clientRequest'], pins['clientRequestSha256']),
                           (declared['brokerBootstrapPolicy'], pins['bootstrapPolicySha256']),
                           (declared['approvedBaseline'], request.recovery_sha256),
                           (declared['trustConfig'], request.trust_sha256)):
        _keys(record, ('path', 'sha256'))
        require(type(record['path']) is str and 0 < len(record['path']) <= 32767 and
                not any(c in record['path'] for c in '\x00\r\n') and _hex(record['sha256']) == digest)
    # Native declaration/stock/trust/file/profile validation stays in the existing
    # admission driver and provisioner. Binding original bytes cannot replace it.


def prepare_request(value, *, client_request, declaration):
    """Own and validate a proposed envelope; produce no approval or file.

    Call only after the two original input files have been produced and retained.
    The returned digest is review data, not authority to approve those inputs.
    """
    request = _request(_encode(value))
    _originals(request, client_request, declaration)
    return request


@dataclass(frozen=True, slots=True)
class BoundApproval:
    request: Request
    original_approval_bytes: bytes
    approval_sha256: str
    issued_at: datetime
    expires_at: datetime
    original_client_request_bytes: bytes
    original_declaration_bytes: bytes


def bind_approval(*, request_bytes, approval_bytes, client_request, declaration,
                  approved_request_sha256, approved_approval_sha256, approved_pins,
                  expected_runner, expected_challenge, expected_candidate_artifact,
                  expected_operator_kit_artifact, now):
    """Bind already independently approved originals to this live pending job.

    All approved_* inputs must arrive through the caller's independently
    authenticated operator channel, not from these records or a self-hash.
    Expected context/artifacts must be independently authenticated by the caller.
    Recheck expiry immediately before using the record. Parsing consumes no nonce.
    """
    _clock(now)
    require(type(request_bytes) is bytes and 0 < len(request_bytes) <= MAX_RECORD_BYTES and
            type(approval_bytes) is bytes and 0 < len(approval_bytes) <= MAX_RECORD_BYTES)
    require(sha256(request_bytes) == _hex(approved_request_sha256) and
            sha256(approval_bytes) == _hex(approved_approval_sha256))
    request = _request(request_bytes)
    require(request.runner == _runner(expected_runner) and request.challenge == _hex(expected_challenge) and
            request.candidate == _artifact(expected_candidate_artifact, 16 << 30) and
            request.operator_kit == _artifact(expected_operator_kit_artifact, 768 << 20) and
            request.pins == _pins(approved_pins) and request.created_at <= now < request.expires_at)
    _originals(request, client_request, declaration)
    approval = _canonical(approval_bytes)
    _keys(approval, ('schema', 'purpose', 'requestSha256', 'runner', 'challenge', 'issuedAt', 'expiresAt'))
    require(type(approval['schema']) is int and approval['schema'] == 1 and approval['purpose'] == APPROVAL_PURPOSE and
            _hex(approval['requestSha256']) == approved_request_sha256 and
            _runner(approval['runner']) == request.runner and _hex(approval['challenge']) == request.challenge)
    issued, expires = _time(approval['issuedAt']), _time(approval['expiresAt'])
    require(request.created_at < issued <= now < expires <= request.expires_at and
            timedelta(0) < expires - issued <= MAX_APPROVAL_LIFETIME)
    return BoundApproval(request, approval_bytes, approved_approval_sha256, issued, expires,
                         client_request, declaration)

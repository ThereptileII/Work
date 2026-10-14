"""One operator-authored commit-status locator; never artifact discovery/grant.

TrustedStatusLocator comes from independently authorized configuration, never the
pending request, current actor or repository owner. The operator posts the exact
ApprovalSource descriptor after reviewing the originals. This reader supplies
that descriptor to the unchanged approval transport; it never follows its URL.
"""
import base64
from dataclasses import asdict, dataclass, fields
from datetime import datetime, timezone
import hashlib
import re
import time

import installed_updater_approval as binding
import installed_updater_approval_transport as transport

MAX_PAGES = 5
FAILURE = 'Installed updater approval rendezvous refused'


def require(value):
    if not value:
        raise ValueError(FAILURE)


@dataclass(frozen=True, slots=True)
class TrustedStatusLocator:
    status_creator_id: str
    producer_commit: str
    producer_actor_id: str
    producer_triggering_actor_id: str

    def __post_init__(self):
        for value in (self.status_creator_id, self.producer_actor_id, self.producer_triggering_actor_id):
            binding._decimal(value)
        binding._hex(self.producer_commit, 40)


def status_context(request):
    require(type(request) is binding.Request)
    raw = binding._encode(dict(runner=asdict(request.runner), challenge=request.challenge,
                               requestSha256=request.request_sha256))
    return 'installed-b0-source/' + hashlib.sha256(raw).hexdigest()


def source_target_url(source):
    """Encode a descriptor already independently approved; grant no authority."""
    require(type(source) is transport.ApprovalSource)
    raw = binding._encode(asdict(source))
    encoded = base64.urlsafe_b64encode(raw).decode('ascii').rstrip('=')
    result = f'https://github.com/{binding.REPOSITORY}/actions/runs/{source.run_id}#installed-b0-source={encoded}'
    require(len(result) <= 2048)
    return result


def _source(url, policy, request):
    require(type(url) is str and len(url) <= 2048)
    pattern = r'https://github\.com/ThereptileII/Work/actions/runs/([1-9][0-9]{0,19})#installed-b0-source=([A-Za-z0-9_-]+)'
    match = re.fullmatch(pattern, url)
    require(match is not None)
    encoded = match[2]
    raw = base64.b64decode(encoded + '=' * (-len(encoded) % 4), altchars=b'-_', validate=True)
    value = binding._canonical(raw)
    binding._keys(value, tuple(f.name for f in fields(transport.ApprovalSource)))
    source = transport.ApprovalSource(**value)
    transport._source(source, request)
    require(source_target_url(source) == url and source.run_id == match[1] and
            source.commit == policy.producer_commit and source.actor_id == policy.producer_actor_id and
            source.triggering_actor_id == policy.producer_triggering_actor_id)
    return source


@dataclass(frozen=True, slots=True)
class ResolvedSource:
    source: transport.ApprovalSource
    status_bytes: bytes


def _scan(policy, request, read_api, remaining, utcnow, check):
    matches = []
    for page in range(1, MAX_PAGES + 1):
        check()
        endpoint = f'repos/{binding.REPOSITORY}/commits/{request.runner.head_sha}/statuses?per_page=100&page={page}'
        rows = read_api(endpoint, timeout_seconds=min(30, remaining()), max_bytes=transport.MAX_API_BYTES)
        remaining()
        require(type(rows) is list and len(rows) <= 100)
        for status in rows:
            require(type(status) is dict and type(status.get('context')) is str)
            if status['context'].casefold() != status_context(request):
                continue
            require(status['context'] == status_context(request) and type(status.get('id')) is int and status['id'] > 0 and
                    status.get('state') == 'success' and type(status.get('creator')) is dict and
                    type(status['creator'].get('id')) is int and
                    str(status['creator']['id']) == policy.status_creator_id)
            created = binding._time(status.get('created_at'))
            require(status.get('updated_at') == status['created_at'] and
                    request.created_at < created <= binding._clock(utcnow()) < request.expires_at)
            source = _source(status.get('target_url'), policy, request)
            # Retain exact attestation fields; mutable creator profile/avatar
            # metadata is not authority. Duplicate/conflicting entries refuse.
            original = binding._encode(dict(id=status['id'], context=status['context'],
                creatorID=policy.status_creator_id, state=status['state'],
                createdAt=status['created_at'], updatedAt=status['updated_at'], targetURL=status['target_url']))
            matches.append(ResolvedSource(source, original))
            require(len(matches) == 1)
        if len(rows) < 100:
            return matches[0] if matches else None
    raise ValueError(FAILURE)  # Never accept a partial pagination prefix.


def resolve_source(*, locator, request, read_api, timeout_seconds=3600, poll_seconds=5,
                   cancelled=lambda: False, check=lambda: None, monotonic=time.monotonic,
                   sleep=time.sleep, utcnow=lambda: datetime.now(timezone.utc), expected=None):
    """Wait for one fixed locator or recheck its exact original attestation.

    expected recheck performs one complete bounded scan, never waits for a
    replacement. This method neither downloads artifacts nor follows URLs.
    """
    require(type(locator) is TrustedStatusLocator and type(request) is binding.Request and
            type(timeout_seconds) in (int, float) and 0 < timeout_seconds <= 3600 and
            type(poll_seconds) in (int, float) and 0 < poll_seconds <= 30 and
            (expected is None or type(expected) is ResolvedSource))
    deadline = monotonic() + timeout_seconds

    def remaining():
        now = binding._clock(utcnow())
        value = min(deadline - monotonic(), (request.expires_at - now).total_seconds())
        require(not cancelled() and request.created_at <= now and value > 0)
        return value

    while True:
        remaining()
        result = _scan(locator, request, read_api, remaining, utcnow, check)
        check()
        remaining()
        if expected is not None:
            require(result == expected)
            return result
        if result is not None:
            return result
        sleep(min(poll_seconds, remaining()))

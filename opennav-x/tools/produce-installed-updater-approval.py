#!/usr/bin/env python3
"""Retain three independently authored approval records; never grant or enroll.

Only an authorized operator may dispatch this producer after reviewing original
pending-job inputs. The receiving job separately authenticates that operator,
this exact producer and artifact. Matching records alone are not authority.
No approval fields or pins are generated from the applicant's request.
"""
import base64
from datetime import datetime, timezone, timedelta
import os
from pathlib import Path
import sys

import installed_updater_approval as binding
import installed_updater_approval_transport as transport

FAILURE = 'Installed updater approval producer refused'
INPUTS = {'request.json': 'SKAGER_REQUEST_B64', 'approval.json': 'SKAGER_APPROVAL_B64',
          'approval-pins.json': 'SKAGER_PINS_B64'}


def require(condition):
    if not condition:
        raise ValueError(FAILURE)


def decode(value):
    require(type(value) is str and 0 < len(value) <= 4 * ((binding.MAX_RECORD_BYTES + 2) // 3))
    raw = base64.b64decode(value, validate=True)
    require(base64.b64encode(raw).decode('ascii') == value and 0 < len(raw) <= binding.MAX_RECORD_BYTES)
    return raw


def validate(records, run_id, run_attempt, now):
    binding._keys(records, transport.MEMBERS)
    binding._decimal(run_id)
    binding._decimal(run_attempt)
    now = binding._clock(now)
    request = binding._request(records['request.json'])
    require(request.runner.run_id != run_id and request.created_at < now < request.expires_at)
    approval = binding._canonical(records['approval.json'])
    binding._keys(approval, ('schema', 'purpose', 'requestSha256', 'runner', 'challenge', 'issuedAt', 'expiresAt'))
    require(type(approval['schema']) is int and approval['schema'] == 1 and
            approval['purpose'] == binding.APPROVAL_PURPOSE and
            binding._hex(approval['requestSha256']) == request.request_sha256 and
            binding._runner(approval['runner']) == request.runner and
            binding._hex(approval['challenge']) == request.challenge)
    issued, expires = binding._time(approval['issuedAt']), binding._time(approval['expiresAt'])
    require(request.created_at < issued <= now < expires <= request.expires_at and
            timedelta(0) < expires - issued <= binding.MAX_APPROVAL_LIFETIME)
    pins = binding._canonical(records['approval-pins.json'])
    binding._keys(pins, ('schema', 'purpose', 'requestSha256', 'approvalSha256', 'pins'))
    require(type(pins['schema']) is int and pins['schema'] == 1 and
            pins['purpose'] == transport.PINS_PURPOSE and
            binding._hex(pins['requestSha256']) == request.request_sha256 and
            binding._hex(pins['approvalSha256']) == binding.sha256(records['approval.json']) and
            binding._pins(pins['pins']) == request.pins)
    return transport.artifact_name(run_id, run_attempt, request.request_sha256)


def retain(records, output, run_id, run_attempt, now):
    """Validate all original bytes before any output; refuse prior destinations."""
    name = validate(records, run_id, run_attempt, now)
    require(isinstance(output, Path) and output.is_absolute())
    require(not any(parent.is_symlink() for parent in (output, *output.parents)))
    output.mkdir(mode=0o700)  # No overwrite, retry or automatic cleanup.
    for filename in sorted(transport.MEMBERS):
        with (output / filename).open('xb') as stream:
            stream.write(records[filename])
    require({p.name for p in output.iterdir()} == transport.MEMBERS and
            all((output / name).read_bytes() == raw for name, raw in records.items()))
    return name


def main():
    try:
        require(len(sys.argv) == 1 and os.environ.get('GITHUB_ACTIONS') == 'true' and
                os.environ.get('GITHUB_EVENT_NAME') == 'workflow_dispatch' and
                os.environ.get('GITHUB_REPOSITORY') == binding.REPOSITORY and
                os.environ.get('GITHUB_JOB') == 'approval' and
                os.environ.get('GITHUB_WORKFLOW_REF', '').startswith(binding.REPOSITORY + '/' + transport.WORKFLOW + '@'))
        run_id, attempt = os.environ['GITHUB_RUN_ID'], os.environ['GITHUB_RUN_ATTEMPT']
        binding._decimal(run_id)
        binding._decimal(attempt)
        root = Path(os.environ['RUNNER_TEMP'])
        require(root.is_absolute() and root.is_dir() and not any(ord(c) < 32 for c in str(root)))
        output = root / ('installed-b0-approval-' + run_id + '-attempt' + attempt)
        records = {filename: decode(os.environ[variable]) for filename, variable in INPUTS.items()}
        name = retain(records, output, run_id, attempt, datetime.now(timezone.utc))
        with Path(os.environ['GITHUB_OUTPUT']).open('a', encoding='utf-8') as stream:
            stream.write('artifact_name=' + name + '\n' + 'directory=' + str(output) + '\n')
        print('Original approval records retained; recipient authorization and native gates remain required.')
        return 0
    except (Exception, KeyboardInterrupt):
        print(FAILURE, file=sys.stderr)  # Never echo submitted data or exception bodies.
        return 1


if __name__ == '__main__':
    raise SystemExit(main())

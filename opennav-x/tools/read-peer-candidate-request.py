#!/usr/bin/env python3
"""Resolve strict reviewed probe inputs without invoking the candidate probe."""
import json
import os
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
FIELDS = {'run': r'[1-9][0-9]{0,19}', 'commit': r'[0-9a-f]{40}',
          'artifact': r'[1-9][0-9]{0,19}', 'digest': r'[0-9a-f]{64}'}
MAX_BYTES = 2048


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('duplicate request field')
        result[key] = value
    return result


def parse_request(data):
    if len(data) > MAX_BYTES:
        raise ValueError('request exceeds 2048 bytes')
    request = json.loads(data.decode('utf-8'), object_pairs_hook=unique_object)
    if not isinstance(request, dict) or set(request) != set(FIELDS):
        raise ValueError('request must contain exactly run, commit, artifact, digest')
    for key, pattern in FIELDS.items():
        if not isinstance(request[key], str) or not re.fullmatch(pattern, request[key]):
            raise ValueError('invalid request field: ' + key)
    return request


def main():
    event = os.environ['GITHUB_EVENT_NAME']
    if event == 'push':
        if os.environ['GITHUB_REF'] != 'refs/heads/skager-candidate-security-probes':
            raise ValueError('request push is restricted to the candidate security probe branch')
        with (ROOT / 'tools/peer-candidate-request.json').open('rb') as source:
            request = parse_request(source.read(MAX_BYTES + 1))
    elif event == 'workflow_dispatch':
        request = parse_request(json.dumps({key: os.environ['CANDIDATE_' + key.upper()]
                                           for key in FIELDS}).encode('utf-8'))
    else:
        raise ValueError('unsupported probe event')
    # Restricted ASCII values cannot introduce workflow output commands/lines.
    with open(os.environ['GITHUB_OUTPUT'], 'a', encoding='utf-8') as output:
        output.write(''.join(key + '=' + request[key] + '\n' for key in FIELDS))


if __name__ == '__main__':
    main()

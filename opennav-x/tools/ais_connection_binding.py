#!/usr/bin/env python3
"""Strict, offline AIS attribution evidence validation; never permits filtering.

The expected image/module inventory must be reviewed and supplied independently.
Matching captured evidence cannot bind the module loader after collection or
establish current socket liveness. No network or filtering APIs are used here.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timedelta, timezone
import ipaddress
import json
import math
from pathlib import Path
import re
import sys
import unicodedata

MAX_EVIDENCE_BYTES = 1024 * 1024
MAX_EXPECTED_BYTES = 128 * 1024
MAX_DIAGNOSTICS_BYTES = 4 * 1024 * 1024
MAX_DEPTH = 12
MAX_NODES = 50000
MAX_STRING = 1024
MAX_MODULES = 256
MAX_TCP_ROWS = 4096
MAX_DNS_ADDRESSES = 128
UINT64_MAX = (1 << 64) - 1
ERROR = 'AIS attribution evidence refused.'
TIMESTAMP = re.compile(r'[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}Z\Z')
HEX_ADDRESS = re.compile(r'[0-9a-f]{32}\Z')
HEX_HASH = re.compile(r'[0-9a-f]{64}\Z')
DECIMAL = re.compile(r'(?:0|[1-9][0-9]{0,19})\Z')
ROOT_KEYS = {'schema', 'purpose', 'started_utc', 'finished_utc',
             'observation_before', 'observation_after', 'process_before',
             'process_after', 'tcp_before', 'tcp_after', 'dns_before', 'dns_after'}
OBSERVATION_KEYS = {'schema', 'source', 'available', 'reason', 'process_id',
                    'observed_utc', 'family', 'generation', 'captured_monotonic_ns',
                    'observed_monotonic_ns', 'age_ms', 'local', 'remote'}
PROCESS_KEYS = {'pid', 'creation_filetime', 'image_path', 'image_sha256',
                'modules', 'same_image_processes'}


class EvidenceError(ValueError):
    """A redacted refusal safe to show without leaking input values."""

    def __init__(self):
        super().__init__(ERROR)


def _require(condition):
    if not condition:
        raise EvidenceError()


def _bounded(value, allow_floats=False):
    """Bound even direct Python callers, including cyclic or exotic objects."""
    pending = [(value, 0)]
    count = 0
    while pending:
        item, depth = pending.pop()
        count += 1
        _require(count <= MAX_NODES and depth <= MAX_DEPTH)
        kind = type(item)
        if kind is dict:
            _require(len(item) <= MAX_TCP_ROWS)
            for key, child in item.items():
                _require(type(key) is str and len(key) <= MAX_STRING and
                         not any(0xd800 <= ord(c) <= 0xdfff for c in key))
                pending.append((child, depth + 1))
        elif kind is list:
            _require(len(item) <= MAX_TCP_ROWS)
            pending.extend((child, depth + 1) for child in item)
        elif kind is str:
            _require(len(item) <= MAX_STRING and
                     not any(0xd800 <= ord(c) <= 0xdfff for c in item))
        else:
            _require(kind in (int, bool, type(None)) or (allow_floats and kind is float))
            if kind is int:
                _require((-UINT64_MAX if allow_floats else 0) <= item <= UINT64_MAX)
            elif kind is float:
                _require(math.isfinite(item))


def _object(value, keys):
    _require(type(value) is dict and set(value) == keys)
    return value


def _integer(value, minimum=0, maximum=(1 << 32) - 1):
    _require(type(value) is int and minimum <= value <= maximum)
    return value


def _decimal(value, positive=False):
    _require(type(value) is str and DECIMAL.fullmatch(value) is not None)
    result = int(value)
    _require((1 if positive else 0) <= result <= UINT64_MAX)
    return result


def _time(value):
    _require(type(value) is str and TIMESTAMP.fullmatch(value) is not None)
    try:
        return datetime.strptime(value, '%Y-%m-%dT%H:%M:%SZ').replace(tzinfo=timezone.utc)
    except ValueError:
        raise EvidenceError() from None


def _now(value):
    if type(value) is str:
        return _time(value)
    _require(type(value) is datetime and value.tzinfo is not None)
    _require(value.utcoffset() == timedelta(0))
    return value


def _path(value):
    # Accept only one conservative DOS spelling. UNC, ADS, device, relative,
    # short-name and normalization aliases cannot establish independent pins.
    _require(type(value) is str and 4 <= len(value) <= MAX_STRING)
    _require(re.match(r'[A-Za-z]:\\', value) is not None)
    _require(unicodedata.normalize('NFC', value) == value)
    parts = value[3:].split('\\')
    for part in parts:
        _require(bool(part) and part not in ('.', '..') and part[-1] not in '. ')
        _require(not any(ord(c) < 32 or c in '<>:"/|?*~' for c in part))
        stem = part.split('.')[0].rstrip(' ').casefold()
        _require(stem not in {'con', 'prn', 'aux', 'nul', 'clock$', 'conin$', 'conout$'})
        _require(re.fullmatch(r'(?:com|lpt)[0-9¹²³]', stem) is None)
    return value


def _hash(value):
    _require(type(value) is str and HEX_HASH.fullmatch(value) is not None)
    return value


def _modules(value):
    _require(type(value) is list and 1 <= len(value) <= MAX_MODULES)
    result = {}
    folded = set()
    for module in value:
        _object(module, {'path', 'sha256'})
        path = _path(module['path'])
        key = path.casefold()
        _require(key not in folded)
        folded.add(key)
        result[path] = _hash(module['sha256'])
    return result


def _address(family, value, scope, public=False):
    _require(type(family) is str and family in ('ipv4', 'ipv6'))
    _require(type(value) is str and HEX_ADDRESS.fullmatch(value) is not None)
    _integer(scope)
    raw = bytes.fromhex(value)
    if family == 'ipv4':
        _require(raw[4:] == bytes(12) and scope == 0 and raw[0] != 0)
        addr = ipaddress.IPv4Address(raw[:4])
    else:
        addr = ipaddress.IPv6Address(raw)
        _require(addr.ipv4_mapped is None)
        _require((addr.is_link_local and scope > 0) or
                 (not addr.is_link_local and scope == 0))
    _require(not (addr.is_unspecified or addr.is_multicast or addr.is_reserved))
    if public:
        _require(addr.is_global and not addr.is_loopback and not addr.is_link_local)
    return (family, value, scope)


def _endpoint(value, family, remote=False):
    _object(value, {'address', 'port', 'scope'})
    address = _address(family, value['address'], value['scope'], public=remote)
    port = _integer(value['port'], 1, 65535)
    if remote:
        _require(port == 443)
    return address + (port,)


def _observation(value):
    _object(value, OBSERVATION_KEYS)
    _require(type(value['schema']) is int and value['schema'] == 1)
    _require(value['source'] == 'AISSTREAM_ONLINE' and value['available'] is True)
    _require(value['reason'] == 'observed')
    pid = _integer(value['process_id'], 1)
    observed = _time(value['observed_utc'])
    generation = _decimal(value['generation'], positive=True)
    captured_ns = _decimal(value['captured_monotonic_ns'])
    observed_ns = _decimal(value['observed_monotonic_ns'])
    age_ms = _decimal(value['age_ms'])
    _require(captured_ns <= observed_ns and
             age_ms == (observed_ns - captured_ns) // 1000000)
    family = value['family']
    local = _endpoint(value['local'], family)
    remote = _endpoint(value['remote'], family, remote=True)
    return {'identity': (pid, generation, captured_ns, family, local, remote),
            'pid': pid, 'utc': observed, 'monotonic_ns': observed_ns,
            'tuple': (pid, family, local, remote), 'remote': remote[:3]}


def _process(value):
    _object(value, PROCESS_KEYS)
    pid = _integer(value['pid'], 1)
    created = _decimal(value['creation_filetime'], positive=True)
    path = _path(value['image_path'])
    digest = _hash(value['image_sha256'])
    modules = _modules(value['modules'])
    peers = value['same_image_processes']
    _require(type(peers) is list and len(peers) == 1)
    _object(peers[0], {'pid', 'creation_filetime'})
    _require(_integer(peers[0]['pid'], 1) == pid and
             _decimal(peers[0]['creation_filetime'], positive=True) == created)
    return (pid, created, path, digest, modules)


def _tcp(value, selected):
    _require(type(value) is list and 1 <= len(value) <= MAX_TCP_ROWS)
    matches = 0
    tuple_matches = 0
    for row in value:
        _object(row, {'pid', 'state', 'family', 'local', 'remote'})
        pid = _integer(row['pid'], 1)
        _require(row['state'] == 'ESTABLISHED')
        family = row['family']
        local = _endpoint(row['local'], family)
        # Other established TCP connections need not use TLS port 443 or a
        # public peer; only the selected AIS tuple has those requirements.
        remote = _endpoint(row['remote'], family)
        matches += (pid, family, local, remote) == selected
        tuple_matches += (family, local, remote) == selected[1:]
    _require(matches == 1 and tuple_matches == 1)


def _dns(value):
    _object(value, {'host', 'addresses'})
    _require(value['host'] == 'stream.aisstream.io')
    addresses = value['addresses']
    _require(type(addresses) is list and 1 <= len(addresses) <= MAX_DNS_ADDRESSES)
    result = set()
    for row in addresses:
        _object(row, {'family', 'address', 'scope'})
        address = _address(row['family'], row['address'], row['scope'], public=True)
        _require(address not in result)
        result.add(address)
    return result


def validate_expected(expected):
    """Validate independent reviewed pins before a collector opens any paths."""
    _bounded(expected)
    _object(expected, {'schema', 'image_path', 'image_sha256', 'modules'})
    _require(type(expected['schema']) is int and expected['schema'] == 1)
    image = _path(expected['image_path'])
    digest = _hash(expected['image_sha256'])
    modules = _modules(expected['modules'])
    for path, module_digest in modules.items():
        if path.casefold() == image.casefold():
            _require(path == image and module_digest == digest)
    return expected


def validate(evidence, expected, now):
    """Return a bounded observation status or raise a redacted EvidenceError.

    ``now`` is an explicit canonical UTC timestamp or aware UTC datetime. Old
    connection capture times are allowed: freshness applies to the live reads,
    and capture age alone is neither liveness evidence nor an expiry policy.
    """
    _bounded(evidence)
    validate_expected(expected)
    _object(evidence, ROOT_KEYS)
    _require(type(evidence['schema']) is int and evidence['schema'] == 1)
    _require(evidence['purpose'] == 'read-only-ais-attribution')
    pins = (_path(expected['image_path']), _hash(expected['image_sha256']),
            _modules(expected['modules']))
    current = _now(now)
    started = _time(evidence['started_utc'])
    finished = _time(evidence['finished_utc'])
    _require(started <= finished <= current and finished - started <= timedelta(seconds=15))
    before = _observation(evidence['observation_before'])
    after = _observation(evidence['observation_after'])
    for sample in (before, after):
        _require(started <= sample['utc'] <= finished)
        _require(timedelta(0) <= current - sample['utc'] <= timedelta(seconds=10))
    _require(before['utc'] <= after['utc'] and
             before['monotonic_ns'] < after['monotonic_ns'] and
             before['identity'] == after['identity'])
    process_before = _process(evidence['process_before'])
    process_after = _process(evidence['process_after'])
    _require(process_before == process_after and process_before[0] == before['pid'])
    _require(process_before[2:] == pins)
    _tcp(evidence['tcp_before'], before['tuple'])
    _tcp(evidence['tcp_after'], after['tuple'])
    dns_before = _dns(evidence['dns_before'])
    dns_after = _dns(evidence['dns_after'])
    _require(dns_before == dns_after and before['remote'] in dns_before)
    return {'schema': 1, 'state': 'OBSERVATION_MATCHED', 'authorizes_filter': False}


def _pairs(pairs):
    result = {}
    for key, value in pairs:
        _require(key not in result)
        result[key] = value
    return result


def _reject_constant(_value):
    raise EvidenceError()


def _json_integer(value):
    _require(len(value) <= 20 and not value.startswith('-'))
    result = int(value)
    _require(result <= UINT64_MAX)
    return result


def _finite_float(value):
    _require(len(value) <= 32)
    result = float(value)
    _require(math.isfinite(result))
    return result


def _diagnostic_integer(value):
    _require(len(value.lstrip('-')) <= 20)
    result = int(value)
    _require(-UINT64_MAX <= result <= UINT64_MAX)
    return result


def strict_json_loads(raw, limit=MAX_EVIDENCE_BYTES, *, allow_floats=False):
    """Decode bounded JSON with no duplicate keys or nonfinite numbers.

    ``allow_floats`` is only for reading a full product diagnostic before
    extracting its exact observation object. Evidence/expected inputs and
    ``validate`` always reject floats, signed integers and other invalid types.
    """
    try:
        _require(type(raw) in (bytes, str))
        _require(type(allow_floats) is bool)
        maximum = MAX_DIAGNOSTICS_BYTES if allow_floats else MAX_EVIDENCE_BYTES
        _require(type(limit) is int and 0 < limit <= maximum)
        if type(raw) is str:
            _require(len(raw) <= limit)
            raw = raw.encode('utf-8', errors='strict')
        _require(len(raw) <= limit)
        text = raw.decode('utf-8', errors='strict')
        # Count structure before decoding, ignoring escaped/string brackets.
        depth = 0
        in_string = False
        escaped = False
        for char in text:
            if in_string:
                if escaped:
                    escaped = False
                elif char == '\\':
                    escaped = True
                elif char == '"':
                    in_string = False
            elif char == '"':
                in_string = True
            elif char in '[{':
                depth += 1
                _require(depth <= MAX_DEPTH)
            elif char in ']}':
                depth -= 1
                _require(depth >= 0)
        result = json.loads(text, object_pairs_hook=_pairs,
                            parse_constant=_reject_constant,
                            parse_float=_finite_float if allow_floats else _reject_constant,
                            parse_int=_diagnostic_integer if allow_floats else _json_integer)
        _bounded(result, allow_floats=allow_floats)
        return result
    except (UnicodeError, ValueError, RecursionError):
        raise EvidenceError() from None


def read_json(path, limit=MAX_EVIDENCE_BYTES):
    """Read only limit+1 bytes and apply the strict evidence JSON decoder."""
    try:
        _require(type(limit) is int and 0 < limit <= MAX_EVIDENCE_BYTES)
        with Path(path).open('rb') as stream:
            raw = stream.read(limit + 1)
        return strict_json_loads(raw, limit)
    except (OSError, ValueError, RecursionError):
        raise EvidenceError() from None


class _Parser(argparse.ArgumentParser):
    def error(self, _message):
        raise EvidenceError()


def main(argv=None):
    try:
        parser = _Parser(description=__doc__)
        parser.add_argument('--evidence', required=True)
        parser.add_argument('--expected', required=True)
        args = parser.parse_args(argv)
        evidence = read_json(args.evidence)
        expected = read_json(args.expected, MAX_EXPECTED_BYTES)
        result = validate(evidence, expected, datetime.now(timezone.utc))
    except EvidenceError:
        print(ERROR, file=sys.stderr)
        return 2
    print(json.dumps(result, sort_keys=True, separators=(',', ':')))
    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""Operator-only, read-only native Windows AIS attribution (Python 3.8+ stdlib).

This is a qualification helper, not a product runtime dependency or a filter.
It opens no application configuration and sends only a DNS lookup for the fixed
AIS hostname. Disk hashes describe retained on-disk files, not mapped bytes.
Two inventories cannot prove absence of transient loads between observations.
Synchronous native calls are checked after return; the CLI's separate collector
child has a watchdog. Only that child can be terminated, never the application.
Native Windows fixture qualification is required before relying on this tool.
"""
import argparse
import ctypes
from datetime import datetime, timezone
import hashlib
import ipaddress
import json
import multiprocessing
import ntpath
import os
import re
import queue
import socket
import sys
import time
import threading
import unicodedata

import ais_connection_binding as binding

HOST = 'stream.aisstream.io'
MAX_DIAGNOSTICS = 4 * 1024 * 1024
MAX_EVIDENCE = 1024 * 1024
MAX_EXPECTED = 128 * 1024
MAX_MODULES = 256
MAX_TCP_ROWS = 4096
MAX_TABLE_BYTES = 1024 * 1024
MAX_DNS = 128
MAX_FILE_BYTES = 128 * 1024 * 1024
MAX_HASH_BYTES = 512 * 1024 * 1024
WATCHDOG_SECONDS = 15
REFUSAL = 'AIS attribution refused.\n'


class CollectorError(ValueError):
    """Details are deliberately never rendered by the operator CLI."""


def require(condition):
    if not condition:
        raise CollectorError('collection unavailable')


def local_path(value):
    """Accept only explicit, unambiguous local DOS paths, including outputs."""
    require(isinstance(value, str) and 3 < len(value) <= 1024)
    require(unicodedata.normalize('NFC', value) == value)
    require(re.match(r'^[A-Za-z]:\\', value) is not None)
    require('/' not in value and ntpath.normpath(value) == value)
    reserved = re.compile(r'^(?:CON|PRN|AUX|NUL|CLOCK\$|CONIN\$|CONOUT\$|COM[0-9¹²³]|LPT[0-9¹²³])(?:\.|$)', re.I)
    for part in value[3:].split('\\'):
        require(bool(part) and part not in ('.', '..'))
        require(not part.endswith((' ', '.')) and not reserved.match(part))
        require(not any(ord(c) < 32 or c in ':<>"|?*~' for c in part))
    return value


def key(path):
    return local_path(path).casefold()


def utc_stamp(value):
    require(value.tzinfo is not None)
    return value.astimezone(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')


class Clock:
    monotonic = staticmethod(time.monotonic)
    utc = staticmethod(lambda: datetime.now(timezone.utc))
    sleep = staticmethod(time.sleep)


class Budget:
    def __init__(self, clock):
        self.clock = clock
        self.started_mono = self.last_mono = clock.monotonic()
        self.started_utc = self.last_utc = clock.utc()
        require(self.started_utc.tzinfo is not None)

    def check(self):
        mono, utc = self.clock.monotonic(), self.clock.utc()
        elapsed = mono - self.started_mono
        utc_elapsed = (utc - self.started_utc).total_seconds()
        require(mono >= self.last_mono and utc >= self.last_utc)
        require(0 <= elapsed <= 15 and 0 <= utc_elapsed <= 15)
        require(abs(utc_elapsed - elapsed) <= 1)
        self.last_mono, self.last_utc = mono, utc
        return utc

    def wait_publication(self):
        self.clock.sleep(1.1)
        self.check()


def diagnostic_json(raw):
    """Parse the full report strictly, permitting unrelated finite sensor values.

    Only the extracted observation enters the final strict evidence validator.
    """
    return binding.strict_json_loads(raw, limit=MAX_DIAGNOSTICS, allow_floats=True)


def observation(raw, pid):
    report = diagnostic_json(raw)
    require(type(report) is dict)
    require(report.get('build_purpose') == 'INSTALLED PRODUCT')
    require(report.get('data_mode') == 'OPENCPN selected navigation')
    require(report.get('test_fixtures') is False)
    require(isinstance(report.get('build_commit'), str) and
            re.fullmatch('[0-9a-f]{40}', report['build_commit']) is not None)
    runtime = report.get('runtime')
    require(type(runtime) is dict and type(runtime.get('online_ais')) is dict)
    row = runtime['online_ais'].get('connection_observation')
    require(type(row) is dict and type(row.get('process_id')) is int and row['process_id'] == pid)
    # Validate only the extracted structure's strict JSON domain now. The final
    # binding validates its exact keys, timestamps and relationship to sockets.
    return binding.strict_json_loads(json.dumps(row, ensure_ascii=True).encode(), limit=MAX_EVIDENCE)


def collect(pid, diagnostics, expected_path, *, provider_factory=None, clock=None):
    """Portable orchestration seam; native provider is never called by tests."""
    require(type(pid) is int and 0 < pid <= 0xffffffff)
    for path in (diagnostics, expected_path):
        local_path(path)
    budget = Budget(clock or Clock())
    provider = (provider_factory or NativeWindows)(budget)
    try:
        expected = binding.strict_json_loads(provider.expected_bytes(expected_path), limit=MAX_EXPECTED)
        binding.validate_expected(expected)
        budget.check()
        provider.prepare(pid, expected)
        budget.check()
        budget.wait_publication()
        evidence = {'schema': 1, 'purpose': 'read-only-ais-attribution',
                    'started_utc': utc_stamp(budget.started_utc)}
        for phase in ('before', 'after'):
            if phase == 'after':
                budget.wait_publication()
            evidence['process_' + phase] = provider.process_sample()
            budget.check()
            evidence['tcp_' + phase] = provider.tcp_sample(pid)
            budget.check()
            evidence['dns_' + phase] = provider.dns_sample()
            budget.check()
            evidence['observation_' + phase] = observation(provider.diagnostics_bytes(diagnostics), pid)
            budget.check()
        provider.verify_retained()
        evidence['finished_utc'] = utc_stamp(budget.check())
    finally:
        # Closing every owner is required even if a sample, validation or close
        # fails. No output is opened while any native owner remains held.
        provider.close()
    now = budget.check()
    verdict = binding.validate(evidence, expected, now)
    require(verdict == {'schema': 1, 'state': 'OBSERVATION_MATCHED', 'authorizes_filter': False})
    encoded = json.dumps(evidence, sort_keys=True, separators=(',', ':'), ensure_ascii=True).encode('ascii')
    require(0 < len(encoded) <= MAX_EVIDENCE)
    return encoded


class NativeWindows:
    """Read-only process, file, TCP-table and fixed-host DNS Windows provider."""
    def __init__(self, budget):
        require(sys.platform == 'win32')
        from ctypes import wintypes as w
        self.w, self.budget = w, budget
        self.kernel = ctypes.WinDLL('kernel32', use_last_error=True, winmode=0x00000800)
        self.psapi = ctypes.WinDLL('psapi', use_last_error=True, winmode=0x00000800)
        self.iphlp = ctypes.WinDLL('iphlpapi', use_last_error=True, winmode=0x00000800)
        self.handles, self.retained, self.hash_bytes = [], {}, 0
        self.process, self.pid, self.expected = None, None, None
        self._configure()

    def _configure(self):
        w = self.w
        class Info(ctypes.Structure):
            _fields_ = [('attributes', w.DWORD), ('creation', w.FILETIME), ('access', w.FILETIME),
                        ('write', w.FILETIME), ('volume', w.DWORD), ('size_high', w.DWORD),
                        ('size_low', w.DWORD), ('links', w.DWORD), ('index_high', w.DWORD), ('index_low', w.DWORD)]
        class ProcessEntry(ctypes.Structure):
            _fields_ = [('size', w.DWORD), ('usage', w.DWORD), ('pid', w.DWORD),
                        ('heap', ctypes.c_size_t), ('module', w.DWORD), ('threads', w.DWORD),
                        ('parent', w.DWORD), ('priority', w.LONG), ('flags', w.DWORD), ('exe', w.WCHAR * 260)]
        class Tcp4(ctypes.Structure):
            _fields_ = [(name, w.DWORD) for name in ('state', 'local_address', 'local_port', 'remote_address', 'remote_port', 'pid')]
        class Tcp6(ctypes.Structure):
            _fields_ = [('local_address', ctypes.c_ubyte * 16), ('local_scope', w.DWORD),
                        ('local_port', w.DWORD), ('remote_address', ctypes.c_ubyte * 16),
                        ('remote_scope', w.DWORD), ('remote_port', w.DWORD), ('state', w.DWORD), ('pid', w.DWORD)]
        self.Info, self.ProcessEntry, self.Tcp4, self.Tcp6 = Info, ProcessEntry, Tcp4, Tcp6
        specs = [
            ('OpenProcess', w.HANDLE, [w.DWORD, w.BOOL, w.DWORD]),
            ('CloseHandle', w.BOOL, [w.HANDLE]),
            ('WaitForSingleObject', w.DWORD, [w.HANDLE, w.DWORD]),
            ('QueryFullProcessImageNameW', w.BOOL, [w.HANDLE, w.DWORD, w.LPWSTR, ctypes.POINTER(w.DWORD)]),
            ('GetProcessTimes', w.BOOL, [w.HANDLE] + [ctypes.POINTER(w.FILETIME)] * 4),
            ('CreateFileW', w.HANDLE, [w.LPCWSTR, w.DWORD, w.DWORD, ctypes.c_void_p, w.DWORD, w.DWORD, w.HANDLE]),
            ('GetFileType', w.DWORD, [w.HANDLE]),
            ('GetCurrentProcess', w.HANDLE, []),
            ('DuplicateHandle', w.BOOL, [w.HANDLE, w.HANDLE, w.HANDLE,
                                       ctypes.POINTER(w.HANDLE), w.DWORD, w.BOOL, w.DWORD]),
            ('SetFileInformationByHandle', w.BOOL, [w.HANDLE, ctypes.c_int, ctypes.c_void_p, w.DWORD]),
            ('GetFileInformationByHandle', w.BOOL, [w.HANDLE, ctypes.POINTER(Info)]),
            ('GetFinalPathNameByHandleW', w.DWORD, [w.HANDLE, w.LPWSTR, w.DWORD, w.DWORD]),
            ('GetDriveTypeW', w.UINT, [w.LPCWSTR]),
            ('ReadFile', w.BOOL, [w.HANDLE, ctypes.c_void_p, w.DWORD, ctypes.POINTER(w.DWORD), ctypes.c_void_p]),
            ('CreateToolhelp32Snapshot', w.HANDLE, [w.DWORD, w.DWORD]),
            ('Process32FirstW', w.BOOL, [w.HANDLE, ctypes.POINTER(ProcessEntry)]),
            ('Process32NextW', w.BOOL, [w.HANDLE, ctypes.POINTER(ProcessEntry)]),
        ]
        for name, restype, argtypes in specs:
            fn = getattr(self.kernel, name)
            fn.restype, fn.argtypes = restype, argtypes
        self.psapi.EnumProcessModulesEx.restype = w.BOOL
        self.psapi.EnumProcessModulesEx.argtypes = [w.HANDLE, ctypes.POINTER(w.HMODULE), w.DWORD, ctypes.POINTER(w.DWORD), w.DWORD]
        self.psapi.GetModuleFileNameExW.restype = w.DWORD
        self.psapi.GetModuleFileNameExW.argtypes = [w.HANDLE, w.HMODULE, w.LPWSTR, w.DWORD]
        self.iphlp.GetExtendedTcpTable.restype = w.DWORD
        self.iphlp.GetExtendedTcpTable.argtypes = [ctypes.c_void_p, ctypes.POINTER(w.DWORD), w.BOOL, w.ULONG, w.DWORD, w.ULONG]

    def call(self, function, *args):
        self.budget.check()
        result = function(*args)
        self.budget.check()
        return result

    def open_handle(self, function, *args, on_acquired=None):
        # Own returned handles before checking the post-call deadline. A slow
        # successful native open must still be drained on refusal.
        self.budget.check()
        require(len(self.handles) < 4096)
        handle = function(*args)
        if handle not in (None, 0, ctypes.c_void_p(-1).value):
            self.handles.append(handle)
            if on_acquired is not None:
                on_acquired(handle)
        self.budget.check()
        require(handle not in (None, 0, ctypes.c_void_p(-1).value))
        return handle

    def close_one(self, handle):
        if handle in self.handles:
            # Closing is never skipped merely because a budget has elapsed.
            okay = bool(self.kernel.CloseHandle(handle))
            if okay:
                self.handles.remove(handle)
            require(okay)

    def release(self, handle):
        self.close_one(handle)
        self.budget.check()

    def close(self):
        okay = True
        # No budget checks during drain: deadline exhaustion must still close
        # every owner, and cannot replace a failed collection with partial data.
        for handle in list(reversed(self.handles)):
            try:
                if self.kernel.CloseHandle(handle):
                    self.handles.remove(handle)
                else:
                    okay = False
            except Exception:
                okay = False
        require(okay)

    def path_parts(self, path):
        local_path(path)
        require(self.call(self.kernel.GetDriveTypeW, path[:3]) in (2, 3, 6))
        # GetFinalPathNameByHandle plus OPEN_REPARSE_POINT on every component
        # rejects aliases/reparse traversal, including parent junctions.
        parts = path[3:].split('\\')
        require(len(parts) <= 64)
        return [path[:3] + '\\'.join(parts[:n]) for n in range(1, len(parts))]

    def lock_parents(self, path, *, retain):
        parents = []
        for parent in self.path_parts(path):
            handle = self.open_handle(self.kernel.CreateFileW, parent, 0x80,
                                        1 if retain else 1 | 2, None, 3,
                                        0x02000000 | 0x00200000, None)
            parents.append(handle)
            self.file_identity(handle, parent, directory=True)
        return parents

    def open_plain(self, path, *, retain=False, directory=False, pinned_binary=False):
        w = self.w
        parents = []
        try:
            parents = self.lock_parents(path, retain=retain)
            flags = 0x00200000 | (0x02000000 if directory else 0)
            share = 1 if retain else 1 | 2 | 4
            handle = self.open_handle(self.kernel.CreateFileW, path, 0x80 if directory else 0x80000000,
                                        share, None, 3, flags, None)
            stamp = self.file_identity(handle, path, directory=directory, pinned_binary=pinned_binary)
            # Keep parent handles (deny DELETE) with retained image/module
            # handles, preventing namespace replacement until the drain.
            if not retain:
                for parent in parents:
                    self.release(parent)
            return handle, stamp
        except Exception:
            # All owners, including the partly acquired parents, remain in the
            # owner list for the collection's final drain.
            raise

    def file_identity(self, handle, path, *, directory=False, pinned_binary=False):
        w = self.w
        require(self.call(self.kernel.GetFileType, handle) == 1)
        info = self.Info()
        require(bool(self.call(self.kernel.GetFileInformationByHandle, handle, ctypes.byref(info))))
        require(not (info.attributes & 0x400))
        require(bool(info.attributes & 0x10) == directory)
        # Windows system DLLs can be component-store hardlinks. Only reviewed,
        # retained binary files permit that; manifest, diagnostic and output
        # files keep the generic single-link rule. Identity/linkcount remain
        # stable and the file handle denies WRITE/DELETE across all its names.
        require(directory or (info.links >= 1 if pinned_binary else info.links == 1))
        final = ctypes.create_unicode_buffer(1025)
        size = self.call(self.kernel.GetFinalPathNameByHandleW, handle, final, len(final), 0)
        require(0 < size < len(final) and final.value.startswith('\\\\?\\'))
        require(key(final.value[4:]) == key(path))
        filetime = lambda t: (t.dwHighDateTime << 32) | t.dwLowDateTime
        return (info.volume, info.index_high, info.index_low, info.attributes, info.links,
                (info.size_high << 32) | info.size_low, filetime(info.creation), filetime(info.write))

    def read_handle(self, handle, size, limit):
        require(0 <= size <= limit)
        result = bytearray()
        while len(result) < size:
            count = min(65536, size - len(result))
            block, read = ctypes.create_string_buffer(count), self.w.DWORD()
            require(bool(self.call(self.kernel.ReadFile, handle, block, count, ctypes.byref(read), None)))
            require(0 < read.value <= count)
            result.extend(block.raw[:read.value])
        # Explicit EOF prevents a growing file from producing a truncated pass.
        block, read = ctypes.create_string_buffer(1), self.w.DWORD()
        require(bool(self.call(self.kernel.ReadFile, handle, block, 1, ctypes.byref(read), None)))
        require(read.value == 0)
        return bytes(result)

    def read_path(self, path, limit, *, retain):
        handle, before = self.open_plain(path, retain=retain)
        raw = self.read_handle(handle, before[5], limit)
        require(self.file_identity(handle, path) == before)
        if retain:
            self.retained[key(path)] = (handle, path, before, None, False)
        else:
            self.release(handle)
        return raw

    def expected_bytes(self, path):
        return self.read_path(path, MAX_EXPECTED, retain=True)

    def diagnostics_bytes(self, path):
        return self.read_path(path, MAX_DIAGNOSTICS, retain=False)

    def identity(self, handle, pid):
        require(self.call(self.kernel.WaitForSingleObject, handle, 0) == 0x102)
        buffer, size = ctypes.create_unicode_buffer(1025), self.w.DWORD(1025)
        require(bool(self.call(self.kernel.QueryFullProcessImageNameW, handle, 0, buffer, ctypes.byref(size))))
        require(0 < size.value < 1025)
        path = local_path(buffer.value)
        times = [self.w.FILETIME() for _ in range(4)]
        require(bool(self.call(self.kernel.GetProcessTimes, handle, *(ctypes.byref(item) for item in times))))
        created = (times[0].dwHighDateTime << 32) | times[0].dwLowDateTime
        require(created > 0)
        return {'pid': pid, 'creation_filetime': str(created), 'image_path': path}

    def inventory(self):
        modules = (self.w.HMODULE * MAX_MODULES)()
        needed = self.w.DWORD()
        require(bool(self.call(self.psapi.EnumProcessModulesEx, self.process, modules,
                               ctypes.sizeof(modules), ctypes.byref(needed), 3)))
        unit = ctypes.sizeof(self.w.HMODULE)
        require(0 < needed.value <= ctypes.sizeof(modules) and needed.value % unit == 0)
        rows = []
        for module in modules[:needed.value // unit]:
            name = ctypes.create_unicode_buffer(1025)
            length = self.call(self.psapi.GetModuleFileNameExW, self.process, module, name, len(name))
            require(0 < length < len(name) - 1)
            rows.append((key(name.value), local_path(name.value), int(module)))
        require(len({row[0] for row in rows}) == len(rows))
        return sorted(rows)

    def prepare(self, pid, expected):
        self.pid, self.expected = pid, expected
        # QUERY_INFORMATION | VM_READ | SYNCHRONIZE; no write, suspend or
        # termination rights are requested on the selected process.
        self.process = self.open_handle(self.kernel.OpenProcess, 0x400 | 0x10 | 0x100000, False, pid)
        self.original_identity = self.identity(self.process, pid)
        require(key(self.original_identity['image_path']) == key(expected['image_path']))
        require(ntpath.basename(expected['image_path']).casefold() == 'opencpn.exe')
        self.original_inventory = self.inventory()
        pins = {key(row['path']): row for row in expected['modules']}
        require({row[0] for row in self.original_inventory} == set(pins))
        # Inventory set equality is checked before any module/image file opens.
        files = dict(pins)
        image_key = key(expected['image_path'])
        if image_key in files:
            require(files[image_key]['sha256'] == expected['image_sha256'])
        else:
            files[image_key] = {'path': expected['image_path'], 'sha256': expected['image_sha256']}
        self.hashes = {}
        for path_key, pin in sorted(files.items()):
            require(path_key not in self.retained)  # expected manifest may not alias a binary
            handle, before = self.open_plain(pin['path'], retain=True, pinned_binary=True)
            require(before[5] <= MAX_FILE_BYTES and self.hash_bytes + before[5] <= MAX_HASH_BYTES)
            digest = hashlib.sha256()
            remaining = before[5]
            while remaining:
                count = min(65536, remaining)
                block, read = ctypes.create_string_buffer(count), self.w.DWORD()
                require(bool(self.call(self.kernel.ReadFile, handle, block, count, ctypes.byref(read), None)))
                require(0 < read.value <= count)
                digest.update(block.raw[:read.value])
                remaining -= read.value
            block, read = ctypes.create_string_buffer(1), self.w.DWORD()
            require(bool(self.call(self.kernel.ReadFile, handle, block, 1, ctypes.byref(read), None)))
            require(read.value == 0 and self.file_identity(handle, pin['path'], pinned_binary=True) == before)
            self.hash_bytes += before[5]
            sha256 = digest.hexdigest()
            require(sha256 == pin['sha256'])
            self.hashes[path_key] = sha256
            self.retained[path_key] = (handle, pin['path'], before, sha256, True)
        require(self.inventory() == self.original_inventory)

    def same_image_processes(self):
        snapshot = self.open_handle(self.kernel.CreateToolhelp32Snapshot, 2, 0)
        rows, count = [], 0
        entry = self.ProcessEntry()
        entry.size = ctypes.sizeof(entry)
        require(bool(self.call(self.kernel.Process32FirstW, snapshot, ctypes.byref(entry))))
        while True:
            count += 1
            require(count <= 32768)
            if entry.exe.casefold() == 'opencpn.exe':
                handle = self.open_handle(self.kernel.OpenProcess, 0x1000 | 0x100000, False, entry.pid)
                identity = self.identity(handle, entry.pid)
                if key(identity['image_path']) == key(self.expected['image_path']):
                    rows.append({'pid': entry.pid, 'creation_filetime': identity['creation_filetime']})
                self.release(handle)
            if not self.call(self.kernel.Process32NextW, snapshot, ctypes.byref(entry)):
                require(ctypes.get_last_error() == 18)
                break
        self.release(snapshot)
        return sorted(rows, key=lambda row: (row['pid'], row['creation_filetime']))

    def process_sample(self):
        require(self.inventory() == self.original_inventory)
        identity = self.identity(self.process, self.pid)
        require(identity == self.original_identity)
        return dict(identity, image_sha256=self.hashes[key(identity['image_path'])],
                    modules=[{'path': row[1], 'sha256': self.hashes[row[0]]} for row in self.original_inventory],
                    same_image_processes=self.same_image_processes())

    @staticmethod
    def port(value):
        # Windows stores a network-order ushort in the low DWORD bytes.
        # MIB DWORD port fields use only their low 16 bits; upper bits are
        # unused by the native table and must not become part of the port.
        return socket.ntohs(value & 0xffff)

    def tcp_sample(self, pid):
        rows = []
        for family, structure, tag in ((socket.AF_INET, self.Tcp4, 'ipv4'), (socket.AF_INET6, self.Tcp6, 'ipv6')):
            size = self.w.DWORD()
            code = self.call(self.iphlp.GetExtendedTcpTable, None, ctypes.byref(size), False, family, 5, 0)
            require(code == 122 and 4 <= size.value <= MAX_TABLE_BYTES)
            table = ctypes.create_string_buffer(size.value)
            require(self.call(self.iphlp.GetExtendedTcpTable, table, ctypes.byref(size), False, family, 5, 0) == 0)
            require(4 <= size.value <= len(table))
            count = self.w.DWORD.from_buffer_copy(table.raw[:4]).value
            require(count <= MAX_TCP_ROWS and 4 + count * ctypes.sizeof(structure) <= size.value)
            for index in range(count):
                row = structure.from_buffer_copy(table.raw, 4 + index * ctypes.sizeof(structure))
                if row.state != 5 or self.port(row.remote_port) != 443:
                    continue
                endpoints = {}
                for side in ('local', 'remote'):
                    value = getattr(row, side + '_address')
                    if tag == 'ipv4':
                        address, scope = int(value).to_bytes(4, 'little') + bytes(12), 0
                    else:
                        address, scope = bytes(value), int(getattr(row, side + '_scope'))
                    endpoints[side] = {'address': address.hex(), 'port': self.port(getattr(row, side + '_port')), 'scope': scope}
                rows.append(dict(pid=int(row.pid), state='ESTABLISHED', family=tag, **endpoints))
                require(len(rows) <= MAX_TCP_ROWS)
        def endpoint_tuple(row):
            return (row['family'], *(row[side][field] for side in ('local', 'remote')
                                    for field in ('address', 'port', 'scope')))
        selected = {endpoint_tuple(row) for row in rows if row['pid'] == pid}
        # Retain only selected-PID remote443 rows and competing owners of an
        # exact selected tuple. Unrelated foreign connections never leave here.
        rows = [row for row in rows if row['pid'] == pid or endpoint_tuple(row) in selected]
        return sorted(rows, key=lambda row: (row['family'], row['local']['address'], row['local']['port'], row['remote']['address']))

    def dns_sample(self):
        self.budget.check()
        # No API key, endpoint override, environment or configuration reads.
        answers = socket.getaddrinfo(HOST, 443, socket.AF_UNSPEC, socket.SOCK_STREAM, socket.IPPROTO_TCP)
        self.budget.check()
        require(0 < len(answers) <= MAX_DNS)
        rows = set()
        for family, kind, protocol, _, endpoint in answers:
            require(kind == socket.SOCK_STREAM and protocol == socket.IPPROTO_TCP)
            require(family in (socket.AF_INET, socket.AF_INET6))
            require('%' not in endpoint[0])
            address = ipaddress.ip_address(endpoint[0])
            require(address.version == (4 if family == socket.AF_INET else 6))
            scope = 0 if family == socket.AF_INET else endpoint[3]
            packed = address.packed + (bytes(12) if address.version == 4 else b'')
            rows.add(('ipv4' if address.version == 4 else 'ipv6', packed.hex(), scope))
        require(0 < len(rows) <= MAX_DNS)
        return {'host': HOST, 'addresses': [{'family': family, 'address': address, 'scope': scope}
                                            for family, address, scope in sorted(rows)]}

    def verify_retained(self):
        require(self.identity(self.process, self.pid) == self.original_identity)
        require(self.inventory() == self.original_inventory)
        for handle, path, before, _, pinned_binary in self.retained.values():
            require(self.file_identity(handle, path, pinned_binary=pinned_binary) == before)


def _worker(connection, pid, diagnostics, expected):
    try:
        encoded = collect(pid, diagnostics, expected)
        connection.send_bytes(encoded)
    except BaseException:
        # Deliberately no path, OS exception, DNS answer, diagnostic content or
        # application output is printed, even on a partially acquired capture.
        try:
            connection.send_bytes(b'')
        except BaseException:
            pass
    finally:
        connection.close()


def run_with_watchdog(pid, diagnostics, expected):
    """Bound collection and partial IPC reads; terminate only our own child.

    A pipe becoming readable does not mean its entire frame has arrived. One
    daemon reader owns the blocking receive; the parent waits on a bounded
    queue, then drains/terminates its child and closes only its own IPC owners.
    Draining has separate bounded join intervals after the collection timeout.
    """
    context = multiprocessing.get_context('spawn')
    receiver, sender = context.Pipe(duplex=False)
    child = context.Process(target=_worker, args=(sender, pid, diagnostics, expected))
    completed = queue.Queue(maxsize=1)
    reader = None

    def receive():
        try:
            completed.put(receiver.recv_bytes(MAX_EVIDENCE))
        except BaseException:
            completed.put(b'')

    try:
        started = time.monotonic()
        child.start()
        sender.close()
        reader = threading.Thread(target=receive, daemon=True)
        reader.start()
        remaining = WATCHDOG_SECONDS - (time.monotonic() - started)
        require(remaining > 0)
        try:
            encoded = completed.get(timeout=remaining)
        except queue.Empty:
            raise CollectorError('collector watchdog expired') from None
        require(time.monotonic() - started <= WATCHDOG_SECONDS)
        child.join(1)
        require(not child.is_alive() and child.exitcode == 0 and bool(encoded))
        return encoded
    finally:
        sender.close()
        if child.pid is not None:
            if child.is_alive():
                child.terminate()
            child.join(1)
            if child.is_alive():
                child.kill()
                child.join(1)
            child.close()
        # Killing only our child closes its writer and releases a partial-frame
        # receive. No application/native worker thread is asynchronously killed.
        if reader is not None:
            reader.join(1)
        receiver.close()


def write_new_output(path, encoded):
    """Create a new output and bind failure deletion to its retained handle.

    The original CREATE_NEW handle denies all sharing and has DELETE access.
    Python's x-mode stream owns a duplicate. The original stays held after
    Python closes; no cleanup ever deletes a pathname's later occupant.
    """
    import msvcrt
    native = NativeWindows(Budget(Clock()))
    raw_handle, failure = None, None
    try:
        native.lock_parents(path, retain=True)

        def exclusive_opener(filename, flags):
            nonlocal raw_handle
            def mark_created(handle):
                nonlocal raw_handle
                raw_handle = handle
            raw_handle = native.open_handle(native.kernel.CreateFileW, filename,
                                             0x40010080, 0, None, 1, 0x00200000, None,
                                             on_acquired=mark_created)
            native.file_identity(raw_handle, filename)
            process = native.kernel.GetCurrentProcess()  # pseudo-handle, never closed
            duplicate = native.w.HANDLE()
            native.budget.check()
            okay = bool(native.kernel.DuplicateHandle(process, raw_handle, process,
                                                       ctypes.byref(duplicate), 0, False, 2))
            # DuplicateHandle returns BOOL plus an out-handle. Register that
            # out-handle before the post-call deadline, like open_handle does.
            if okay:
                native.handles.append(duplicate.value)
            native.budget.check()
            require(okay)
            descriptor = msvcrt.open_osfhandle(duplicate.value, os.O_WRONLY | os.O_BINARY)
            native.handles.remove(duplicate.value)  # descriptor now owns duplicate
            return descriptor

        with open(path, 'xb', opener=exclusive_opener) as output:
            before = native.file_identity(raw_handle, path)
            require(before[5] == 0)
            output.write(encoded + b'\n')
            output.flush()
            after = native.file_identity(raw_handle, path)
            require(after[:5] == before[:5] and after[6] == before[6])
            require(after[5] == len(encoded) + 1)
        # Drain parent owners before releasing the retained output. A parent
        # close failure therefore still has a live handle for failure deletion.
        for handle in list(native.handles):
            if handle != raw_handle:
                native.close_one(handle)
        native.close_one(raw_handle)
    except BaseException as error:
        failure = error
    finally:
        if failure is not None and raw_handle in native.handles:
            # FILE_DISPOSITION_INFO.DeleteFile is a one-byte BOOLEAN. Cleanup
            # deliberately has no deadline checks and acts on the owned file.
            delete = ctypes.c_ubyte(1)
            try:
                require(bool(native.kernel.SetFileInformationByHandle(
                    raw_handle, 4, ctypes.byref(delete), ctypes.sizeof(delete))))
            except BaseException as error:
                failure = error
        try:
            native.close()
        except BaseException as error:
            failure = error
    if failure is not None:
        raise failure


class RedactedParser(argparse.ArgumentParser):
    def error(self, message):
        raise CollectorError('invalid arguments')


def main(argv=None, *, runner=None, windows=None, writer=None):
    parser = RedactedParser(description=__doc__)
    parser.add_argument('--process-id', type=int, required=True)
    parser.add_argument('--diagnostics', required=True)
    parser.add_argument('--expected', required=True)
    parser.add_argument('--output', required=True, help='New, explicit local JSON path; existing files are refused')
    try:
        args = parser.parse_args(argv)
        require((sys.platform == 'win32') if windows is None else windows)
        require(0 < args.process_id <= 0xffffffff)
        for path in (args.diagnostics, args.expected, args.output):
            local_path(path)
        require(len({key(args.diagnostics), key(args.expected), key(args.output)}) == 3)
        encoded = (runner or run_with_watchdog)(args.process_id, args.diagnostics, args.expected)
        # Parse returned child bytes strictly before the only output mutation.
        binding.strict_json_loads(encoded, limit=MAX_EVIDENCE)
        (writer or write_new_output)(args.output, encoded)
        return 0
    except Exception:
        sys.stderr.write(REFUSAL)
        return 1


if __name__ == '__main__':
    multiprocessing.freeze_support()
    raise SystemExit(main())

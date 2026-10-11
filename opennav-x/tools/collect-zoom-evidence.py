#!/usr/bin/env python3
"""Passive Windows SCRUM-317 observations; never launches or controls the target."""
import argparse
import ctypes
import hashlib
import json
import os
from pathlib import Path
import time


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def reject_nonfinite(value):
    raise ValueError(f'Non-finite JSON number: {value}')


def subset(value, keys):
    return {key: value.get(key) for key in keys}


def observation(snapshot):
    """Allowlist: retain counts/scale, never positions, names, IDs or credentials."""
    runtime = snapshot.get('runtime', {})
    chart = runtime.get('chart', {})
    members = chart.get('quilt_members')
    return {
        'build_commit': snapshot.get('build_commit'),
        'publication_monotonic_ms': snapshot.get('publication_monotonic_ms'),
        'chart': {**subset(chart, ['scale_ppm', 'opengl_enabled', 'quilt',
                                  'database_entries']),
                  'canvas_pixels': subset(chart.get('canvas_pixels', {}), ['width', 'height']),
                  'quilt_members': None if members is None else [
                      subset(item, ['type', 'native_scale']) for item in members],
                  'viewport_scale_denominator': None},
        'counts': {
            'online_ais': subset(runtime.get('online_ais', {}),
                ['chart_marks', 'cached_positions', 'in_radius_positions', 'targets']),
            'active_route_waypoints': snapshot.get('route', {}).get('waypoint_count'),
            'weather_wind_records': runtime.get('weather', {}).get('winds'),
            'drawn_chart_objects': None, 'drawn_labels': None,
            'all_route_waypoints': None, 'local_ais_targets': None,
        },
        'ui_update': subset(runtime.get('ui_update', {}),
                            ['ticks', 'last_ms', 'mean_ms', 'maximum_ms', 'timer_period_ms']),
        'chart_frame_time_ms': None,
    }


class Heartbeat:
    def __init__(self):
        self.token = None
        self.changed_at = None

    def observe(self, snapshot, now):
        token = snapshot.get('publication_monotonic_ms')
        if token is None:
            return {'status': 'publication_timestamp_unavailable', 'unchanged_ms': None}
        first = self.changed_at is None
        changed = token != self.token
        if changed:
            self.token, self.changed_at = token, now
        return {'status': 'first_observation' if first else 'new_publication' if changed else 'unchanged',
                'unchanged_ms': round(1000 * (now - self.changed_at), 3)}


class WindowsProcess:
    """One retained process handle: no injection, window messages or termination."""
    def __init__(self, pid, expected_exe):
        from ctypes import wintypes as w
        self.kernel = ctypes.WinDLL('kernel32', use_last_error=True)
        self.psapi = ctypes.WinDLL('psapi', use_last_error=True)
        k = self.kernel
        k.OpenProcess.argtypes = [w.DWORD, w.BOOL, w.DWORD]
        k.OpenProcess.restype = w.HANDLE
        k.CloseHandle.argtypes = [w.HANDLE]
        k.CloseHandle.restype = w.BOOL
        k.GetProcessTimes.argtypes = [w.HANDLE] + [ctypes.POINTER(w.FILETIME)] * 4
        k.GetProcessTimes.restype = w.BOOL
        k.GetExitCodeProcess.argtypes = [w.HANDLE, ctypes.POINTER(w.DWORD)]
        k.GetExitCodeProcess.restype = w.BOOL
        k.QueryFullProcessImageNameW.argtypes = [w.HANDLE, w.DWORD, w.LPWSTR, ctypes.POINTER(w.DWORD)]
        k.QueryFullProcessImageNameW.restype = w.BOOL
        class Memory(ctypes.Structure):
            _fields_ = [('cb', w.DWORD), ('faults', w.DWORD)] + [
                (name, ctypes.c_size_t) for name in ['peak', 'working', 'paged_peak',
                'paged', 'nonpaged_peak', 'nonpaged', 'pagefile', 'peak_pagefile']]
        self.Memory = Memory
        self.psapi.GetProcessMemoryInfo.argtypes = [w.HANDLE, ctypes.POINTER(Memory), w.DWORD]
        self.psapi.GetProcessMemoryInfo.restype = w.BOOL
        self.handle = k.OpenProcess(0x410, False, pid)  # QUERY_INFORMATION | VM_READ
        if not self.handle:
            raise ctypes.WinError(ctypes.get_last_error())
        try:
            length = w.DWORD(32768)
            path = ctypes.create_unicode_buffer(length.value)
            self.check(k.QueryFullProcessImageNameW(self.handle, 0, path, ctypes.byref(length)))
            self.exe = Path(path.value)
            if not os.path.samefile(self.exe, expected_exe):
                raise ValueError('PID executable differs from --exe')
            self.created_filetime, _, _ = self.usage()
        except BaseException:
            self.close()
            raise

    @staticmethod
    def check(result):
        if not result:
            raise ctypes.WinError(ctypes.get_last_error())

    def usage(self):
        from ctypes import wintypes as w
        exit_code = w.DWORD()
        self.check(self.kernel.GetExitCodeProcess(self.handle, ctypes.byref(exit_code)))
        if exit_code.value != 259:  # STILL_ACTIVE
            raise ProcessLookupError('Observed process exited')
        times = [w.FILETIME() for _ in range(4)]
        self.check(self.kernel.GetProcessTimes(self.handle, *[ctypes.byref(t) for t in times]))
        values = [(t.dwHighDateTime << 32) | t.dwLowDateTime for t in times]
        memory = self.Memory()
        memory.cb = ctypes.sizeof(memory)
        self.check(self.psapi.GetProcessMemoryInfo(self.handle, ctypes.byref(memory), memory.cb))
        return values[0], (values[2] + values[3]) / 1e7, memory.working

    def close(self):
        if self.handle:
            self.kernel.CloseHandle(self.handle)
            self.handle = None


def collect(args):
    if os.name != 'nt':
        raise ValueError('Live collection requires native Windows')
    profile, diagnostic = args.disposable_profile.resolve(), args.diagnostics.resolve()
    if profile not in diagnostic.parents:
        raise ValueError('--diagnostics must be inside the attested disposable profile')
    output = args.output.resolve()
    if output == profile or profile in output.parents:
        raise ValueError('--output must be outside the disposable profile')
    if not 5 <= args.seconds <= 120:
        raise ValueError('--seconds must be between 5 and 120')
    if args.pid <= 0:
        raise ValueError('--pid must be positive')
    target = WindowsProcess(args.pid, args.exe.resolve())
    try:
        # Fail before creating evidence if the selected diagnostic belongs to another build.
        first = json.loads(diagnostic.read_text(encoding='utf-8'), parse_constant=reject_nonfinite)
        if first.get('build_commit') != args.package_commit:
            raise ValueError('Diagnostic build_commit differs from --package-commit')
        output.mkdir(parents=True, exist_ok=False)
        metadata = {
            'schema': 1, 'issue': 'SCRUM-317', 'pid': args.pid,
            'process_creation_filetime': target.created_filetime,
            'exe_sha256': sha256(target.exe), 'package_commit': args.package_commit,
            'collector_sha256': sha256(Path(__file__)),
            'started_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
            'duration_requested_seconds': args.seconds, 'sample_period_seconds': 0.25,
            'cpu_basis': '100 percent = one logical CPU; may exceed 100',
            'disposable_profile_attested': args.confirm_disposable,
            'diagnostic_pid_association': 'operator attested; JSON contains no PID',
            'limits': ['Diagnostic publication is throttled to 1 second; file or UI delay can stall it.',
                       'UI callback duration excludes asynchronous chart painting.',
                       'Counts are model inputs, not counts of actually drawn objects.',
                       'Null values are unavailable, never zero. Viewport PPM is not a scale denominator.',
                       'Frame-time and UI-thread stacks require separate native trace evidence.'],
        }
        (output / 'metadata.json').write_text(json.dumps(metadata, indent=2) + '\n', encoding='utf-8')
        heartbeat = Heartbeat()
        start = time.monotonic()
        previous_time, previous_cpu = None, None
        reason = 'duration_complete'
        with (output / 'samples.jsonl').open('x', encoding='utf-8') as stream:
            try:
                while time.monotonic() - start < args.seconds:
                    now = time.monotonic()
                    row = {'elapsed_ms': round((now - start) * 1000, 3),
                           'utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())}
                    try:
                        _, cpu, rss = target.usage()
                    except (OSError, ProcessLookupError) as error:
                        row['process_error'] = type(error).__name__
                        stream.write(json.dumps(row) + '\n'); stream.flush()
                        reason = 'process_unavailable'
                        break
                    row.update(resident_bytes=rss, cpu_seconds_total=cpu,
                        cpu_percent_one_core=None if previous_time is None else
                        100 * (cpu - previous_cpu) / (now - previous_time))
                    previous_time, previous_cpu = now, cpu
                    try:
                        # One read per sample: never block sampling through a rename gap.
                        with diagnostic.open('r', encoding='utf-8') as source:
                            modified = os.fstat(source.fileno()).st_mtime
                            snapshot = json.load(source, parse_constant=reject_nonfinite)
                        if snapshot.get('build_commit') != args.package_commit:
                            raise ValueError('build_identity_changed')
                        row['diagnostic'] = observation(snapshot)
                        row['publication_observation'] = heartbeat.observe(snapshot, now)
                        row['diagnostic_file_age_ms'] = round((time.time() - modified) * 1000, 3)
                    except (OSError, ValueError, TypeError, AttributeError) as error:
                        row['diagnostic_error'] = type(error).__name__
                    stream.write(json.dumps(row, allow_nan=False) + '\n'); stream.flush()
                    time.sleep(max(0, 0.25 - (time.monotonic() - now)))
            except KeyboardInterrupt:
                reason = 'operator_stopped_collector'
        (output / 'completion.json').write_text(json.dumps({
            'reason': reason, 'elapsed_seconds': time.monotonic() - start,
            'acceptance': 'not_assessed', 'chart_frame_time_ms': None}, indent=2) + '\n', encoding='utf-8')
    finally:
        target.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pid', required=True, type=int)
    parser.add_argument('--exe', required=True, type=Path)
    parser.add_argument('--package-commit', required=True)
    parser.add_argument('--disposable-profile', required=True, type=Path)
    parser.add_argument('--diagnostics', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--confirm-disposable', required=True, action='store_true',
                        help='Attest PID and diagnostic file belong to the same disposable test process')
    args = parser.parse_args()
    try:
        collect(args)
    except (OSError, ValueError) as error:
        parser.exit(1, f'Collection stopped: {error}\n')


if __name__ == '__main__':
    main()

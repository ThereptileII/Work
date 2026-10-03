#!/usr/bin/env python3
"""Exercise actual PortableProfile with the collector's disposable copy paths.

The small probe links unchanged production C++; the identity fixture's dummy
opencpn.exe is never executed. This is not a native application launch gate.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys

from recovery_capture_inputs_tests import PackageBoundary, inputs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--probe', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    probe = args.probe.resolve(strict=True)
    fixture = PackageBoundary()
    fixture.setUp()
    checks = []
    try:
        audited = fixture.verify()
        output = fixture.base/'new capture with spaces'
        output.mkdir()
        copied = inputs.stage_disposable_package(fixture.package, output, audited)
        def run(executable, profile):
            return subprocess.run([str(probe), str(executable), str(profile)],
                                  capture_output=True, text=True, timeout=10)
        external = output/'profile'
        old = run(fixture.package/'app/opencpn.exe', external)
        assert old.returncode == 1 and 'outside its own profile folder' in old.stderr, old
        assert not external.exists(), 'Old collector profile was created despite refusal'
        checks.append('original collector invocation refused by actual production guard')
        for requested in (copied['profile'], ''):
            result = run(copied['executable'], requested)
            assert result.returncode == 0, result.stderr
            paths = [Path(value).resolve() for value in result.stdout.splitlines()]
            assert paths == [Path(copied[key]).resolve() for key in ('root', 'profile', 'logs')], paths
        checks.append('actual guard accepts copied package with own explicit/default profile and logs')
        # The test-profile marker is not permission to escape package ownership.
        wrong = run(copied['executable'], external)
        assert wrong.returncode == 1 and not external.exists(), wrong
        checks.append('copied portable marker still refuses external configdir')
        diagnostics = Path(copied['logs'])/'opennav-diagnostics.json'
        diagnostics.write_text('{"test_observation":true}\n')
        assert not (Path(copied['profile'])/diagnostics.name).exists()
        assert inputs.verify_disposable_package(copied)
        assert fixture.verify() == audited
        checks.append('portable log writes preserve copied immutable bytes and complete audited original')
        receipt = dict(scope='Production portable path contract only; no application launch',
                       platform=sys.platform, probe_sha256=inputs.sha(probe),
                       production_source_sha256=inputs.sha(Path(__file__).resolve().parents[1]/
                                                          'src/platform/PortableProfile.cpp'),
                       checks=checks, result='PASS')
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(receipt, indent=2)+'\n')
        print(f'{len(checks)} production portable-boundary checks passed')
    finally:
        fixture.tearDown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Produce the operator-only native Windows broker-enrollment console tool.

Use an isolated install copy in launcher -> broker -> provisioner producer
order, all from one clean commit. Authenticate original native producer
provenance before any first execution: these local checks do not establish
publisher trust. This producer never elevates or runs its output, creates trust
configuration, installs a service, or enables runtime/CI activation.
"""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import zipfile

import broker_package as broker
import provisioner_package as provisioner

_spec = importlib.util.spec_from_file_location('_provisioner_broker_producer',
                                              Path(__file__).with_name('package-update-broker.py'))
producer = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(producer)
BINARY = provisioner.BINARY
RECORD_RELATIVE = provisioner.RECORD_RELATIVE
BUILD_RECIPE = provisioner.BUILD_RECIPE


def package(install, commit, go):
    """The command, recipe and console identity are fixed in source."""
    return producer._package_component(install, commit, go, component=provisioner.PROVISIONER,
                                       prerequisite=broker.broker_prerequisite)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--install', type=Path, required=True)
    parser.add_argument('--commit', default=os.environ.get('GITHUB_SHA', ''))
    parser.add_argument('--go', default='go')
    args = parser.parse_args()
    try:
        result = package(args.install, args.commit, args.go)
    except (ValueError, OSError, KeyError, zipfile.BadZipFile, subprocess.SubprocessError) as error:
        raise SystemExit(str(error)) from error
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()

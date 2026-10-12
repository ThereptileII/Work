#!/usr/bin/env python3
"""Build the inert baseline handoff Win32 console from the retained source.

Run after exact same-commit launcher, broker and provisioner production in the
same protected install copy. Authenticate original native producer provenance
separately before first execution. This step does not run or elevate the tool.
"""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import zipfile

import baseline_handoff_package as handoff

_spec = importlib.util.spec_from_file_location('_baseline_handoff_broker_producer',
                                              Path(__file__).with_name('package-update-broker.py'))
producer = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(producer)


def package(install, commit, go):
    """Use the fixed generic Component recipe, with exact provisioner custody."""
    return producer._package_component(install, commit, go, component=handoff.BASELINE_HANDOFF,
                                       prerequisite=handoff.provisioner_prerequisite)


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

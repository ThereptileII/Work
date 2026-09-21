#!/usr/bin/env python3
"""Classify two reproduced upstream-only baseline failures; never a release gate."""
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

path = Path(sys.argv[1])
root = ET.parse(path).getroot()
cases = list(root.iter('testcase'))
if len(cases) < 50:
    raise SystemExit('Baseline suite missing or unexpectedly small')
known = {'DriverRegistry.RegisterDriver', 'IpcServer.Commands'}
failed = {case.attrib['name'] for case in cases if case.find('failure') is not None or case.find('error') is not None}
unexpected = failed - known
report = {'ctest_entries': len(cases), 'observed_failures': sorted(failed),
          'known_upstream_failures': sorted(failed & known), 'unexpected_failures': sorted(unexpected),
          'release_acceptance': False,
          'note': 'Pristine comparison only. Integrated regression tests must pass without exceptions.'}
path.with_suffix('.classification.json').write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2))
if unexpected:
    raise SystemExit('Unexpected pristine baseline failures')
if failed:
    print('::warning::Pristine upstream baseline has reproduced failures; not an all-tests-pass result.')

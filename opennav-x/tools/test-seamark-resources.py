#!/usr/bin/env python3
"""Focused verification of an already-generated SCRUM-264 resource set."""
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tests')]
from chart_seamark_resources_tests import verify_seamarks
p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--generated',type=Path,required=True);a=p.parse_args()
checks=0
def check(value):
    global checks
    checks+=1
    assert value,f'Seamark check {checks}'
verify_seamarks(a.source,a.generated,json.loads((a.generated/'manifest.json').read_text()),check)
print(f'{checks} focused seamark resource checks passed, including eight negative whole-resource cases')

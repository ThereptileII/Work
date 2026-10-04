#!/usr/bin/env python3
"""Additional Windows functional checks on retained CI fixtures; no design pass."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
from release_manifest import verify

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--release',type=Path,required=True)
    args=p.parse_args()
    if sys.platform != 'win32' or os.environ.get('GITHUB_ACTIONS') != 'true':
        raise SystemExit('Retained functional qualification requires disposable native Windows CI')
    manifest=verify(args.release)
    env=dict(os.environ,SKAGER_DESIGN_VALIDATION='false')
    checks=[('smoke-modes-windows.py',),('smoke-navigation.py',),
            ('smoke-navigation.py','--route-fixture'),('smoke-navigation.py','--route-fixture-standard'),
            ('smoke-navigation.py','--instruments'),('smoke-navigation.py','--n2k'),
            ('smoke-navigation.py','--boat'),('smoke-signalk.py',),('smoke-recording.py',),
            ('smoke-pilot.py',),('smoke-navigation.py','--objects'),
            ('smoke-user-flows.py',),('smoke-recovery.py',)]
    report={'status':'running','commit':manifest['commit'],'harnessCommit':os.environ['GITHUB_SHA'],
            'designReview':'not-requested','checks':[]}
    out=ROOT/'evidence/local/production-functional.json'
    out.parent.mkdir(parents=True,exist_ok=True)
    try:
        for script,*arguments in checks:
            subprocess.run([sys.executable,str(ROOT/'tools'/script),*arguments],check=True,env=env)
            report['checks'].append([script,*arguments])
        report['status']='passed'
    finally:
        out.write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':
    main()

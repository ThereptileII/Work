#!/usr/bin/env python3
"""Retain independently rendered Windows tab geometry without editing the HTML."""
import argparse
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
STATES=('settings-day','settings-dusk','settings-night','sensors-day','display-day','system-day')
LABELS=('Vessel','Navigation','Sensors','Autopilot','Radar','Display','System','Help')

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    reference=json.loads(args.capture.read_text())
    canonical=json.loads((ROOT/'docs/design/prototype/reference/windows/capture.json').read_text())
    assert reference['platform']=='Windows' and reference['viewport']=={'width':1280,'height':800}
    assert reference['deviceScaleFactor']==1 and reference['htmlSha256']==canonical['htmlSha256']
    result=dict(htmlSha256=reference['htmlSha256'],platform='Windows',viewport=reference['viewport'],
                referenceCaptureSha256=hashlib.sha256(args.capture.read_bytes()).hexdigest(),states={})
    for name in STATES:
        state=reference['states'][name]
        assert state['screenshotSha256']==canonical['states'][name]['screenshotSha256'], 'Canonical pixels changed: '+name
        assert hashlib.sha256((args.capture.parent/(name+'.png')).read_bytes()).hexdigest()==state['screenshotSha256']
        tabs=state['components']['.settings-tabs button']
        assert len(tabs)==8
        assert all(tab['style']['fontSize']=='11px' and tab['style']['fontWeight']=='400' for tab in tabs)
        result['states'][name]=dict(screenshotSha256=state['screenshotSha256'],
            fonts=state['platformFonts']['.settings-tabs button'],
            drawer=state['components']['.drawer'][0]['rect'],
            tabs={label:tab['rect'] for label,tab in zip(LABELS,tabs)})
    result['responsive']={}
    for percent in (125,150):
        capture=args.capture.parent/f'responsive-{percent}/capture.json'
        d=json.loads(capture.read_text());state=d['states']['settings-day']
        assert d['platform']=='Windows' and d['htmlSha256']==result['htmlSha256']
        assert d['deviceScaleFactor']==percent/100
        assert hashlib.sha256((capture.parent/'settings-day.png').read_bytes()).hexdigest()==state['screenshotSha256']
        result['responsive'][str(percent)]=dict(viewport=d['viewport'],deviceScaleFactor=d['deviceScaleFactor'],
            referenceCaptureSha256=hashlib.sha256(capture.read_bytes()).hexdigest(),
            screenshotSha256=state['screenshotSha256'],drawer=state['components']['.drawer'][0]['rect'])
    args.output.write_text(json.dumps(result,indent=2)+'\n')

if __name__=='__main__':main()

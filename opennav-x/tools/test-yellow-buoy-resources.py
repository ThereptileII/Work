#!/usr/bin/env python3
"""Focused two-alias proof against the sealed preceding generated resources."""
import argparse,copy,json,sys,xml.etree.ElementTree as ET
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'tests'))
from chart_yellow_buoy_resources_tests import verify_yellow_buoy,restore_yellow,TABLES
from chart_raster_ink import decode
from chart_yellow_buoy_art import restore_for_validation,canonical
p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--generated',type=Path,required=True);p.add_argument('--before',type=Path,required=True);a=p.parse_args();checks=0

def check(value):
 global checks
 checks+=1
 assert value,checks
verify_yellow_buoy(a.source,a.generated,check)
before=ET.parse(a.before/'chartsymbols.xml').getroot();after=ET.parse(a.generated/'chartsymbols.xml').getroot();restore_for_validation(before,after);check(canonical(before)==canonical(after))
for _,_,filename in TABLES:
 _,old=decode((a.before/filename).read_bytes());_,new=decode((a.generated/filename).read_bytes());new=bytearray(new)
 restore_yellow(old,new);check(old==new)
 # Unowned neighboring byte mutation must fail the whole-canvas inverse.
 new[0]^=1;check(old!=new)
for xpath,key,value in [("bitmap/pivot",'x','13'),('bitmap','width','25'),('bitmap/graphics-location','x','724')]:
 bad=ET.parse(a.generated/'chartsymbols.xml').getroot();bad.find("symbols/symbol[name='XNSPPT01']/"+xpath).set(key,value)
 try:restore_for_validation(before,bad)
 except AssertionError:checks+=1
 else:raise AssertionError('Owned head mutation accepted')
check((a.before/'S52RAZDS.RLE').read_bytes()==(a.generated/'S52RAZDS.RLE').read_bytes())
print(json.dumps({'passed':True,'checks':checks,'wholePrecedingXmlAndAllAtlasInverse':True,'neighborsAndRleUnchanged':True,'scope':'Two owned aliases only; no full resource suite'},indent=2))

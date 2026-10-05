#!/usr/bin/env python3
"""Execute actual bounded s52plib methods; inspect persistence separately.

This is a method-level fixture, not an application/renderer acceptance test.
The renderer and configuration gates must still run on the exact application.
"""
import argparse
from pathlib import Path
import re
import subprocess

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--source',required=True,type=Path)
p.add_argument('--output',required=True,type=Path)
p.add_argument('--cxx',default='c++')
p.add_argument('--enum-source',type=Path)
p.add_argument('--expect-negative',action='store_true')
a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
root=Path(__file__).resolve().parents[1]
h=(a.source/'libs/s52plib/src/s52plib.h').read_text()
cpp=(a.source/'libs/s52plib/src/s52plib.cpp').read_text()
def method(text,signature):
    start=text.index(signature); brace=text.index('{',start); depth=0
    for pos in range(brace,len(text)):
        depth += (text[pos]=='{')-(text[pos]=='}')
        if depth==0:return text[start:pos+1]
    raise AssertionError('Unclosed actual method '+signature)
methods=[method(h,'void EnablePresentationSimplifiedSymbols()'),
         method(h,'LUPname GetEffectiveSymbolStyle() const')]
flag=re.findall(r'bool m_presentationSimpleSymbols = false;',h)
assert len(flag)==1
assert 'm_nSymbolStyle = PAPER_CHART;' in cpp
mariner=method(cpp,'void s52plib::UpdateMarinerParams(void)')
assert 'GetEffectiveSymbolStyle()' in mariner
enum_text=(a.enum_source or a.source/'libs/s52plib/src/s52s57.h').read_text()
actual_enum=re.search(r'typedef enum _LUPname \{.*?\} LUPname;',enum_text,re.S)[0]
if a.expect_negative:
    assert 'm_presentationSimpleSymbols ? SIMPLIFIED : m_nSymbolStyle' in methods[1]
    methods[1]=methods[1].replace('m_presentationSimpleSymbols ? SIMPLIFIED : m_nSymbolStyle','m_nSymbolStyle')
stub=actual_enum+'''
enum { S52_MAR_SYMPLIFIED_PNT, S52_MAR_SYMBOLIZED_BND };
static double parameters[2]{};
void S52_setMarinerParam(int key, double value) { parameters[key]=value; }
class s52plib {
 public:
  LUPname m_nSymbolStyle=PAPER_CHART, m_nBoundaryStyle=PLAIN_BOUNDARIES;
  void UpdateMarinerParams(void);
'''+ '\n'.join(methods)+'\n private:\n'+flag[0]+'\n};\n'+mariner+'\n'
(a.output/'actual-symbol-style.inc').write_text(stub)
exe=a.output/'symbol-style-test'
subprocess.run([a.cxx,'-std=c++17','-Wall','-Wextra','-pedantic','-I'+str(a.output),str(root/'tests/chart_symbol_style_test.cpp'),'-o',str(exe)],check=True)
result=subprocess.run([str(exe)],capture_output=True,text=True)
print(result.stdout+result.stderr,end='')
if a.expect_negative:
    assert result.returncode==1 and 'Effective symbol style check 7' in result.stderr
    print('Negative control: ignoring display policy rejected at actual effective selection')
else:result.check_returncode()

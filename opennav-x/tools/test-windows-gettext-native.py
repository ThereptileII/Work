#!/usr/bin/env python3
"""Disposable native gettext prerequisite proof, never an app/dependency build."""
import argparse
import gettext
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import windows_gettext as prerequisite

ROOT=Path(__file__).resolve().parents[1]

def main():
    p=argparse.ArgumentParser();p.add_argument('--evidence',type=Path,required=True);args=p.parse_args()
    if sys.platform!='win32' or os.environ.get('GITHUB_ACTIONS')!='true':
        raise SystemExit('Native gettext proof requires disposable Windows CI; never the boat')
    evidence=args.evidence.resolve();evidence.mkdir(parents=True,exist_ok=False)
    summary={'passed':False,'nativeProductAcceptance':False,'candidate':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
             'inputs':{n:prerequisite.identity(ROOT/n) for n in ['tools/windows_gettext.py','tools/test-windows-gettext-native.py','tests/windows_gettext_tests.py','tools/build-pristine-windows.ps1']}}
    try:
        # Missing/failing/version/retry/known-directory fixtures exercise the
        # same helper first, before any provider operation.
        command=[sys.executable,str(ROOT/'tests/windows_gettext_tests.py')]
        result,_=prerequisite.native(command,60,evidence/'contracts')
        if result['exitCode']!=0:raise RuntimeError('Native gettext prerequisite contracts failed')
        receipt=evidence/'gettext.json'
        selected=prerequisite.ensure(receipt,allow_install=True)
        os.environ['PATH']=selected['directory']+os.pathsep+os.environ['PATH']
        for name in prerequisite.TOOLS:
            if Path(shutil.which(name)).resolve()!=Path(selected['tools'][name]['path']):
                raise RuntimeError('Selected native gettext tool is shadowed')
        prerequisite.verify(receipt)
        header='msgid ""\nmsgstr ""\n"Content-Type: text/plain; charset=UTF-8\\n"\n\n'
        po=evidence/'sample.po';pot=evidence/'sample.pot';merged=evidence/'merged.po';mo=evidence/'sample.mo'
        po.write_text(header+'msgid "SKAGER source"\nmsgstr "SKAGER översättning"\n',encoding='utf-8')
        pot.write_text(header+'msgid "SKAGER source"\nmsgstr ""\n',encoding='utf-8')
        commands=[('msgmerge',[selected['tools']['msgmerge.exe']['path'],'--quiet','--output-file='+str(merged),str(po),str(pot)]),
                  ('msgfmt',[selected['tools']['msgfmt.exe']['path'],'--check','--output-file='+str(mo),str(merged)])]
        for name,command in commands:
            result,_=prerequisite.native(command,30,evidence/name)
            if result['exitCode']!=0:raise RuntimeError('Actual Poedit '+name+' translation operation failed')
        with mo.open('rb') as stream:
            if gettext.GNUTranslations(stream).gettext('SKAGER source')!='SKAGER översättning':
                raise RuntimeError('Native compiled catalog lost the supplied UTF-8 translation')
        prerequisite.verify(receipt)
        summary.update(passed=True,receipt=prerequisite.identity(receipt),catalog=prerequisite.identity(mo),
                       mergedCatalog=prerequisite.identity(merged),functionalTranslation='passed',
                       limits=['No app or dependency compilation','No product runtime, GUI, installer or boat acceptance'])
    except Exception as error:
        summary['error']=str(error);raise
    finally:
        (evidence/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')

if __name__=='__main__':main()

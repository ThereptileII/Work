from pathlib import Path
import sys, importlib.util, json, hashlib, subprocess
root=Path.cwd();sys.path[:0]=[str(root/'tools'),str(root/'tests')]
spec=importlib.util.spec_from_file_location('chart_generator',root/'tools/generate-xnav-chart-style.py')
g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
from chart_seamark_resources_tests import verify_seamarks
from chart_construction_hatch_tests import verify as verify_hatch
source=Path('/home/standard/Projects/X-nav-worktrees/scrum259-linux-6dafd29/upstream/OpenCPN/data/s57data')
output=root/'.local/combined-light-hatch/generated'
metadata=g.generate(source,output)
checks=0
def check(value):
    global checks
    checks+=1
    assert value, 'Combined assertion '+str(checks)
verify_seamarks(source,output,metadata,check)
seamark_checks=checks
verify_hatch(source,output,check)
receipt={'status':'passed','source':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'checks':checks,'seamark_checks':seamark_checks,'hatch_checks':checks-seamark_checks,'manifest_sha256':hashlib.sha256((output/'manifest.json').read_bytes()).hexdigest(),'scope':'One combined generation, unchanged seamark and hatch oracles including whole XML/atlas inverse and retained negative controls; no app or native acceptance.'}
(root/'.local/combined-light-hatch/receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(json.dumps(receipt,indent=2))

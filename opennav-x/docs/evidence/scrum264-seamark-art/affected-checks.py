from pathlib import Path
import sys,json,importlib.util,re,xml.etree.ElementTree as ET
root=Path.cwd();sys.path[:0]=[str(root/'tools'),str(root/'tests')]
source=Path('/home/standard/Projects/X-nav/upstream/OpenCPN/data/s57data');output=root/'.local/seamark-proof/generated';data=json.loads((output/'manifest.json').read_text());checks=0
for table in data['palette'].values():
 for role,rgb in table.items():table[role]=tuple(rgb)
spec=importlib.util.spec_from_file_location('g',root/'tools/generate-xnav-chart-style.py');g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
def check(value):
 global checks
 checks+=1
 assert value,checks
from chart_seamark_resources_tests import verify_seamarks,SELECTED,ALIASES
if '--remaining' not in sys.argv:verify_seamarks(source,output,data,check)
print('seamarks',checks,flush=True)
from chart_anchor_resources_tests import verify_anchor
from chart_service_resources_tests import verify_services
from chart_cardinal_resources_tests import verify_cardinals
from chart_day_neutral_resources_tests import verify_day_neutral
for test in ((verify_day_neutral,) if '--remaining' in sys.argv else (verify_anchor,verify_services,verify_cardinals,verify_day_neutral)):
 test(source,output,data,check);print(test.__name__,checks,flush=True)
# Execute the existing independent complete XML lookup/symbol inverse oracle.
s=(root/'tests/chart_presentation_resources_tests.py').read_text();start=s.index("    a,b=ET.parse(source/");end=s.index("    html=(ROOT/",start)
import textwrap
exec(compile(textwrap.dedent(s[start:end]),'existing-resource-xml-inverse','exec'))
print(checks,'affected existing atlas and complete XML inverse checks passed',flush=True)

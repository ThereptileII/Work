"""Seal actual final staged identities after the single authorized build."""
import argparse,hashlib,importlib.util,json,re
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--expected-commit',required=True);a=p.parse_args()
here=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('private_inputs',here/'cache-inputs-readonly.py')
i=importlib.util.module_from_spec(spec);spec.loader.exec_module(i)
i.require(re.fullmatch('[a-f0-9]{40}',a.expected_commit),'Full source commit required')
frozen=i.verify(False);i.require(frozen['commit']==a.expected_commit,'Final source mismatch')
cache=here/'inputs';root=i.APP;build=root/'build/xnav-linux';install=root/'build/xnav-install'
header=(build/'include/OpenNavBuild.h').read_text();i.require(a.expected_commit in header,'Generated build header differs')
manifest=install/'share/opencpn/opennav/chart-style/v1/manifest.json';m=json.loads(manifest.read_text())
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
for name,identity in m['files'].items():
    raw=(manifest.parent/name).read_bytes()
    i.require(len(raw)==identity['bytes'] and hashlib.sha256(raw).hexdigest()==identity['sha256'],'Installed resource differs: '+name)
config=(build/'CMakeCache.txt').read_text();flags={}
for key in ('OCPN_BUILD_TEST','OPENNAV_ENABLE_ROUTE_SCENARIO','XNAV_ENABLE_PILOT_LOOPBACK_TESTS','XNAV_ENABLE_TEST_FIXTURES'):
    flags[key]=re.search(r'^'+key+r':[A-Z_]+=(.*)$',config,re.M)[1]
inputs=('src/integration/ChartNameTypography.h','src/integration/ChartTextFace.h','src/integration/ChartCogPredictor.h','src/integration/ChartPresentation.cpp','patches/opencpn-5.12.4-xnav.patch','patches/opencpn-5.12.4-chart-presentation.patch','resources/chart-style/v1/definition.json','docs/design/prototype-tokens.json','src/application/SkagerBrandAsset.h','resources/branding/provenance.json','tools/chart_cable_paint.py','tools/chart_day_neutral_ink.py','src/ui/SkagerWordmark.h','src/integration/OpenCPNIntegration.cpp')
inputs=inputs+('src/integration/ChartCaLightPoint.h','src/ais/Provider.h','src/ais/AisStreamProvider.cpp','src/ais/AisStreamSession.cpp')
inputs=tuple(dict.fromkeys(inputs+tuple(str(q.relative_to(root)) for q in sorted((root/'tools').glob('chart_*.py')))))
staged={'schema':1,'commit':a.expected_commit,'binary_sha256':digest(install/'bin/opencpn'),'build_flags':flags,'manifest_sha256':digest(manifest),'resource_files':m['files'],'source_evidence':{name:digest(root/name) for name in inputs},'authority':'Private Linux development capture only; not Windows or boat acceptance','runtime_identity':'Still required from loader self-test and actual capture diagnostics'}
for name,value in [('frozen-inputs.json',frozen),('staged-inputs.json',staged)]:
    path=cache/name;i.require(not path.exists(),'Refusing to replace earlier sealed inputs');path.write_text(json.dumps(value,indent=2)+'\n')
print(json.dumps(staged,indent=2))

"""One-generation bounded SCRUM-231 proof against the retained 585fd0f baseline."""
import argparse,hashlib,importlib.util,json,re,sys,time
from pathlib import Path
import xml.etree.ElementTree as ET
root=Path(__file__).resolve().parents[3];sys.path.insert(0,str(root/'tools'))
spec=importlib.util.spec_from_file_location('generator',root/'tools/generate-xnav-chart-style.py');g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
sha=lambda b:hashlib.sha256(b).hexdigest()
baseline=json.loads((root/'docs/evidence/scrum254-service-glyphs/generated-manifest.json').read_text())
checks=0
def check(value,message):
 global checks
 checks+=1
 assert value,message
def luminance(rgb):
 v=[x/255 for x in rgb];return sum((x/12.92 if x<=.04045 else ((x+.055)/1.055)**2.4)*w for x,w in zip(v,(.2126,.7152,.0722)))
def contrast(a,b):
 light,dark=sorted((luminance(a),luminance(b)),reverse=True);return (light+.05)/(dark+.05)
inputs={n:sha((a.source/n).read_bytes()) for n in baseline['files']}
start=time.monotonic();data=g.generate(a.source,a.output)
xml=(a.output/'chartsymbols.xml').read_text();restored=xml
expected={'DAY_BRIGHT':(238,238,226),'DUSK':(78,97,93),'NIGHT':(29,41,37)}
contrasts={}
for table,rgb in expected.items():
 colors=data['palette'][table];old=baseline['palette'][table]
 check(colors['XNBUA']==rgb==colors['LANDA'],'Exact prototype land and effective Night fill')
 check(colors['XNBUA'] not in [colors[n] for n in ('DEPDW','DEPMD','DEPMS','DEPVS','DEPIT')],'All water/depth roles remain distinct')
 check({n:list(v) for n,v in colors.items() if n!='XNBUA'}=={n:v for n,v in old.items() if n!='XNBUA'},'All other palette roles unchanged')
 pattern=r'(<color-table name="'+table+r'">)(.*?)(</color-table>)'
 m=re.search(pattern,restored,re.S);check(m is not None,'Unique theme table exists')
 current='<color name="XNBUA" r="%s" g="%s" b="%s"/>'%rgb
 prior='<color name="XNBUA" r="%s" g="%s" b="%s"/>'%tuple(old['XNBUA'])
 check(m[2].count(current)==1,'Only one dedicated fill role')
 restored=restored[:m.start(2)]+m[2].replace(current,prior)+restored[m.end(2):]
 contrasts[table]={n:contrast(colors['CHBLK'],colors[n]) for n in ('LANDA','DEPDW','DEPVS')}
 if table!='DAY_BRIGHT':
  for name,minimum in [('LANDA',3),('DEPDW',4),('DEPVS',2)]:check(contrasts[table][name]>=minimum,'Existing safety ink contrast unchanged')
check(sha(restored.encode())==baseline['files']['chartsymbols.xml']['sha256'],'Restoring only three XNBUA RGB nodes yields exact baseline XML bytes')
for name,entry in baseline['files'].items():
 if name!='chartsymbols.xml':check(data['files'][name]==entry,'Sprites and RLE byte-identical to baseline')
check(inputs=={n:sha((a.source/n).read_bytes()) for n in inputs},'Pinned original source resources untouched')
# Reuse complete semantic reverse-equality guard, including the two prior AC
# tokens and all already-reviewed typography/artwork; no new lookup exception.
g.validate_resource_changes((a.source/'chartsymbols.xml').read_bytes().replace(b'\r\n',b'\n'),xml,data['palette']);checks+=1
# Focused mutations must still fail: global CHBRN, wrong fill, and BUAARE labels.
for mutation in ('CHBRN','XNBUA','label'):
 tree=ET.fromstring(xml)
 if mutation=='label':tree.find("lookups/lookup[@id='16']/instruction").text+=';TX(OBJNAM,1,1,1,1)'
 else:tree.find("color-tables/color-table[@name='DAY_BRIGHT']/color[@name='"+mutation+"']").set('r','1')
 try:g.validate_resource_changes((a.source/'chartsymbols.xml').read_bytes(),ET.tostring(tree),data['palette'])
 except AssertionError:checks+=1
 else:raise AssertionError('Mutation accepted: '+mutation)
# Execute only the affected existing assertions and adjacent contrast guards;
# avoid rerunning the independent multi-minute sprite/artwork suite.
script=(root/'tests/chart_presentation_resources_tests.py').read_text()
start_line=script.index("    for table,digest in [('DAY_BRIGHT'")
end_line=script.index('    # Fail closed on accidental broad recoloring',start_line)
import textwrap
scope={'data':data,'g':g,'ET':ET,'hashlib':hashlib,'output':a.output,'luminance':luminance,'contrast':contrast,'check':lambda v:check(v,'Affected existing resource assertion')}
exec(compile(textwrap.dedent(script[start_line:end_line]),'affected-existing-resource-assertions','exec'),scope)
record={'base':'585fd0f','upstream':data['upstreamCommit'],'checks':checks,'elapsedSeconds':time.monotonic()-start,'palette':expected,'contrasts':contrasts,'restoredXmlSha256':sha(restored.encode()),'baselineXmlSha256':baseline['files']['chartsymbols.xml']['sha256'],'generatedFiles':data['files'],'scope':'One generation; exact three-role reverse equality, affected existing assertions, contrasts and three negative semantic cases. No app/CI/boat.'}
(root/'docs/evidence/scrum231-land-fill/result.json').write_text(json.dumps(record,indent=2)+'\n');print(json.dumps(record,indent=2))

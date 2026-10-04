from pathlib import Path
import json,subprocess,hashlib
root=Path.cwd();out=root/'.local/final-private-object';reports=[]
for item in json.loads((root/'.local/compiled-commands.json').read_text())[:1]+json.loads((out/'commands.json').read_text()):
 cmd=item['command'];base=cmd[:cmd.index('-c')];source=cmd[cmd.index('-c')+1];name=item['name']
 macros=subprocess.check_output(base+['-E','-dM',source],text=True)
 text=subprocess.check_output(base+['-E','-P',source],text=True)
 for symbol in ['FishingPatternEligible(m_presentationLightSymbols, prule)','ComposeFishingPattern(', 'CableWaveRule(', 'DrawCableWaveGL(']:assert symbol in text,(name,symbol)
 selected={k:('#define '+k+' ') in macros or ('#define '+k+'\n') in macros for k in ['OPENNAV_X','SKAGER_OCHARTS_ADAPTER','ocpnUSE_GL','ocpnUSE_GLSL']}
 assert selected['OPENNAV_X']==name.startswith('core') and selected['SKAGER_OCHARTS_ADAPTER']==name.startswith('private'),selected
 reports.append({'name':name,'macros':selected,'preprocessedSha256':hashlib.sha256(text.encode()).hexdigest(),'combinedFeatureCallsPresent':True})
(root/'.local/production-branches.json').write_text(json.dumps(reports,indent=2)+'\n')

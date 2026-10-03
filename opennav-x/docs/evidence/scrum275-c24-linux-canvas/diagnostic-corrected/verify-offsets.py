from pathlib import Path
import subprocess,hashlib,re,json
p=Path(__file__).resolve().parent;exe=p/'inputs/install/bin/opencpn'
assert hashlib.sha256(exe.read_bytes()).hexdigest()=='13d6dcdf1c52f55237afd166f022d09252a7e362e66d962187fde08286fdb76d'
sym='_ZN7s52plib30RenderPresentationCaLightPointEP12_ObjRazRules'
nm=subprocess.check_output(['nm',str(exe)],text=True);start=int(re.search(r'^([0-9a-f]+) T '+sym+'$',nm,re.M)[1],16)
dis=subprocess.check_output(['objdump','-d','--start-address='+str(start),'--stop-address='+str(start+0x420),str(exe)],text=True)
expected={0x150:('48 8b 05 49 93 51 00','mov'),0x28f:('4d 85 ed','test')}
rows={int(m[0],16):(m[1].strip(),m[2]) for m in re.findall(r'^\s*([0-9a-f]+):\s*\t([0-9a-f ]+)\t([^\n]+)',dis,re.M)}
for off,(raw,insn) in expected.items():
 actual=rows[start+off];assert actual[0]==raw and actual[1].startswith(insn),(off,actual)
(p/'output/point-disassembly.txt').write_text(dis)
(p/'output/offset-receipt.json').write_text(json.dumps({'binarySha256':hashlib.sha256(exe.read_bytes()).hexdigest(),'symbol':sym,'start':hex(start),'instructionOffsets':{hex(k):v for k,v in expected.items()},'note':'0x150 after Take, 0x28f dictionary lookup complete before guard. Offsets validated at instruction boundaries, exact bytes.'},indent=2)+'\n')
print('Exact ELF diagnostic instruction offsets/bytes verified')

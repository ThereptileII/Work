"""Read-only record inventory of actual retained native SENC caches.

OSENC V2 packed records and feature/attribute IDs follow pinned Osenc.h and
s57objectclasses.csv/s57attributes.csv, not byte-string searches for CATCAM.
No decoder, chart download, application or hardware is launched.
"""
from pathlib import Path
import collections,csv,hashlib,json,struct,sys
source=Path(sys.argv[1]);captures=Path(sys.argv[2]);sha=lambda b:hashlib.sha256(b).hexdigest()
header=source/'gui/include/gui/Osenc.h';text=header.read_text()
assert '#define FEATURE_ID_RECORD 64' in text and '#define FEATURE_ATTRIBUTE_RECORD 65' in text
assert 'uint16_t feature_type_code;' in text and 'uint16_t attribute_type;' in text and '#pragma pack(push, 1)' in text
classes=source/'data/s57data/s57objectclasses.csv';attributes=source/'data/s57data/s57attributes.csv'
class_rows=list(csv.reader(classes.open()));attribute_rows=list(csv.reader(attributes.open()))
assert any(r[:3]==['14','Buoy, cardinal','BOYCAR'] for r in class_rows)
assert any(r[:3]==['5','Beacon, cardinal','BCNCAR'] for r in class_rows)
assert any(r[:3]==['13','Category of cardinal mark','CATCAM'] for r in attribute_rows)
result={'source_files':{str(p):sha(p.read_bytes()) for p in (header,classes,attributes)},'schema':'pinned packed little-endian OSENC V2; uint16 type, uint32 total record length','captures':[]}
for renderer in ('software','opengl'):
 for style in ('SKAGER','Standard'):
  files=list((captures/f'capture-78eccb8-{renderer}'/f'{style}-profile/SENC').glob('*US5SEAFL.S57'))
  assert len(files)==1
  p=files[0];data=p.read_bytes();offset=0;records=collections.Counter();features=collections.Counter();cardinal_attributes=[]
  while offset<len(data):
   kind,length=struct.unpack_from('<HI',data,offset)
   assert length>=6 and offset+length<=len(data)
   records[kind]+=1
   if kind==64:
    assert length==11
    code,identity,primitive=struct.unpack_from('<HHB',data,offset+6);features[code]+=1
   if kind==65:
    assert length>=9
    attribute,value_type=struct.unpack_from('<HB',data,offset+6)
    if attribute==13:cardinal_attributes.append({'feature_code':code,'feature_id':identity,'value_type':value_type,'payload_hex':data[offset+9:offset+length].hex()})
   offset+=length
  assert offset==len(data) and records[1]==1 and sum(features.values())==1930
  assert features[14]==features[5]==0 and not cardinal_attributes
  result['captures'].append({'renderer':renderer,'style':style,'path':str(p),'bytes':len(data),'sha256':sha(data),'features':sum(features.values()),'attributes':records[65],'BOYCAR':features[14],'BCNCAR':features[5],'CATCAM':cardinal_attributes,'feature_class_counts':dict(sorted(features.items()))})
result['conclusion']='No cardinal buoy/beacon or CATCAM occurs anywhere in the actual decoded public ENC fixture, so no cardinal can be visible in these views. Resource/loader proof does not establish real-ENC cardinal recognition.'
print(json.dumps(result,indent=2))

set pagination off
set confirm off
set breakpoint pending on
set debuginfod enabled off
python
import gdb,json,os,struct,math
path=os.environ['SKAGER_SEAMARK_TRACE'];scene=json.loads(os.environ['SKAGER_SEAMARK_SCENE'])
layout=json.load(open(os.environ['SKAGER_SEAMARK_LAYOUT']))
assert layout['pointerBytes']==8
all_wanted={(f['class'],round(f['attributes']['latitude'],6),round(f['attributes']['longitude'],6)) for f in scene['features']}
wanted={'RenderSY':{k for k in all_wanted if k[0]=='FOGSIG'},
        'RenderCARC':{k for k in all_wanted if k[0]=='LIGHTS'},
        'RenderRasterSymbol':{k for k in all_wanted if k[0]=='FOGSIG' or os.environ['SKAGER_SEAMARK_STYLE']=='XNav'}}
def emit(value):
 with open(path,'a') as f:f.write(json.dumps(value)+'\n')
def mem(address,size):
 assert 4096<=address<(1<<47) and 0<size<=8
 return bytes(gdb.selected_inferior().read_memory(address,size))
def field(ptr,key,fmt):
 return struct.unpack(fmt,mem(ptr+layout[key],struct.calcsize(fmt)))[0]
def pointer(address):
 value=struct.unpack('<Q',mem(address,8))[0]
 assert 4096<=value<(1<<47) and value%8==0
 return value
def reg(name,alignment=8):
 value=int(gdb.parse_and_eval('$'+name));assert 4096<=value<(1<<47) and value%alignment==0;return value
def read_args(kind):
 raster=kind=='RenderRasterSymbol'
 owner=reg('rdi');rz=reg('rsi');third=reg('rdx')
 vp=owner+layout['s52plib.vp_plib']
 if (field(vp,'VPointCompat.pix_width','<i'),field(vp,'VPointCompat.pix_height','<i'))!=(1014,566):return None
 obj=pointer(rz+layout['ObjRazRules.obj']);lup=pointer(rz+layout['ObjRazRules.LUP'])
 cls=mem(obj+layout['S57Obj.FeatureName'],6).decode('ascii')
 if (cls,round(field(obj,'S57Obj.m_lat','<d'),6),round(field(obj,'S57Obj.m_lon','<d'),6)) not in all_wanted:return None
 lat=field(obj,'S57Obj.m_lat','<d');lon=field(obj,'S57Obj.m_lon','<d')
 assert math.isfinite(lat) and -90<=lat<=90 and math.isfinite(lon) and -180<=lon<=180
 value={'kind':kind,'class':cls,'object_index':field(obj,'S57Obj.Index','<i'),'latitude':lat,'longitude':lon,'lookup_table':field(lup,'LUPrec.TNAM','<i'),'lookup_rcid':field(lup,'LUPrec.RCID','<i'),'priority':field(lup,'LUPrec.DPRI','<i'),'effective_symbol_table':field(owner,'s52plib.m_nSymbolStyle','<i')}
 if kind=='RenderCARC':
  ptr=field(third,'Rules.INSTstr','<Q');assert 4096<=ptr<(1<<47);raw=b''
  for n in range(160):
   c=mem(ptr+n,1)
   if c==b'\0':break
   raw+=c
  value['instruction']=raw.decode('ascii')
 else:
  rule=third if raster else pointer(third+layout['Rules.razRule'])
  symbol=mem(rule+layout['Rule.name'],8).decode('ascii');assert len(symbol)==8 and symbol.isalnum()
  value.update(symbol=symbol,definition=field(rule,'Rule.definition','<B'))
 if cls=='LIGHTS':
  count=field(obj,'S57Obj.n_attr','<i');assert 0<=count<=4096
  attributes=pointer(obj+layout['S57Obj.att_array']) if count else 0
  names=[mem(attributes+6*i,6).decode('ascii') for i in range(count)]
  value.update(attribute_names=names,has_ORIENT='ORIENT' in names)
 if raster:
  point=reg('rcx',4);x=field(point,'wxPoint.x','<i');y=field(point,'wxPoint.y','<i')
  assert -10000<x<10000 and -10000<y<10000
  value.update(pixel_x=x,pixel_y=y,canvas_width=1014,canvas_height=566)
 return value
class Start(gdb.Breakpoint):
 def stop(self):
  assert gdb.selected_frame().architecture().name()=='i386:x86-64'
  with open(os.environ['SKAGER_SEAMARK_PID'],'w') as f:f.write(str(gdb.selected_inferior().pid))
  emit({'kind':'abi','machine':'x86-64 SysV','argument_boundary':'Exact mangled symbol entry before prologue','layout':layout})
  return False
class Probe(gdb.Breakpoint):
 def __init__(self,symbol,kind):
  self.kind=kind;self.hits=0;self.seen=set();self.records=set();super().__init__('*'+symbol,internal=True)
 def stop(self):
  self.hits+=1;kind=self.kind
  if self.hits>4000:
   emit({'kind':'trace_disabled','probe':kind,'reason':'bounded_hit_limit','hits':self.hits});self.enabled=False;return False
  try:
   value=read_args(self.kind)
   if value is None:return False
   key=(value['class'],round(value['latitude'],6),round(value['longitude'],6))
   desired=wanted[self.kind]
   if key not in desired:return False
   identity=json.dumps(value,sort_keys=True)
   if identity not in self.records:self.records.add(identity);emit(value)
   self.seen.add(key)
   if desired<=self.seen:
    emit({'kind':'trace_disabled','probe':kind,'reason':'all_requested_geometry_observed','hits':self.hits});self.enabled=False
  except Exception as error:
   emit({'kind':'trace_error','probe':kind,'error':str(error)});self.enabled=False
  return False
Start('main',temporary=True,internal=True)
Probe('_ZN7s52plib8RenderSYEP12_ObjRazRulesP6_Rules','RenderSY')
Probe('_ZN7s52plib10RenderCARCEP12_ObjRazRulesP6_Rules','RenderCARC')
Probe('_ZN7s52plib18RenderRasterSymbolEP12_ObjRazRulesP5_RuleR7wxPointf','RenderRasterSymbol')
def exited(event):emit({'kind':'inferior_exit','exit_code':getattr(event,'exit_code',None)})
gdb.events.exited.connect(exited)
end
run

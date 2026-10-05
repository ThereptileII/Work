set pagination off
set confirm off
set breakpoint pending on
set debuginfod enabled off
python
import gdb,json,os,struct,math
path=os.environ['SKAGER_SEAMARK_TRACE'];layout=json.load(open(os.environ['SKAGER_SEAMARK_LAYOUT']))
seq=0
active=None
def emit(x):
 global seq
 seq+=1;x['sequence']=seq
 with open(path,'a') as f:f.write(json.dumps(x)+'\n')
def mem(p,n):
 assert 4096<=p<(1<<47) and 0<n<=8
 return bytes(gdb.selected_inferior().read_memory(p,n))
def read(p,fmt):return struct.unpack(fmt,mem(p,struct.calcsize(fmt)))[0]
def field(p,k,fmt):return read(p+layout[k],fmt)
def reg(k):return int(gdb.parse_and_eval('$'+k))
def text(p,n=160):
 if not p:return None
 raw=b''
 for i in range(n):
  c=mem(p+i,1)
  if c==b'\0':break
  raw+=c
 return raw.decode('ascii',errors='replace')
def obj(rz):
 p=field(rz,'ObjRazRules.obj','<Q');lu=field(rz,'ObjRazRules.LUP','<Q')
 val={'rz':hex(rz),'object':hex(p),'class':mem(p+layout['S57Obj.FeatureName'],6).decode('ascii'),'index':field(p,'S57Obj.Index','<i'),'lat':field(p,'S57Obj.m_lat','<d'),'lon':field(p,'S57Obj.m_lon','<d'),'primitive':field(p,'S57Obj.Primitive_type','<i'),'context':hex(field(p,'S57Obj.m_chart_context','<Q')),'x':field(p,'S57Obj.x','<d'),'y':field(p,'S57Obj.y','<d')}
 if lu:
  val['lookup']={k:field(lu,'LUPrec.'+k,'<i') for k in ['RCID','FTYP','RPRI','DISC','LUCM','TNAM','DPRI']}
  rule=field(lu,'LUPrec.ruleList','<Q');val['ruleLoaded']=bool(rule)
  if rule:
   val['rule']={'type':field(rule,'Rules.ruleType','<i'),'next':hex(field(rule,'Rules.next','<Q')),'private':field(rule,'Rules.b_private_razRule','<B'),'instruction':text(field(rule,'Rules.INSTstr','<Q'))}
   s=field(rule,'Rules.razRule','<Q')
   if s and val['rule']['type']==3:
    val['symbol']={'rcid':field(s,'Rule.RCID','<i'),'name':mem(s+layout['Rule.name'],8).decode('ascii'),'definition':field(s,'Rule.definition','<B'),'position':{k:field(s,'Rule.pos.symb.'+k,'<i') for k in ['bnbox_w.SYHL','bnbox_h.SYVL','pivot_x.SYCL','pivot_y.SYRW','bnbox_x.SBXC','bnbox_y.SBXR']}}
 n=field(p,'S57Obj.n_attr','<i');assert 0<=n<=4096
 a=field(p,'S57Obj.att_array','<Q');val['attributes']=[mem(a+6*i,6).decode('ascii') for i in range(n)]
 return val
def target(o):return abs(o['lat']+32.3760351)<1e-6 and abs(o['lon']-61.0307025)<1e-6
def members(p):
 n=read(p+layout['inventoryMapCountOffset'],'<Q');assert n<=32768
 node=read(p+layout['inventoryMapHeadOffset'],'<Q');out=[]
 while node:
  key=read(node+layout['nodeKeyOffset'],'<Q');value=read(node+layout['nodeValueOffset'],'<Q')
  out.append({'object':hex(key),'name':text(value,8)})
  assert len(out)<=n
  node=read(node,'<Q')
 assert len(out)==n
 return out
class Start(gdb.Breakpoint):
 def stop(self):
  with open(os.environ['SKAGER_SEAMARK_PID'],'w') as f:f.write(str(gdb.selected_inferior().pid))
  emit({'kind':'abi','layout':layout,'map_layout':'count offset24 verified by separate same-compiler populated-map helper and actual ELF disassembly; no inferior calls'})
  return False
class Finish(gdb.FinishBreakpoint):
 def __init__(self,p,entry):self.p=p;self.entry=entry;super().__init__(gdb.newest_frame(),internal=True)
 def stop(self):
  emit({'kind':'inventory_constructed','inventory':hex(self.p),'entry':self.entry,'size':read(self.p+layout['inventoryMapCountOffset'],'<Q'),'members':members(self.p)});return False
class Inventory(gdb.Breakpoint):
 def __init__(self):self.count=0;super().__init__('*_ZN7opennav11integration21CaLightPointInventoryC1ILm10ELm5EEEbRAT__AT0__KP12_ObjRazRulesj',internal=True)
 def stop(self):
  self.count+=1
  if self.count>20:self.enabled=False;return False
  try:
   p=reg('rdi');heads=reg('rdx');col=reg('rcx');assert col<5
   record={'kind':'inventory_entry','inventory':hex(p),'enabled':reg('rsi')&255,'column':col,'target_objects':[],'bad_nodes':[],'count':0};seen=set()
   for i in range(10):
    rz=read(heads+8*(i*5+col),'<Q')
    while rz:
     record['count']+=1;assert record['count']<=32768
     o=obj(rz)
     if o['object'] in seen:record['bad_nodes'].append({'reason':'duplicate','object':o});break
     seen.add(o['object'])
     if o['primitive']==0 and (o['context']=='0x0' or not math.isfinite(o['x']) or not math.isfinite(o['y'])):record['bad_nodes'].append({'reason':'invalid context/coordinate','object':o})
     if target(o):record['target_objects'].append(o)
     rz=field(rz,'ObjRazRules.next','<Q')
   emit(record);Finish(p,seq)
  except Exception as e:emit({'kind':'trace_error','probe':'inventory','error':str(e)});self.enabled=False
  return False
class Point(gdb.Breakpoint):
 def __init__(self):self.count=0;super().__init__('*_ZN7s52plib30RenderPresentationCaLightPointEP12_ObjRazRules',internal=True)
 def stop(self):
  try:
   global active
   owner=reg('rdi');rz=reg('rsi');o=obj(rz);active=o
   if not target(o):return False
   self.count+=1
   if self.count>40:self.enabled=False;return False
   p=field(owner,'s52plib.m_presentationCaLights','<Q');vp=owner+layout['s52plib.vp_plib']
   emit({'kind':'point_entry','object':o,'enabled':field(owner,'s52plib.m_presentationLightSymbols','<B'),'glsl':field(owner,'s52plib.m_useGLSL','<B'),'dc':hex(field(owner,'s52plib.m_pdc','<Q')),'table':field(owner,'s52plib.m_nSymbolStyle','<i'),'inventory':hex(p),'inventory_size':read(p+layout['inventoryMapCountOffset'],'<Q') if p else None,'members':members(p) if p else None,'canvas':[field(vp,'VPointCompat.pix_width','<i'),field(vp,'VPointCompat.pix_height','<i')]})
  except Exception as e:emit({'kind':'trace_error','probe':'point','error':str(e)});self.enabled=False
  return False
class Taken(gdb.Breakpoint):
 def __init__(self):self.count=0;super().__init__('*_ZN7s52plib30RenderPresentationCaLightPointEP12_ObjRazRules+0x150',internal=True)
 def stop(self):
  self.count+=1
  if self.count>40:self.enabled=False;return False
  try:emit({'kind':'take_succeeded','name':text(reg('rsi'),8),'inventory':hex(reg('r12')),'object':active})
  except Exception as e:emit({'kind':'trace_error','probe':'taken','error':str(e)});self.enabled=False
  return False
class Dictionary(gdb.Breakpoint):
 def __init__(self):self.count=0;super().__init__('*_ZN7s52plib30RenderPresentationCaLightPointEP12_ObjRazRules+0x28f',internal=True)
 def stop(self):
  self.count+=1
  if self.count>40:self.enabled=False;return False
  try:
   node=reg('r13');rule=read(node+0x38,'<Q') if node else 0
   emit({'kind':'dictionary_result','object':active,'found':bool(node),'rule_present':bool(rule),'rule':{'name':mem(rule+layout['Rule.name'],8).decode('ascii'),'definition':field(rule,'Rule.definition','<B'),'RCID':field(rule,'Rule.RCID','<i')} if rule else None})
  except Exception as e:emit({'kind':'trace_error','probe':'dictionary','error':str(e)});self.enabled=False
  return False
Start('main',temporary=True,internal=True);Inventory();Point();Taken();Dictionary()
def exited(event):emit({'kind':'inferior_exit','exit_code':getattr(event,'exit_code',None)})
gdb.events.exited.connect(exited)
end
run

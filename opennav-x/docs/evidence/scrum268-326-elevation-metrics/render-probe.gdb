set pagination off
set confirm off
set breakpoint pending on
set debuginfod enabled off
python
import gdb,json,os,struct,time
path=os.environ['SKAGER_SEAMARK_TRACE'];layout=json.load(open(os.environ['SKAGER_SEAMARK_LAYOUT']))
base=os.path.dirname(os.environ['SKAGER_SEAMARK_LAYOUT']);entries=json.load(open(base+'/entries.json'))
active=None;target=None;sequence=0;hits=0

def emit(v):
 v['monotonic']=time.monotonic()
 try:v['phase']=json.load(open(base+'/phase.json'))
 except FileNotFoundError:v['phase']={'index':0,'theme':'Day','startup':True}
 with open(path,'a') as f:f.write(json.dumps(v)+'\n')
def read(p,n):
 assert 4096<=p<(1<<47) and 0<n<=256
 return bytes(gdb.selected_inferior().read_memory(p,n))
def value(p,fmt):return struct.unpack(fmt,read(p,struct.calcsize(fmt)))[0]
def field(p,k,fmt):return value(p+layout[k],fmt)
def reg(n):return int(gdb.parse_and_eval('$'+n))
def string(p):
 out=b''
 for n in range(160):
  c=read(p+n,1)
  if c==b'\0':return out.decode('utf-8',errors='replace')
  out+=c
 raise RuntimeError('Unbounded source text')
def object_record(p):
 return {'object':p,'class':read(p+layout['S57Obj.FeatureName'],6).decode('ascii'),'index':field(p,'S57Obj.Index','<i'),'latitude':field(p,'S57Obj.m_lat','<d'),'longitude':field(p,'S57Obj.m_lon','<d')}
def matches(r):return r['class']=='LNDELV' and abs(r['latitude']+32.3740304)<1e-7 and abs(r['longitude']-61.0376363)<1e-7

def metrics(owner,text):
 v={k:field(text,'S52_TextC.'+k,fmt) for k,fmt in [('bsize','<i'),('avgCharWidth','<i'),('pFont','<Q'),('xoffs','<i'),('yoffs','<i'),('hjust','<B'),('vjust','<B'),('rendered_char_height','<i'),('letter_spacing','<d'),('text_opacity','<B'),('light_label','<?'),('bspecial_char','<?'),('texobj','<i')]}
 v.update({k:field(owner,'s52plib.'+k,fmt) for k,fmt in [('m_colortable_index','<i'),('m_TextScaleFactor','<d'),('m_dipfactor','<d'),('m_ContentScaleFactor','<d'),('m_FinalTextScaleFactor','<d')]})
 caches=[]
 for i in range(8):
  p=owner+layout['s52plib.s_txf']+i*layout['TexFontCache.size']
  caches.append({'slot':i,'key':field(p,'TexFontCache.key','<Q'),'cache':field(p,'TexFontCache.cache','<Q')})
 vp=owner+layout['s52plib.vp_plib']
 v['canvas']=[field(vp,'VPointCompat.pix_width','<i'),field(vp,'VPointCompat.pix_height','<i')]
 v['stored_rText']=list(struct.unpack('<iiii',read(text+layout['S52_TextC.rText'],16)))
 v['caches']=caches;v['matching_cache']=[c for c in caches if c['key']==v['pFont']]
 return v

class Return(gdb.FinishBreakpoint):
 def __init__(self,kind,record):self.kind=kind;self.record=record;super().__init__(gdb.newest_frame(),internal=True)
 def out_of_scope(self):emit({'kind':'trace_error','probe':self.kind+'_return','error':'FinishBreakpoint left scope without normal return','sequence':self.record.get('sequence')})
 def stop(self):
  global active,target
  try:
   r=dict(self.record);r['kind']=self.kind+'_return'
   if self.kind=='RenderText':
    r.update(metrics(r['owner'],r['text']));r['drawn']=bool(reg('rax')&255);r['rectangle']=list(struct.unpack('<iiii',read(r['rect'],16)));active=None
   elif self.kind=='RenderT_All':target=None
   elif self.kind.startswith('GetTextExtent'):
    r['width']=value(r['wptr'],'<i') if r['wptr'] else None;r['height']=value(r['hptr'],'<i') if r['hptr'] else None
   emit(r)
  except Exception as e:emit({'kind':'trace_error','probe':self.kind+'_return','error':str(e)})
  return False

class Probe(gdb.Breakpoint):
 def __init__(self,symbol,kind):self.kind=kind;super().__init__('*'+symbol,internal=True)
 def stop(self):
  global active,target,sequence,hits
  hits+=1
  if hits>18000:emit({'kind':'trace_error','error':'Bounded 18000 hit limit'});gdb.execute('quit');return False
  try:
   k=self.kind
   if k=='RenderT_All':
    rz=reg('rsi');obj=field(rz,'ObjRazRules.obj','<Q');r=object_record(obj)
    if not matches(r):return False
    lup=field(rz,'ObjRazRules.LUP','<Q');rules=reg('rdx')
    r.update(kind=k,owner=reg('rdi'),lookup_rcid=field(lup,'LUPrec.RCID','<i'),lookup_table=field(lup,'LUPrec.TNAM','<i'),instruction=string(field(rules,'Rules.INSTstr','<Q')),bTX=reg('rcx')&255,cached_FText=field(obj,'S57Obj.FText','<Q'),bFText_Added=field(obj,'S57Obj.bFText_Added','<?'))
    sequence+=1;r['sequence']=sequence;target=r;emit(r);Return(k,r)
   elif k=='RenderText':
    obj=value(reg('rsp')+8,'<Q')
    if not target or obj!=target['object']:return False
    r={'kind':k,'sequence':sequence,'owner':reg('rdi'),'text':reg('rdx'),'pdc':reg('rsi'),'anchor_x':reg('ecx'),'anchor_y':reg('r8d'),'rect':reg('r9'),'object':obj}
    r.update(metrics(r['owner'],r['text']));active=r;emit(r);Return(k,r)
   elif active:
    r={'kind':k,'sequence':active['sequence'],'cache':reg('rdi')}
    if k=='Build':r['font']=reg('rsi')
    else:
     r['input_pointer']=reg('rsi');r['is_label_wxstring']=reg('rsi')==active['text']+layout['S52_TextC.frmtd']
     if k.endswith('_char'):r['input']=string(reg('rsi'))
     if k.startswith('GetTextExtent'):r.update(wptr=reg('rdx'),hptr=reg('rcx'))
     else:r.update(x=reg('edx'),y=reg('ecx'))
    active['nested_events']=active.get('nested_events',0)+1
    if active['nested_events']>64:raise RuntimeError('Bounded64 events per target call')
    emit(r)
    if not k.startswith('RenderString'):Return(k,r)
  except Exception as e:emit({'kind':'trace_error','probe':self.kind,'error':str(e)})
  return False

class Start(gdb.Breakpoint):
 def stop(self):
  assert gdb.selected_frame().architecture().name()=='i386:x86-64'
  with open(os.environ['SKAGER_SEAMARK_PID'],'w') as f:f.write(str(gdb.selected_inferior().pid))
  for symbol,entry in entries.items():
   addr=int(gdb.parse_and_eval('&'+symbol));actual=read(addr,16).hex();assert actual==entry['bytes'],(symbol,actual,entry)
   Probe(symbol,entry['kind'])
  emit({'kind':'abi','machine':'x86-64 SysV','layout':layout,'validated_entries':entries,'scope':'External memory/register reads only; no inferior calls or writes; entry breakpoint bytes verified against unchanged ELF'})
  return False
Start('main',temporary=True,internal=True)
def exited(event):emit({'kind':'inferior_exit','exit_code':getattr(event,'exit_code',None)})
gdb.events.exited.connect(exited)
end
run

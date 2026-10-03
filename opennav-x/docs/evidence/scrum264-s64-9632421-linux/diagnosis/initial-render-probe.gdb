set pagination off
set confirm off
set breakpoint pending on
set debuginfod enabled off
python
import gdb,json,os
path=os.environ['SKAGER_SEAMARK_TRACE']
scene=json.loads(os.environ['SKAGER_SEAMARK_SCENE'])
wanted={(f['class'],round(f['attributes']['latitude'],6),round(f['attributes']['longitude'],6)) for f in scene['features']}
for f in scene['features']:
 for related in f['related']:
  if related['class']=='LIGHTS':
   a=related['attributes'];wanted.add(('LIGHTS',round(a['latitude'],6),round(a['longitude'],6)))
seen=set();records=set()
raster_wanted={(f['class'],round(f['attributes']['latitude'],6),round(f['attributes']['longitude'],6)) for f in scene['features']}
if os.environ['SKAGER_SEAMARK_STYLE']=='XNav':
 for f in scene['features']:
  for related in f['related']:
   a=related['attributes']
   if related['class']=='LIGHTS' and a.get('COLOUR') in (['1'],['6'],['9']):raster_wanted.add(('LIGHTS',round(a['latitude'],6),round(a['longitude'],6)))
raster_seen=set()
def emit(value):
 with open(path,'a') as f:f.write(json.dumps(value)+'\n')
class Start(gdb.Breakpoint):
 def stop(self):
  with open(os.environ['SKAGER_SEAMARK_PID'],'w') as f:f.write(str(gdb.selected_inferior().pid))
  return False
class Render(gdb.Breakpoint):
 hits=0
 def stop(self):
  self.hits+=1
  if self.hits>4000:
   emit({'kind':'trace_disabled','probe':'RenderSY','reason':'bounded_hit_limit','hits':self.hits});self.enabled=False;return False
  try:
   frame=gdb.newest_frame();owner=frame.read_var('this')
   if int(owner['vp_plib']['pix_width'])!=1014 or int(owner['vp_plib']['pix_height'])!=566:return False
   rz=frame.read_var('rzRules');rules=frame.read_var('rules');obj=rz['obj'].dereference()
   cls=obj['FeatureName'].string(length=6)
   if cls not in ('BOYLAT','BOYISD','BOYSAW','BOYCAR','BOYSPP','LIGHTS','TOPMAR'):return False
   lat=float(obj['m_lat']);lon=float(obj['m_lon']);key=(cls,round(lat,6),round(lon,6))
   rule=rules['razRule'];symbol=rule['name']['SYNM'].string(length=8) if int(rule) else None
   identity=(cls,int(obj['Index']),lat,lon,symbol)
   if identity not in records:
    records.add(identity)
    emit({'kind':'RenderSY','class':cls,'object_index':int(obj['Index']),'latitude':lat,'longitude':lon,'symbol':symbol,'definition':int(rule['definition']['SYDF']) if int(rule) else None,'lookup_table':int(rz['LUP']['TNAM']),'effective_symbol_table':int(frame.read_var('this')['m_nSymbolStyle'])})
   seen.add(key)
   if wanted<=seen:
    emit({'kind':'trace_disabled','probe':'RenderSY','reason':'all_requested_geometry_observed','hits':self.hits});self.enabled=False
  except Exception as error:
   emit({'kind':'trace_error','error':str(error)});self.enabled=False
  return False
class Raster(gdb.Breakpoint):
 hits=0
 def stop(self):
  self.hits+=1
  if self.hits>4000:
   emit({'kind':'trace_disabled','probe':'RenderRasterSymbol','reason':'bounded_hit_limit','hits':self.hits});self.enabled=False;return False
  try:
   frame=gdb.newest_frame();owner=frame.read_var('this')
   if int(owner['vp_plib']['pix_width'])!=1014 or int(owner['vp_plib']['pix_height'])!=566:return False
   rz=frame.read_var('rzRules');obj=rz['obj'].dereference();cls=obj['FeatureName'].string(length=6)
   lat=float(obj['m_lat']);lon=float(obj['m_lon']);key=(cls,round(lat,6),round(lon,6))
   if key not in raster_wanted:return False
   rule=frame.read_var('prule');point=frame.read_var('r')
   emit({'kind':'RenderRasterSymbol','class':cls,'object_index':int(obj['Index']),'latitude':lat,'longitude':lon,'symbol':rule['name']['SYNM'].string(length=8),'pixel_x':int(point['x']),'pixel_y':int(point['y']),'canvas_width':int(owner['vp_plib']['pix_width']),'canvas_height':int(owner['vp_plib']['pix_height'])})
   raster_seen.add(key)
   if raster_wanted<=raster_seen:
    emit({'kind':'trace_disabled','probe':'RenderRasterSymbol','reason':'all_requested_geometry_observed','hits':self.hits});self.enabled=False
  except Exception as error:
   emit({'kind':'trace_error','probe':'RenderRasterSymbol','error':str(error)});self.enabled=False
  return False
Start('main',temporary=True,internal=True)
Render('s52plib::RenderSY',internal=True)
Raster('s52plib::RenderRasterSymbol',internal=True)
def exited(event):emit({'kind':'inferior_exit','exit_code':getattr(event,'exit_code',None)})
gdb.events.exited.connect(exited)
end
run

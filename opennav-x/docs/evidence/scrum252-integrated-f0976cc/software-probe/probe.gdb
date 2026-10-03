set pagination off
set confirm off
set breakpoint pending on
set debuginfod enabled off
python
import gdb,json
counts={}
def reg(n):return int(gdb.parse_and_eval('$'+n))
def emit(kind,**values):gdb.write('ROUTE_PROBE '+json.dumps(dict(kind=kind,**values))+'\n')
class Return(gdb.FinishBreakpoint):
 def __init__(self,kind,values):
  self.kind=kind;self.values=values
  super().__init__(gdb.newest_frame(),internal=True)
 def stop(self):
  emit(self.kind,return_rax=reg('rax'),**self.values);return False
class Probe(gdb.Breakpoint):
 def __init__(self,sym,kind):
  self.kind=kind;super().__init__(sym,internal=True)
 def stop(self):
  stack=gdb.execute('bt 5',to_string=True)
  if self.kind=='context' and 'ChartRoute' not in stack:return False
  n=counts.get(self.kind,0);counts[self.kind]=n+1
  if n>=24:self.enabled=False;return False
  values={'call':n,'stack':stack}
  if self.kind=='ordinal':values['pinned_icon']=reg('rdx') & 255
  if self.kind=='label_prepare':values['ordinal']=reg('rdx') & 0xffffffff
  if self.kind=='waypoint_draw':values['ordinal']=reg('r8') & 0xffffffff
  if self.kind=='context':
   ptr=reg('rdi');vtable=int(gdb.parse_and_eval('*(void**)'+str(ptr)))
   values['native_dc_vtable']=gdb.execute('info symbol '+str(vtable),to_string=True)
  Return(self.kind,values);return False
class Main(gdb.Breakpoint):
 def stop(self):
  import os
  with open(os.environ['SKAGER_ROUTE252_OUTPUT']+'/inferior.pid','w') as f:f.write(str(gdb.selected_inferior().pid))
  return False
Main('main',temporary=True,internal=True)
Probe('opennav::integration::ChartRouteWaypointOrdinal','ordinal')
Probe('opennav::integration::PrepareChartRouteLabel','label_prepare')
Probe('opennav::integration::DrawChartRouteWaypoint','waypoint_draw')
Probe('wxGraphicsContext::CreateFromUnknownDC','context')
end
run

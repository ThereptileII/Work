import ctypes as c, os, time
X=c.CDLL('libX11.so.6'); W=c.c_ulong; D=c.c_void_p
class Attr(c.Structure):
 _fields_=[('x',c.c_int),('y',c.c_int),('width',c.c_int),('height',c.c_int),('border_width',c.c_int),('depth',c.c_int),('visual',D),('root',W),('class_',c.c_int),('bit_gravity',c.c_int),('win_gravity',c.c_int),('backing_store',c.c_int),('backing_planes',W),('backing_pixel',W),('save_under',c.c_int),('colormap',W),('map_installed',c.c_int),('map_state',c.c_int),('all_event_masks',c.c_long),('your_event_mask',c.c_long),('do_not_propagate_mask',c.c_long),('override_redirect',c.c_int),('screen',D)]
X.XOpenDisplay.argtypes=[c.c_char_p];X.XOpenDisplay.restype=D
X.XDefaultRootWindow.argtypes=[D];X.XDefaultRootWindow.restype=W
X.XQueryTree.argtypes=[D,W,c.POINTER(W),c.POINTER(W),c.POINTER(c.POINTER(W)),c.POINTER(c.c_uint)]
X.XGetWindowAttributes.argtypes=[D,W,c.POINTER(Attr)]
X.XFetchName.argtypes=[D,W,c.POINTER(c.c_char_p)]
X.XFree.argtypes=[D];X.XCloseDisplay.argtypes=[D]
def window_tree(display):
 t=time.monotonic_ns();d=X.XOpenDisplay(display.encode());assert d
 def visit(w,depth):
  a=Attr();assert X.XGetWindowAttributes(d,w,c.byref(a))
  n=c.c_char_p();X.XFetchName(d,w,c.byref(n));name=n.value.decode(errors='replace') if n.value else ''
  if n:X.XFree(n)
  r,p=W(),W();kids=c.POINTER(W)();num=c.c_uint();X.XQueryTree(d,w,c.byref(r),c.byref(p),c.byref(kids),c.byref(num))
  rows=[{'id':int(w),'parent':int(p.value),'depth':depth,'name':name,'xywh':[a.x,a.y,a.width,a.height],'map_state':a.map_state,'override_redirect':a.override_redirect}]
  for i in range(num.value):rows+=visit(kids[i],depth+1)
  if kids:X.XFree(kids)
  return rows
 try:return {'monotonic_ns':t,'completed_monotonic_ns':time.monotonic_ns(),'ordering':'XQueryTree siblings bottom to top','windows':visit(X.XDefaultRootWindow(d),0)}
 finally:X.XCloseDisplay(d)

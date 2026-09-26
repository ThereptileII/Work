// Native interop for one hash-gated official OpenCPN setup process only.
// Tree interop uses a bounded temporary TVITEM32 buffer, not installer internals.
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;

public static class OpenNavOfficialWizard {
  public sealed class Control {
    public long Handle; public string Text, Class; public bool Enabled;
  }
  public sealed class TreeItem {
    public long Handle; public string Text; public int State, Depth;
  }
  [StructLayout(LayoutKind.Sequential)] public struct RECT { public int Left, Top, Right, Bottom; }
  [StructLayout(LayoutKind.Sequential)] struct POINT { public int X,Y; public POINT(int x,int y){X=x;Y=y;} }
  delegate bool EnumProc(IntPtr h, IntPtr data);
  [DllImport("user32.dll")] static extern bool EnumWindows(EnumProc callback,IntPtr data);
  [DllImport("user32.dll")] static extern bool EnumChildWindows(IntPtr parent,EnumProc callback,IntPtr data);
  [DllImport("user32.dll")] static extern uint GetWindowThreadProcessId(IntPtr h,out uint pid);
  [DllImport("user32.dll")] static extern bool IsWindowVisible(IntPtr h);
  [DllImport("user32.dll")] static extern bool IsWindowEnabled(IntPtr h);
  [DllImport("user32.dll")] static extern bool IsIconic(IntPtr h);
  [DllImport("user32.dll")] static extern int GetSystemMetrics(int index);
  [DllImport("user32.dll",CharSet=CharSet.Unicode)] static extern int GetClassName(IntPtr h,StringBuilder text,int count);
  [DllImport("user32.dll",CharSet=CharSet.Unicode)] static extern IntPtr SendMessageTimeout(IntPtr h,uint message,IntPtr w,IntPtr l,uint flags,uint timeout,out IntPtr result);
  [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h,out RECT rect);
  [DllImport("user32.dll")] static extern bool SetForegroundWindow(IntPtr h);
  [DllImport("user32.dll")] static extern bool SetProcessDPIAware();
  [DllImport("user32.dll")] static extern IntPtr GetForegroundWindow();
  [DllImport("user32.dll")] static extern IntPtr GetAncestor(IntPtr h,uint flag);
  [DllImport("user32.dll")] static extern bool IsChild(IntPtr parent,IntPtr child);
  [DllImport("user32.dll")] static extern bool SetCursorPos(int x,int y);
  [DllImport("user32.dll")] static extern IntPtr WindowFromPoint(POINT p);
  [DllImport("user32.dll")] static extern void mouse_event(uint flags,uint x,uint y,uint data,UIntPtr extra);
  [DllImport("kernel32.dll",SetLastError=true)] static extern IntPtr OpenProcess(uint access,bool inherit,uint pid);
  [DllImport("kernel32.dll")] static extern bool IsWow64Process(IntPtr process,out bool wow);
  [DllImport("kernel32.dll")] static extern bool CloseHandle(IntPtr h);
  [DllImport("kernel32.dll",SetLastError=true)] static extern IntPtr VirtualAllocEx(IntPtr process,IntPtr address,UIntPtr size,uint kind,uint protect);
  [DllImport("kernel32.dll")] static extern bool VirtualFreeEx(IntPtr process,IntPtr address,UIntPtr size,uint kind);
  [DllImport("kernel32.dll",SetLastError=true)] static extern bool WriteProcessMemory(IntPtr process,IntPtr address,byte[] data,UIntPtr size,out UIntPtr written);
  [DllImport("kernel32.dll",SetLastError=true)] static extern bool ReadProcessMemory(IntPtr process,IntPtr address,byte[] data,UIntPtr size,out UIntPtr read);
  public static void Initialize() { SetProcessDPIAware(); }
  public static void Own(long handle,int pid) {
    uint owner; GetWindowThreadProcessId(new IntPtr(handle),out owner);
    if(owner!=(uint)pid || !IsWindowVisible(new IntPtr(handle))) throw new InvalidOperationException("Window is not visible and owned by this setup PID");
  }
  public static long Message(long handle,int pid,uint message,long w,long l) {
    Own(handle,pid); IntPtr result;
    if(SendMessageTimeout(new IntPtr(handle),message,new IntPtr(w),new IntPtr(l),2,3000,out result)==IntPtr.Zero)
      throw new InvalidOperationException("Owned control did not respond");
    return result.ToInt64();
  }
  public static string Class(long handle) { var text=new StringBuilder(128);GetClassName(new IntPtr(handle),text,128);return text.ToString(); }
  public static string Text(long handle,int pid) {
    int count=(int)Math.Min(8191,Math.Max(0,Message(handle,pid,14,0,0)));
    IntPtr buffer=Marshal.AllocHGlobal((count+1)*2);
    try { Marshal.WriteInt16(buffer,0);Message(handle,pid,13,count+1,buffer.ToInt64());return Marshal.PtrToStringUni(buffer)??""; }
    finally {Marshal.FreeHGlobal(buffer);}
  }
  static Control Describe(IntPtr handle,int pid) {
    return new Control {Handle=handle.ToInt64(),Text=Text(handle.ToInt64(),pid),Class=Class(handle.ToInt64()),Enabled=IsWindowEnabled(handle)};
  }
  public static Control[] Windows(int pid) {
    var result=new List<Control>();Exception failure=null;
    EnumWindows((h,d)=>{try{uint owner;GetWindowThreadProcessId(h,out owner);if(owner==(uint)pid && IsWindowVisible(h))result.Add(Describe(h,pid));return true;}catch(Exception e){failure=e;return false;}},IntPtr.Zero);
    if(failure!=null)throw new InvalidOperationException("Cannot inspect exact setup windows",failure);
    return result.ToArray();
  }
  public static Control[] Children(long parent,int pid) {
    Own(parent,pid);var result=new List<Control>();Exception failure=null;
    EnumChildWindows(new IntPtr(parent),(h,d)=>{try{if(IsWindowVisible(h)){Own(h.ToInt64(),pid);if(result.Count>=256)throw new InvalidOperationException("Too many setup controls");result.Add(Describe(h,pid));}return true;}catch(Exception e){failure=e;return false;}},IntPtr.Zero);
    if(failure!=null)throw new InvalidOperationException("Cannot inspect owned setup controls",failure);
    return result.ToArray();
  }
  public static void Foreground(long handle,int pid) {
    Own(handle,pid);SetForegroundWindow(new IntPtr(handle));Thread.Sleep(150);
    AssertForeground(handle,pid);
  }
  public static void AssertForeground(long handle,int pid) {
    Own(handle,pid);var foreground=GetForegroundWindow();uint owner;GetWindowThreadProcessId(foreground,out owner);
    if(owner!=(uint)pid || GetAncestor(foreground,2)!=GetAncestor(new IntPtr(handle),2) || IsIconic(new IntPtr(handle)))
      throw new InvalidOperationException("Exact setup window does not own visible foreground; no input or capture allowed");
  }
  public static RECT CaptureBounds(long handle,int pid) {
    AssertForeground(handle,pid);RECT rect;
    if(!GetWindowRect(new IntPtr(handle),out rect))throw new InvalidOperationException("Setup capture bounds unavailable");
    int x=GetSystemMetrics(76),y=GetSystemMetrics(77),width=GetSystemMetrics(78),height=GetSystemMetrics(79);
    if(rect.Right<=rect.Left || rect.Bottom<=rect.Top || rect.Left<x || rect.Top<y || (long)rect.Right>(long)x+width || (long)rect.Bottom>(long)y+height)
      throw new InvalidOperationException("Setup bounds are not fully inside the visible desktop");
    return rect;
  }
  public static void Click(long handle,int pid) {
    Own(handle,pid);if(Class(handle)!="Button" || !IsWindowEnabled(new IntPtr(handle)))throw new InvalidOperationException("Expected enabled setup button");
    var root=GetAncestor(new IntPtr(handle),2);Foreground(root.ToInt64(),pid);
    RECT r;if(!GetWindowRect(new IntPtr(handle),out r) || r.Right<=r.Left || r.Bottom<=r.Top)throw new InvalidOperationException("Invalid button bounds");
    var point=new POINT((r.Left+r.Right)/2,(r.Top+r.Bottom)/2);
    if(!SetCursorPos(point.X,point.Y))throw new InvalidOperationException("Cannot position pointer");
    Thread.Sleep(100);Own(handle,pid);
    var hit=WindowFromPoint(point);if(hit!=new IntPtr(handle)&&!IsChild(new IntPtr(handle),hit))throw new InvalidOperationException("Setup button is obscured; no click sent");
    uint owner;GetWindowThreadProcessId(GetForegroundWindow(),out owner);if(owner!=(uint)pid)throw new InvalidOperationException("Foreground changed; no click sent");
    mouse_event(2,0,0,0,UIntPtr.Zero);mouse_event(4,0,0,0,UIntPtr.Zero);
  }
  public static void English(long combo,int pid) {
    if(Class(combo)!="ComboBox")throw new InvalidOperationException("Expected language selector");
    var text=Marshal.StringToHGlobalUni("English");
    try {long index=Message(combo,pid,0x158,-1,text.ToInt64());if(index<0)throw new InvalidOperationException("English not offered");if(Message(combo,pid,0x14e,index,0)!=index)throw new InvalidOperationException("Language not selected");}
    finally {Marshal.FreeHGlobal(text);}
    if(Text(combo,pid)!="English")throw new InvalidOperationException("English selection did not persist");
  }
  public static void Check(long button,int pid,bool value) {
    if(Class(button)!="Button")throw new InvalidOperationException("Expected check/radio button");
    Message(button,pid,0xf1,value?1:0,0);
    if(Message(button,pid,0xf0,0,0)!=(value?1:0))throw new InvalidOperationException("Check/radio state not confirmed");
  }
  public static TreeItem[] Tree(long tree,int pid) {
    Own(tree,pid);if(Class(tree)!="SysTreeView32")throw new InvalidOperationException("Expected NSIS component tree");
    IntPtr process=OpenProcess(0x1038,false,(uint)pid),memory=IntPtr.Zero;
    if(process==IntPtr.Zero)throw new Win32Exception();
    try {
      bool wow;if(!Environment.Is64BitOperatingSystem || !IsWow64Process(process,out wow) || !wow)throw new InvalidOperationException("Only approved x86 setup on Windows x64 supported");
      memory=VirtualAllocEx(process,IntPtr.Zero,new UIntPtr(2048),0x3000,4);
      if(memory==IntPtr.Zero || memory.ToInt64()>UInt32.MaxValue)throw new InvalidOperationException("Cannot allocate bounded x86 tree query buffer");
      var result=new List<TreeItem>();var pending=new Stack<KeyValuePair<long,int>>();
      long first=Message(tree,pid,0x110a,0,0);if(first!=0)pending.Push(new KeyValuePair<long,int>(first,0));
      while(pending.Count>0) {
        var item=pending.Pop();if(result.Count>=64 || item.Value>8)throw new InvalidOperationException("Unknown component-tree size");
        var buffer=new byte[2048];Array.Copy(BitConverter.GetBytes((uint)0x19),0,buffer,0,4);
        Array.Copy(BitConverter.GetBytes(checked((uint)item.Key)),0,buffer,4,4);
        Array.Copy(BitConverter.GetBytes((uint)0xf000),0,buffer,12,4);
        Array.Copy(BitConverter.GetBytes(checked((uint)(memory.ToInt64()+64))),0,buffer,16,4);
        Array.Copy(BitConverter.GetBytes(512),0,buffer,20,4);
        UIntPtr count;if(!WriteProcessMemory(process,memory,buffer,new UIntPtr(2048),out count)||count.ToUInt64()!=2048)throw new Win32Exception();
        if(Message(tree,pid,0x113e,0,memory.ToInt64())==0)throw new InvalidOperationException("Cannot read component item");
        if(!ReadProcessMemory(process,memory,buffer,new UIntPtr(2048),out count)||count.ToUInt64()!=2048)throw new Win32Exception();
        string label=Encoding.Unicode.GetString(buffer,64,1024).Split('\0')[0];
        result.Add(new TreeItem{Handle=item.Key,Text=label,State=(BitConverter.ToInt32(buffer,8)&0xf000)>>12,Depth=item.Value});
        long sibling=Message(tree,pid,0x110a,1,item.Key),child=Message(tree,pid,0x110a,4,item.Key);
        if(sibling!=0)pending.Push(new KeyValuePair<long,int>(sibling,item.Value));
        if(child!=0)pending.Push(new KeyValuePair<long,int>(child,item.Value+1));
      }
      if(result.Count==0)throw new InvalidOperationException("Empty component tree");return result.ToArray();
    } finally {if(memory!=IntPtr.Zero)VirtualFreeEx(process,memory,UIntPtr.Zero,0x8000);CloseHandle(process);}
  }
  public static void ToggleTree(long tree,long item,int pid) {
    // NSIS's normal WM_CHAR Space handler changes SectionFlags and repaints the
    // tree; changing a state-image directly would not change installer actions.
    if(Class(tree)!="SysTreeView32")throw new InvalidOperationException("Expected component tree");
    if(Message(tree,pid,0x110b,9,item)==0)throw new InvalidOperationException("Cannot select component");
    if(Message(tree,pid,0x110a,9,0)!=item)throw new InvalidOperationException("Component selection mismatch");
    Message(tree,pid,0x102,32,0);Thread.Sleep(150);
  }
}

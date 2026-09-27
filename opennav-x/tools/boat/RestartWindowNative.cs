using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;

namespace OpenNavX {
  // Tooling only: exact installed modes, one source-reviewed UI command. There
  // is no public arbitrary message, key, caption, coordinate or command-ID API.
  public static class RestartWindowNative {
    [StructLayout(LayoutKind.Sequential)] public struct Rect {
      public int Left,Top,Right,Bottom;
      public int Width { get { return Right-Left; } }
      public int Height { get { return Bottom-Top; } }
    }
    [StructLayout(LayoutKind.Sequential)] private struct Point {public int X,Y;}
    [StructLayout(LayoutKind.Sequential)] private struct MonitorInfo {public uint Size;public Rect Monitor,Work;public uint Flags;}
    [StructLayout(LayoutKind.Sequential)] private struct MenuBarInfo {public uint Size;public Rect Bar;public IntPtr Menu,Window;public uint Focus;}
    public sealed class WindowInfo {public long Handle;public int ProcessId;public uint Dpi;public Rect Bounds;public string Mode;public int OwnedLegacyWindowCount;}
    public sealed class ModeCommand {public long Target;public uint MenuId;public string Caption,Method,FromMode,ToMode;}
    private delegate bool EnumCallback(IntPtr h,IntPtr p);
    [DllImport("user32.dll")] private static extern bool EnumWindows(EnumCallback callback,IntPtr parameter);
    [DllImport("user32.dll")] private static extern bool EnumChildWindows(IntPtr parent,EnumCallback callback,IntPtr parameter);
    [DllImport("user32.dll",CharSet=CharSet.Unicode)] private static extern int GetWindowTextW(IntPtr h,StringBuilder text,int length);
    [DllImport("user32.dll",CharSet=CharSet.Unicode)] private static extern int GetClassNameW(IntPtr h,StringBuilder text,int length);
    [DllImport("user32.dll")] private static extern uint GetWindowThreadProcessId(IntPtr h,out uint pid);
    [DllImport("user32.dll")] private static extern IntPtr GetForegroundWindow();
    [DllImport("user32.dll")] private static extern bool SetForegroundWindow(IntPtr h);
    [DllImport("user32.dll")] private static extern bool IsWindowVisible(IntPtr h);
    [DllImport("user32.dll")] private static extern bool IsWindowEnabled(IntPtr h);
    [DllImport("user32.dll")] private static extern bool IsIconic(IntPtr h);
    [DllImport("user32.dll")] private static extern bool IsZoomed(IntPtr h);
    [DllImport("user32.dll")] private static extern bool ShowWindow(IntPtr h,int command);
    [DllImport("user32.dll")] private static extern IntPtr GetParent(IntPtr h);
    [DllImport("user32.dll")] private static extern IntPtr GetWindow(IntPtr h,uint relation);
    [DllImport("user32.dll")] private static extern bool IsChild(IntPtr parent,IntPtr child);
    [DllImport("user32.dll")] private static extern bool GetWindowRect(IntPtr h,out Rect rect);
    [DllImport("user32.dll")] private static extern bool GetClientRect(IntPtr h,out Rect rect);
    [DllImport("user32.dll")] private static extern IntPtr WindowFromPoint(Point point);
    [DllImport("user32.dll")] private static extern IntPtr MonitorFromWindow(IntPtr h,uint flags);
    [DllImport("user32.dll",CharSet=CharSet.Unicode)] private static extern bool GetMonitorInfoW(IntPtr h,ref MonitorInfo info);
    [DllImport("user32.dll")] private static extern uint GetDpiForWindow(IntPtr h);
    [DllImport("user32.dll")] public static extern IntPtr SetThreadDpiAwarenessContext(IntPtr context);
    [DllImport("dwmapi.dll")] private static extern int DwmGetWindowAttribute(IntPtr h,uint attr,out Rect rect,uint bytes);
    [DllImport("user32.dll",SetLastError=true)] private static extern bool SetWindowPos(IntPtr h,IntPtr after,int x,int y,int width,int height,uint flags);
    [DllImport("user32.dll",SetLastError=true)] private static extern IntPtr SendMessageTimeoutW(IntPtr h,uint msg,UIntPtr w,IntPtr l,uint flags,uint timeout,out UIntPtr result);
    [DllImport("user32.dll")] private static extern IntPtr GetMenu(IntPtr h);
    [DllImport("user32.dll")] private static extern int GetMenuItemCount(IntPtr menu);
    [DllImport("user32.dll")] private static extern IntPtr GetSubMenu(IntPtr menu,int position);
    [DllImport("user32.dll")] private static extern uint GetMenuItemID(IntPtr menu,int position);
    [DllImport("user32.dll")] private static extern uint GetMenuState(IntPtr menu,uint item,uint flags);
    [DllImport("user32.dll",CharSet=CharSet.Unicode)] private static extern int GetMenuStringW(IntPtr menu,uint item,StringBuilder text,int length,uint flags);
    [DllImport("user32.dll")] private static extern bool GetMenuBarInfo(IntPtr h,int objectId,int itemId,ref MenuBarInfo info);
    private static uint Owner(IntPtr h) {uint pid;GetWindowThreadProcessId(h,out pid);return pid;}
    private static string Text(IntPtr h) {var s=new StringBuilder(2048);GetWindowTextW(h,s,s.Capacity);return s.ToString();}
    private static string Class(IntPtr h) {var s=new StringBuilder(128);GetClassNameW(h,s,s.Capacity);return s.ToString();}
    private static bool Contains(Rect a,Rect b) {return b.Width>0 && b.Height>0 && b.Left>=a.Left && b.Top>=a.Top && b.Right<=a.Right && b.Bottom<=a.Bottom;}
    private static bool Intersects(Rect a,Rect b) {return b.Right>a.Left && b.Left<a.Right && b.Bottom>a.Top && b.Top<a.Bottom;}
    private static Rect Bounds(IntPtr h) {Rect r;if(DwmGetWindowAttribute(h,9,out r,16)!=0 && !GetWindowRect(h,out r))throw new InvalidOperationException("Window bounds unavailable.");return r;}
    private static MonitorInfo Monitor(IntPtr h) {var m=new MonitorInfo();m.Size=(uint)Marshal.SizeOf(typeof(MonitorInfo));if(!GetMonitorInfoW(MonitorFromWindow(h,2),ref m))throw new InvalidOperationException("Monitor unavailable.");return m;}
    public static string Title(string mode) {
      switch(mode) {case "--xnav":return "OpenNav X / OpenCPN";case "--legacy":return "OpenCPN / Legacy";case "--safe-mode":return "OpenNav Safe Mode / OpenCPN";default:throw new InvalidOperationException("Unknown installed mode.");}
    }
    public static string Caption(string from,string to) {
      Title(from);Title(to);
      if(from!="--xnav") {if(to=="--xnav")return "Switch to XNav";throw new InvalidOperationException("Legacy/Safe exposes only its actual XNav return action.");}
      switch(to) {case "--xnav":return "Restart XNav";case "--legacy":return "Open Legacy OpenCPN";case "--safe-mode":return "Safe Mode";default:throw new InvalidOperationException("Unknown mode.");}
    }
    public static WindowInfo AssertFrame(IntPtr h,int pid,string mode) {
      if(h==IntPtr.Zero || Owner(h)!=(uint)pid || GetForegroundWindow()!=h || GetParent(h)!=IntPtr.Zero ||
          IsIconic(h) || !IsWindowVisible(h) || !IsWindowEnabled(h) || Text(h)!=Title(mode))
        throw new InvalidOperationException("Exact installed mode main frame must be foreground, enabled and unobscured by a modal.");
      var r=Bounds(h);var dpi=GetDpiForWindow(h);
      if(r.Width<100 || r.Height<100 || r.Width>7680 || r.Height>4320 || !Contains(Monitor(h).Monitor,r) || dpi<72 || dpi>384)
        throw new InvalidOperationException("Unexpected frame bounds/DPI.");
      return new WindowInfo{Handle=h.ToInt64(),ProcessId=pid,Dpi=dpi,Bounds=r,Mode=mode};
    }
    public static void Foreground(IntPtr h,int pid,string mode) {if(Owner(h)!=(uint)pid || Text(h)!=Title(mode) || IsIconic(h))throw new InvalidOperationException("Mode frame unavailable.");SetForegroundWindow(h);Thread.Sleep(250);AssertFrame(h,pid,mode);}
    private static bool OwnedLegacyWindow(IntPtr frame,int pid,IntPtr other,string mode) {
      // Floating plugin panes are part of the normal Legacy workspace. The
      // process/owner chain, not a caption, establishes application ownership.
      // Modal windows still fail AssertFrame's enabled/foreground checks.
      if(mode!="--legacy" || Owner(other)!=(uint)pid)return false;
      var seen=new HashSet<IntPtr>();
      for(int depth=0;depth<8;depth++) {
        if(!seen.Add(other))return false;
        other=GetWindow(other,4); // GW_OWNER; parent/child membership is not ownership.
        if(other==frame)return true;
        if(other==IntPtr.Zero || Owner(other)!=(uint)pid)return false;
      }
      return false;
    }
    public static void AssertCapture(IntPtr h,int pid,WindowInfo expected) {
      var current=AssertFrame(h,pid,expected.Mode);
      if(current.Bounds.Left!=expected.Bounds.Left || current.Bounds.Top!=expected.Bounds.Top || current.Bounds.Right!=expected.Bounds.Right || current.Bounds.Bottom!=expected.Bounds.Bottom || current.Dpi!=expected.Dpi)throw new InvalidOperationException("Capture geometry changed.");
      bool obscured=false;int owned=0;Exception failure=null;
      EnumWindows(delegate(IntPtr other,IntPtr p){try{if(other==h)return false;if(IsWindowVisible(other) && !IsIconic(other) && Intersects(expected.Bounds,Bounds(other))) {
        if(OwnedLegacyWindow(h,pid,other,expected.Mode))owned++;else obscured=true;
      }return true;}catch(Exception e){failure=e;return false;}},IntPtr.Zero);
      if(failure!=null || obscured)throw new InvalidOperationException("Another window obscures the capture.",failure);
      expected.OwnedLegacyWindowCount=owned;
    }
    private static void AssertMenuUnobscured(IntPtr frame,Rect bar) {
      bool obscured=false;Exception failure=null;
      EnumWindows(delegate(IntPtr other,IntPtr p){try{if(other==frame)return false;
        if(IsWindowVisible(other) && !IsIconic(other) && Intersects(bar,Bounds(other)))obscured=true;
        return true;}catch(Exception e){failure=e;return false;}},IntPtr.Zero);
      if(failure!=null || obscured)throw new InvalidOperationException("The actual mode menu is obscured; no command sent.",failure);
    }
    public static void Resize1280x800(IntPtr h,int pid,string mode) {
      AssertFrame(h,pid,mode);var m=Monitor(h);if(m.Work.Width<1280 || m.Work.Height<800)throw new InvalidOperationException("Physical 1280x800 does not fit current work area; display is not changed.");
      if(IsZoomed(h)){ShowWindow(h,9);Thread.Sleep(150);AssertFrame(h,pid,mode);}
      Rect outer;if(!GetWindowRect(h,out outer))throw new InvalidOperationException("Outer frame bounds unavailable.");var visible=Bounds(h);
      if(!SetWindowPos(h,IntPtr.Zero,m.Work.Left-(visible.Left-outer.Left),m.Work.Top-(visible.Top-outer.Top),1280+outer.Width-visible.Width,800+outer.Height-visible.Height,0x14))throw new Win32Exception(Marshal.GetLastWin32Error());
      Thread.Sleep(300);var after=AssertFrame(h,pid,mode);if(after.Bounds.Width!=1280 || after.Bounds.Height!=800)throw new InvalidOperationException("Resize not accepted; no retry.");
    }
    private static void FindMenu(IntPtr menu,string caption,List<uint> ids,int depth) {
      int count=GetMenuItemCount(menu);if(depth>8 || count<0 || count>256)throw new InvalidOperationException("Unexpected menu structure.");
      for(int i=0;i<count;i++) {
        uint state=GetMenuState(menu,(uint)i,0x400);if(state==UInt32.MaxValue || (state&3)!=0)continue;
        var sub=GetSubMenu(menu,i);if(sub!=IntPtr.Zero){FindMenu(sub,caption,ids,depth+1);continue;}
        var text=new StringBuilder(512);GetMenuStringW(menu,(uint)i,text,text.Capacity,0x400);
        if(text.ToString()==caption){uint id=GetMenuItemID(menu,i);if(id==0 || id==UInt32.MaxValue || id>65535)throw new InvalidOperationException("Invalid live menu command.");ids.Add(id);}
      }
    }
    public static ModeCommand InspectModeCommand(IntPtr frame,int pid,string from,string to) {
      var root=AssertFrame(frame,pid,from);string caption=Caption(from,to);
      AssertCapture(frame,pid,root);
      if(from!="--xnav") {
        var menu=GetMenu(frame);var bar=new MenuBarInfo();bar.Size=(uint)Marshal.SizeOf(typeof(MenuBarInfo));
        if(menu==IntPtr.Zero || !GetMenuBarInfo(frame,-3,0,ref bar) || bar.Menu!=menu || !Contains(root.Bounds,bar.Bar))throw new InvalidOperationException("Visible native mode menu required; no hidden command invocation.");
        AssertMenuUnobscured(frame,bar.Bar);
        var ids=new List<uint>();FindMenu(menu,caption,ids,0);if(ids.Count!=1)throw new InvalidOperationException("Unique enabled source-reviewed mode menu required.");
        return new ModeCommand{Target=frame.ToInt64(),MenuId=ids[0],Caption=caption,FromMode=from,ToMode=to,Method="Exact current native menu caption resolved to its own command"};
      }
      var matches=new List<IntPtr>();var systems=new List<IntPtr>();Exception failure=null;
      EnumChildWindows(frame,delegate(IntPtr h,IntPtr p){try {
        if(!IsWindowVisible(h) || Owner(h)!=(uint)pid)return true;
        if(Text(h)=="OpenNav product page: System")systems.Add(h);
        if(Text(h)!=caption || Class(h)=="Static" || !IsWindowEnabled(h))return true;
        Rect r;if(!GetWindowRect(h,out r) || !Contains(root.Bounds,r))return true;
        for(var parent=GetParent(h);parent!=IntPtr.Zero && parent!=frame;parent=GetParent(parent)){Rect pr;if(!GetWindowRect(parent,out pr) || !Contains(pr,r))return true;}
        matches.Add(h);return true;
      }catch(Exception e){failure=e;return false;}},IntPtr.Zero);
      if(failure!=null || systems.Count!=1 || matches.Count!=1 || !IsChild(systems[0],matches[0]))throw new InvalidOperationException("Visible System page and its unique enabled mode button required.",failure);
      return new ModeCommand{Target=matches[0].ToInt64(),Caption=caption,FromMode=from,ToMode=to,Method="One target-local reviewed System button press/release"};
    }
    public static void RequestMode(IntPtr frame,int pid,ModeCommand expected) {
      if(expected==null)throw new InvalidOperationException("Inspected command required.");
      var actual=InspectModeCommand(frame,pid,expected.FromMode,expected.ToMode);
      if(actual.Target!=expected.Target || actual.MenuId!=expected.MenuId || actual.Caption!=expected.Caption || actual.Method!=expected.Method)throw new InvalidOperationException("Mode control changed before one action.");
      UIntPtr result;
      if(actual.MenuId!=0) {
        if(SendMessageTimeoutW(frame,0x111,new UIntPtr(actual.MenuId),IntPtr.Zero,0x2,1000,out result)==IntPtr.Zero)throw new InvalidOperationException("Mode menu result uncertain; never retry.");
      } else {
        var button=new IntPtr(actual.Target);Rect screen,client;
        if(!GetWindowRect(button,out screen) || !GetClientRect(button,out client) || client.Width<48 || client.Height<48)throw new InvalidOperationException("Mode button geometry unavailable.");
        var hit=WindowFromPoint(new Point{X=(screen.Left+screen.Right)/2,Y=(screen.Top+screen.Bottom)/2});if(hit!=button && !IsChild(button,hit))throw new InvalidOperationException("Mode button is obscured.");
        AssertFrame(frame,pid,actual.FromMode);var point=new IntPtr((client.Width/2)|((client.Height/2)<<16));
        var down=SendMessageTimeoutW(button,0x201,new UIntPtr(1),point,0x2,1000,out result);
        if(down==IntPtr.Zero)throw new InvalidOperationException("Mode press result uncertain; no release/retry is sent to an unverified window.");
        var held=InspectModeCommand(frame,pid,actual.FromMode,actual.ToMode);Rect heldScreen,heldClient;
        if(held.Target!=actual.Target || held.MenuId!=0 || held.Caption!=actual.Caption || !GetWindowRect(button,out heldScreen) || !GetClientRect(button,out heldClient) ||
           heldScreen.Left!=screen.Left || heldScreen.Top!=screen.Top || heldScreen.Right!=screen.Right || heldScreen.Bottom!=screen.Bottom ||
           heldClient.Width!=client.Width || heldClient.Height!=client.Height)throw new InvalidOperationException("Mode button changed during press; consumed action is uncertain.");
        var up=SendMessageTimeoutW(button,0x202,UIntPtr.Zero,point,0x2,1000,out result);
        if(up==IntPtr.Zero)throw new InvalidOperationException("Mode press result uncertain; never retry.");
      }
      // The parent can now close. Only broker completion/receipt may establish
      // child success; a successful UI send is deliberately not a launch proof.
    }
  }
}

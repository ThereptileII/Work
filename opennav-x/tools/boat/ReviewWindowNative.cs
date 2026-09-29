using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Runtime.InteropServices;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading;

namespace OpenNavX {
  // Review tooling only. No arbitrary key, mouse coordinate or window-message
  // entry point: callers select a single source-reviewed display action.
  public static class ReviewWindowNative {
    [StructLayout(LayoutKind.Sequential)] public struct Rect {
      public int Left,Top,Right,Bottom;
      public int Width { get { return Right-Left; } }
      public int Height { get { return Bottom-Top; } }
    }
    [StructLayout(LayoutKind.Sequential)] private struct Point { public int X,Y; }
    [StructLayout(LayoutKind.Sequential)] private struct MonitorInfo { public uint Size;public Rect Monitor,Work;public uint Flags; }
    [StructLayout(LayoutKind.Sequential)] private struct GuiInfo {
      public uint Size,Flags;public IntPtr Active,Focus,Capture,MenuOwner,MoveSize,Caret;public Rect CaretBounds;
    }
    public sealed class WindowInfo {
      public long Handle;public int ProcessId;public uint Dpi;public Rect Bounds;public bool Maximized;
      public string Shell;public SurfaceInfo[] Surfaces;
    }
    public sealed class SurfaceInfo {
      public long Handle;public string Title,Signature;public uint Dpi;public Rect Bounds;
    }
    public sealed class ResizeInfo {
      public WindowInfo Before,Restored,After;public Rect MonitorBounds,WorkArea;public bool RestoreRequested;
    }
    public sealed class SelectionRow {
      public long Handle;public string Label;public int Top,Left;
      public bool Enabled,Visible,DirectChild;
    }
    private delegate bool EnumCallback(IntPtr window,IntPtr parameter);
    [DllImport("user32.dll")] private static extern bool EnumChildWindows(IntPtr parent,EnumCallback callback,IntPtr parameter);
    [DllImport("user32.dll")] private static extern bool EnumWindows(EnumCallback callback,IntPtr parameter);
    [DllImport("user32.dll",CharSet=CharSet.Unicode)] private static extern int GetWindowTextW(IntPtr window,StringBuilder text,int maximum);
    [DllImport("user32.dll",CharSet=CharSet.Unicode)] private static extern int GetClassNameW(IntPtr window,StringBuilder text,int maximum);
    [DllImport("user32.dll")] private static extern uint GetWindowThreadProcessId(IntPtr window,out uint process);
    [DllImport("user32.dll")] private static extern IntPtr GetForegroundWindow();
    [DllImport("user32.dll")] private static extern bool SetForegroundWindow(IntPtr window);
    [DllImport("user32.dll")] private static extern bool IsWindowVisible(IntPtr window);
    [DllImport("user32.dll")] private static extern bool IsWindowEnabled(IntPtr window);
    [DllImport("user32.dll")] private static extern bool IsIconic(IntPtr window);
    [DllImport("user32.dll")] private static extern bool IsZoomed(IntPtr window);
    [DllImport("user32.dll")] private static extern bool ShowWindow(IntPtr window,int command);
    [DllImport("user32.dll")] private static extern bool GetWindowRect(IntPtr window,out Rect rect);
    [DllImport("user32.dll")] private static extern bool GetClientRect(IntPtr window,out Rect rect);
    [DllImport("user32.dll")] private static extern IntPtr WindowFromPoint(Point point);
    [DllImport("user32.dll")] private static extern bool IsChild(IntPtr parent,IntPtr child);
    [DllImport("user32.dll")] private static extern IntPtr GetParent(IntPtr child);
    [DllImport("user32.dll")] private static extern IntPtr GetWindow(IntPtr window,uint command);
    [DllImport("user32.dll")] private static extern bool ClientToScreen(IntPtr window,ref Point point);
    [DllImport("user32.dll")] private static extern IntPtr MonitorFromWindow(IntPtr window,uint flags);
    [DllImport("user32.dll",CharSet=CharSet.Unicode)] private static extern bool GetMonitorInfoW(IntPtr monitor,ref MonitorInfo info);
    [DllImport("user32.dll",SetLastError=true)] private static extern bool SetWindowPos(IntPtr window,IntPtr after,int x,int y,int width,int height,uint flags);
    [DllImport("user32.dll")] public static extern IntPtr SetThreadDpiAwarenessContext(IntPtr value);
    [DllImport("user32.dll")] private static extern uint GetDpiForWindow(IntPtr window);
    [DllImport("dwmapi.dll")] private static extern int DwmGetWindowAttribute(IntPtr window,uint attribute,out Rect rect,uint bytes);
    [DllImport("user32.dll")] private static extern bool GetGUIThreadInfo(uint thread,ref GuiInfo info);
    [DllImport("user32.dll")] private static extern short GetAsyncKeyState(int key);
    [DllImport("user32.dll",SetLastError=true)] private static extern IntPtr SendMessageTimeoutW(IntPtr window,uint message,UIntPtr wparam,IntPtr lparam,uint flags,uint timeout,out UIntPtr result);

    private static string Text(IntPtr h) { var text=new StringBuilder(2048);GetWindowTextW(h,text,text.Capacity);return text.ToString(); }
    private static string Class(IntPtr h) { var text=new StringBuilder(128);GetClassNameW(h,text,text.Capacity);return text.ToString(); }
    private static uint Owner(IntPtr h) { uint pid;GetWindowThreadProcessId(h,out pid);return pid; }
    private static Rect Bounds(IntPtr h) {
      Rect rect;
      if(DwmGetWindowAttribute(h,9,out rect,16)!=0 && !GetWindowRect(h,out rect))throw new InvalidOperationException("Window bounds unavailable.");
      return rect;
    }
    private static bool Contains(Rect a,Rect b) {return b.Left>=a.Left && b.Top>=a.Top && b.Right<=a.Right && b.Bottom<=a.Bottom;}
    private static bool Intersects(Rect a,Rect b) {return b.Right>a.Left && b.Left<a.Right && b.Bottom>a.Top && b.Top<a.Bottom;}
    private static MonitorInfo Monitor(IntPtr h) {
      var info=new MonitorInfo();info.Size=(uint)Marshal.SizeOf(typeof(MonitorInfo));
      if(!GetMonitorInfoW(MonitorFromWindow(h,2),ref info))throw new InvalidOperationException("Monitor unavailable.");
      return info;
    }
    private static List<IntPtr> Children(IntPtr frame) {
      var result=new List<IntPtr>();
      EnumChildWindows(frame,delegate(IntPtr h,IntPtr p){if(IsWindowVisible(h))result.Add(h);return true;},IntPtr.Zero);
      return result;
    }
    public static bool IsPrototypeNavigation(string[] labels) {
      if(labels==null || (labels.Length!=8 && labels.Length!=9))return false;
      var expected=new HashSet<string>(new string[]{"Chart","Passage","Traffic","Energy","Instruments","Anchor","Radar","Settings"},StringComparer.Ordinal);
      if(labels.Length==9)expected.Add("Vessel profile");
      foreach(var label in labels)if(!expected.Remove(label))return false;
      return expected.Count==0;
    }
    private static bool SameLabels(string[] actual,params string[] expected) {
      if(actual==null || actual.Length!=expected.Length)return false;
      var remaining=new HashSet<string>(expected,StringComparer.Ordinal);
      foreach(var label in actual)if(!remaining.Remove(label))return false;
      return remaining.Count==0;
    }
    // Signatures authorize capture only. They never authorize clicking a sheet
    // control (in particular key storage, route changes or hardware commands).
    public static bool IsPrototypeSurface(string title,string[] directLabels,string[] headingLabels) {
      switch(title) {
        case "OpenNav chart tools":return SameLabels(directLabels,"Measure","Waypoint","+","\u2212");
        case "OpenNav chart orientation":return SameLabels(directLabels,"North") || SameLabels(directLabels,"Course");
        case "OpenNav follow boat":return SameLabels(directLabels,"Follow boat");
        case "OpenNav preferences":return SameLabels(directLabels) && SameLabels(headingLabels,"Close");
        case "OpenNav passage":return SameLabels(directLabels) && SameLabels(headingLabels,"Close");
        case "OpenNav vessel traffic":return SameLabels(directLabels) && (SameLabels(headingLabels,"Close") || SameLabels(headingLabels,"Back"));
        default:return false;
      }
    }
    private static string[] DirectLabels(IntPtr parent,int pid) {
      var labels=new List<string>();
      foreach(var h in Children(parent))if(GetParent(h)==parent && Owner(h)==(uint)pid && Class(h)!="Static" && Text(h).Length>0)labels.Add(Text(h));
      return labels.ToArray();
    }
    private static IntPtr PrototypeNavigation(IntPtr frame,int pid) {
      var result=IntPtr.Zero;
      foreach(var h in Children(frame))if(GetParent(h)==frame && Owner(h)==(uint)pid && IsPrototypeNavigation(DirectLabels(h,pid))) {
        if(result!=IntPtr.Zero)throw new InvalidOperationException("Ambiguous prototype navigation rail.");
        result=h;
      }
      return result;
    }
    private static SurfaceInfo[] PrototypeSurfaces(IntPtr frame,int pid,uint dpi) {
      Rect client;var origin=new Point();
      if(!GetClientRect(frame,out client) || !ClientToScreen(frame,ref origin))throw new InvalidOperationException("Frame client geometry unavailable.");
      client.Left+=origin.X;client.Right+=origin.X;client.Top+=origin.Y;client.Bottom+=origin.Y;
      var result=new List<SurfaceInfo>();var names=new HashSet<string>(StringComparer.Ordinal);Exception failure=null;int drawers=0;
      EnumWindows(delegate(IntPtr h,IntPtr ignored) {
        try {
          if(!IsWindowVisible(h) || IsIconic(h) || GetWindow(h,4)!=frame || Owner(h)!=(uint)pid || IsChild(frame,h))return true;
          var title=Text(h);
          if(title!="OpenNav chart tools" && title!="OpenNav chart orientation" && title!="OpenNav follow boat" && title!="OpenNav passage" && title!="OpenNav vessel traffic" && title!="OpenNav preferences")return true;
          var direct=DirectLabels(h,pid);var heading=new List<string>();
          foreach(var child in Children(h))if(GetParent(child)==h && Owner(child)==(uint)pid)
            foreach(var label in DirectLabels(child,pid))if(label=="Close" || label=="Back")heading.Add(label);
          var rect=Bounds(h);
          // Bounded reasons contain only allowlisted surface titles and booleans,
          // never arbitrary child text (which could be a typed credential).
          bool unique=names.Add(title),enabled=IsWindowEnabled(h),sameDpi=GetDpiForWindow(h)==dpi,
               size=rect.Width>=24 && rect.Height>=24,contained=Contains(client,rect),
               signature=IsPrototypeSurface(title,direct,heading.ToArray());
          if(!unique || !enabled || !sameDpi || !size || !contained || !signature)
            throw new InvalidOperationException(String.Format("Owned prototype surface {0} refused: unique={1}, enabled={2}, dpi={3}, size={4}, contained={5}, signature={6}.",title,unique,enabled,sameDpi,size,contained,signature));
          if((title=="OpenNav passage" || title=="OpenNav vessel traffic" || title=="OpenNav preferences") && ++drawers>1)throw new InvalidOperationException("More than one prototype sheet is visible.");
          Array.Sort(direct,StringComparer.Ordinal);heading.Sort(StringComparer.Ordinal);
          result.Add(new SurfaceInfo{Handle=h.ToInt64(),Title=title,Signature=String.Join("|",direct)+"/"+String.Join("|",heading.ToArray()),Dpi=dpi,Bounds=rect});
          return true;
        } catch(Exception error){failure=error;return false;}
      },IntPtr.Zero);
      if(failure!=null)throw new InvalidOperationException("Prototype owned-window verification failed.",failure);
      result.Sort(delegate(SurfaceInfo a,SurfaceInfo b){return a.Handle.CompareTo(b.Handle);});
      return result.ToArray();
    }
    public static void Foreground(IntPtr frame,int pid) {
      if(Owner(frame)!=(uint)pid || IsIconic(frame) || !IsWindowVisible(frame))throw new InvalidOperationException("Reviewed frame unavailable or minimized.");
      SetForegroundWindow(frame);Thread.Sleep(250);AssertFrame(frame,pid);
    }
    private static WindowInfo FrameIdentity(IntPtr frame,int pid,bool requireForeground) {
      if(frame==IntPtr.Zero || Owner(frame)!=(uint)pid || GetParent(frame)!=IntPtr.Zero ||
         IsIconic(frame) || !IsWindowVisible(frame) || !IsWindowEnabled(frame))throw new InvalidOperationException("Exact reviewed XNav frame must be foreground and enabled; dismiss other windows manually.");
      int menu=0,navigation=0;
      foreach(var child in Children(frame)) {
        var text=Text(child);
        if(text=="Demo" || text.StartsWith("DEMO /",StringComparison.Ordinal))throw new InvalidOperationException("Synthetic interface refused.");
        if(text=="Menu" && Class(child)!="Static")menu++;
        if(text=="Navigation" && Class(child)!="Static")navigation++;
      }
      bool prototype=PrototypeNavigation(frame,pid)!=IntPtr.Zero;
      if((prototype && (menu!=0 || navigation!=0)) || (!prototype && (menu!=1 || navigation!=1)))throw new InvalidOperationException("Normal installed XNav shell was not uniquely identified.");
      var rect=Bounds(frame);
      if(rect.Width<100 || rect.Height<100 || rect.Width>7680 || rect.Height>4320)throw new InvalidOperationException("Reviewed frame dimensions are outside bounded display geometry.");
      var dpi=GetDpiForWindow(frame);
      if(dpi<72 || dpi>384)throw new InvalidOperationException("Unexpected application DPI.");
      var surfaces=prototype?PrototypeSurfaces(frame,pid,dpi):new SurfaceInfo[0];
      var foreground=GetForegroundWindow();bool knownFocus=foreground==frame;
      foreach(var surface in surfaces)if(foreground.ToInt64()==surface.Handle)knownFocus=true;
      if(requireForeground && !knownFocus)throw new InvalidOperationException("Exact reviewed frame or verified owned prototype surface must be foreground.");
      return new WindowInfo {Handle=frame.ToInt64(),ProcessId=pid,Dpi=dpi,Bounds=rect,Maximized=IsZoomed(frame),Shell=prototype?"prototype":"legacy-xnav",Surfaces=surfaces};
    }
    public static WindowInfo AssertFrame(IntPtr frame,int pid) {
      var info=FrameIdentity(frame,pid,true);
      if(!Contains(Monitor(frame).Monitor,info.Bounds))throw new InvalidOperationException("Complete reviewed frame must fit on its current monitor.");
      return info;
    }
    public static void AssertCapture(IntPtr frame,int pid,WindowInfo expected) {
      var current=AssertFrame(frame,pid);
      if(expected==null || current.Handle!=expected.Handle || current.ProcessId!=expected.ProcessId || current.Shell!=expected.Shell ||
         current.Bounds.Left!=expected.Bounds.Left || current.Bounds.Top!=expected.Bounds.Top ||
         current.Bounds.Right!=expected.Bounds.Right || current.Bounds.Bottom!=expected.Bounds.Bottom || current.Dpi!=expected.Dpi)
        throw new InvalidOperationException("Reviewed window moved or changed DPI during capture.");
      if(expected.Surfaces==null || current.Surfaces.Length!=expected.Surfaces.Length)throw new InvalidOperationException("Owned prototype surfaces changed during capture.");
      for(int i=0;i<current.Surfaces.Length;i++) {
        var a=current.Surfaces[i];var b=expected.Surfaces[i];
        if(b==null || a.Handle!=b.Handle || a.Title!=b.Title || a.Signature!=b.Signature || a.Dpi!=b.Dpi || !SameRect(a.Bounds,b.Bounds))
          throw new InvalidOperationException("Owned prototype surface changed during capture.");
      }
      bool obscured=false;Exception failure=null;
      // Never let an exception escape a reverse-P/Invoke enumeration callback.
      // Any inaccessible/disappearing overlaid window invalidates the capture.
      EnumWindows(delegate(IntPtr h,IntPtr p){
        try {
          if(h==frame)return false;
          if(IsWindowVisible(h) && !IsIconic(h) && Intersects(expected.Bounds,Bounds(h))) {
            bool known=false;foreach(var surface in current.Surfaces)if(surface.Handle==h.ToInt64())known=true;
            if(!known)obscured=true;
          }
          return true;
        } catch(Exception error) {failure=error;return false;}
      },IntPtr.Zero);
      if(failure!=null)throw new InvalidOperationException("Window changed during capture visibility check.",failure);
      if(obscured)throw new InvalidOperationException("Another visible window obscures the reviewed frame; no screenshot published.");
    }
    private static bool SameRect(Rect a,Rect b) {return a.Left==b.Left && a.Top==b.Top && a.Right==b.Right && a.Bottom==b.Bottom;}
    private static void ValidateResizeWorkArea(Rect work) {
      if(work.Width<1280 || work.Height<800)throw new InvalidOperationException("1280x800 does not fit the current monitor work area. Display/DPI and taskbar are not changed.");
    }
    private static void AssertResizeMonitor(IntPtr handle,MonitorInfo expected) {
      var current=new MonitorInfo();current.Size=(uint)Marshal.SizeOf(typeof(MonitorInfo));
      if(!GetMonitorInfoW(handle,ref current) || !SameRect(current.Monitor,expected.Monitor) || !SameRect(current.Work,expected.Work))
        throw new InvalidOperationException("Monitor or work area changed during the fixed resize.");
    }
    private static string Geometry(WindowInfo value) {
      if(value==null)return "unavailable";
      var r=value.Bounds;
      return String.Format(System.Globalization.CultureInfo.InvariantCulture,"{0},{1},{2},{3};dpi={4};max={5}",r.Left,r.Top,r.Right,r.Bottom,value.Dpi,value.Maximized);
    }
    private static string CurrentGeometry(IntPtr frame,int pid) {
      try {
        if(frame==IntPtr.Zero || Owner(frame)!=(uint)pid || GetParent(frame)!=IntPtr.Zero)return "identity-unavailable";
        return Geometry(new WindowInfo {Bounds=Bounds(frame),Dpi=GetDpiForWindow(frame),Maximized=IsZoomed(frame)});
      } catch {return "unavailable";}
    }
    public static ResizeInfo Resize1280x800(IntPtr frame,int pid) {
      var result=new ResizeInfo();string stage="identity";
      try {
        // This one fixed operation may recover an offscreen normal rectangle.
        // Capture/Close/ordinary Foreground retain their full-containment guard.
        result.Before=FrameIdentity(frame,pid,false);
        var monitorHandle=MonitorFromWindow(frame,2);var monitor=Monitor(frame);
        result.MonitorBounds=monitor.Monitor;result.WorkArea=monitor.Work;
        stage="work-area";ValidateResizeWorkArea(monitor.Work);
        if(!Intersects(monitor.Monitor,result.Before.Bounds))throw new InvalidOperationException("Reviewed frame does not intersect its selected monitor.");
        stage="foreground";SetForegroundWindow(frame);Thread.Sleep(250);FrameIdentity(frame,pid,true);
        AssertResizeMonitor(monitorHandle,monitor);
        result.RestoreRequested=IsZoomed(frame);
        if(result.RestoreRequested){stage="restore";ShowWindow(frame,9);Thread.Sleep(150);}
        result.Restored=FrameIdentity(frame,pid,true);
        if(result.Restored.Maximized)throw new InvalidOperationException("Reviewed frame did not restore; no resize sent.");
        stage="fixed-placement";AssertResizeMonitor(monitorHandle,monitor);
        Rect outer;if(!GetWindowRect(frame,out outer))throw new InvalidOperationException("Outer frame bounds unavailable.");
        var visible=result.Restored.Bounds;
        // Only DWM's bounded invisible resize border may expand the outer rect.
        int left=visible.Left-outer.Left,top=visible.Top-outer.Top;
        int right=outer.Right-visible.Right,bottom=outer.Bottom-visible.Bottom;
        if(left<0 || top<0 || right<0 || bottom<0 || left>128 || top>128 || right>128 || bottom>128)
          throw new InvalidOperationException("Unexpected outer/visible frame border; no resize sent.");
        var stable=FrameIdentity(frame,pid,true);
        if(!SameRect(stable.Bounds,result.Restored.Bounds) || stable.Dpi!=result.Restored.Dpi)
          throw new InvalidOperationException("Restored frame geometry changed before fixed placement.");
        if(!SetWindowPos(frame,IntPtr.Zero,monitor.Work.Left-left,monitor.Work.Top-top,1280+left+right,800+top+bottom,0x14))
          throw new Win32Exception(Marshal.GetLastWin32Error(),"Window resize failed.");
        stage="verify";Thread.Sleep(300);AssertResizeMonitor(monitorHandle,monitor);result.After=AssertFrame(frame,pid);
        if(result.After.Maximized || result.After.Bounds.Width!=1280 || result.After.Bounds.Height!=800 || !Contains(monitor.Work,result.After.Bounds))
          throw new InvalidOperationException("Application did not accept exactly 1280x800 physical pixels in the pinned work area; no resize retry.");
        return result;
      } catch(Exception error) {
        throw new InvalidOperationException("Fixed XNav resize refused at "+stage+"; before="+Geometry(result.Before)+"; restored="+Geometry(result.Restored)+"; after="+Geometry(result.After)+"; current="+CurrentGeometry(frame,pid)+". "+error.Message,error);
      }
    }
    public static void Escape(IntPtr frame,int pid) {
      AssertFrame(frame,pid);uint owner;var thread=GetWindowThreadProcessId(frame,out owner);
      var info=new GuiInfo();info.Size=(uint)Marshal.SizeOf(typeof(GuiInfo));
      if(!GetGUIThreadInfo(thread,ref info) || info.Focus==IntPtr.Zero || Owner(info.Focus)!=(uint)pid ||
         (info.Focus!=frame && !IsChild(frame,info.Focus)))throw new InvalidOperationException("Reviewed focus unavailable.");
      UIntPtr result;
      // Escape goes to this process's current focused HWND only. No global
      // SendInput, modifier keys, Enter, accelerators or arbitrary messages.
      var down=SendMessageTimeoutW(info.Focus,0x100,new UIntPtr(0x1b),new IntPtr(1),0x2,1000,out result);
      var up=SendMessageTimeoutW(info.Focus,0x101,new UIntPtr(0x1b),new IntPtr(unchecked((int)0xc0000001)),0x2,1000,out result);
      if(down==IntPtr.Zero || up==IntPtr.Zero)throw new InvalidOperationException("Escape did not respond; no retry.");
      Thread.Sleep(300);AssertFrame(frame,pid);
    }
    public static void PanRight(IntPtr frame,int pid,Rect chart) {
      var root=AssertFrame(frame,pid);
      if(VisiblePageLabels(frame).Length!=0 || !Contains(root.Bounds,chart) ||
         chart.Width<root.Bounds.Width/2 || chart.Height<root.Bounds.Height/2)
        throw new InvalidOperationException("Chart pan requires the unobscured Navigation canvas.");
      // No held modifiers or pointer drag may turn this fixed arrow into a
      // chart-stack shortcut, route edit or continuation of another gesture.
      foreach(int key in new int[]{1,2,4,5,6,0x10,0x11,0x12,0x25,0x26,0x27,0x28,0x5b,0x5c})
        if((GetAsyncKeyState(key)&0x8000)!=0)throw new InvalidOperationException("Release all mouse buttons and modifiers before chart pan.");
      uint owner;var thread=GetWindowThreadProcessId(frame,out owner);
      var gui=new GuiInfo();gui.Size=(uint)Marshal.SizeOf(typeof(GuiInfo));
      if(!GetGUIThreadInfo(thread,ref gui) || gui.Capture!=IntPtr.Zero || gui.MenuOwner!=IntPtr.Zero || gui.MoveSize!=IntPtr.Zero)
        throw new InvalidOperationException("Another native gesture or menu is active.");
      var point=new Point{X=(chart.Left+chart.Right)/2,Y=(chart.Top+chart.Bottom)/2};
      var candidates=new List<IntPtr>();
      foreach(var h in Children(frame)) {
        Rect observed;
        if(Owner(h)==(uint)pid && GetParent(h)==frame && IsWindowEnabled(h) && GetWindowRect(h,out observed) &&
           observed.Left==chart.Left && observed.Top==chart.Top && observed.Right==chart.Right && observed.Bottom==chart.Bottom)candidates.Add(h);
      }
      if(candidates.Count!=1)
        throw new InvalidOperationException("Fresh chart geometry does not identify the exact native canvas.");
      var canvas=candidates[0];var hit=WindowFromPoint(point);
      // OpenGL owns a child surface inside the same source-identified canvas.
      if(Owner(hit)!=(uint)pid || (hit!=canvas && !IsChild(canvas,hit)))
        throw new InvalidOperationException("Another window obscures the chart pan target.");
      var kind=Class(canvas);var caption=Text(canvas);UIntPtr result;
      AssertFrame(frame,pid);
      // Pinned ChartCanvas::OnKeyDown/OnKeyUp: an unmodified Right arrow pans
      // the existing viewport and releases timed movement. No global input,
      // chart clicks, route activation, arbitrary key or command ID is exposed.
      if(SendMessageTimeoutW(canvas,0x100,new UIntPtr(0x27),new IntPtr(0x014d0001),0x2,1000,out result)==IntPtr.Zero)
        throw new InvalidOperationException("Chart pan press uncertain; no retry.");
      Thread.Sleep(150);
      if(Owner(canvas)!=(uint)pid || GetParent(canvas)!=frame || Class(canvas)!=kind || Text(canvas)!=caption)
        throw new InvalidOperationException("Chart identity changed during pan; no message to a replacement window.");
      // Release this exact target even if foreground changes while held.
      if(SendMessageTimeoutW(canvas,0x101,new UIntPtr(0x27),new IntPtr(unchecked((int)0xc14d0001)),0x2,1000,out result)==IntPtr.Zero)
        throw new InvalidOperationException("Chart pan release uncertain; no retry.");
      Thread.Sleep(300);AssertFrame(frame,pid);
    }
    public static string[] ActionLabels(string action) {
      switch(action) {
        case "Menu":return new string[]{"Menu"};case "Navigation":return new string[]{"Navigation"};
        case "Routes":return new string[]{"Routes"};case "Waypoints":return new string[]{"Waypoints"};
        case "AIS":return new string[]{"AIS targets"};case "Instruments":return new string[]{"Vessel instruments"};
        case "Advice":return new string[]{"SmartNav advisories"};case "PilotView":return new string[]{"Autopilot"};
        case "Anchor":return new string[]{"Anchor watch"};case "Settings":return new string[]{"Settings"};
        case "Sources":return new string[]{"SENSORS"};case "Route":return new string[]{"Route"};
        case "Display":return new string[]{"DISPLAY"};case "ToggleFullscreen":return new string[]{"Fullscreen / window"};
        case "ToggleOrientation":return new string[]{"North","Course"};
        case "Energy":return new string[]{"Energy"};case "Diagnostics":return new string[]{"Diagnostics"};
        case "System":return new string[]{"System"};case "Alerts":return new string[]{"Alerts"};
        case "CyclePalette":return new string[]{"Day","Dusk","Night"};
        case "ZoomIn":return new string[]{"+"};case "ZoomOut":return new string[]{"\u2212"};
        case "Center":return new string[]{"Follow boat"};case "PageUp":return new string[]{"Up"};case "PageDown":return new string[]{"Down"};
        default:throw new InvalidOperationException("Unsupported review pointer action.");
      }
    }
    public static string ActionContext(string action) {
      ActionLabels(action); // Unknown actions have no context, even without a window.
      switch(action) {
        case "Display":return "OpenNav product page: Settings";
        case "ToggleFullscreen":return "OpenNav product page: Display";
        case "ToggleOrientation":return "Navigation chart tools";
        case "CyclePalette":return "Navigation status bar";
        default:return "Installed XNav shell";
      }
    }
    private static bool ScopedButton(IntPtr frame,int pid,IntPtr button,string action) {
      if(action=="CyclePalette") {
        var parent=GetParent(button);int menus=0;
        if(parent==IntPtr.Zero || GetParent(parent)!=frame)return false;
        var sibling=PrototypeNavigation(frame,pid)==IntPtr.Zero?"Menu":"Alerts";
        foreach(var h in Children(parent))if(GetParent(h)==parent && Owner(h)==(uint)pid && Class(h)!="Static" && Text(h)==sibling)menus++;
        return menus==1;
      }
      if(action=="Display" || action=="ToggleFullscreen") {
        string page=ActionContext(action);var pages=new List<IntPtr>();
        foreach(var h in Children(frame))if(Owner(h)==(uint)pid && Text(h)==page && IsWindowEnabled(h))pages.Add(h);
        return pages.Count==1 && GetParent(button)==pages[0];
      }
      if(action=="ToggleOrientation") {
        if(VisiblePageLabels(frame).Length!=0)return false;
        var parent=GetParent(button);
        if(parent==IntPtr.Zero || GetParent(parent)!=frame)return false;
        foreach(var label in new string[]{"+","\u2212","Center"}) {
          int count=0;
          foreach(var h in Children(parent))if(GetParent(h)==parent && Owner(h)==(uint)pid && Class(h)!="Static" && Text(h)==label)count++;
          if(count!=1)return false;
        }
      }
      return true;
    }
    public static string PrototypeRailLabel(string action) {
      ActionLabels(action); // Same fixed read-only action allowlist.
      switch(action) {
        case "Menu":case "Settings":return "Settings";
        case "Navigation":return "Chart";
        case "Route":return "Passage";
        case "AIS":return "Traffic";
        case "Energy":return "Energy";
        case "Instruments":return "Instruments";
        case "Anchor":return "Anchor";
        default:return null;
      }
    }
    private static IntPtr ResolveButton(IntPtr frame,int pid,string action) {
      var labels=ActionLabels(action);var root=AssertFrame(frame,pid);var matches=new List<IntPtr>();
      var rail=root.Shell=="prototype"?PrototypeNavigation(frame,pid):IntPtr.Zero;
      var railLabel=rail==IntPtr.Zero?null:PrototypeRailLabel(action);
      if(railLabel!=null)labels=new string[]{railLabel};
      foreach(var h in Children(frame)) {
        if(Array.IndexOf(labels,Text(h))<0 || Class(h)=="Static" || !IsWindowEnabled(h) || Owner(h)!=(uint)pid)continue;
        if(railLabel!=null && GetParent(h)!=rail)continue;
        Rect r;if(!GetWindowRect(h,out r) || !Contains(root.Bounds,r))continue;
        bool visible=true;for(var parent=GetParent(h);parent!=IntPtr.Zero && parent!=frame;parent=GetParent(parent)) {
          Rect pr;if(!GetWindowRect(parent,out pr) || !Contains(pr,r))visible=false;
        }
        if(visible && (action!="CyclePalette" || ScopedButton(frame,pid,h,action)))matches.Add(h);
      }
      if(matches.Count!=1 || !ScopedButton(frame,pid,matches[0],action))throw new InvalidOperationException("Reviewed button must be unique, enabled, fully visible and in its exact source-reviewed page or chart rail.");
      return matches[0];
    }
    public static void Click(IntPtr frame,int pid,string action) {
      var button=ResolveButton(frame,pid,action);
      ClickReviewedButton(frame,pid,button,delegate{return ResolveButton(frame,pid,action);});
    }
    private static void ClickReviewedButton(IntPtr frame,int pid,IntPtr button,Func<IntPtr> resolve=null) {
      AssertFrame(frame,pid);
      if(!IsWindowVisible(button) || !IsWindowEnabled(button) || Owner(button)!=(uint)pid || !IsChild(frame,button))
        throw new InvalidOperationException("Reviewed button identity changed before press.");
      Rect screen,client;
      if(!GetWindowRect(button,out screen) || !GetClientRect(button,out client) || client.Width<24 || client.Height<24)throw new InvalidOperationException("Reviewed button geometry unavailable.");
      var hit=WindowFromPoint(new Point{X=(screen.Left+screen.Right)/2,Y=(screen.Top+screen.Bottom)/2});
      if(hit!=button && !IsChild(button,hit))throw new InvalidOperationException("Reviewed button is obscured; no click sent.");
      var parent=GetParent(button);var caption=Text(button);var kind=Class(button);var parentCaption=Text(parent);
      AssertFrame(frame,pid);
      var position=new IntPtr((client.Width/2)|((client.Height/2)<<16));UIntPtr result;
      // One target-local press/release. No global input or click retry.
      var down=SendMessageTimeoutW(button,0x201,new UIntPtr(1),position,0x2,1000,out result);
      if(down==IntPtr.Zero)throw new InvalidOperationException("Reviewed press result uncertain; no release or retry to an unverified control.");
      AssertFrame(frame,pid);Rect heldScreen,heldClient;
      if(!IsWindowVisible(button) || !IsWindowEnabled(button) || Owner(button)!=(uint)pid || !IsChild(frame,button) ||
          GetParent(button)!=parent || Text(button)!=caption || Class(button)!=kind || Text(parent)!=parentCaption ||
          !GetWindowRect(button,out heldScreen) || !GetClientRect(button,out heldClient) ||
          heldScreen.Left!=screen.Left || heldScreen.Top!=screen.Top || heldScreen.Right!=screen.Right || heldScreen.Bottom!=screen.Bottom ||
          heldClient.Width!=client.Width || heldClient.Height!=client.Height || (resolve!=null && resolve()!=button))
        throw new InvalidOperationException("Reviewed control changed during press; no release or retry.");
      hit=WindowFromPoint(new Point{X=(screen.Left+screen.Right)/2,Y=(screen.Top+screen.Bottom)/2});
      if(hit!=button && !IsChild(button,hit))throw new InvalidOperationException("Reviewed control became obscured during press; no release or retry.");
      var up=SendMessageTimeoutW(button,0x202,UIntPtr.Zero,position,0x2,1000,out result);
      if(up==IntPtr.Zero)throw new InvalidOperationException("Reviewed button did not respond; no click retry.");
      Thread.Sleep(300);AssertFrame(frame,pid);
    }
    public static string SelectionPage(string action) {
      switch(action) {
        case "SelectFirstVisibleWaypoint":return "OpenNav product page: Waypoints";
        case "SelectFirstVisibleAis":return "OpenNav product page: AIS targets";
        default:throw new InvalidOperationException("Unsupported read-only row selection.");
      }
    }
    public static bool IsSelectionLabel(string action,string label) {
      SelectionPage(action); // Reject unknown actions even for empty labels.
      if(String.IsNullOrEmpty(label) || label.Length>2046 || label.IndexOfAny(new char[]{'\r','\n','\0'})>=0)return false;
      if(action=="SelectFirstVisibleWaypoint")
        return Regex.IsMatch(label,@"^.+ / (?:mark|in route)$",RegexOptions.CultureInvariant);
      // Exact source-derived health prefixes; optional upstream status text may
      // be localized. Fixed page actions do not satisfy this grammar.
      return Regex.IsMatch(label,@"^.+ / (?:Active|Inactive|Lost|Position doubtful|Active distress beacon|Distress beacon testing)(?: / .+)?$",RegexOptions.CultureInvariant);
    }
    public static SelectionRow ChooseSelectionRow(string action,string page,SelectionRow[] rows) {
      if(page!=SelectionPage(action) || rows==null || rows.Length>4096)
        throw new InvalidOperationException("Exact reviewed list page and bounded rows required.");
      var candidates=new List<SelectionRow>();var identities=new HashSet<long>();
      foreach(var row in rows) {
        if(row==null || row.Handle<=0 || !identities.Add(row.Handle))throw new InvalidOperationException("Ambiguous row identity.");
        if(row.Enabled && row.Visible && row.DirectChild && IsSelectionLabel(action,row.Label))candidates.Add(row);
      }
      if(candidates.Count==0)throw new InvalidOperationException("No reviewed list row is fully visible; no selection sent.");
      candidates.Sort(delegate(SelectionRow a,SelectionRow b){int y=a.Top.CompareTo(b.Top);return y!=0?y:a.Left.CompareTo(b.Left);});
      if(candidates.Count>1 && candidates[0].Top==candidates[1].Top && candidates[0].Left==candidates[1].Left)
        throw new InvalidOperationException("Overlapping first rows are ambiguous.");
      return candidates[0];
    }
    public static SelectionRow SelectRow(IntPtr frame,int pid,string action) {
      var pageLabel=SelectionPage(action);var root=AssertFrame(frame,pid);var pages=new List<IntPtr>();
      foreach(var h in Children(frame))
        if(Text(h)==pageLabel && Owner(h)==(uint)pid && IsWindowEnabled(h))pages.Add(h);
      if(pages.Count!=1)throw new InvalidOperationException("Exactly one reviewed waypoint/AIS list page must be visible.");
      var page=pages[0];Rect pageBounds;
      if(!GetWindowRect(page,out pageBounds))throw new InvalidOperationException("List page geometry unavailable.");
      var rows=new List<SelectionRow>();
      foreach(var h in Children(page)) {
        if(GetParent(h)!=page || Owner(h)!=(uint)pid || Class(h)=="Static")continue;
        Rect r;if(!GetWindowRect(h,out r))throw new InvalidOperationException("List changed while reading row bounds.");
        rows.Add(new SelectionRow{Handle=h.ToInt64(),Label=Text(h),Top=r.Top,Left=r.Left,Enabled=IsWindowEnabled(h),
          Visible=Contains(root.Bounds,r) && Contains(pageBounds,r),DirectChild=true});
      }
      var chosen=ChooseSelectionRow(action,Text(page),rows.ToArray());
      var button=new IntPtr(chosen.Handle);Rect final;
      // No caller supplies an HWND/name/coordinate. Recheck the live page and
      // chosen row immediately before the sole bounded press/release.
      AssertFrame(frame,pid);
      if(Text(page)!=pageLabel || GetParent(button)!=page || Text(button)!=chosen.Label || !GetWindowRect(button,out final) ||
          final.Top!=chosen.Top || final.Left!=chosen.Left || !Contains(pageBounds,final) || !Contains(root.Bounds,final))
        throw new InvalidOperationException("Selected list/page changed before interaction; no retry.");
      ClickReviewedButton(frame,pid,button);
      var expected=action=="SelectFirstVisibleWaypoint"?"OpenNav product page: Waypoint detail":"OpenNav product page: AIS target";
      if(Array.IndexOf(VisiblePageLabels(frame),expected)<0)
        throw new InvalidOperationException("Selection did not expose its read-only detail page; inspect saved before image without retrying.");
      return chosen;
    }
    public static string[] VisiblePageLabels(IntPtr frame) {
      var result=new List<string>();foreach(var h in Children(frame)) {
        var text=Text(h);if(text.StartsWith("OpenNav product page:",StringComparison.Ordinal) || text.StartsWith("OpenNav page:",StringComparison.Ordinal))result.Add(text);
      }return result.ToArray();
    }
  }
}

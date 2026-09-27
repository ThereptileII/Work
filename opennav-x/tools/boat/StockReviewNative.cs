using System;
using System.ComponentModel;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;

namespace OpenNavX {
  // Review tooling only. Callers select fixed source-reviewed display actions;
  // no arbitrary keys, mouse coordinates, menu IDs or window messages.
  public static class StockReviewNative {
    [StructLayout(LayoutKind.Sequential)] public struct Rect {
      public int Left,Top,Right,Bottom;
      public int Width { get { return Right-Left; } }
      public int Height { get { return Bottom-Top; } }
    }
    [StructLayout(LayoutKind.Sequential)] private struct MonitorInfo { public uint Size;public Rect Monitor,Work;public uint Flags; }
    public sealed class WindowInfo {
      public long Handle;public int ProcessId;public uint Dpi;public Rect Bounds;public bool Maximized;
    }
    public sealed class ResizeInfo {
      public WindowInfo Before,Restored,After;public Rect MonitorBounds,WorkArea;public bool RestoreRequested;
    }
    private delegate bool EnumCallback(IntPtr window,IntPtr parameter);
    [DllImport("user32.dll")] private static extern bool EnumWindows(EnumCallback callback,IntPtr parameter);
    [DllImport("user32.dll",CharSet=CharSet.Unicode)] private static extern int GetWindowTextW(IntPtr window,StringBuilder text,int maximum);
    [DllImport("user32.dll")] private static extern uint GetWindowThreadProcessId(IntPtr window,out uint process);
    [DllImport("user32.dll")] private static extern IntPtr GetForegroundWindow();
    [DllImport("user32.dll")] private static extern bool SetForegroundWindow(IntPtr window);
    [DllImport("user32.dll")] private static extern bool IsWindowVisible(IntPtr window);
    [DllImport("user32.dll")] private static extern bool IsWindowEnabled(IntPtr window);
    [DllImport("user32.dll")] private static extern bool IsIconic(IntPtr window);
    [DllImport("user32.dll")] private static extern bool IsZoomed(IntPtr window);
    [DllImport("user32.dll")] private static extern bool ShowWindow(IntPtr window,int command);
    [DllImport("user32.dll")] private static extern bool GetWindowRect(IntPtr window,out Rect rect);
    [DllImport("user32.dll")] private static extern IntPtr GetParent(IntPtr child);
    [DllImport("user32.dll")] private static extern IntPtr MonitorFromWindow(IntPtr window,uint flags);
    [DllImport("user32.dll",CharSet=CharSet.Unicode)] private static extern bool GetMonitorInfoW(IntPtr monitor,ref MonitorInfo info);
    [DllImport("user32.dll",SetLastError=true)] private static extern bool SetWindowPos(IntPtr window,IntPtr after,int x,int y,int width,int height,uint flags);
    [DllImport("user32.dll")] public static extern IntPtr SetThreadDpiAwarenessContext(IntPtr value);
    [DllImport("user32.dll")] private static extern uint GetDpiForWindow(IntPtr window);
    [DllImport("dwmapi.dll")] private static extern int DwmGetWindowAttribute(IntPtr window,uint attribute,out Rect rect,uint bytes);

    private static string Text(IntPtr h) { var text=new StringBuilder(2048);GetWindowTextW(h,text,text.Capacity);return text.ToString(); }
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
    public static void Foreground(IntPtr frame,int pid) {
      if(Owner(frame)!=(uint)pid || IsIconic(frame) || !IsWindowVisible(frame))throw new InvalidOperationException("Reviewed frame unavailable or minimized.");
      SetForegroundWindow(frame);Thread.Sleep(250);AssertFrame(frame,pid);
    }
    private static WindowInfo FrameIdentity(IntPtr frame,int pid,bool requireForeground) {
      if(frame==IntPtr.Zero || Owner(frame)!=(uint)pid || (requireForeground && GetForegroundWindow()!=frame) ||
         IsIconic(frame) || !IsWindowVisible(frame) || !IsWindowEnabled(frame))throw new InvalidOperationException("Exact reviewed stock frame must be foreground and enabled; dismiss other windows manually.");
      // Executable and exact process/start identity are verified by StockReview.
      // The native surface exposes capture, fixed resize and one audited zoom command.
      if(GetParent(frame)!=IntPtr.Zero || !Text(frame).StartsWith("OpenCPN",StringComparison.Ordinal))
        throw new InvalidOperationException("Expected the official stock OpenCPN main frame.");
      var rect=Bounds(frame);
      if(rect.Width<100 || rect.Height<100 || rect.Width>7680 || rect.Height>4320)throw new InvalidOperationException("Reviewed frame dimensions are outside bounded display geometry.");
      var dpi=GetDpiForWindow(frame);
      if(dpi<72 || dpi>384)throw new InvalidOperationException("Unexpected application DPI.");
      return new WindowInfo {Handle=frame.ToInt64(),ProcessId=pid,Dpi=dpi,Bounds=rect,Maximized=IsZoomed(frame)};
    }
    public static WindowInfo AssertFrame(IntPtr frame,int pid) {
      var info=FrameIdentity(frame,pid,true);
      if(!Contains(Monitor(frame).Monitor,info.Bounds))throw new InvalidOperationException("Complete reviewed frame must fit on its current monitor.");
      return info;
    }
    public static void AssertCapture(IntPtr frame,int pid,WindowInfo expected) {
      var current=AssertFrame(frame,pid);
      if(current.Bounds.Left!=expected.Bounds.Left || current.Bounds.Top!=expected.Bounds.Top ||
         current.Bounds.Right!=expected.Bounds.Right || current.Bounds.Bottom!=expected.Bounds.Bottom || current.Dpi!=expected.Dpi)
        throw new InvalidOperationException("Reviewed window moved or changed DPI during capture.");
      IntPtr obscurer=IntPtr.Zero;Exception failure=null;
      // Never let an exception escape a reverse-P/Invoke enumeration callback.
      // Any inaccessible/disappearing overlaid window invalidates the capture.
      EnumWindows(delegate(IntPtr h,IntPtr p){
        try {
          if(h==frame)return false;
          if(IsWindowVisible(h) && !IsIconic(h) && Intersects(expected.Bounds,Bounds(h)) && obscurer==IntPtr.Zero)obscurer=h;
          return true;
        } catch(Exception error) {failure=error;return false;}
      },IntPtr.Zero);
      if(failure!=null)throw new InvalidOperationException("Window changed during capture visibility check.",failure);
      if(obscurer!=IntPtr.Zero)throw new InvalidOperationException("Another visible window obscures the reviewed frame; no screenshot published. obscuringWindow="+SafeObscurerDiagnostic(obscurer));
    }
    [DllImport("user32.dll",CharSet=CharSet.Unicode)] private static extern int GetClassNameW(IntPtr window,StringBuilder text,int maximum);
    [DllImport("user32.dll")] private static extern IntPtr GetWindow(IntPtr window,uint relation);
    [DllImport("dwmapi.dll")] private static extern int DwmGetWindowAttribute(IntPtr window,uint attribute,out uint value,uint bytes);
    private static string DiagnosticString(string value) {
      // Private refusal evidence only. Bound captions and escape control text;
      // never reinterpret cloak/owner metadata as capture permission.
      if(value==null)value="";if(value.Length>256)value=value.Substring(0,256);
      var result=new StringBuilder("\"");
      foreach(char c in value) {
        if(c=='"' || c=='\\')result.Append('\\').Append(c);
        else if(c<32 || Char.IsSurrogate(c))result.Append("\\u").Append(((int)c).ToString("x4",System.Globalization.CultureInfo.InvariantCulture));
        else result.Append(c);
      }
      return result.Append('"').ToString();
    }
    private static string FormatObscurerDiagnostic(long hwnd,uint pid,long owner,string windowClass,string title,Rect bounds,int cloakResult,uint cloak) {
      return "{\"hwnd\":"+hwnd.ToString(System.Globalization.CultureInfo.InvariantCulture)+
        ",\"pid\":"+pid.ToString(System.Globalization.CultureInfo.InvariantCulture)+
        ",\"owner\":"+owner.ToString(System.Globalization.CultureInfo.InvariantCulture)+
        ",\"class\":"+DiagnosticString(windowClass)+",\"title\":"+DiagnosticString(title)+
        ",\"bounds\":{\"left\":"+bounds.Left.ToString(System.Globalization.CultureInfo.InvariantCulture)+
        ",\"top\":"+bounds.Top.ToString(System.Globalization.CultureInfo.InvariantCulture)+
        ",\"right\":"+bounds.Right.ToString(System.Globalization.CultureInfo.InvariantCulture)+
        ",\"bottom\":"+bounds.Bottom.ToString(System.Globalization.CultureInfo.InvariantCulture)+"}"+
        ",\"cloakResult\":"+cloakResult.ToString(System.Globalization.CultureInfo.InvariantCulture)+
        ",\"cloak\":"+(cloakResult==0?cloak.ToString(System.Globalization.CultureInfo.InvariantCulture):"null")+"}";
    }
    private static string SafeObscurerDiagnostic(IntPtr window) {
      try {
        var name=new StringBuilder(257);var caption=new StringBuilder(257);
        GetClassNameW(window,name,name.Capacity);GetWindowTextW(window,caption,caption.Capacity);
        uint cloak;int result=DwmGetWindowAttribute(window,14,out cloak,4);
        return FormatObscurerDiagnostic(window.ToInt64(),Owner(window),GetWindow(window,4).ToInt64(),name.ToString(),caption.ToString(),Bounds(window),result,cloak);
      } catch {return "{\"observationUnavailable\":true}";}
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
        throw new InvalidOperationException("Fixed stock resize refused at "+stage+"; before="+Geometry(result.Before)+"; restored="+Geometry(result.Restored)+"; after="+Geometry(result.After)+"; current="+CurrentGeometry(frame,pid)+". "+error.Message,error);
      }
    }
    public sealed class ZoomInfo {public int CommandId;public string Navigation,ZoomIn,ZoomOut;public long Menu,Submenu;}
    [StructLayout(LayoutKind.Sequential)] private struct ChartGuiInfo {
      public uint Size,Flags;public IntPtr Active,Focus,Capture,MenuOwner,MoveSize,Caret;public Rect CaretBounds;
    }
    [DllImport("user32.dll")] private static extern IntPtr GetMenu(IntPtr frame);
    [DllImport("user32.dll")] private static extern IntPtr GetSubMenu(IntPtr menu,int position);
    [DllImport("user32.dll")] private static extern int GetMenuItemCount(IntPtr menu);
    [DllImport("user32.dll")] private static extern uint GetMenuItemID(IntPtr menu,int position);
    [DllImport("user32.dll")] private static extern uint GetMenuState(IntPtr menu,uint item,uint flags);
    [DllImport("user32.dll",CharSet=CharSet.Unicode)] private static extern int GetMenuStringW(IntPtr menu,uint item,StringBuilder value,int length,uint flags);
    [DllImport("user32.dll")] private static extern short GetAsyncKeyState(int key);
    [DllImport("user32.dll")] private static extern bool GetGUIThreadInfo(uint thread,ref ChartGuiInfo info);
    [DllImport("user32.dll",SetLastError=true)] private static extern IntPtr SendMessageTimeoutW(IntPtr window,uint message,IntPtr wparam,IntPtr lparam,uint flags,uint timeout,out IntPtr result);
    private static string MenuText(IntPtr menu,int index) {
      var value=new StringBuilder(512);int length=GetMenuStringW(menu,(uint)index,value,value.Capacity,0x400);
      if(length<=0 || length>=511)throw new InvalidOperationException("Bounded native menu label unavailable.");
      return value.ToString();
    }
    private static bool ZoomTuple(string navigation,string zoomIn,string zoomOut) {
      return (navigation=="&Navigate" && zoomIn=="Zoom In\t+" && zoomOut=="Zoom Out\t-") ||
             (navigation=="&Navigation" && zoomIn=="Zooma in\t+" && zoomOut=="Zooma ut\t-");
    }
    private static bool OrdinaryZoomEntry(uint state,IntPtr child) {
      // Enabled, unchecked, plain text leaf. Never accept a submenu, bitmap,
      // owner-drawn callback or active/highlighted menu entry.
      return state!=0xffffffff && (state&0x99f)==0 && child==IntPtr.Zero;
    }
    private static void CountZoomCommands(IntPtr menu,int depth,ref int count,ref int visited) {
      int length=GetMenuItemCount(menu);
      if(depth>4 || length<0 || length>64)throw new InvalidOperationException("Unexpected native menu structure.");
      for(int i=0;i<length;i++) {
        if(++visited>256)throw new InvalidOperationException("Native menu inventory exceeds bound.");
        if(GetMenuItemID(menu,i)==2001)count++;
        IntPtr child=GetSubMenu(menu,i);if(child!=IntPtr.Zero)CountZoomCommands(child,depth+1,ref count,ref visited);
      }
    }
    private static ZoomInfo InspectZoomOut(IntPtr frame,int pid) {
      var window=AssertFrame(frame,pid);AssertCapture(frame,pid,window);
      uint owner;uint thread=GetWindowThreadProcessId(frame,out owner);
      var gui=new ChartGuiInfo();gui.Size=(uint)Marshal.SizeOf(typeof(ChartGuiInfo));
      if(!GetGUIThreadInfo(thread,ref gui) || (gui.Flags&30)!=0 || gui.Capture!=IntPtr.Zero || gui.Active!=frame)
        throw new InvalidOperationException("Stock chart must have idle exact foreground input context.");
      foreach(int key in new[]{1,2,4,5,6,16,17,18,91,92})if((GetAsyncKeyState(key)&0x8000)!=0)
        throw new InvalidOperationException("User mouse or modifier input is active; no zoom command sent.");
      IntPtr menu=GetMenu(frame);if(menu==IntPtr.Zero)throw new InvalidOperationException("Visible native stock menu bar required.");
      int count=0,visited=0;CountZoomCommands(menu,0,ref count,ref visited);
      if(count!=1)throw new InvalidOperationException("Exactly one pinned Zoom Out command required.");
      IntPtr navigation=GetSubMenu(menu,0);if(navigation==IntPtr.Zero)throw new InvalidOperationException("Pinned first navigation menu absent.");
      string zoomIn=null,zoomOut=null;int n=GetMenuItemCount(navigation);
      for(int i=0;i<n;i++) {
        uint id=GetMenuItemID(navigation,i);
        if(id==2000 || id==2001) {
          uint state=GetMenuState(navigation,(uint)i,0x400);
          if(!OrdinaryZoomEntry(state,GetSubMenu(navigation,i)))
            throw new InvalidOperationException("Pinned zoom menu entry must be enabled ordinary text.");
          if(id==2000)zoomIn=MenuText(navigation,i);else zoomOut=MenuText(navigation,i);
        }
      }
      string title=MenuText(menu,0);
      if(!ZoomTuple(title,zoomIn,zoomOut))throw new InvalidOperationException("Pinned English/Swedish stock zoom tuple differs.");
      return new ZoomInfo{CommandId=2001,Navigation=title,ZoomIn=zoomIn,ZoomOut=zoomOut,Menu=menu.ToInt64(),Submenu=navigation.ToInt64()};
    }
    public static ZoomInfo ZoomOut(IntPtr frame,int pid) {
      var before=InspectZoomOut(frame,pid);var final=InspectZoomOut(frame,pid);
      if(before.Menu!=final.Menu || before.Submenu!=final.Submenu || before.Navigation!=final.Navigation || before.ZoomIn!=final.ZoomIn || before.ZoomOut!=final.ZoomOut)
        throw new InvalidOperationException("Stock zoom menu changed before command; nothing dispatched.");
      // Pinned 37fd0cd idents.h:2001 -> MyFrame::OnToolLeftClick only invokes
      // focusedCanvas.ZoomCanvas(1/g_plus_minus_zoom_factor,false).
      IntPtr result;
      if(SendMessageTimeoutW(frame,0x111,new IntPtr(2001),IntPtr.Zero,3,5000,out result)==IntPtr.Zero)
        throw new Win32Exception(Marshal.GetLastWin32Error(),"Zoom delivery uncertain; no retry.");
      Thread.Sleep(300);AssertFrame(frame,pid);
      return before;
    }
  }
}

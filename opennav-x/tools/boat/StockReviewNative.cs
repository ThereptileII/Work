using System;
using System.ComponentModel;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;

namespace OpenNavX {
  // Review tooling only. No key, mouse coordinate or window-message
  // entry point: callers select a single source-reviewed display action.
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
    public static WindowInfo AssertFrame(IntPtr frame,int pid) {
      if(frame==IntPtr.Zero || Owner(frame)!=(uint)pid || GetForegroundWindow()!=frame ||
         IsIconic(frame) || !IsWindowVisible(frame) || !IsWindowEnabled(frame))throw new InvalidOperationException("Exact reviewed stock frame must be foreground and enabled; dismiss other windows manually.");
      // Executable and exact process/start identity are verified by StockReview.
      // This native surface exposes only capture/resize; no menu or key injection.
      if(GetParent(frame)!=IntPtr.Zero || !Text(frame).StartsWith("OpenCPN",StringComparison.Ordinal))
        throw new InvalidOperationException("Expected the official stock OpenCPN main frame.");
      var rect=Bounds(frame);
      if(rect.Width<100 || rect.Height<100 || rect.Width>7680 || rect.Height>4320 || !Contains(Monitor(frame).Monitor,rect))throw new InvalidOperationException("Complete reviewed frame must fit on its current monitor.");
      var dpi=GetDpiForWindow(frame);
      if(dpi<72 || dpi>384)throw new InvalidOperationException("Unexpected application DPI.");
      return new WindowInfo {Handle=frame.ToInt64(),ProcessId=pid,Dpi=dpi,Bounds=rect,Maximized=IsZoomed(frame)};
    }
    public static void AssertCapture(IntPtr frame,int pid,WindowInfo expected) {
      var current=AssertFrame(frame,pid);
      if(current.Bounds.Left!=expected.Bounds.Left || current.Bounds.Top!=expected.Bounds.Top ||
         current.Bounds.Right!=expected.Bounds.Right || current.Bounds.Bottom!=expected.Bounds.Bottom || current.Dpi!=expected.Dpi)
        throw new InvalidOperationException("Reviewed window moved or changed DPI during capture.");
      bool obscured=false;Exception failure=null;
      // Never let an exception escape a reverse-P/Invoke enumeration callback.
      // Any inaccessible/disappearing overlaid window invalidates the capture.
      EnumWindows(delegate(IntPtr h,IntPtr p){
        try {
          if(h==frame)return false;
          if(IsWindowVisible(h) && !IsIconic(h) && Intersects(expected.Bounds,Bounds(h)))obscured=true;
          return true;
        } catch(Exception error) {failure=error;return false;}
      },IntPtr.Zero);
      if(failure!=null)throw new InvalidOperationException("Window changed during capture visibility check.",failure);
      if(obscured)throw new InvalidOperationException("Another visible window obscures the reviewed frame; no screenshot published.");
    }
    public static void Resize1280x800(IntPtr frame,int pid) {
      AssertFrame(frame,pid);var monitor=Monitor(frame);
      if(monitor.Work.Width<1280 || monitor.Work.Height<800)throw new InvalidOperationException("1280x800 does not fit the current monitor work area. Display/DPI and taskbar are not changed.");
      if(IsZoomed(frame)){ShowWindow(frame,9);Thread.Sleep(150);AssertFrame(frame,pid);}
      Rect outer;if(!GetWindowRect(frame,out outer))throw new InvalidOperationException("Outer frame bounds unavailable.");
      var visible=Bounds(frame);int dx=visible.Left-outer.Left,dy=visible.Top-outer.Top;
      int width=1280+outer.Width-visible.Width,height=800+outer.Height-visible.Height;
      if(!SetWindowPos(frame,IntPtr.Zero,monitor.Work.Left-dx,monitor.Work.Top-dy,width,height,0x14))throw new Win32Exception(Marshal.GetLastWin32Error(),"Window resize failed.");
      Thread.Sleep(300);var after=AssertFrame(frame,pid);
      if(after.Bounds.Width!=1280 || after.Bounds.Height!=800)throw new InvalidOperationException("Application did not accept exactly 1280x800 physical pixels; no resize retry.");
    }
  }
}

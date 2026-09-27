using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;

namespace OpenNavX {
  // The pinned OpenCPN ShowNavWarning -> AlertDialog only. This is deliberately
  // separate from StockReviewNative: its ordinary enabled-frame guard stays strict.
  public static class StockWelcomeNative {
    [StructLayout(LayoutKind.Sequential)] public struct Rect {
      public int Left,Top,Right,Bottom;
      public int Width { get { return Right-Left; } }
      public int Height { get { return Bottom-Top; } }
    }
    [StructLayout(LayoutKind.Sequential)] private struct MonitorInfo { public uint Size;public Rect Monitor,Work;public uint Flags; }
    public sealed class NoticeInfo {
      public long Frame,Modal,Agree,Cancel,Html;
      public int ProcessId,AgreeId,CancelId;
      public uint Dpi;
      public Rect Bounds;
      public string Title,ModalClass,HtmlClass,HtmlName,AgreeText,CancelText;
    }
    private delegate bool EnumCallback(IntPtr h,IntPtr data);
    [DllImport("user32.dll")] private static extern bool EnumWindows(EnumCallback callback,IntPtr data);
    [DllImport("user32.dll")] private static extern bool EnumChildWindows(IntPtr parent,EnumCallback callback,IntPtr data);
    [DllImport("user32.dll")] private static extern uint GetWindowThreadProcessId(IntPtr h,out uint pid);
    [DllImport("user32.dll",CharSet=CharSet.Unicode)] private static extern int GetWindowTextW(IntPtr h,StringBuilder s,int max);
    [DllImport("user32.dll",CharSet=CharSet.Unicode)] private static extern int GetClassNameW(IntPtr h,StringBuilder s,int max);
    [DllImport("user32.dll")] private static extern IntPtr GetWindow(IntPtr h,uint command);
    [DllImport("user32.dll")] private static extern IntPtr GetParent(IntPtr h);
    [DllImport("user32.dll")] private static extern int GetDlgCtrlID(IntPtr h);
    [DllImport("user32.dll")] private static extern bool IsWindowVisible(IntPtr h);
    [DllImport("user32.dll")] private static extern bool IsWindowEnabled(IntPtr h);
    [DllImport("user32.dll")] private static extern bool IsIconic(IntPtr h);
    [DllImport("user32.dll")] private static extern IntPtr GetForegroundWindow();
    [DllImport("user32.dll")] private static extern bool SetForegroundWindow(IntPtr h);
    [DllImport("user32.dll")] private static extern bool GetWindowRect(IntPtr h,out Rect r);
    [DllImport("user32.dll")] private static extern uint GetDpiForWindow(IntPtr h);
    [DllImport("user32.dll")] private static extern IntPtr MonitorFromWindow(IntPtr h,uint flags);
    [DllImport("user32.dll",CharSet=CharSet.Unicode)] private static extern bool GetMonitorInfoW(IntPtr h,ref MonitorInfo info);
    [DllImport("user32.dll",SetLastError=true)] private static extern IntPtr SendMessageTimeoutW(IntPtr h,uint message,IntPtr w,IntPtr l,uint flags,uint timeout,out UIntPtr result);
    private static uint Pid(IntPtr h){uint p;GetWindowThreadProcessId(h,out p);return p;}
    private static string Text(IntPtr h){var s=new StringBuilder(2048);GetWindowTextW(h,s,s.Capacity);return s.ToString();}
    private static string Class(IntPtr h){var s=new StringBuilder(256);GetClassNameW(h,s,s.Capacity);return s.ToString();}
    private static Rect Bounds(IntPtr h){Rect r;if(!GetWindowRect(h,out r))throw new InvalidOperationException("Warning bounds unavailable.");return r;}
    private static bool Same(Rect a,Rect b){return a.Left==b.Left&&a.Top==b.Top&&a.Right==b.Right&&a.Bottom==b.Bottom;}
    private static bool Contains(Rect a,Rect b){return b.Left>=a.Left&&b.Top>=a.Top&&b.Right<=a.Right&&b.Bottom<=a.Bottom;}
    private static bool Intersects(Rect a,Rect b){return b.Left<a.Right&&b.Right>a.Left&&b.Top<a.Bottom&&b.Bottom>a.Top;}
    private static List<IntPtr> Windows(IntPtr parent){
      var result=new List<IntPtr>();Exception failed=null;
      EnumCallback callback=delegate(IntPtr h,IntPtr p){try{result.Add(h);if(result.Count>2048)throw new InvalidOperationException("Window inventory exceeded bound.");return true;}catch(Exception e){failed=e;return false;}};
      if(parent==IntPtr.Zero)EnumWindows(callback,IntPtr.Zero);else EnumChildWindows(parent,callback,IntPtr.Zero);
      if(failed!=null)throw failed;return result;
    }
    private static bool SupportedTuple(string title,string agree,string cancel){
      return (title=="Welcome to OpenCPN"&&agree=="Agree"&&cancel=="Cancel")||
             (title=="V\u00e4lkommen till OpenCPN"&&agree=="Acceptera"&&cancel=="Avbryt");
    }
    private static bool SameNotice(NoticeInfo a,NoticeInfo b){
      return a!=null&&b!=null&&a.Frame==b.Frame&&a.Modal==b.Modal&&a.Agree==b.Agree&&a.Cancel==b.Cancel&&a.Html==b.Html&&
        a.ProcessId==b.ProcessId&&a.AgreeId==b.AgreeId&&a.CancelId==b.CancelId&&a.Dpi==b.Dpi&&Same(a.Bounds,b.Bounds)&&
        a.Title==b.Title&&a.ModalClass==b.ModalClass&&a.HtmlClass==b.HtmlClass&&a.HtmlName==b.HtmlName&&a.AgreeText==b.AgreeText&&a.CancelText==b.CancelText;
    }
    private static NoticeInfo Find(int pid){
      if(pid<=0)throw new InvalidOperationException("Exact launched process required.");
      IntPtr modal=IntPtr.Zero;var windows=Windows(IntPtr.Zero);
      foreach(var h in windows)if(Pid(h)==(uint)pid&&IsWindowVisible(h)&&(Text(h)=="Welcome to OpenCPN"||Text(h)=="V\u00e4lkommen till OpenCPN")){
        if(modal!=IntPtr.Zero)throw new InvalidOperationException("Ambiguous first-start warning.");modal=h;
      }
      if(modal==IntPtr.Zero||Class(modal)!="#32770"||!IsWindowEnabled(modal)||IsIconic(modal))throw new InvalidOperationException("Exact pinned English or Swedish navigation caution is not available.");
      var frame=GetWindow(modal,4); // GW_OWNER, not an arbitrary caller-supplied HWND.
      if(frame==IntPtr.Zero||Pid(frame)!=(uint)pid||GetWindow(frame,4)!=IntPtr.Zero||!Text(frame).StartsWith("OpenCPN",StringComparison.Ordinal)||
         !IsWindowVisible(frame)||IsWindowEnabled(frame)||IsIconic(frame))throw new InvalidOperationException("Warning must uniquely own the disabled stock main frame.");
      foreach(var h in windows)if(Pid(h)==(uint)pid&&IsWindowVisible(h)&&h!=modal&&h!=frame)throw new InvalidOperationException("Unexpected additional visible application window.");
      IntPtr agree=IntPtr.Zero,cancel=IntPtr.Zero,html=IntPtr.Zero;int buttons=0;
      foreach(var h in Windows(modal)){
        if(Pid(h)!=(uint)pid||GetParent(h)!=modal||!IsWindowVisible(h))throw new InvalidOperationException("Unexpected warning child hierarchy.");
        var cls=Class(h);var name=Text(h);
        if(cls=="Button"){
          ++buttons;
          if(GetDlgCtrlID(h)==5100&&agree==IntPtr.Zero)agree=h;
          else if(GetDlgCtrlID(h)==5101&&cancel==IntPtr.Zero)cancel=h;
          else throw new InvalidOperationException("Unexpected warning action.");
          if(!IsWindowEnabled(h))throw new InvalidOperationException("Warning action unavailable.");
        }else if(cls=="wxWindowNR"&&name=="htmlWindow"&&html==IntPtr.Zero)html=h;
        else if(cls!="Static")throw new InvalidOperationException("Unexpected warning content class: "+cls+" / "+name);
      }
      if(buttons!=2||agree==IntPtr.Zero||cancel==IntPtr.Zero||html==IntPtr.Zero)throw new InvalidOperationException("Pinned AlertDialog structure differs.");
      if(!SupportedTuple(Text(modal),Text(agree),Text(cancel)))throw new InvalidOperationException("Mixed or unreviewed warning language tuple.");
      var bounds=Bounds(modal);var monitor=new MonitorInfo();monitor.Size=(uint)Marshal.SizeOf(typeof(MonitorInfo));
      if(bounds.Width<200||bounds.Height<150||bounds.Width>3840||bounds.Height>2160||!GetMonitorInfoW(MonitorFromWindow(modal,2),ref monitor)||!Contains(monitor.Monitor,bounds))throw new InvalidOperationException("Complete warning must be visible on the current monitor.");
      foreach(var h in new[]{agree,cancel,html})if(!Contains(bounds,Bounds(h)))throw new InvalidOperationException("Warning content is clipped.");
      var dpi=GetDpiForWindow(modal);if(dpi<72||dpi>384)throw new InvalidOperationException("Unexpected warning DPI.");
      return new NoticeInfo{Frame=frame.ToInt64(),Modal=modal.ToInt64(),Agree=agree.ToInt64(),Cancel=cancel.ToInt64(),Html=html.ToInt64(),ProcessId=pid,AgreeId=5100,CancelId=5101,Dpi=dpi,Bounds=bounds,Title=Text(modal),ModalClass=Class(modal),HtmlClass=Class(html),HtmlName=Text(html),AgreeText=Text(agree),CancelText=Text(cancel)};
    }
    public static NoticeInfo Inspect(int pid){
      var info=Find(pid);SetForegroundWindow(new IntPtr(info.Modal));Thread.Sleep(250);AssertUnchanged(pid,info);return info;
    }
    public static void AssertUnchanged(int pid,NoticeInfo expected){
      if(expected==null)throw new InvalidOperationException("A captured warning is required.");
      var now=Find(pid);
      if(!SameNotice(now,expected)||GetForegroundWindow()!=new IntPtr(now.Modal))throw new InvalidOperationException("Warning identity, text, geometry or foreground changed.");
      bool found=false;foreach(var h in Windows(IntPtr.Zero)){
        if(h==new IntPtr(now.Modal)){found=true;break;}
        if(IsWindowVisible(h)&&!IsIconic(h)&&Intersects(now.Bounds,Bounds(h)))throw new InvalidOperationException("Warning is obscured; no capture or acknowledgement.");
      }
      if(!found)throw new InvalidOperationException("Warning disappeared during visibility verification.");
    }
    public static void Agree(int pid,NoticeInfo expected){
      AssertUnchanged(pid,expected);
      UIntPtr result;
      // Exactly the existing Agree button. No caller-selectable message, key,
      // coordinate, caption or action. Timeout is uncertain, never retried.
      if(SendMessageTimeoutW(new IntPtr(expected.Agree),0x00F5,IntPtr.Zero,IntPtr.Zero,0x0003,5000,out result)==IntPtr.Zero)throw new InvalidOperationException("Agree delivery timed out or failed; inspect normally, never retry automatically.");
    }
  }
}

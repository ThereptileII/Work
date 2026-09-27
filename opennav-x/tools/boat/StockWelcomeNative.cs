using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;

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
    [DllImport("kernel32.dll")] private static extern uint GetCurrentThreadId();
    [DllImport("user32.dll",SetLastError=true)] private static extern IntPtr GetThreadDesktop(uint thread);
    [DllImport("user32.dll",SetLastError=true)] private static extern IntPtr OpenInputDesktop(uint flags,[MarshalAs(UnmanagedType.Bool)] bool inherit,uint access);
    [DllImport("user32.dll",SetLastError=true)] private static extern bool CloseDesktop(IntPtr desktop);
    [DllImport("user32.dll",SetLastError=true,CharSet=CharSet.Unicode,ExactSpelling=true)] private static extern bool GetUserObjectInformationW(IntPtr desktop,int index,IntPtr buffer,uint bytes,out uint needed);
    [StructLayout(LayoutKind.Sequential)] private struct GuiThreadInfo {
      public uint Size,Flags;
      public IntPtr Active,Focus,Capture,MenuOwner,MoveSize,Caret;
      public Rect CaretRect;
    }
    [DllImport("user32.dll",SetLastError=true)] private static extern bool GetGUIThreadInfo(uint thread,ref GuiThreadInfo info);
    private static bool? GuiFlag(uint? flags,uint mask){return flags.HasValue?(bool?)((flags.Value&mask)!=0):null;}
    private static string GuiFlagsJson(uint? flags){
      return "{\"raw\":"+(flags.HasValue?flags.Value.ToString(System.Globalization.CultureInfo.InvariantCulture):"null")+
        ",\"menu\":"+JsonBool(GuiFlag(flags,0x04))+",\"moveSize\":"+JsonBool(GuiFlag(flags,0x02))+
        ",\"systemMenu\":"+JsonBool(GuiFlag(flags,0x08))+",\"popupMenu\":"+JsonBool(GuiFlag(flags,0x10))+"}";
    }
    private static string ForegroundDiagnostic(){
      var window=GetForegroundWindow();uint process=0;uint thread=window==IntPtr.Zero?0:GetWindowThreadProcessId(window,out process);
      uint? flags=null;int? error=null;
      if(thread!=0){
        var info=new GuiThreadInfo();info.Size=(uint)Marshal.SizeOf(typeof(GuiThreadInfo));
        if(GetGUIThreadInfo(thread,ref info)){flags=info.Flags;error=0;}else error=Marshal.GetLastWin32Error();
      }
      bool stable=GetForegroundWindow()==window;
      return "{\"hwnd\":"+Number(window.ToInt64())+",\"pid\":"+Number(process)+",\"threadId\":"+Number(thread)+
        ",\"unchangedDuringQuery\":"+JsonBool(stable)+",\"guiError\":"+(error.HasValue?error.Value.ToString(System.Globalization.CultureInfo.InvariantCulture):"null")+
        ",\"flags\":"+GuiFlagsJson(flags)+"}";
    }
    private sealed class DesktopState {
      public string Name;
      public bool? ReceivesInput;
      public int HandleError,NameError,InputError;
      public string Json(){return "{\"name\":"+JsonString(Name)+",\"receivesInput\":"+JsonBool(ReceivesInput)+",\"handleError\":"+Number(HandleError)+",\"nameError\":"+Number(NameError)+",\"inputError\":"+Number(InputError)+"}";}
    }
    private static string Number(long value){return value.ToString(System.Globalization.CultureInfo.InvariantCulture);}
    private static string JsonString(string value){
      if(value==null)return "null";
      var text=new StringBuilder("\"");
      foreach(char c in value){
        if(c=='\\'||c=='\"'){text.Append('\\');text.Append(c);}
        else if(c<32||c>126){text.Append("\\u");text.Append(((int)c).ToString("x4",System.Globalization.CultureInfo.InvariantCulture));}
        else text.Append(c);
      }
      return text.Append('\"').ToString();
    }
    private static string JsonBool(bool? value){return value.HasValue?(value.Value?"true":"false"):"null";}
    private static DesktopState ReadDesktopState(IntPtr desktop,int handleError){
      var result=new DesktopState();
      if(desktop==IntPtr.Zero){result.HandleError=handleError==0?-1:handleError;return result;}
      // Fixed 1024-byte Unicode name buffer; no variable-size native allocation
      // from an external length. UOI_IO is a Win32 BOOL, exactly four bytes.
      IntPtr buffer=Marshal.AllocHGlobal(1024);
      try{
        uint needed;
        if(GetUserObjectInformationW(desktop,2,buffer,1024,out needed)){
          if(needed<2||needed>1024||(needed%2)!=0)result.NameError=-1;
          else {
            var name=Marshal.PtrToStringUni(buffer,(int)(needed/2));
            if(name.Length==0||name[name.Length-1]!='\0'||name.IndexOf('\0')!=name.Length-1)result.NameError=-1;
            else result.Name=name.Substring(0,name.Length-1);
          }
        }else result.NameError=Marshal.GetLastWin32Error();
        if(GetUserObjectInformationW(desktop,6,buffer,4,out needed)){
          int value=Marshal.ReadInt32(buffer);
          if(needed!=4||(value!=0&&value!=1))result.InputError=-1;
          else result.ReceivesInput=value!=0;
        }else result.InputError=Marshal.GetLastWin32Error();
      }finally{Marshal.FreeHGlobal(buffer);}
      return result;
    }
    private static string DesktopRelation(string helperName,bool? helperInput,int helperError,string inputName,bool? input,int inputError){
      if(inputError!=0)return "INPUT_DESKTOP_UNAVAILABLE";
      if(helperError!=0||helperName==null||inputName==null||!helperInput.HasValue||!input.HasValue)return "DESKTOP_METADATA_INCOMPLETE";
      if(!input.Value)return "INPUT_DESKTOP_CHANGED_OR_DISCONNECTED";
      if(helperInput.Value&&String.Equals(helperName,inputName,StringComparison.OrdinalIgnoreCase))return "HELPER_DESKTOP_RECEIVES_INPUT";
      return "HELPER_DESKTOP_NOT_INPUT_DESKTOP";
    }
    private static string DesktopDiagnostic(){
      // This runs only on an already-refused, exact-process warning path.
      // No desktop switch, activation, credential operation, hooks or access edits.
      var helperHandle=GetThreadDesktop(GetCurrentThreadId());
      var helper=ReadDesktopState(helperHandle,helperHandle==IntPtr.Zero?Marshal.GetLastWin32Error():0);
      var inputHandle=OpenInputDesktop(0,false,0x0001); // DESKTOP_READOBJECTS only.
      DesktopState input;bool? closed=null;int closeError=0;
      try{input=ReadDesktopState(inputHandle,inputHandle==IntPtr.Zero?Marshal.GetLastWin32Error():0);}
      finally{
        // Never close the borrowed handle returned by GetThreadDesktop.
        if(inputHandle!=IntPtr.Zero){closed=CloseDesktop(inputHandle);if(closed==false)closeError=Marshal.GetLastWin32Error();}
      }
      string relation=DesktopRelation(helper.Name,helper.ReceivesInput,helper.HandleError|helper.NameError|helper.InputError,input.Name,input.ReceivesInput,input.HandleError|input.NameError|input.InputError);
      return "{\"foreground\":"+ForegroundDiagnostic()+",\"relation\":"+JsonString(relation)+",\"helperDesktop\":"+helper.Json()+",\"inputDesktop\":"+input.Json()+",\"inputHandleClosed\":"+JsonBool(closed)+",\"closeError\":"+Number(closeError)+",\"interpretation\":\"Metadata only; access denial does not establish a locked desktop.\"}";
    }
    private static string SafeDesktopDiagnostic(){
      try{return DesktopDiagnostic();}
      catch(Exception e){return "{\"probeError\":"+JsonString(e.GetType().Name)+"}";}
    }
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
    private static string ChangedFields(NoticeInfo a,NoticeInfo b){
      if(a==null||b==null)return "observation";
      var fields=new List<string>();
      if(a.Frame!=b.Frame)fields.Add("Frame");
      if(a.Modal!=b.Modal)fields.Add("Modal");
      if(a.Agree!=b.Agree)fields.Add("Agree");
      if(a.Cancel!=b.Cancel)fields.Add("Cancel");
      if(a.Html!=b.Html)fields.Add("Html");
      if(a.ProcessId!=b.ProcessId)fields.Add("ProcessId");
      if(a.AgreeId!=b.AgreeId)fields.Add("AgreeId");
      if(a.CancelId!=b.CancelId)fields.Add("CancelId");
      if(a.Dpi!=b.Dpi)fields.Add("Dpi");
      if(a.Title!=b.Title)fields.Add("Title");
      if(a.ModalClass!=b.ModalClass)fields.Add("ModalClass");
      if(a.HtmlClass!=b.HtmlClass)fields.Add("HtmlClass");
      if(a.HtmlName!=b.HtmlName)fields.Add("HtmlName");
      if(a.AgreeText!=b.AgreeText)fields.Add("AgreeText");
      if(a.CancelText!=b.CancelText)fields.Add("CancelText");
      if(!Same(a.Bounds,b.Bounds))fields.Add("Bounds");
      return String.Join(",",fields.ToArray());
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
    private static void VerifyActivation(bool requestReturned,bool rendezvousCompleted,Action verify){
      string result="Warning focusRequestReturned="+JsonBool(requestReturned)+". ";
      if(!rendezvousCompleted)throw new InvalidOperationException(result+"WM_NULL activation rendezvous failed or timed out; no capture or acknowledgement.");
      try{verify();}
      catch(InvalidOperationException error){throw new InvalidOperationException(result+error.Message,error);}
    }
    public static NoticeInfo Inspect(int pid){
      var info=Find(pid);var modal=new IntPtr(info.Modal);
      bool requested=SetForegroundWindow(modal);UIntPtr ignored;
      // Cross-thread activation is asynchronous. Wait for this exact modal to
      // process the nudge, without sharing input queues or assuming permission.
      // Microsoft: devblogs.microsoft.com/oldnewthing/20161118-00/?p=94745
      bool completed=SendMessageTimeoutW(modal,0x0000,IntPtr.Zero,IntPtr.Zero,0x0003,5000,out ignored)!=IntPtr.Zero;
      try{VerifyActivation(requested,completed,delegate{AssertUnchanged(pid,info);});}
      catch(InvalidOperationException error){
        if(!completed)throw new InvalidOperationException(error.Message+" desktopDiagnostic="+SafeDesktopDiagnostic(),error);
        throw;
      }
      return info;
    }
    public static void AssertUnchanged(int pid,NoticeInfo expected){
      if(expected==null)throw new InvalidOperationException("A captured warning is required.");
      var now=Find(pid);
      if(!SameNotice(now,expected))throw new InvalidOperationException("Warning captured fields changed: "+ChangedFields(now,expected)+".");
      var foreground=GetForegroundWindow();
      if(foreground!=new IntPtr(now.Modal))throw new InvalidOperationException("Warning foreground differs: expected HWND "+now.Modal+" PID "+pid+"; observed HWND "+foreground.ToInt64()+" PID "+Pid(foreground)+". No capture or acknowledgement. desktopDiagnostic="+SafeDesktopDiagnostic());
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

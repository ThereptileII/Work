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
    // Fixed warning title-bar focus only. There is no public point, HWND,
    // caption, input event or message selector. Agreement remains separate.
    [StructLayout(LayoutKind.Sequential)] private struct CaptionPoint { public int X,Y; }
    [StructLayout(LayoutKind.Sequential)] private struct TitleBarInfo {
      public uint Size;public Rect Bounds;
      [MarshalAs(UnmanagedType.ByValArray,SizeConst=6)] public uint[] States;
    }
    [StructLayout(LayoutKind.Sequential)] private struct MouseInput {
      public int X,Y;public uint Data,Flags,Time;public UIntPtr Extra;
    }
    [StructLayout(LayoutKind.Explicit)] private struct InputUnion { [FieldOffset(0)] public MouseInput Mouse; }
    [StructLayout(LayoutKind.Sequential)] private struct NativeInput { public uint Type;public InputUnion Value; }
    private sealed class CaptionCandidate { public NoticeInfo Notice;public Rect TitleBar;public int X,Y; }
    [DllImport("user32.dll",SetLastError=true)] private static extern bool GetTitleBarInfo(IntPtr window,ref TitleBarInfo info);
    [DllImport("user32.dll")] private static extern IntPtr WindowFromPoint(CaptionPoint point);
    [DllImport("user32.dll")] private static extern short GetAsyncKeyState(int key);
    [DllImport("user32.dll")] private static extern int GetSystemMetrics(int index);
    [DllImport("user32.dll",SetLastError=true)] private static extern uint SendInput(uint count,[In] NativeInput[] inputs,int size);
    [DllImport("dwmapi.dll")] private static extern int DwmGetWindowAttribute(IntPtr window,uint attribute,out uint value,uint bytes);
    private static string WindowDiagnostic(IntPtr window){
      uint cloak=0;int result=DwmGetWindowAttribute(window,14,out cloak,4); // DWMWA_CLOAKED, observation only.
      return "{\"hwnd\":"+Number(window.ToInt64())+",\"pid\":"+Number(Pid(window))+",\"class\":"+JsonString(Class(window))+
        ",\"cloakResult\":"+Number(result)+",\"cloak\":"+(result==0?Number(cloak):"null")+"}";
    }
    private static void AssertNotObscured(NoticeInfo notice){
      bool found=false;foreach(var h in Windows(IntPtr.Zero)){
        if(h==new IntPtr(notice.Modal)){found=true;break;}
        // Preserve the existing strict capture rule. A cloaked observation is
        // reported, never used to silently ignore an overlapping window.
        if(IsWindowVisible(h)&&!IsIconic(h)&&Intersects(notice.Bounds,Bounds(h)))
          throw new InvalidOperationException("Warning is obscured; obscuringWindow="+WindowDiagnostic(h));
      }
      if(!found)throw new InvalidOperationException("Warning disappeared during visibility verification.");
    }
    private static CaptionPoint ChooseCaptionPoint(Rect window,Rect title,uint state){
      if(window.Width<200||window.Height<150||title.Width<32||title.Height<8||!Contains(window,title)||
         (state&0x00018009)!=0)throw new InvalidOperationException("A fully visible available title bar is required; titleState="+Number(state)+".");
      long x=((long)title.Left+title.Right)/2,y=((long)title.Top+title.Bottom)/2;
      if(x<short.MinValue||x>short.MaxValue||y<short.MinValue||y>short.MaxValue)throw new InvalidOperationException("Caption point exceeds signed native hit-test coordinates.");
      return new CaptionPoint{X=(int)x,Y=(int)y};
    }
    private static CaptionCandidate InspectCaptionCandidate(int pid){
      var notice=Find(pid);var modal=new IntPtr(notice.Modal);
      uint cloak;int cloakResult=DwmGetWindowAttribute(modal,14,out cloak,4);
      if(cloakResult!=0||cloak!=0)throw new InvalidOperationException("Warning target visibility unavailable; target="+WindowDiagnostic(modal));
      var title=new TitleBarInfo{Size=(uint)Marshal.SizeOf(typeof(TitleBarInfo)),States=new uint[6]};
      if(!GetTitleBarInfo(modal,ref title))throw new InvalidOperationException("Native title bar information unavailable; error="+Number(Marshal.GetLastWin32Error())+".");
      var point=ChooseCaptionPoint(notice.Bounds,title.Bounds,title.States[0]);
      var actual=WindowFromPoint(point);
      if(actual!=modal)throw new InvalidOperationException("Caption point is covered or belongs to another window; hitWindow="+WindowDiagnostic(actual));
      int packed=unchecked((int)((uint)(ushort)point.X|((uint)(ushort)point.Y<<16)));UIntPtr hit;
      bool hitCompleted=SendMessageTimeoutW(modal,0x0084,IntPtr.Zero,new IntPtr(packed),0x0003,5000,out hit)!=IntPtr.Zero;
      if(!hitCompleted||hit.ToUInt64()!=2)
        throw new InvalidOperationException("Caption requires HTCAPTION=2; completed="+JsonBool(hitCompleted)+" hitTest="+hit.ToUInt64().ToString(System.Globalization.CultureInfo.InvariantCulture)+" target="+WindowDiagnostic(modal));
      var now=Find(pid);actual=WindowFromPoint(point);
      if(!SameNotice(now,notice))throw new InvalidOperationException("Warning changed during caption inspection: "+ChangedFields(now,notice));
      if(actual!=modal)throw new InvalidOperationException("Caption point changed; hitWindow="+WindowDiagnostic(actual));
      AssertNotObscured(notice);
      return new CaptionCandidate{Notice=notice,TitleBar=title.Bounds,X=point.X,Y=point.Y};
    }
    private static bool InputDesktopSuitable(DesktopState helper,DesktopState input){
      return helper.HandleError==0&&helper.NameError==0&&helper.InputError==0&&input.HandleError==0&&input.NameError==0&&input.InputError==0&&
        helper.Name=="Default"&&input.Name=="Default"&&helper.ReceivesInput==true&&input.ReceivesInput==true;
    }
    private static void AssertIdleInput(IntPtr modal){
      var helperHandle=GetThreadDesktop(GetCurrentThreadId());
      var helper=ReadDesktopState(helperHandle,helperHandle==IntPtr.Zero?Marshal.GetLastWin32Error():0);
      var inputHandle=OpenInputDesktop(0,false,1);DesktopState input;
      try{input=ReadDesktopState(inputHandle,inputHandle==IntPtr.Zero?Marshal.GetLastWin32Error():0);}
      finally{if(inputHandle!=IntPtr.Zero&&!CloseDesktop(inputHandle))throw new InvalidOperationException("Input desktop handle release failed.");}
      if(!InputDesktopSuitable(helper,input))throw new InvalidOperationException("Caption focus requires the same active Default input desktop. desktopDiagnostic="+SafeDesktopDiagnostic());
      var foreground=GetForegroundWindow();uint ignored;uint thread=GetWindowThreadProcessId(foreground,out ignored);
      var state=new GuiThreadInfo{Size=(uint)Marshal.SizeOf(typeof(GuiThreadInfo))};
      if(thread==0||!GetGUIThreadInfo(thread,ref state)||!GuiInputIdle(state.Flags,state.Capture)||GetForegroundWindow()!=foreground)
        throw new InvalidOperationException("Foreground input is busy, changed or unavailable. desktopDiagnostic="+SafeDesktopDiagnostic());
      uint targetThread=GetWindowThreadProcessId(modal,out ignored);
      var targetState=new GuiThreadInfo{Size=(uint)Marshal.SizeOf(typeof(GuiThreadInfo))};
      if(targetThread==0||!GetGUIThreadInfo(targetThread,ref targetState)||!GuiInputIdle(targetState.Flags,targetState.Capture))
        throw new InvalidOperationException("Exact warning GUI input is busy or unavailable; target="+WindowDiagnostic(modal));
      // Never compensate for a held modifier/button by releasing the user's input.
      foreach(int key in new[]{1,2,4,5,6,16,17,18,91,92})if((GetAsyncKeyState(key)&0x8000)!=0)
        throw new InvalidOperationException("Caption focus refused while a mouse button or modifier is held; virtualKey="+Number(key)+".");
    }
    private static bool GuiInputIdle(uint flags,IntPtr capture){return (flags&0x1e)==0&&capture==IntPtr.Zero;}
    private static int AbsoluteCoordinate(int pixel,int origin,int length){
      if(length<2||length>65536||(long)pixel<origin||(long)pixel>=(long)origin+length)throw new InvalidOperationException("Caption point is outside the bounded virtual desktop.");
      return (int)Math.Round(((long)pixel-origin)*65535.0/(length-1),MidpointRounding.AwayFromZero);
    }
    private static NativeInput MouseEvent(int x,int y,uint flags){
      return new NativeInput{Type=0,Value=new InputUnion{Mouse=new MouseInput{X=x,Y=y,Flags=flags}}};
    }
    private static void DeliverCaptionInput(Func<uint> deliver,Func<uint> release,Action verify){
      uint sent=deliver();
      if(sent!=3){
        // One release-only cleanup for uncertain partial delivery. Never repeat
        // a down/move or claim success. A failed cleanup remains explicitly unknown.
        uint? released=null;if(sent>0)released=release();
        throw new InvalidOperationException("Caption input delivery uncertain; inserted="+Number(sent)+" expected=3; releaseOnlyCleanup="+(released.HasValue?Number(released.Value):"not-required")+". No retry or acknowledgement; verify button state before further input.");
      }
      verify(); // A submitted click alone is never proof of focus.
    }
    private static void AssertFocusProcess(int pid,long startedUtcTicks){
      using(var process=System.Diagnostics.Process.GetProcessById(pid))
      using(var caller=System.Diagnostics.Process.GetCurrentProcess()){
        if(process.HasExited||process.SessionId!=caller.SessionId||process.StartTime.ToUniversalTime().Ticks!=startedUtcTicks)
          throw new InvalidOperationException("Caption focus process/session/start identity changed.");
      }
    }
    public static NoticeInfo FocusCaption(int pid,long startedUtcTicks){
      AssertFocusProcess(pid,startedUtcTicks);
      var first=InspectCaptionCandidate(pid);AssertIdleInput(new IntPtr(first.Notice.Modal));
      if(GetForegroundWindow()==new IntPtr(first.Notice.Modal)){AssertUnchanged(pid,first.Notice);return first.Notice;}
      int x=AbsoluteCoordinate(first.X,GetSystemMetrics(76),GetSystemMetrics(78));
      int y=AbsoluteCoordinate(first.Y,GetSystemMetrics(77),GetSystemMetrics(79));
      var inputs=new[]{MouseEvent(x,y,0xc001),MouseEvent(0,0,0x0002),MouseEvent(0,0,0x0004)};
      AssertFocusProcess(pid,startedUtcTicks);AssertIdleInput(new IntPtr(first.Notice.Modal));
      var final=InspectCaptionCandidate(pid);
      if(!SameNotice(first.Notice,final.Notice)||!Same(first.TitleBar,final.TitleBar)||first.X!=final.X||first.Y!=final.Y)
        throw new InvalidOperationException("Caption candidate changed before input; no click sent.");
      if(x!=AbsoluteCoordinate(final.X,GetSystemMetrics(76),GetSystemMetrics(78))||y!=AbsoluteCoordinate(final.Y,GetSystemMetrics(77),GetSystemMetrics(79)))
        throw new InvalidOperationException("Virtual desktop changed before caption input.");
      int size=Marshal.SizeOf(typeof(NativeInput));
      int deliveryError=0,releaseError=0;
      try{DeliverCaptionInput(delegate{uint sent=SendInput(3,inputs,size);deliveryError=Marshal.GetLastWin32Error();return sent;},
        delegate{uint released=SendInput(1,new[]{MouseEvent(0,0,0x0004)},size);releaseError=Marshal.GetLastWin32Error();return released;},delegate{
        // Sent messages can overtake queued input. Observe actual activation
        // before using WM_NULL as a rendezvous; never resend the click.
        var wait=System.Diagnostics.Stopwatch.StartNew();
        while(true){
          AssertFocusProcess(pid,startedUtcTicks);var observed=Find(pid);
          if(!SameNotice(first.Notice,observed))throw new InvalidOperationException("Warning changed while caption input was pending: "+ChangedFields(first.Notice,observed));
          if(GetForegroundWindow()==new IntPtr(first.Notice.Modal)&&(GetAsyncKeyState(1)&0x8000)==0)break;
          if(wait.ElapsedMilliseconds>=5000)throw new InvalidOperationException("Caption input submitted but focus/release was not observed; no retry. desktopDiagnostic="+SafeDesktopDiagnostic());
          System.Threading.Thread.Sleep(20);
        }
        UIntPtr ignored;
        if(SendMessageTimeoutW(new IntPtr(first.Notice.Modal),0,IntPtr.Zero,IntPtr.Zero,3,5000,out ignored)==IntPtr.Zero)
          throw new InvalidOperationException("Caption click submitted but exact-modal rendezvous failed; focus uncertain, no retry.");
        AssertFocusProcess(pid,startedUtcTicks);AssertUnchanged(pid,first.Notice);
        if((GetAsyncKeyState(1)&0x8000)!=0)throw new InvalidOperationException("Caption click submitted but button state remains down; no further input.");
      });}catch(InvalidOperationException error){
        throw new InvalidOperationException(error.Message+" sendInputError="+Number(deliveryError)+" releaseInputError="+Number(releaseError)+". Error codes alone do not identify UIPI.",error);
      }
      return first.Notice;
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
      AssertNotObscured(now);
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

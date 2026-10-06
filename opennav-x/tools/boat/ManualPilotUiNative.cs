using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;
using System.Text.RegularExpressions;

namespace OpenNavX {
  // Separate from ReviewWindowNative's read-only allowlist. Only one source-
  // reviewed native control event is exposed; no coordinates or raw messages.
  public static class ManualPilotUiNative {
    public const string Page="SKAGER product page: Autopilot configuration";
    public sealed class Control {
      public long Handle,Parent;public string Context,Label,Class;
      public bool Enabled,Visible,Contained;public int Left,Top,Right,Bottom;
    }
    public sealed class Identity {public string Name;public int Address;}
    public sealed class Snapshot {public long Frame,Surface;public string Modal,InterfaceValue,NameValue;public Control[] Controls;public Identity[] Identities;}
    [StructLayout(LayoutKind.Sequential)] private struct Rect {public int Left,Top,Right,Bottom;}
    [StructLayout(LayoutKind.Sequential)] private struct Point {public int X,Y;}
    private delegate bool EnumCallback(IntPtr h,IntPtr p);
    [DllImport("user32.dll")] private static extern bool EnumWindows(EnumCallback callback,IntPtr p);
    [DllImport("user32.dll")] private static extern bool EnumChildWindows(IntPtr h,EnumCallback callback,IntPtr p);
    [DllImport("user32.dll")] private static extern uint GetWindowThreadProcessId(IntPtr h,out uint p);
    [DllImport("user32.dll")] private static extern IntPtr GetParent(IntPtr h);
    [DllImport("user32.dll")] private static extern IntPtr GetWindow(IntPtr h,uint command);
    [DllImport("user32.dll")] private static extern bool IsWindowVisible(IntPtr h);
    [DllImport("user32.dll")] private static extern bool IsWindowEnabled(IntPtr h);
    [DllImport("user32.dll")] private static extern bool IsIconic(IntPtr h);
    [DllImport("user32.dll")] private static extern IntPtr GetForegroundWindow();
    [DllImport("user32.dll")] private static extern bool GetWindowRect(IntPtr h,out Rect r);
    [DllImport("user32.dll")] private static extern bool GetClientRect(IntPtr h,out Rect r);
    [DllImport("user32.dll")] private static extern IntPtr WindowFromPoint(Point p);
    [DllImport("user32.dll",CharSet=CharSet.Unicode)] private static extern int GetWindowTextW(IntPtr h,StringBuilder b,int n);
    [DllImport("user32.dll",CharSet=CharSet.Unicode)] private static extern int GetClassNameW(IntPtr h,StringBuilder b,int n);
    [DllImport("user32.dll",CharSet=CharSet.Unicode)] private static extern IntPtr SendMessageTimeoutW(IntPtr h,uint message,UIntPtr w,IntPtr l,uint flags,uint ms,out UIntPtr result);
    [ComImport,Guid("618736e0-3c3d-11cf-810c-00aa00389b71"),InterfaceType(ComInterfaceType.InterfaceIsIDispatch)]
    private interface Accessible { [DispId(-5003)] string this[[In,MarshalAs(UnmanagedType.Struct)] object child] {[return:MarshalAs(UnmanagedType.BStr)]get;} }
    [DllImport("oleacc.dll")] private static extern int AccessibleObjectFromWindow(IntPtr h,uint id,ref Guid iid,[MarshalAs(UnmanagedType.Interface)] out Accessible a);
    private static string Text(IntPtr h){var b=new StringBuilder(2048);GetWindowTextW(h,b,b.Capacity);return b.ToString();}
    private static string Class(IntPtr h){var b=new StringBuilder(128);GetClassNameW(h,b,b.Capacity);return b.ToString();}
    private static uint Pid(IntPtr h){uint p;GetWindowThreadProcessId(h,out p);return p;}
    private static string Name(IntPtr h){Accessible a=null;var id=new Guid("618736e0-3c3d-11cf-810c-00aa00389b71");try{
      if(AccessibleObjectFromWindow(h,unchecked((uint)-4),ref id,out a)!=0 || a==null)throw new InvalidOperationException("Identity field accessibility unavailable.");
      var name=a[0];if(name==null || name.Length>128)throw new InvalidOperationException("Invalid accessible field name.");return name;
    }finally{if(a!=null && Marshal.IsComObject(a))Marshal.ReleaseComObject(a);}}
    private static string EditText(IntPtr h){var buffer=Marshal.AllocHGlobal(402);try{UIntPtr result;
      if(SendMessageTimeoutW(h,0x000d,new UIntPtr(201),buffer,2,1000,out result)==IntPtr.Zero || result.ToUInt64()>200)throw new InvalidOperationException("Identity field read unavailable.");
      return Marshal.PtrToStringUni(buffer,(int)result.ToUInt64());
    }finally{Marshal.FreeHGlobal(buffer);}}
    private static List<IntPtr> Children(IntPtr h){var list=new List<IntPtr>();EnumChildWindows(h,delegate(IntPtr c,IntPtr p){list.Add(c);return list.Count<=4096;},IntPtr.Zero);if(list.Count>4096)throw new InvalidOperationException("Too many native controls.");return list;}
    private static Rect Bounds(IntPtr h){Rect r;if(!GetWindowRect(h,out r))throw new InvalidOperationException("Control disappeared.");return r;}
    private static bool Contains(Rect a,Rect b){return b.Left>=a.Left && b.Top>=a.Top && b.Right<=a.Right && b.Bottom<=a.Bottom;}
    private static bool Owned(IntPtr h,IntPtr frame,int pid){for(int i=0;i<8 && h!=IntPtr.Zero;i++,h=GetWindow(h,4)){if(Pid(h)!=(uint)pid)return false;if(h==frame)return true;}return false;}
    private static bool Rail(IntPtr h,int pid){var labels=new List<string>();foreach(var c in Children(h))if(GetParent(c)==h && IsWindowVisible(c) && Pid(c)==(uint)pid && Class(c)!="Static")labels.Add(Text(c));return ReviewWindowNative.IsPrototypeNavigation(labels.ToArray());}
    public static bool IsPassiveChartSurface(string title){return title=="SKAGER chart tools" || title=="SKAGER chart orientation" || title=="SKAGER chart layers" || title=="SKAGER follow boat";}
    private static bool Modal(string title){return title=="ST4000 translator identity" || title=="Permit manual pilot commands?" || title=="Enable physical pilot control?" || title=="Request AUTO";}
    // Pure policy is shared by the actual HWND resolver and inert fixtures.
    public static string[] Target(string action){switch(action){
      case "ScrollUp":return new[]{"pilot-page-scroll","Up"};
      case "ScrollDown":return new[]{"pilot-page-scroll","Down"};
      case "OpenSettings":return new[]{"rail","Settings"};
      case "OpenPilot":return new[]{"frame","Autopilot"};
      case "PilotTab":return new[]{"SKAGER preferences","Autopilot"};
      case "PilotConnection":return new[]{"SKAGER preferences","Pilot connection"};
      case "BackToPilot":return new[]{Page,"Back to manual autopilot"};
      case "Advanced":return new[]{Page,"Advanced connection setup"};
      case "OpenIdentity":return new[]{Page,"Configure translator identity"};
      case "RefreshIdentity":return new[]{Page,"Refresh device identity"};
      case "Permit":return new[]{Page,"Permit manual commissioning..."};
      case "DisplayOnly":return new[]{Page,"Return to display-only"};
      case "AcceptPermission":return new[]{"Permit manual pilot commands?","Save manual permission"};
      case "Enable":case "Disable":return new[]{"SKAGER autopilot","Enable control"};
      case "AcceptEnable":return new[]{"Enable physical pilot control?","Enable manual control"};
      case "Auto":return new[]{"SKAGER autopilot","Auto"};
      case "AcceptAuto":return new[]{"Request AUTO","Request AUTO"};
      case "Standby":return new[]{"SKAGER autopilot","Standby"};
      case "Minus1":return new[]{"SKAGER autopilot","\u22121\u00b0"};
      case "Plus1":return new[]{"SKAGER autopilot","+1\u00b0"};
      case "Minus10":return new[]{"SKAGER autopilot","\u221210\u00b0"};
      case "Plus10":return new[]{"SKAGER autopilot","+10\u00b0"};
      case "SetInterface":return new[]{"ST4000 translator identity","OpenCPN NMEA2000 interface"};
      case "SetName":return new[]{"ST4000 translator identity","Observed translator NAME"};
      case "CancelIdentity":return new[]{"ST4000 translator identity","Cancel"};
      case "CancelPermission":return new[]{"Permit manual pilot commands?","Cancel"};
      case "CancelEnable":return new[]{"Enable physical pilot control?","Cancel"};
      case "CancelAuto":return new[]{"Request AUTO","Cancel"};
      case "SaveIdentity":return new[]{"ST4000 translator identity","Save"};
      default:throw new InvalidOperationException("Unknown manual UI action.");
    }}
    public static Control Choose(string action,string modal,Control[] controls){
      var target=Target(action);if((Modal(target[0])?target[0]:"")!=(modal??""))throw new InvalidOperationException("Unexpected or missing exact permission/identity sheet.");
      Control found=null;foreach(var c in controls){if(c.Context!=target[0] || c.Label!=target[1])continue;
        if(found!=null)throw new InvalidOperationException("Ambiguous native control.");found=c;}
      if(found==null || found.Handle==0 || !found.Enabled || !found.Visible || !found.Contained)throw new InvalidOperationException("Control missing, disabled, hidden or clipped; expose it normally first.");
      bool edit=action=="SetName" || action=="SetInterface";
      if(edit?(found.Class!="Edit"):(found.Class=="Static" || found.Class=="Edit"))throw new InvalidOperationException("Unexpected native control type.");
      return found;
    }
    public static bool CompatibleName(string name){ulong n;return name!=null && Regex.IsMatch(name,"\\A[a-f0-9]{16}\\z") && UInt64.TryParse(name,System.Globalization.NumberStyles.HexNumber,System.Globalization.CultureInfo.InvariantCulture,out n) && ((n>>21)&2047)==1851 && ((n>>40)&255)==135 && ((n>>49)&127)==40 && ((n>>60)&7)==4;}
    public static Identity[] ParseIdentities(string text){var result=new List<Identity>();foreach(Match m in Regex.Matches(text??"","(?:^|\\n)COM8\\s*/\\s*NAME\\s+([a-f0-9]{16})\\s*/\\s*address\\s+([0-9]{1,3})(?=\\s*(?:\\n|$))")){
      int address=Int32.Parse(m.Groups[2].Value,System.Globalization.CultureInfo.InvariantCulture);if(address<254 && CompatibleName(m.Groups[1].Value))result.Add(new Identity{Name=m.Groups[1].Value,Address=address});}return result.ToArray();}
    private static bool Known(string context,string label){foreach(var action in new[]{"ScrollUp","ScrollDown","OpenSettings","OpenPilot","PilotTab","PilotConnection","BackToPilot","Advanced","OpenIdentity","RefreshIdentity","Permit","DisplayOnly","AcceptPermission","Enable","AcceptEnable","Auto","AcceptAuto","Standby","Minus1","Plus1","Minus10","Plus10","SetInterface","SetName","SaveIdentity","CancelIdentity","CancelPermission","CancelEnable","CancelAuto"}){var t=Target(action);if(t[0]==context && t[1]==label)return true;}return false;}
    public static Snapshot Observe(IntPtr frame,int pid){
      if(frame==IntPtr.Zero || Pid(frame)!=(uint)pid || GetParent(frame)!=IntPtr.Zero || !IsWindowVisible(frame) || IsIconic(frame))throw new InvalidOperationException("Exact main frame unavailable.");
      var children=Children(frame);bool pilotPage=false;foreach(var h in children)if(IsWindowVisible(h) && Text(h)==Page)pilotPage=true;int rails=0;foreach(var h in children)if(GetParent(h)==frame && Rail(h,pid))rails++;
      if(rails!=1)throw new InvalidOperationException("Unique installed prototype frame required.");
      var passive=new List<IntPtr>();var roots=new List<IntPtr>();roots.Add(frame);IntPtr sheet=IntPtr.Zero;string modal="";Exception failure=null;
      EnumWindows(delegate(IntPtr h,IntPtr p){try{
        if(h==frame || !IsWindowVisible(h) || Pid(h)!=(uint)pid)return true;
        if(!Owned(h,frame,pid))throw new InvalidOperationException("Unrelated process window is visible.");
        var title=Text(h);
        // Existing capture policy validates these normal chart overlays. They
        // remain passive: do not enumerate their children as action targets.
        if(IsPassiveChartSurface(title)){passive.Add(h);return true;}
        if(Modal(title)) {if(sheet!=IntPtr.Zero)throw new InvalidOperationException("Multiple modal sheets.");sheet=h;modal=title;}
        else if(title!="SKAGER autopilot" && title!="SKAGER preferences")throw new InvalidOperationException("Unreviewed owned window is visible.");
        roots.Add(h);return true;
      }catch(Exception e){failure=e;return false;}},IntPtr.Zero);
      if(failure!=null)throw failure;
      var foreground=GetForegroundWindow();if(sheet!=IntPtr.Zero){
        if(passive.Count!=0 || IsWindowEnabled(frame) || foreground!=sheet || !IsWindowEnabled(sheet))throw new InvalidOperationException("Exact owned modal must hold foreground.");
      }else{
        var verified=ReviewWindowNative.AssertFrame(frame,pid);
        foreach(var h in passive){bool found=false;
          foreach(var surface in verified.Surfaces)if(surface.Handle==h.ToInt64() && surface.Title==Text(h))found=true;
          if(!found)throw new InvalidOperationException("Passive chart overlay was not validated by the read-only surface policy.");
        }
      }
      var controls=new List<Control>();var identities=new List<Identity>();string iface=null,name=null;
      foreach(var root in roots)foreach(var h in Children(root)){
        if(Pid(h)!=(uint)pid)throw new InvalidOperationException("Foreign child window.");
        var cls=Class(h);var text=Text(h);string context=Text(root);if(root==frame){context="frame";
          if(GetParent(GetParent(h))==frame && Rail(GetParent(h),pid))context="rail";
          for(var p=GetParent(h);p!=IntPtr.Zero && p!=frame;p=GetParent(p))if(Text(p)==Page){context=Page;break;}}
        if(context=="frame" && pilotPage && (text=="Up" || text=="Down"))context="pilot-page-scroll";
        if(context==Page && cls=="Static" && IsWindowVisible(h))identities.AddRange(ParseIdentities(text));
        string label=cls=="Edit" && root==sheet && modal=="ST4000 translator identity"?Name(h):text;
        if(!Known(context,label))continue;
        if(cls=="Edit" && label=="OpenCPN NMEA2000 interface"){if(iface!=null)throw new InvalidOperationException("Duplicate identity interface field.");iface=EditText(h);}
        if(cls=="Edit" && label=="Observed translator NAME"){if(name!=null)throw new InvalidOperationException("Duplicate identity NAME field.");name=EditText(h);}
        var bounds=Bounds(h);bool contained=bounds.Right-bounds.Left>=24 && bounds.Bottom-bounds.Top>=16 && Contains(Bounds(frame),bounds);
        bool enabled=IsWindowEnabled(h),visible=IsWindowVisible(h);
        for(var p=GetParent(h);p!=IntPtr.Zero;p=GetParent(p)){
          if(!Contains(Bounds(p),bounds))contained=false;
          if(!IsWindowEnabled(p))enabled=false;if(!IsWindowVisible(p))visible=false;
          if(p==root)break;
        }
        controls.Add(new Control{Handle=h.ToInt64(),Parent=GetParent(h).ToInt64(),Context=context,Label=label,Class=cls,Enabled=enabled,Visible=visible,Contained=contained,Left=bounds.Left,Top=bounds.Top,Right=bounds.Right,Bottom=bounds.Bottom});
      }
      return new Snapshot{Frame=frame.ToInt64(),Surface=sheet.ToInt64(),Modal=modal,InterfaceValue=iface,NameValue=name,Controls=controls.ToArray(),Identities=identities.ToArray()};
    }
    private static void Same(Control a,Control b){if(a.Handle!=b.Handle || a.Parent!=b.Parent || a.Class!=b.Class || a.Left!=b.Left || a.Top!=b.Top || a.Right!=b.Right || a.Bottom!=b.Bottom)throw new InvalidOperationException("Native control changed; no retry.");}
    public static void Act(IntPtr frame,int pid,string action,string value){
      var snapshot=Observe(frame,pid);var selected=Choose(action,snapshot.Modal,snapshot.Controls);var h=new IntPtr(selected.Handle);
      var center=new Point{X=(selected.Left+selected.Right)/2,Y=(selected.Top+selected.Bottom)/2};
      if(WindowFromPoint(center)!=h)throw new InvalidOperationException("Control is obscured; no input sent.");
      bool edit=action=="SetName" || action=="SetInterface";
      if(action=="SaveIdentity"){
        if(snapshot.InterfaceValue!="COM8" || snapshot.NameValue==null)throw new InvalidOperationException("Exact COM8 identity fields required.");
        if(snapshot.NameValue.Length>0){int count=0;foreach(var i in snapshot.Identities)if(i.Name==snapshot.NameValue)count++;if(count!=1)throw new InvalidOperationException("Only the actually observed compatible NAME may be saved.");}
      }
      if(edit){
        if(action=="SetInterface"?value!="COM8":!CompatibleName(value))throw new InvalidOperationException("Unreviewed identity field value.");
        if(action=="SetName"){int count=0;foreach(var i in snapshot.Identities)if(i.Name==value)count++;if(count!=1)throw new InvalidOperationException("NAME must be unique in actual compatible identity controls.");}
        IntPtr buffer=Marshal.StringToHGlobalUni(value);try{UIntPtr result;
          var again=Observe(frame,pid);Same(selected,Choose(action,again.Modal,again.Controls));
          if(action=="SetName"){int count=0;foreach(var i in again.Identities)if(i.Name==value)count++;if(count!=1)throw new InvalidOperationException("Observed NAME changed before field input.");}
          if(SendMessageTimeoutW(h,0x000c,UIntPtr.Zero,buffer,2,1000,out result)==IntPtr.Zero || result==UIntPtr.Zero)throw new InvalidOperationException("Field input outcome uncertain; inspect, never retry.");
        }finally{Marshal.FreeHGlobal(buffer);}return;
      }
      if(!String.IsNullOrEmpty(value))throw new InvalidOperationException("Button actions accept no value.");
      Rect client;if(!GetClientRect(h,out client))throw new InvalidOperationException("Control disappeared.");
      var point=new Point{X=(client.Right-client.Left)/2,Y=(client.Bottom-client.Top)/2};var packed=new IntPtr((point.Y<<16)|(point.X&65535));UIntPtr ignored;
      if(SendMessageTimeoutW(h,0x0201,new UIntPtr(1),packed,2,1000,out ignored)==IntPtr.Zero)throw new InvalidOperationException("Button down outcome uncertain; no retry.");
      var next=Observe(frame,pid);Same(selected,Choose(action,next.Modal,next.Controls));
      if(action=="SaveIdentity" && (snapshot.InterfaceValue!=next.InterfaceValue || snapshot.NameValue!=next.NameValue))throw new InvalidOperationException("Identity fields changed before Save; no retry.");
      if(WindowFromPoint(center)!=h)throw new InvalidOperationException("Control changed after button down; no retry.");
      if(SendMessageTimeoutW(h,0x0202,UIntPtr.Zero,packed,2,1000,out ignored)==IntPtr.Zero)throw new InvalidOperationException("Button up outcome uncertain; no retry.");
      // A sheet may open/close asynchronously. Do not retry or infer pilot ack.
    }
  }
}

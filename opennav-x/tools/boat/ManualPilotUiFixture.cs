// Disposable native Windows fixture only. No product, process attachment, serial
// device or NMEA dependency. Its sole actionable button increments a counter.
using System;
using System.Runtime.InteropServices;
namespace OpenNavX {
  public sealed class ManualPilotUiFixture : IDisposable {
    [UnmanagedFunctionPointer(CallingConvention.Winapi)] private delegate IntPtr Procedure(IntPtr h,uint m,IntPtr w,IntPtr l);
    [StructLayout(LayoutKind.Sequential,CharSet=CharSet.Unicode)] private struct WindowClass {
      public uint Style;public Procedure Proc;public int ClassBytes,WindowBytes;public IntPtr Instance,Icon,Cursor,Background;
      public string Menu,Name;
    }
    [StructLayout(LayoutKind.Sequential)] private struct Message {public IntPtr Window;public uint Id;public UIntPtr W;public IntPtr L;public uint Time;public int X,Y;public uint Private;}
    [DllImport("user32.dll",CharSet=CharSet.Unicode)] private static extern ushort RegisterClassW(ref WindowClass c);
    [DllImport("user32.dll",CharSet=CharSet.Unicode)] private static extern bool UnregisterClassW(string name,IntPtr instance);
    [DllImport("user32.dll",CharSet=CharSet.Unicode)] private static extern IntPtr CreateWindowExW(uint ex,string cls,string text,uint style,int x,int y,int w,int h,IntPtr parent,IntPtr menu,IntPtr instance,IntPtr parameter);
    [DllImport("user32.dll")] private static extern IntPtr DefWindowProcW(IntPtr h,uint m,IntPtr w,IntPtr l);
    [DllImport("user32.dll")] private static extern bool DestroyWindow(IntPtr h);
    [DllImport("user32.dll")] private static extern bool SetForegroundWindow(IntPtr h);
    [DllImport("user32.dll")] private static extern bool EnableWindow(IntPtr h,bool enabled);
    [DllImport("user32.dll")] private static extern bool PeekMessageW(out Message m,IntPtr h,uint first,uint last,uint remove);
    [DllImport("user32.dll")] private static extern bool TranslateMessage(ref Message m);
    [DllImport("user32.dll")] private static extern IntPtr DispatchMessageW(ref Message m);
    [DllImport("kernel32.dll",CharSet=CharSet.Unicode)] private static extern IntPtr GetModuleHandleW(string name);
    private readonly Procedure procedure;private readonly string name;private readonly IntPtr instance;private IntPtr button,overlay;
    public IntPtr Frame {get;private set;} public int Clicks {get;private set;}
    public ManualPilotUiFixture(){
      name="OpenNavXInertPilotUi"+Guid.NewGuid().ToString("N");instance=GetModuleHandleW(null);
      procedure=delegate(IntPtr h,uint m,IntPtr w,IntPtr l){if(m==0x0111 && l==button && ((w.ToInt64()>>16)&65535)==0)Clicks++;return DefWindowProcW(h,m,w,l);};
      var cls=new WindowClass{Proc=procedure,Instance=instance,Name=name};
      if(RegisterClassW(ref cls)==0)throw new InvalidOperationException("Inert class creation failed.");
      try{
        Frame=CreateWindowExW(0,name,"Inert manual helper fixture",0x10cf0000,20,20,1000,650,IntPtr.Zero,IntPtr.Zero,instance,IntPtr.Zero);
        if(Frame==IntPtr.Zero)throw new InvalidOperationException("Inert frame creation failed.");
        var rail=Child(name,"",Frame,5,5,130,540,0);
        int y=5;foreach(var label in new[]{"Chart","Passage","Traffic","Energy","Instruments","Anchor","Radar","Settings"}){Child("BUTTON",label,rail,5,y,115,40,0);y+=55;}
        var page=Child(name,"SKAGER product page: Autopilot configuration",Frame,150,5,700,540,0);
        button=Child("BUTTON","Advanced connection setup",page,30,30,300,48,77);
        SetForegroundWindow(Frame);Pump();
      }catch{Dispose();throw;}
    }
    private IntPtr Child(string cls,string text,IntPtr parent,int x,int y,int w,int h,int id){var child=CreateWindowExW(0,cls,text,0x50000000,x,y,w,h,parent,new IntPtr(id),instance,IntPtr.Zero);if(child==IntPtr.Zero)throw new InvalidOperationException("Inert child creation failed.");return child;}
    public void ShowOverlay(string title,bool outside){
      ClearOverlay();
      overlay=CreateWindowExW(0,name,title,0x90000000,outside?1100:230,450,300,60,Frame,IntPtr.Zero,instance,IntPtr.Zero);
      if(overlay==IntPtr.Zero)throw new InvalidOperationException("Inert overlay creation failed.");
      var labels=title=="SKAGER chart tools"?new[]{"Measure","Waypoint","+","\u2212"}:
        title=="SKAGER chart orientation"?new[]{"North"}:title=="SKAGER chart layers"?new[]{"Layers"}:new[]{"Follow boat"};
      int x=0;foreach(var label in labels){Child("BUTTON",label,overlay,x,5,70,45,0);x+=75;}
      SetForegroundWindow(Frame);Pump();
    }
    public void ClearOverlay(){if(overlay!=IntPtr.Zero){DestroyWindow(overlay);overlay=IntPtr.Zero;}Pump();}
    public void Disable(){EnableWindow(button,false);Pump();}
    public void Pump(){Message m;int count=0;while(PeekMessageW(out m,IntPtr.Zero,0,0,1)){if(++count>2000)throw new InvalidOperationException("Inert message pump exceeded bound.");TranslateMessage(ref m);DispatchMessageW(ref m);}}
    public void Dispose(){if(Frame!=IntPtr.Zero){DestroyWindow(Frame);Frame=IntPtr.Zero;}UnregisterClassW(name,instance);GC.KeepAlive(procedure);}
  }
}

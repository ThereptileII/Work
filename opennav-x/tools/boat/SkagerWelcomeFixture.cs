// Disposable native HWND fixture only. Never loaded by the boat helper.
using System;
using System.ComponentModel;
using System.Runtime.InteropServices;
using System.Threading;

public static class SkagerPointerDeliveryFixture {
  public static int Deliveries,Releases,Verifications;
  public static uint Count,ReleaseCount;
  public static bool RejectVerification;
  public static Func<uint> Delivery(){return delegate{Deliveries++;return Count;};}
  public static Func<uint> Release(){return delegate{Releases++;return ReleaseCount;};}
  public static Action Verify(){return delegate{Verifications++;if(RejectVerification)throw new InvalidOperationException("Fixture verification refused");};}
  public static void Reset(uint count,uint release,bool reject){Deliveries=Releases=Verifications=0;Count=count;ReleaseCount=release;RejectVerification=reject;}
}

public sealed class SkagerWelcomeFixture : IDisposable {
  [StructLayout(LayoutKind.Sequential)] private struct Point { public int X,Y; }
  [StructLayout(LayoutKind.Sequential)] private struct Rect { public int Left,Top,Right,Bottom; }
  [StructLayout(LayoutKind.Sequential)] private struct Message {
    public IntPtr Window; public uint Id; public UIntPtr WParam; public IntPtr LParam;
    public uint Time; public Point Position; public uint Private;
  }
  private delegate IntPtr Procedure(IntPtr window,uint message,IntPtr wParam,IntPtr lParam);
  [StructLayout(LayoutKind.Sequential,CharSet=CharSet.Unicode)] private struct WindowClass {
    public uint Size,Style; public IntPtr Procedure; public int ClassExtra,WindowExtra;
    public IntPtr Instance,Icon,Cursor,Background; public string Menu,Name; public IntPtr SmallIcon;
  }
  [DllImport("user32.dll",CharSet=CharSet.Unicode,SetLastError=true)] private static extern ushort RegisterClassExW(ref WindowClass value);
  [DllImport("user32.dll",CharSet=CharSet.Unicode,SetLastError=true)] private static extern bool UnregisterClassW(string name,IntPtr module);
  [DllImport("user32.dll",CharSet=CharSet.Unicode,SetLastError=true)]
  private static extern IntPtr CreateWindowExW(uint extended,string kind,string text,uint style,int x,int y,int width,int height,IntPtr parent,IntPtr menu,IntPtr module,IntPtr parameter);
  [DllImport("user32.dll")] private static extern IntPtr DefWindowProcW(IntPtr window,uint message,IntPtr wParam,IntPtr lParam);
  [DllImport("user32.dll",SetLastError=true)] private static extern int GetMessageW(out Message message,IntPtr window,uint minimum,uint maximum);
  [DllImport("user32.dll")] private static extern bool TranslateMessage(ref Message message);
  [DllImport("user32.dll")] private static extern IntPtr DispatchMessageW(ref Message message);
  [DllImport("user32.dll")] private static extern bool DestroyWindow(IntPtr window);
  [DllImport("user32.dll")] private static extern bool IsWindow(IntPtr window);
  [DllImport("user32.dll")] private static extern bool IsWindowEnabled(IntPtr window);
  [DllImport("user32.dll")] private static extern bool EnableWindow(IntPtr window,bool enabled);
  [DllImport("user32.dll")] private static extern bool ShowWindow(IntPtr window,int action);
  [DllImport("user32.dll")] private static extern bool SetForegroundWindow(IntPtr window);
  [DllImport("user32.dll")] private static extern IntPtr SetCapture(IntPtr window);
  [DllImport("user32.dll")] private static extern bool ReleaseCapture();
  [DllImport("user32.dll")] private static extern bool GetClientRect(IntPtr window,out Rect rectangle);
  [DllImport("user32.dll")] private static extern int GetSystemMetrics(int index);
  [DllImport("user32.dll")] private static extern int GetWindowLongW(IntPtr window,int index);
  [DllImport("user32.dll")] private static extern int SetWindowLongW(IntPtr window,int index,int value);
  [DllImport("user32.dll")] private static extern bool SetWindowPos(IntPtr window,IntPtr after,int x,int y,int width,int height,uint flags);
  [DllImport("user32.dll",SetLastError=true)] private static extern bool PostThreadMessageW(uint thread,uint message,UIntPtr wParam,IntPtr lParam);
  [DllImport("user32.dll",EntryPoint="SendMessageTimeoutW",CharSet=CharSet.Unicode,SetLastError=true)]
  private static extern IntPtr SetTextMessage(IntPtr window,uint message,IntPtr wParam,string text,uint flags,uint timeout,out UIntPtr result);
  [DllImport("user32.dll",SetLastError=true)] private static extern IntPtr SendMessageTimeoutW(IntPtr window,uint message,IntPtr wParam,IntPtr lParam,uint flags,uint timeout,out UIntPtr result);
  [DllImport("user32.dll")] private static extern IntPtr SetThreadDpiAwarenessContext(IntPtr value);
  [DllImport("kernel32.dll",CharSet=CharSet.Unicode)] private static extern IntPtr GetModuleHandleW(string name);
  [DllImport("kernel32.dll")] private static extern uint GetCurrentThreadId();
  private readonly ManualResetEvent ready=new ManualResetEvent(false);
  private readonly Thread thread;
  private readonly string notice;
  private readonly Procedure frameProcedure,actionProcedure;
  private IntPtr frame,modal,body,agree,gripper,pressed;
  private uint threadId;
  private Exception failure;
  private int agreeCount,cancelCount;
  public IntPtr Frame { get { return frame; } }
  public IntPtr Modal { get { return modal; } }
  public IntPtr Body { get { return body; } }
  public IntPtr Gripper { get { return gripper; } }
  public int AgreeCount { get { return Interlocked.CompareExchange(ref agreeCount,0,0); } }
  public int CancelCount { get { return Interlocked.CompareExchange(ref cancelCount,0,0); } }
  public bool ModalPresent { get { return IsWindow(modal); } }
  public bool OwnerEnabled { get { return IsWindow(frame)&&IsWindowEnabled(frame); } }
  public static IntPtr EnterDpiAwareness(){return SetThreadDpiAwarenessContext(new IntPtr(-4));}
  public static void RestoreDpiAwareness(IntPtr previous){if(previous!=IntPtr.Zero)SetThreadDpiAwarenessContext(previous);}
  public SkagerWelcomeFixture(string text){
    notice=text;frameProcedure=FrameProcedure;actionProcedure=ActionProcedure;
    thread=new Thread(Run);thread.IsBackground=true;thread.SetApartmentState(ApartmentState.STA);thread.Start();
    if(!ready.WaitOne(5000)){if(threadId!=0)PostThreadMessageW(threadId,0x0012,UIntPtr.Zero,IntPtr.Zero);throw new InvalidOperationException("Native welcome fixture startup timed out");}
    if(failure!=null){thread.Join(5000);ready.Dispose();throw new InvalidOperationException("Native welcome fixture startup failed",failure);}
  }
  private void Register(string name,Procedure procedure,IntPtr module){
    var value=new WindowClass{Size=(uint)Marshal.SizeOf(typeof(WindowClass)),Name=name,Instance=module,
      Procedure=Marshal.GetFunctionPointerForDelegate(procedure),Background=new IntPtr(6)};
    if(RegisterClassExW(ref value)==0)throw new Win32Exception(Marshal.GetLastWin32Error());
  }
  private IntPtr Create(string kind,string title,uint style,int x,int y,int width,int height,IntPtr parent,int id,IntPtr module){
    var value=CreateWindowExW(0,kind,title,style,x,y,width,height,parent,new IntPtr(id),module,IntPtr.Zero);
    if(value==IntPtr.Zero)throw new Win32Exception(Marshal.GetLastWin32Error());return value;
  }
  private void Run(){
    IntPtr module=GetModuleHandleW(null),dpi=IntPtr.Zero;bool frameClass=false,actionClass=false;
    try{
      threadId=GetCurrentThreadId();dpi=EnterDpiAwareness();
      Register("SkagerWelcomeFixtureFrame",frameProcedure,module);frameClass=true;
      Register("wxWindowNR",actionProcedure,module);actionClass=true;
      frame=Create("SkagerWelcomeFixtureFrame","SKAGER / OpenCPN",0x00cf0000,40,40,760,660,IntPtr.Zero,0,module);
      modal=Create("#32770","Welcome to SKAGER",0x80cc0000,60,60,700,600,frame,0,module);
      // wxWidgets 3.2.8 creates this child explicitly for wxRESIZE_BORDER;
      // WS_THICKFRAME alone does not create it in a plain Win32 dialog.
      gripper=Create("SCROLLBAR","",0x4400001c,0,0,0,0,modal,0,module);
      Create("Static","Welcome to SKAGER",0x50000000,20,20,640,30,modal,100,module);
      Create("Static","Navigation safety notice \u00b7 OpenCPN",0x50000000,20,60,640,25,modal,101,module);
      body=Create("Edit",notice,0x50000804,20,100,640,350,modal,102,module);
      Create("wxWindowNR","Cancel",0x50000000,360,470,140,48,modal,5101,module);
      agree=Create("wxWindowNR","Agree",0x50000000,520,470,140,48,modal,5100,module);
      Rect client;int gripWidth=GetSystemMetrics(2),gripHeight=GetSystemMetrics(3);
      if(!GetClientRect(modal,out client)||gripWidth<=0||gripHeight<=0||
         !SetWindowPos(gripper,new IntPtr(1),client.Right-gripWidth,client.Bottom-gripHeight,gripWidth,gripHeight,0x0010))
        throw new InvalidOperationException("Native fixture resize-grip placement failed");
      ShowWindow(gripper,5);
      ShowWindow(frame,5);EnableWindow(frame,false);ShowWindow(modal,5);SetForegroundWindow(modal);
      ready.Set();Message message;int result;
      while((result=GetMessageW(out message,IntPtr.Zero,0,0))>0){TranslateMessage(ref message);DispatchMessageW(ref message);}
      if(result<0)throw new Win32Exception(Marshal.GetLastWin32Error());
    }catch(Exception error){failure=error;}
    finally{
      if(IsWindow(modal))DestroyWindow(modal);if(IsWindow(frame))DestroyWindow(frame);
      if(actionClass)UnregisterClassW("wxWindowNR",module);if(frameClass)UnregisterClassW("SkagerWelcomeFixtureFrame",module);
      RestoreDpiAwareness(dpi);ready.Set();
    }
  }
  private IntPtr FrameProcedure(IntPtr window,uint message,IntPtr wParam,IntPtr lParam){
    if(message==0x8001)return SetWindowPos(agree,IntPtr.Zero,520+wParam.ToInt32(),470,0,0,0x0015)?new IntPtr(1):IntPtr.Zero;
    if(message==0x8002){SetWindowLongW(gripper,-16,wParam.ToInt32());return GetWindowLongW(gripper,-16)==wParam.ToInt32()?new IntPtr(1):IntPtr.Zero;}
    return DefWindowProcW(window,message,wParam,lParam);
  }
  private IntPtr ActionProcedure(IntPtr window,uint message,IntPtr wParam,IntPtr lParam){
    if(message==0x0201&&IsWindowEnabled(window)){pressed=window;SetCapture(window);return IntPtr.Zero;}
    if(message==0x0202){
      Rect bounds;long position=lParam.ToInt64();int x=(short)(position&65535),y=(short)((position>>16)&65535);
      bool activate=pressed==window&&IsWindowEnabled(window)&&GetClientRect(window,out bounds)&&x>=0&&y>=0&&x<bounds.Right&&y<bounds.Bottom;
      pressed=IntPtr.Zero;ReleaseCapture();
      if(activate){
        if(window==agree)Interlocked.Increment(ref agreeCount);else Interlocked.Increment(ref cancelCount);
        DestroyWindow(modal);EnableWindow(frame,true);SetForegroundWindow(frame);
      }
      return IntPtr.Zero;
    }
    if(message==0x0215)pressed=IntPtr.Zero;
    return DefWindowProcW(window,message,wParam,lParam);
  }
  public void SetBody(string text){
    UIntPtr result;if(SetTextMessage(body,0x000c,IntPtr.Zero,text,3,5000,out result)==IntPtr.Zero||result==UIntPtr.Zero)
      throw new InvalidOperationException("Native fixture text update failed");
  }
  public void MoveAgree(int delta){
    UIntPtr result;if(SendMessageTimeoutW(frame,0x8001,new IntPtr(delta),IntPtr.Zero,3,5000,out result)==IntPtr.Zero||result==UIntPtr.Zero)
      throw new InvalidOperationException("Native fixture geometry update failed");
  }
  public void SetGripperStyle(int style){
    UIntPtr result;if(SendMessageTimeoutW(frame,0x8002,new IntPtr(style),IntPtr.Zero,3,5000,out result)==IntPtr.Zero||result==UIntPtr.Zero)
      throw new InvalidOperationException("Native fixture resize-grip style update failed");
  }
  public void Dispose(){
    if(thread.IsAlive&&!PostThreadMessageW(threadId,0x0012,UIntPtr.Zero,IntPtr.Zero))throw new Win32Exception(Marshal.GetLastWin32Error());
    if(!thread.Join(5000))throw new InvalidOperationException("Native fixture shutdown timed out");
    ready.Dispose();if(failure!=null)throw new InvalidOperationException("Native fixture thread failed",failure);
  }
}

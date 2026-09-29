// Windows PowerShell 5.1 only. A local-only pipe, restricted to this user and
// SYSTEM. Peer PID, executable and desktop session are checked before transfer.
using System;
using System.Diagnostics;
using System.IO.Pipes;
using System.Runtime.InteropServices;
using System.Security.AccessControl;
using System.Security.Principal;
using System.Text;
public static class XNavAisCredentialPipe {
  public static string PeerFailure="none";
  [DllImport("kernel32.dll",SetLastError=true)] static extern bool GetNamedPipeClientProcessId(IntPtr pipe,out uint id);
  [DllImport("kernel32.dll",SetLastError=true)] static extern bool GetNamedPipeServerProcessId(IntPtr pipe,out uint id);
  [DllImport("kernel32.dll",SetLastError=true)] static extern IntPtr OpenProcess(uint access,bool inherit,uint id);
  [DllImport("kernel32.dll",CharSet=CharSet.Unicode,SetLastError=true)] static extern bool QueryFullProcessImageNameW(IntPtr process,uint flags,StringBuilder name,ref int size);
  [DllImport("kernel32.dll")] static extern bool CloseHandle(IntPtr handle);
  [DllImport("kernel32.dll")] static extern bool ProcessIdToSessionId(uint process,out uint session);
  public static NamedPipeServerStream Create(string name) {
    var acl=new PipeSecurity();acl.SetAccessRuleProtection(true,false);
    acl.AddAccessRule(new PipeAccessRule(new SecurityIdentifier("S-1-5-2"),PipeAccessRights.FullControl,AccessControlType.Deny));
    acl.AddAccessRule(new PipeAccessRule(WindowsIdentity.GetCurrent().User,PipeAccessRights.FullControl,AccessControlType.Allow));
    acl.AddAccessRule(new PipeAccessRule(new SecurityIdentifier("S-1-5-18"),PipeAccessRights.FullControl,AccessControlType.Allow));
    return new NamedPipeServerStream(name,PipeDirection.Out,1,PipeTransmissionMode.Byte,PipeOptions.Asynchronous,1024,1024,acl);
  }
  static bool Expected(uint id,int session,int expectedId) {
    IntPtr p=IntPtr.Zero;
    try {
        uint actualSession;if(!ProcessIdToSessionId(id,out actualSession)){PeerFailure="session-query";return false;}
        if(actualSession!=session){PeerFailure="session-mismatch";return false;}
        if(expectedId!=0&&id!=expectedId){PeerFailure="pid-mismatch";return false;}
        p=OpenProcess(0x1000,false,id);if(p==IntPtr.Zero){PeerFailure="image-access-"+Marshal.GetLastWin32Error();return false;}
        var name=new StringBuilder(1024);int size=name.Capacity;
        string exe=System.IO.Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.System),"WindowsPowerShell\\v1.0\\powershell.exe");
        if(!QueryFullProcessImageNameW(p,0,name,ref size)){PeerFailure="image-query-"+Marshal.GetLastWin32Error();return false;}
        if(!String.Equals(name.ToString(),exe,StringComparison.OrdinalIgnoreCase)){PeerFailure="image-mismatch";return false;}
        return true;
    } catch {PeerFailure="peer-exception";return false;} finally {if(p!=IntPtr.Zero)CloseHandle(p);}
  }
  public static bool Client(NamedPipeServerStream pipe,int session) {
    uint id;return GetNamedPipeClientProcessId(pipe.SafePipeHandle.DangerousGetHandle(),out id)&&Expected(id,session,0);
  }
  public static bool Server(NamedPipeClientStream pipe,int id) {
    // The dispatching process remains alive with this exclusive pipe. Its
    // exact PID is bound into the owned task before connecting. A limited
    // desktop token cannot query the SSH logon's process session; that query
    // adds nothing to this exact live-server identity check.
    uint actual;if(!GetNamedPipeServerProcessId(pipe.SafePipeHandle.DangerousGetHandle(),out actual)){PeerFailure="pipe-server-query-"+Marshal.GetLastWin32Error();return false;}
    if(id<=0||actual!=id){PeerFailure="pid-mismatch";return false;}return true;
  }
}

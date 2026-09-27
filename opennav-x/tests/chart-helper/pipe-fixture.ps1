# Disposable native pipe marker only. Never loads OpenCPN/vendor helpers.
[CmdletBinding()]
param([Parameter(Mandatory=$true)][string]$Directory,[Parameter(Mandatory=$true)][int]$ParentProcessId,
 [ValidateSet('normal','short','no-reply','nonzero','timeout','arbitrary-reply')][string]$Behaviour='normal')
Set-StrictMode -Version Latest;$ErrorActionPreference='Stop'
if([Environment]::OSVersion.Platform -ne 'Win32NT'){throw 'Windows fixture only.'}
Add-Type -TypeDefinition @'
using System;
using System.IO;
using System.IO.Pipes;
using System.Diagnostics;
using System.Threading;
using System.Threading.Tasks;
public static class ChartPipeFixture {
  static bool Wait(Task task,string release,Stopwatch timer) {
    while(!task.Wait(25)) if(File.Exists(release)||timer.ElapsedMilliseconds>12000) return false;
    task.GetAwaiter().GetResult();return true;
  }
  public static int Run(string directory,string pipeName,string behaviour) {
    string release=Path.Combine(directory,"release");Stopwatch timer=Stopwatch.StartNew();
    using(var pipe=new NamedPipeServerStream(pipeName,PipeDirection.InOut,1,
      PipeTransmissionMode.Byte,PipeOptions.Asynchronous)) {
      using(var self=Process.GetCurrentProcess())
        File.WriteAllText(Path.Combine(directory,"ready.partial"),self.Id.ToString());
      File.Move(Path.Combine(directory,"ready.partial"),Path.Combine(directory,"ready"));
      if(!Wait(pipe.WaitForConnectionAsync(),release,timer))return 0;
      byte[] packet=new byte[1025];int count=0;
      while(count<packet.Length) {
        Task<int> read=pipe.ReadAsync(packet,count,packet.Length-count);
        if(!Wait(read,release,timer))break;
        if(read.Result==0)break;count+=read.Result;
      }
      bool exact=count==1025 && packet[0]==2;
      for(int i=1;i<count;i++)if(packet[i]!=0)exact=false;
      File.WriteAllText(Path.Combine(directory,"received"),count.ToString()+":"+(exact?"exact":"not-exact"));
      if(!exact)return count==0?0:91;
      if(behaviour=="timeout") {
        while(!File.Exists(release)&&timer.ElapsedMilliseconds<12000)Thread.Sleep(25);
        return 0;
      }
      if(behaviour=="no-reply")return 0;
      byte[] response=behaviour=="arbitrary-reply" ? new byte[]{0x58,0x59,0x5a} : new byte[]{0x4f,0x4b,0};
      int length=behaviour=="short"?1:3;
      pipe.Write(response,0,length);
      return behaviour=="nonzero"?17:0;
    }
  }
}
'@
$name='OCPN'+($ParentProcessId%10000).ToString('D4',[Globalization.CultureInfo]::InvariantCulture)
$code=[ChartPipeFixture]::Run($Directory,$name,$Behaviour)
exit $code

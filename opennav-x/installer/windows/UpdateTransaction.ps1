# Dot-source only. Windows PowerShell 5.1 compatible. No launch, profile, shell,
# actuator, network, or installation mutation. The caller owns Lifecycle's lock,
# verifies both generation manifests, persists the attempt BEFORE launch, and
# closes the candidate before applying a returned fallback decision.
Set-StrictMode -Version Latest

function Assert-UpdateIdentity($Identity) {
  foreach ($field in @('generation','commit','packageSha256','executableSha256')) {
    if (-not $Identity -or -not $Identity.PSObject.Properties[$field] -or $Identity.$field -isnot [string]) { throw 'Incomplete update generation identity.' }
  }
  if ($Identity.generation -cnotmatch '^[a-f0-9]{32}$' -or $Identity.commit -cnotmatch '^[a-f0-9]{40}$' -or
      $Identity.packageSha256 -cnotmatch '^[a-f0-9]{64}$' -or $Identity.executableSha256 -cnotmatch '^[a-f0-9]{64}$') { throw 'Invalid update generation identity.' }
}
function Assert-UpdatePendingRecord($Record) {
  if (-not $Record -or ($Record.schema -isnot [int] -and $Record.schema -isnot [long]) -or $Record.schema -ne 1 -or $Record.owner -cne 'SKAGER.UpdateStartup.1' -or
      $Record.transaction -isnot [string] -or $Record.transaction -cnotmatch '^[a-f0-9]{32}$' -or
      ($Record.attempts -isnot [int] -and $Record.attempts -isnot [long]) -or $Record.attempts -notin @(0,1) -or $Record.session -isnot [string] -or
      $Record.session -cnotmatch '^(|[a-f0-9]{32})$' -or
      (($Record.attempts -eq 0) -ne ($Record.session -ceq ''))) { throw 'Unknown or invalid pending update record.' }
  Assert-UpdateIdentity $Record.candidate
  Assert-UpdateIdentity $Record.previous
  if ($Record.candidate.generation -ceq $Record.previous.generation) { throw 'Candidate and recovery generation must differ.' }
}
function New-UpdatePendingRecord($Candidate, $Previous) {
  Assert-UpdateIdentity $Candidate
  Assert-UpdateIdentity $Previous
  $record = [pscustomobject]@{schema=1; owner='SKAGER.UpdateStartup.1'; transaction=[guid]::NewGuid().ToString('N');
    candidate=$Candidate; previous=$Previous; attempts=0; session=''}
  Assert-UpdatePendingRecord $record
  return $record
}
function Assert-UpdateRecordPath([string]$Path) {
  if (-not [IO.Path]::IsPathRooted($Path) -or $Path -match '[\x00-\x1f]' -or $Path.StartsWith('\\')) { throw 'Update record requires an absolute local path.' }
  if ([Environment]::OSVersion.Platform -eq [PlatformID]::Win32NT -and $Path -notmatch '^[A-Za-z]:[\\/]') { throw 'Update record requires an absolute local drive path.' }
  $full = [IO.Path]::GetFullPath($Path)
  $walk = $full
  while ($walk) {
    if (Test-Path -LiteralPath $walk) {
      if ((Get-Item -LiteralPath $walk -Force).Attributes -band [IO.FileAttributes]::ReparsePoint) { throw 'Redirected update record path refused.' }
    }
    $walk = [IO.Path]::GetDirectoryName($walk)
  }
  return $full
}
function Write-UpdatePendingRecord([string]$Path, $Record) {
  Assert-UpdatePendingRecord $Record
  $Path = Assert-UpdateRecordPath $Path
  $temporary = $Path + '.' + [guid]::NewGuid().ToString('N') + '.tmp'
  $encoding = New-Object Text.UTF8Encoding($false)
  $bytes = $encoding.GetBytes(($Record | ConvertTo-Json -Depth 8))
  try {
    $file = New-Object IO.FileStream($temporary, [IO.FileMode]::CreateNew, [IO.FileAccess]::Write, [IO.FileShare]::None)
    try { $file.Write($bytes,0,$bytes.Length); $file.Flush($true) } finally { $file.Dispose() }
    if ([IO.File]::Exists($Path)) { [IO.File]::Replace($temporary,$Path,[Management.Automation.Language.NullString]::Value) }
    else { [IO.File]::Move($temporary,$Path) }
  } finally { if ([IO.File]::Exists($temporary)) { [IO.File]::Delete($temporary) } }
}
function Read-UpdatePendingRecord([string]$Path) {
  $Path = Assert-UpdateRecordPath $Path
  if (-not [IO.File]::Exists($Path)) { return $null }
  if ((Get-Item -LiteralPath $Path).Length -notin 1..4096) { throw 'Invalid pending update record size.' }
  $record = [IO.File]::ReadAllText($Path) | ConvertFrom-Json
  Assert-UpdatePendingRecord $record
  return $record
}
function Resolve-UpdatePendingRecovery($Record, [string]$CurrentGeneration) {
  Assert-UpdatePendingRecord $Record
  # Absence of a live authenticated startup receipt never implies success.
  # A crash before/after publication or before/after launch has one safe result.
  if ($CurrentGeneration -ceq $Record.previous.generation) { return 'retain-previous' }
  if ($CurrentGeneration -ceq $Record.candidate.generation) { return 'restore-previous' }
  return 'manual-recovery'
}
function Initialize-UpdatePipeType {
  if ('Skager.UpdateStartupPipe' -as [type]) { return }
  if ([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT) { throw 'Authenticated startup receipt requires native Windows.' }
  $source = @'
using System;
using System.IO;
using System.IO.Pipes;
using System.Diagnostics;
using System.Security.Cryptography;
using System.Security.Principal;
using System.Runtime.InteropServices;
using System.Text;
using Microsoft.Win32.SafeHandles;
namespace Skager {
 public sealed class UpdateStartupPipe : IDisposable {
  [DllImport("advapi32.dll", CharSet=CharSet.Unicode, SetLastError=true)]
  static extern bool ConvertStringSecurityDescriptorToSecurityDescriptor(string s, uint r, out IntPtr p, out uint n);
  [DllImport("kernel32.dll")] static extern IntPtr LocalFree(IntPtr p);
  [StructLayout(LayoutKind.Sequential)] struct SA { public int length; public IntPtr descriptor; public int inherit; }
  [DllImport("kernel32.dll", CharSet=CharSet.Unicode, SetLastError=true)]
  static extern SafePipeHandle CreateNamedPipe(string name,uint open,uint mode,uint instances,uint output,uint input,uint timeout,ref SA security);
  [DllImport("kernel32.dll", SetLastError=true)]
  static extern bool GetNamedPipeClientProcessId(SafePipeHandle pipe,out uint pid);
  readonly NamedPipeServerStream pipe;
  readonly DateTime created = DateTime.UtcNow;
  bool consumed;
  public UpdateStartupPipe(string name) {
   IntPtr descriptor = IntPtr.Zero; uint size;
   var sid = WindowsIdentity.GetCurrent().User.Value;
   if (!ConvertStringSecurityDescriptorToSecurityDescriptor("D:P(A;;GA;;;"+sid+")",1,out descriptor,out size)) throw new IOException("Cannot restrict startup pipe ACL.");
   try {
    var security = new SA { length=Marshal.SizeOf(typeof(SA)), descriptor=descriptor, inherit=0 };
    // Inbound, overlapped, first instance; byte mode, remote clients refused.
    var handle=CreateNamedPipe(@"\\.\pipe\"+name,0x40080001,8,1,256,256,0,ref security);
    if (handle.IsInvalid) { handle.Dispose(); throw new IOException("Cannot create exclusive local startup pipe."); }
    try { pipe=new NamedPipeServerStream(PipeDirection.In,true,false,handle); }
    catch { handle.Dispose(); throw; }
   } finally { LocalFree(descriptor); }
  }
  static string Hash(string path) {
   using(var file=File.OpenRead(path)) using(var sha=SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(file)).Replace("-","").ToLowerInvariant();
  }
  static bool ExactProcess(Process process,long ticks,string image,string hash) {
   try {
    using(var fresh=Process.GetProcessById(process.Id)) {
     return !process.HasExited && !fresh.HasExited && fresh.StartTime.ToUniversalTime().Ticks==ticks &&
      String.Equals(Path.GetFullPath(fresh.MainModule.FileName),Path.GetFullPath(image),StringComparison.OrdinalIgnoreCase) && Hash(image)==hash;
    }
   } catch { return false; }
  }
  public bool Receive(Process process,string image,string hash,string expected,int timeoutMs) {
   if(consumed) throw new InvalidOperationException("Startup receipt already consumed.");
   consumed=true;
   if(timeoutMs<1 || timeoutMs>180000 || expected.Length>256) throw new ArgumentException("Invalid startup receipt bound.");
   var elapsed=Stopwatch.StartNew();
   try {
    long ticks=process.StartTime.ToUniversalTime().Ticks;
    if(ticks<created.Ticks || !ExactProcess(process,ticks,image,hash)) return false;
    var connect=pipe.WaitForConnectionAsync();
    if(!connect.Wait(timeoutMs)) return false;
    uint client;
    if(!GetNamedPipeClientProcessId(pipe.SafePipeHandle,out client) || client!=(uint)process.Id || !ExactProcess(process,ticks,image,hash)) return false;
    var wanted=Encoding.ASCII.GetBytes(expected);
    var buffer=new byte[257]; int used=0;
    while(used<buffer.Length) {
     int remaining=timeoutMs-(int)elapsed.ElapsedMilliseconds;
     if(remaining<=0) return false;
     var read=pipe.ReadAsync(buffer,used,buffer.Length-used);
     if(!read.Wait(remaining)) return false;
     int count=read.Result;
     if(count==0) break;
     used+=count;
     if(used>wanted.Length) return false;
    }
    if(used!=wanted.Length) return false;
    for(int i=0;i<used;i++) if(buffer[i]!=wanted[i]) return false;
    return elapsed.ElapsedMilliseconds<=timeoutMs && ExactProcess(process,ticks,image,hash);
   } catch { return false; }
   finally { pipe.Dispose(); }
  }
  public void Dispose() { pipe.Dispose(); }
 }
}
'@
  # NSIS may have a native System.dll in its cwd; compile from the framework.
  $previous = [Environment]::CurrentDirectory
  Push-Location -LiteralPath ([Runtime.InteropServices.RuntimeEnvironment]::GetRuntimeDirectory())
  try {
    [Environment]::CurrentDirectory = (Get-Location).Path
    Add-Type -TypeDefinition $source
  } finally { [Environment]::CurrentDirectory = $previous; Pop-Location }
}
function New-UpdateStartupSession($Record) {
  Assert-UpdatePendingRecord $Record
  if ($Record.attempts -ne 0) { throw 'An update candidate gets one startup attempt; recovery never retries it automatically.' }
  Initialize-UpdatePipeType
  $session = [guid]::NewGuid().ToString('N')
  $bytes = New-Object byte[] 32
  $rng = [Security.Cryptography.RandomNumberGenerator]::Create()
  try { $rng.GetBytes($bytes) } finally { $rng.Dispose() }
  $challenge = ([BitConverter]::ToString($bytes)).Replace('-','').ToLowerInvariant()
  $name = 'Skager.Update.' + $session
  $server = New-Object Skager.UpdateStartupPipe($name)
  $Record.attempts = 1
  $Record.session = $session
  # The caller must durably write this modified record before process creation.
  return [pscustomobject]@{server=$server; pipe=$name; challenge=$challenge; session=$session; transaction=$Record.transaction}
}
function Wait-UpdateStartupSuccess($Record, $Session, [Diagnostics.Process]$Process, [string]$ExecutablePath, [int]$TimeoutMilliseconds=90000) {
  Assert-UpdatePendingRecord $Record
  if ($Record.attempts -ne 1 -or $Record.session -cne $Session.session -or $Record.transaction -cne $Session.transaction -or
      $Session.challenge -cnotmatch '^[a-f0-9]{64}$') { throw 'Startup session does not match pending update.' }
  $ExecutablePath = Assert-UpdateRecordPath $ExecutablePath
  $expected = 'SKAGER-UPDATE-READY/1 ' + $Record.candidate.generation + ' ' + $Record.candidate.commit + ' ' + $Session.challenge + "`n"
  return $Session.server.Receive($Process,$ExecutablePath,$Record.candidate.executableSha256,$expected,$TimeoutMilliseconds)
}

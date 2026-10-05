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
    candidate=$Candidate; previous=$Previous; attempts=0; session=''; processId=0; processStartTicks=''}
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
  readonly System.Threading.Tasks.Task connection;
  readonly DateTime created = DateTime.UtcNow;
  bool consumed;
  public string VerifiedFrame { get; private set; }
  public string VerifiedImage { get; private set; }
  public string VerifiedHash { get; private set; }
  public string FailureReason { get; private set; }
  bool Reject(string reason) { FailureReason=reason; return false; }
  public UpdateStartupPipe(string name) {
   IntPtr descriptor = IntPtr.Zero; uint size;
   var sid = WindowsIdentity.GetCurrent().User.Value;
   if (!ConvertStringSecurityDescriptorToSecurityDescriptor("D:P(A;;GA;;;"+sid+")",1,out descriptor,out size)) throw new IOException("Cannot restrict startup pipe ACL.");
   try {
    var security = new SA { length=Marshal.SizeOf(typeof(SA)), descriptor=descriptor, inherit=0 };
    // Inbound, overlapped, first instance; byte mode, remote clients refused.
    var handle=CreateNamedPipe(@"\\.\pipe\"+name,0x40080001,8,1,256,256,0,ref security);
    if (handle.IsInvalid) { handle.Dispose(); throw new IOException("Cannot create exclusive local startup pipe."); }
    try {
     pipe=new NamedPipeServerStream(PipeDirection.In,true,false,handle);
     // Arm ConnectNamedPipe before the caller can launch its child. A fast
     // client may write and close while the caller durably records its PID;
     // starting the connect only in Receive then loses that valid connection.
     connection=pipe.WaitForConnectionAsync();
    }
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
   string stage="process-before-connect";
   try {
    long ticks=process.StartTime.ToUniversalTime().Ticks;
    if(ticks<created.Ticks || !ExactProcess(process,ticks,image,hash)) return Reject(stage);
    stage="connect";
    if(!connection.Wait(timeoutMs)) return Reject("connect-timeout");
    uint client;
    stage="client-identity";
    if(!GetNamedPipeClientProcessId(pipe.SafePipeHandle,out client)) return Reject("client-pid-win32-"+Marshal.GetLastWin32Error());
    if(client!=(uint)process.Id || !ExactProcess(process,ticks,image,hash)) return Reject(stage);
    stage="frame-read";
    var wanted=Encoding.ASCII.GetBytes(expected);
    var buffer=new byte[257]; int used=0;
    while(used<buffer.Length) {
     int remaining=timeoutMs-(int)elapsed.ElapsedMilliseconds;
     if(remaining<=0) return Reject("read-timeout");
     var read=pipe.ReadAsync(buffer,used,buffer.Length-used);
     if(!read.Wait(remaining)) return Reject("read-timeout");
     int count=read.Result;
     if(count==0) break;
     used+=count;
     if(used>wanted.Length) return Reject("frame-oversized");
    }
    if(used!=wanted.Length) return Reject("frame-length");
    for(int i=0;i<used;i++) if(buffer[i]!=wanted[i]) return Reject("frame-mismatch");
    if(elapsed.ElapsedMilliseconds>timeoutMs || !ExactProcess(process,ticks,image,hash)) return Reject("process-after-read");
    VerifiedFrame=expected; VerifiedImage=Path.GetFullPath(image); VerifiedHash=hash;
    return true;
   } catch(Exception error) {
    // Diagnostic category/HRESULT only: never expose frame, challenge or paths.
    error=error.GetBaseException();
    return Reject(stage+":"+error.GetType().Name+":0x"+error.HResult.ToString("X8"));
   } finally { pipe.Dispose(); }
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
function New-UpdateHealthSession($Identity, [string]$Transaction = '') {
  Assert-UpdateIdentity $Identity
  if (-not $Transaction) { $Transaction = [guid]::NewGuid().ToString('N') }
  if ($Transaction -cnotmatch '^[a-f0-9]{32}$') { throw 'Invalid startup transaction.' }
  Initialize-UpdatePipeType
  $session = [guid]::NewGuid().ToString('N')
  $bytes = New-Object byte[] 32
  $rng = [Security.Cryptography.RandomNumberGenerator]::Create()
  try { $rng.GetBytes($bytes) } finally { $rng.Dispose() }
  $challenge = ([BitConverter]::ToString($bytes)).Replace('-','').ToLowerInvariant()
  $name = 'Skager.Update.' + $session
  $server = New-Object Skager.UpdateStartupPipe($name)
  return [pscustomobject]@{server=$server; pipe=$name; challenge=$challenge; session=$session; transaction=$Transaction}
}
function New-UpdateStartupSession($Record) {
  Assert-UpdatePendingRecord $Record
  if ($Record.attempts -ne 0) { throw 'An update candidate gets one startup attempt; recovery never retries it automatically.' }
  $session = New-UpdateHealthSession $Record.candidate $Record.transaction
  $Record.attempts = 1
  $Record.session = $session.session
  # The caller must durably write this modified record before process creation.
  return $session
}
function Get-UpdateReadyFrame($Identity, $Session) {
  Assert-UpdateIdentity $Identity
  if ($Session.challenge -cnotmatch '^[a-f0-9]{64}$' -or $Session.session -cnotmatch '^[a-f0-9]{32}$') { throw 'Invalid startup session.' }
  return 'SKAGER-UPDATE-READY/1 ' + $Identity.generation + ' ' + $Identity.commit + ' ' + $Session.challenge + "`n"
}
function Wait-UpdateGenerationStartupSuccess($Identity, $Session, [Diagnostics.Process]$Process, [string]$ExecutablePath, [int]$TimeoutMilliseconds=90000) {
  $ExecutablePath = Assert-UpdateRecordPath $ExecutablePath
  $expected = Get-UpdateReadyFrame $Identity $Session
  return $Session.server.Receive($Process,$ExecutablePath,$Identity.executableSha256,$expected,$TimeoutMilliseconds)
}
function Wait-UpdateStartupSuccess($Record, $Session, [Diagnostics.Process]$Process, [string]$ExecutablePath, [int]$TimeoutMilliseconds=90000) {
  Assert-UpdatePendingRecord $Record
  if ($Record.attempts -ne 1 -or $Record.session -cne $Session.session -or $Record.transaction -cne $Session.transaction) { throw 'Startup session does not match pending update.' }
  return Wait-UpdateGenerationStartupSuccess $Record.candidate $Session $Process $ExecutablePath $TimeoutMilliseconds
}
function Test-UpdateIdentityEqual($Left, $Right) {
  Assert-UpdateIdentity $Left
  Assert-UpdateIdentity $Right
  return $Left.generation -ceq $Right.generation -and $Left.commit -ceq $Right.commit -and
    $Left.packageSha256 -ceq $Right.packageSha256 -and $Left.executableSha256 -ceq $Right.executableSha256
}
function Write-UpdateReceiptBytes([string]$Path, [byte[]]$Bytes) {
  $Path = Assert-UpdateRecordPath $Path
  $temporary = $Path + '.' + [guid]::NewGuid().ToString('N') + '.tmp'
  try {
    $file = New-Object IO.FileStream($temporary, [IO.FileMode]::CreateNew, [IO.FileAccess]::Write, [IO.FileShare]::None)
    try { $file.Write($Bytes,0,$Bytes.Length); $file.Flush($true) } finally { $file.Dispose() }
    if ([IO.File]::Exists($Path)) { [IO.File]::Replace($temporary,$Path,[Management.Automation.Language.NullString]::Value) }
    else { [IO.File]::Move($temporary,$Path) }
  } finally { if ([IO.File]::Exists($temporary)) { [IO.File]::Delete($temporary) } }
}
function Write-UpdateKnownGoodReceipt([string]$Path, $Identity, $Session, [string]$ExecutablePath) {
  Assert-UpdateIdentity $Identity
  if ([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT) { throw 'Known-good startup proof requires native Windows.' }
  $ExecutablePath = Assert-UpdateRecordPath $ExecutablePath
  if ($Session.server -isnot [Skager.UpdateStartupPipe] -or
      $Session.server.VerifiedFrame -cne (Get-UpdateReadyFrame $Identity $Session) -or
      $Session.server.VerifiedHash -cne $Identity.executableSha256 -or
      -not [string]::Equals($Session.server.VerifiedImage,$ExecutablePath,[StringComparison]::OrdinalIgnoreCase)) { throw 'No authenticated live startup proof for this generation.' }
  $record = [pscustomobject]@{schema=1; owner='SKAGER.KnownGoodStartup.1'; identity=$Identity; session=$Session.session;
    confirmedUtc=[DateTime]::UtcNow.ToString('o'); protocol='SKAGER-UPDATE-READY/1'}
  $bytes = [Text.Encoding]::UTF8.GetBytes(($record | ConvertTo-Json -Depth 6 -Compress))
  Add-Type -AssemblyName System.Security
  $protected = [Security.Cryptography.ProtectedData]::Protect($bytes,[Text.Encoding]::UTF8.GetBytes('SKAGER.KnownGoodStartup.1'),[Security.Cryptography.DataProtectionScope]::CurrentUser)
  Write-UpdateReceiptBytes $Path $protected
}
function Assert-UpdateKnownGoodReceipt([string]$Path, $Identity) {
  Assert-UpdateIdentity $Identity
  $Path = Assert-UpdateRecordPath $Path
  if (-not [IO.File]::Exists($Path) -or (Get-Item -LiteralPath $Path).Length -notin 1..8192) { throw 'No authenticated known-good startup receipt; qualify the current generation first.' }
  if ([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT) { throw 'Known-good startup proof requires native Windows.' }
  Add-Type -AssemblyName System.Security
  try {
    $plain = [Security.Cryptography.ProtectedData]::Unprotect([IO.File]::ReadAllBytes($Path),[Text.Encoding]::UTF8.GetBytes('SKAGER.KnownGoodStartup.1'),[Security.Cryptography.DataProtectionScope]::CurrentUser)
    if ($plain.Length -gt 4096) { throw 'Oversized known-good receipt.' }
    $record = [Text.Encoding]::UTF8.GetString($plain) | ConvertFrom-Json
    if ($record.schema -ne 1 -or $record.owner -cne 'SKAGER.KnownGoodStartup.1' -or $record.protocol -cne 'SKAGER-UPDATE-READY/1' -or
        $record.session -cnotmatch '^[a-f0-9]{32}$' -or -not (Test-UpdateIdentityEqual $record.identity $Identity)) { throw 'Known-good receipt identity mismatch.' }
  } catch { throw ('Known-good startup receipt could not be authenticated: ' + $_.Exception.Message) }
}

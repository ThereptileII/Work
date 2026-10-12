# CI-only routing test. It never runs a broker or OpenCPN: the current image is
# the compiled inert marker_process and skager-start is a marker-only executable.
[CmdletBinding()]
param(
    [Parameter(Mandatory=$true)][string]$Binaries,
    [Parameter(Mandatory=$true)][string]$Evidence,
    [Parameter(Mandatory=$true)][string]$RunnerEnvironment,
    [Parameter(Mandatory=$true)][switch]$ConfirmDisposableHostedRunner
)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
if (-not $ConfirmDisposableHostedRunner -or $RunnerEnvironment -cne 'github-hosted' -or
    $env:GITHUB_ACTIONS -cne 'true' -or $env:RUNNER_OS -cne 'Windows' -or
    $env:GITHUB_REPOSITORY -cne 'ThereptileII/Work' -or
    $env:GITHUB_RUN_ID -cnotmatch '^[1-9][0-9]*$' -or
    $env:GITHUB_SHA -cnotmatch '^[0-9a-f]{40}$' -or
    $PSVersionTable.PSEdition -cne 'Desktop') {
    throw 'Explicit disposable GitHub-hosted Windows PowerShell fixture required'
}
$source = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$workspace = [IO.Path]::GetFullPath($env:GITHUB_WORKSPACE)
if ($source -ine (Join-Path $workspace 'opennav-x')) { throw 'Fixture source must be this checkout' }
$head = (& git -C $workspace rev-parse HEAD).Trim()
if ($LASTEXITCODE -ne 0 -or $head -cne $env:GITHUB_SHA) { throw 'Fixture source is not the exact runner commit' }
$Binaries = [IO.Path]::GetFullPath($Binaries)
$Evidence = [IO.Path]::GetFullPath($Evidence)
$expectedBinaries = [IO.Path]::GetFullPath((Join-Path $source 'build/commissioning-restart/Release'))
if ($Binaries -ine $expectedBinaries -or
    -not $Evidence.StartsWith((Join-Path $source 'build') + '\', [StringComparison]::OrdinalIgnoreCase) -or
    -not [IO.Directory]::Exists($Evidence)) {
    throw 'Retained same-checkout test binaries and existing build evidence directory required'
}

Add-Type -TypeDefinition @'
using System;
using System.ComponentModel;
using System.Runtime.InteropServices;
using System.Security.Cryptography;
using Microsoft.Win32.SafeHandles;
public static class ProtectedRestartFixtureNative {
  const uint Invalid = 0xffffffff;
  [StructLayout(LayoutKind.Sequential)] struct Info {
    public uint Attributes;
    public System.Runtime.InteropServices.ComTypes.FILETIME Created, Accessed, Written;
    public uint Volume, SizeHigh, SizeLow, Links, IndexHigh, IndexLow;
  }
  [DllImport("kernel32.dll", CharSet=CharSet.Unicode, ExactSpelling=true, SetLastError=true)]
  static extern uint GetFileAttributesW(string path);
  [DllImport("kernel32.dll", CharSet=CharSet.Unicode, ExactSpelling=true, SetLastError=true)]
  static extern bool CreateDirectoryW(string path, IntPtr security);
  [DllImport("kernel32.dll", CharSet=CharSet.Unicode, ExactSpelling=true, SetLastError=true)]
  static extern SafeFileHandle CreateFileW(string path, uint access, uint share,
      IntPtr security, uint disposition, uint flags, IntPtr template);
  [DllImport("kernel32.dll", SetLastError=true)]
  static extern bool GetFileInformationByHandle(SafeFileHandle handle, out Info info);
  [StructLayout(LayoutKind.Sequential)] struct FileDisposition { [MarshalAs(UnmanagedType.Bool)] public bool DeleteFile; }
  [DllImport("kernel32.dll", SetLastError=true)]
  static extern bool SetFileInformationByHandle(SafeFileHandle handle, int kind,
      ref FileDisposition disposition, uint size);
  [DllImport("kernel32.dll", SetLastError=true)]
  static extern bool ReadFile(SafeFileHandle handle, byte[] buffer, uint size,
      out uint read, IntPtr overlapped);
  public static bool Absent(string path) {
    if (GetFileAttributesW(path) != Invalid) return false;
    int error = Marshal.GetLastWin32Error();
    if (error == 2 || error == 3) return true;
    throw new Win32Exception(error, "Cannot prove fixture path absent");
  }
  public static void CreateNew(string path) {
    if (!CreateDirectoryW(path, IntPtr.Zero))
      throw new Win32Exception(Marshal.GetLastWin32Error(), "Exclusive fixture directory creation refused");
  }
  static SafeFileHandle Open(string path, bool directory) {
    uint access = directory ? 0x81u : 0x80u;
    uint flags = directory ? 0x02200000u : 0x00200000u;
    var handle = CreateFileW(path, access, 3, IntPtr.Zero, 3, flags, IntPtr.Zero);
    if (handle.IsInvalid) { int error=Marshal.GetLastWin32Error(); handle.Dispose();
      throw new Win32Exception(error, "Fixture identity open refused"); }
    return handle;
  }
  public static SafeFileHandle HoldDirectory(string path) {
    var handle=Open(path,true);
    try { Identity(handle,true); return handle; }
    catch { handle.Dispose(); throw; }
  }
  static string Identity(SafeFileHandle handle, bool directory) {
    Info info;
    if (!GetFileInformationByHandle(handle,out info))
      throw new Win32Exception(Marshal.GetLastWin32Error(), "Fixture identity read refused");
    if ((info.Attributes & 0x400u)!=0 || ((info.Attributes & 0x10u)!=0)!=directory ||
        (!directory && info.Links!=1))
      throw new InvalidOperationException("Fixture path is reparse, wrong kind or linked");
    return string.Format("{0:x8}:{1:x8}:{2:x8}:{3:x8}:{4:x8}",
      info.Volume,info.IndexHigh,info.IndexLow,
      (uint)info.Created.dwHighDateTime,(uint)info.Created.dwLowDateTime);
  }
  public static string HeldIdentity(SafeFileHandle handle) { return Identity(handle,true); }
  public static string PathIdentity(string path, bool directory) {
    using(var handle=Open(path,directory)) return Identity(handle,directory);
  }
  public static SafeFileHandle HoldOwnedFile(string path) {
    var handle=CreateFileW(path,0x80010000u,1,IntPtr.Zero,3,0x00200000u,IntPtr.Zero);
    if (handle.IsInvalid) { int error=Marshal.GetLastWin32Error(); handle.Dispose();
      throw new Win32Exception(error,"Exclusive owned-file delete handle refused"); }
    try { Identity(handle,false); return handle; }
    catch { handle.Dispose(); throw; }
  }
  public static string HeldFileIdentity(SafeFileHandle handle) { return Identity(handle,false); }
  public static string HeldFileDigest(SafeFileHandle handle) {
    Info info;
    if (!GetFileInformationByHandle(handle,out info))
      throw new Win32Exception(Marshal.GetLastWin32Error(),"Owned-file size unavailable");
    ulong remaining=((ulong)info.SizeHigh<<32)|info.SizeLow;
    if (remaining==0 || remaining>32u*1024u*1024u)
      throw new InvalidOperationException("Bounded owned fixture file required");
    using(var hash=SHA256.Create()) {
      var buffer=new byte[65536];
      while (remaining!=0) {
        uint requested=(uint)Math.Min((ulong)buffer.Length,remaining), read;
        if (!ReadFile(handle,buffer,requested,out read,IntPtr.Zero) || read!=requested)
          throw new Win32Exception(Marshal.GetLastWin32Error(),"Owned-file read changed");
        hash.TransformBlock(buffer,0,(int)read,buffer,0);
        remaining-=read;
      }
      uint extra;
      if (!ReadFile(handle,buffer,1,out extra,IntPtr.Zero) || extra!=0)
        throw new InvalidOperationException("Owned fixture file grew during verification");
      hash.TransformFinalBlock(new byte[0],0,0);
      return BitConverter.ToString(hash.Hash).Replace("-","").ToLowerInvariant();
    }
  }
  public static void DeleteHeldFile(SafeFileHandle handle) {
    var disposition=new FileDisposition { DeleteFile=true };
    if (!SetFileInformationByHandle(handle,4,ref disposition,4))
      throw new Win32Exception(Marshal.GetLastWin32Error(),"Exact owned-file disposition refused");
  }
}
'@

function Require([bool]$condition, [string]$message) {
    if (-not $condition) { throw $message }
}
function Digest([string]$path) { return (Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash.ToLowerInvariant() }
function DeleteOwnedFile([string]$path, $record) {
    $handle = [ProtectedRestartFixtureNative]::HoldOwnedFile($path)
    try {
        Require ([ProtectedRestartFixtureNative]::HeldFileIdentity($handle) -ceq $record.id -and
            [ProtectedRestartFixtureNative]::HeldFileDigest($handle) -ceq $record.sha -and
            [ProtectedRestartFixtureNative]::HeldFileIdentity($handle) -ceq $record.id) 'Owned fixture executable changed before deletion'
        [ProtectedRestartFixtureNative]::DeleteHeldFile($handle)
    } finally { $handle.Dispose() }
}
function Probe([string]$path, [string]$argument, [string]$contract) {
    $start = New-Object Diagnostics.ProcessStartInfo
    $start.FileName = $path
    $start.Arguments = $argument
    $start.UseShellExecute = $false
    $start.RedirectStandardOutput = $true
    $process = [Diagnostics.Process]::Start($start)
    try {
        $output = $process.StandardOutput.ReadToEnd()
        Require ($process.WaitForExit(10000) -and $process.ExitCode -eq 0) 'Inert executable probe failed'
        $result = $output | ConvertFrom-Json
        Require ($result.contract -ceq $contract -and $result.profile_accessed -eq $false -and
            $result.marine_code -eq $false) 'Fixture binary is not the expected inert marker'
        if ($contract -ceq 'OpenNavX.ProtectedRestart.LauncherMarker.1') {
            Require ($result.broker -eq $false) 'Fixture launcher contains no broker'
        }
    } finally { $process.Dispose() }
}
function WaitMarker([string]$path) {
    $deadline = [DateTime]::UtcNow.AddSeconds(12)
    while (-not [IO.File]::Exists($path)) {
        if ([DateTime]::UtcNow -ge $deadline) { throw "Closed launcher marker absent: $path" }
        Start-Sleep -Milliseconds 25
    }
}
function AwaitFixtureChildren([string]$helper, [string]$launcher) {
    $deadline = [DateTime]::UtcNow.AddSeconds(12)
    do {
        $remaining = @(
            Get-CimInstance Win32_Process -Filter "Name='opennav-restart.exe' OR Name='skager-start.exe'" |
                Where-Object { $_.ExecutablePath -and
                    ($_.ExecutablePath -ieq $helper -or $_.ExecutablePath -ieq $launcher) }
        )
        if ($remaining.Count -eq 0) { return }
        if ([DateTime]::UtcNow -ge $deadline) { throw 'Inert restart helper or launcher retained after dispatch' }
        Start-Sleep -Milliseconds 50
    } while ($true)
}

$programFiles = [Environment]::GetFolderPath([Environment+SpecialFolder]::ProgramFilesX86)
if (-not $programFiles -or -not [IO.Path]::IsPathRooted($programFiles) -or
    -not [IO.Directory]::Exists($programFiles)) { throw 'Actual ProgramFilesX86 required' }
$root = Join-Path $programFiles 'SKAGER'
if (-not [ProtectedRestartFixtureNative]::Absent($root)) { throw 'Fixed protected namespace already exists; no fixture adoption' }
$sid = [Security.Principal.WindowsIdentity]::GetCurrent().User.Value
if ($sid -cnotmatch '^S-1-5-21-(0|[1-9][0-9]*)-(0|[1-9][0-9]*)-(0|[1-9][0-9]*)-(0|[1-9][0-9]*)$') {
    throw 'Canonical current user SID required'
}
foreach ($key in [Environment]::GetEnvironmentVariables().Keys) {
    if ($key -imatch '^(SKAGER_UPDATE_|OPENNAV_COMMISSIONING_RESTART_(SESSION|RECORD_SHA256)$)') {
        throw 'Fixture inherited a fenced restart environment'
    }
}
$inputs = @{
    'opencpn.exe' = Join-Path $Binaries 'opencpn.exe'
    'opennav-restart.exe' = Join-Path $Binaries 'opennav-restart.exe'
    'skager-start.exe' = Join-Path $Binaries 'protected_launcher_marker.exe'
}
foreach ($binaryInput in $inputs.Values) {
    $item = Get-Item -LiteralPath $binaryInput -ErrorAction Stop
    if ($item.PSIsContainer -or $item.Length -le 0 -or $item.Length -gt 32MB -or
        ($item.Attributes -band [IO.FileAttributes]::ReparsePoint)) {
        throw 'Bounded plain compiled fixture executable required'
    }
}
Probe $inputs['opencpn.exe'] '--marker-self-test' 'OpenNavX.NativeRestart.MarkerOnly.1'
Probe $inputs['skager-start.exe'] '--marker-self-test' 'OpenNavX.ProtectedRestart.LauncherMarker.1'
# The helper's self-test reports role and profile_accessed, but not marine_code.
$helperProbe = & $inputs['opennav-restart.exe'] --commissioning-protocol-self-test | ConvertFrom-Json
Require ($LASTEXITCODE -eq 0 -and $helperProbe.contract -ceq 'OpenNavX.RestartCapability.1' -and
    $helperProbe.role -ceq 'restart-helper' -and
    $helperProbe.profile_accessed -eq $false -and $helperProbe.child_started -eq $false) 'Actual restart helper required'

$generation = [guid]::NewGuid().ToString('N')
$applications = Join-Path $root 'Applications'
$user = Join-Path $applications $sid
$generations = Join-Path $user 'generations'
$version = Join-Path $generations $generation
$app = Join-Path $version 'app'
$directories = @(
    $root,
    $applications,
    $user,
    $generations,
    $version,
    $app
)
$held = New-Object 'Collections.Generic.List[object]'
$files = @{}
$results = New-Object 'Collections.Generic.List[object]'
$report = Join-Path $Evidence 'protected-mode-restart-native.json'
if (-not [ProtectedRestartFixtureNative]::Absent($report)) { throw 'Protected restart evidence already exists; refuse overwrite' }
$completed = $false
try {
    # Recheck at the last point before exclusive creation. Every component is
    # CreateDirectoryW-new and kept open without FILE_SHARE_DELETE.
    Require ([ProtectedRestartFixtureNative]::Absent($root)) 'Fixed protected namespace changed before fixture creation'
    foreach ($directory in $directories) {
        [ProtectedRestartFixtureNative]::CreateNew($directory)
        $handle = [ProtectedRestartFixtureNative]::HoldDirectory($directory)
        $held.Add(@{ path=$directory; handle=$handle; id=[ProtectedRestartFixtureNative]::HeldIdentity($handle) })
    }
    foreach ($name in $inputs.Keys) {
        $target = Join-Path $app $name
        $from = [IO.File]::OpenRead($inputs[$name])
        try {
            $to = [IO.File]::Open($target, [IO.FileMode]::CreateNew, [IO.FileAccess]::Write, [IO.FileShare]::None)
            try { $from.CopyTo($to); $to.Flush($true) } finally { $to.Dispose() }
        } finally { $from.Dispose() }
        $files[$name] = @{ id=[ProtectedRestartFixtureNative]::PathIdentity($target,$false); sha=Digest $target }
        Require ($files[$name].sha -ceq (Digest $inputs[$name])) 'Copied inert fixture differs from compiled source'
    }
    $helper = Join-Path $app 'opennav-restart.exe'
    function RunCase([string]$name, [string]$mode, [string]$expected, [bool]$allow, [bool]$fenced) {
        $case = Join-Path $Evidence ("protected-restart-$generation-$name")
        [ProtectedRestartFixtureNative]::CreateNew($case)
        [IO.File]::WriteAllText((Join-Path $case 'target-mode.txt'), $mode)
        if (-not $allow) { [IO.File]::WriteAllText((Join-Path $case 'parent-fast-exit.txt'), '') }
        $start = New-Object Diagnostics.ProcessStartInfo
        $start.FileName = Join-Path $app 'opencpn.exe'
        $start.Arguments = '--parent'
        $start.WorkingDirectory = $case
        $start.UseShellExecute = $false
        if ($fenced) { $start.EnvironmentVariables['SKAGER_UPDATE_QUALIFY_PROFILE'] = '1' }
        $parent = [Diagnostics.Process]::Start($start)
        try {
            $receipt = Join-Path $case 'parent-armed.txt'
            WaitMarker $receipt
            $lines = [IO.File]::ReadAllLines($receipt)
            Require ($lines.Count -eq 2 -and $lines[0] -ceq $(if($allow){'yes'}else{'no'}) -and
                $lines[1] -ceq [string]$parent.Id) "$name`: actual restart decision differs"
            $marker = Join-Path $case 'protected-launcher-args.txt'
            if ($allow) {
                # Keep the original image alive long enough to observe the
                # helper's wait, rather than relying on a scheduling race.
                Start-Sleep -Milliseconds 600
                Require (-not $parent.HasExited -and -not [IO.File]::Exists($marker)) "$name`: launcher started before original parent release"
                [IO.File]::WriteAllText((Join-Path $case 'parent-release.txt'), '')
            }
            Require ($parent.WaitForExit(15000) -and $parent.ExitCode -eq 0) "$name`: inert parent did not exit normally"
            if ($allow) {
                WaitMarker $marker
                $arguments = [IO.File]::ReadAllLines($marker)
                Require ($arguments.Count -eq 3 -and $arguments[0] -ceq '2' -and
                    $arguments[1] -ceq $expected) "$name`: helper did not deliver one closed broker-current argument"
                $created = [UInt64]::Parse($arguments[2], [Globalization.NumberStyles]::None)
                $parentExit = [UInt64]$parent.ExitTime.ToUniversalTime().ToFileTimeUtc()
                Require ($created -ge $parentExit) "$name`: launcher started before original parent exit"
            } else {
                Start-Sleep -Milliseconds 600
                Require (-not [IO.File]::Exists($marker)) "$name`: refused restart launched the marker"
            }
            AwaitFixtureChildren $helper (Join-Path $app 'skager-start.exe')
            if (-not $allow) {
                Require (-not [IO.File]::Exists($marker)) "$name`: refused restart eventually launched the marker"
            }
            $results.Add(@{case=$name;mode=$mode;armed=$allow;argument=$(if($allow){$expected}else{$null});parentPid=$parent.Id})
        } finally { $parent.Dispose() }
    }
    RunCase 'xnav' '--xnav' "--broker-current=$generation" $true $false
    RunCase 'legacy' '--legacy' "--broker-current=$generation`:legacy" $true $false
    RunCase 'safe' '--safe-mode' "--broker-current=$generation`:safe" $true $false
    RunCase 'fenced-environment' '--xnav' '' $false $true
    $launcher = Join-Path $app 'skager-start.exe'
    Require ([ProtectedRestartFixtureNative]::PathIdentity($launcher,$false) -ceq $files['skager-start.exe'].id -and
        (Digest $launcher) -ceq $files['skager-start.exe'].sha) 'Owned launcher changed before missing-companion case'
    DeleteOwnedFile $launcher $files['skager-start.exe']
    $files.Remove('skager-start.exe')
    RunCase 'missing-companion' '--xnav' '' $false $false

    # Success cleanup is exact and nonrecursive. Directory handles, original
    # identities, expected children and file hashes must still match.
    foreach ($entry in $held) {
        Require ([ProtectedRestartFixtureNative]::HeldIdentity($entry.handle) -ceq $entry.id -and
            [ProtectedRestartFixtureNative]::PathIdentity($entry.path,$true) -ceq $entry.id) 'Held fixture directory changed before cleanup'
    }
    foreach ($name in @('opencpn.exe','opennav-restart.exe')) {
        $path = Join-Path $app $name
        Require ([ProtectedRestartFixtureNative]::PathIdentity($path,$false) -ceq $files[$name].id -and
            (Digest $path) -ceq $files[$name].sha) 'Owned fixture executable changed before cleanup'
    }
    $expectedChildren = @{}
    for ($index=0; $index -lt $directories.Count-1; $index++) {
        $expectedChildren[$directories[$index]] = @([IO.Path]::GetFileName($directories[$index+1]))
    }
    $expectedChildren[$app] = @('opencpn.exe','opennav-restart.exe')
    foreach ($directory in $directories) {
        $actual = @([IO.Directory]::EnumerateFileSystemEntries($directory) | ForEach-Object { [IO.Path]::GetFileName($_) } | Sort-Object)
        $expected = @($expectedChildren[$directory] | Sort-Object)
        Require (@(Compare-Object -ReferenceObject $actual -DifferenceObject $expected).Count -eq 0) 'Unknown child in fixed fixture namespace; refuse cleanup'
    }
    [IO.File]::WriteAllText($report, (@{schema=1;status='routing-validated';generation=$generation;cases=$results.ToArray()} | ConvertTo-Json -Depth 6))
    foreach ($name in @('opencpn.exe','opennav-restart.exe')) {
        DeleteOwnedFile (Join-Path $app $name) $files[$name]
    }
    for ($index=$directories.Count-1; $index -ge 0; $index--) {
        $entry=$held[$index]
        Require ([ProtectedRestartFixtureNative]::HeldIdentity($entry.handle) -ceq $entry.id) 'Held fixture directory drifted during cleanup'
        Require (@([IO.Directory]::EnumerateFileSystemEntries($entry.path)).Count -eq 0) 'Fixture directory not empty; refuse cleanup'
        $entry.handle.Dispose()
        [IO.Directory]::Delete($entry.path,$false)
    }
    $completed = $true
    [IO.File]::WriteAllText($report, (@{schema=1;status='passed';generation=$generation;cases=$results.ToArray();fixedNamespaceRemoved=$true} | ConvertTo-Json -Depth 6))
    Write-Host 'Protected restart helper routed all three modes and refused missing companion/fenced environment.'
} catch {
    [IO.File]::WriteAllText($report, (@{schema=1;status='failed';generation=$generation;reason=$_.Exception.Message;fixedNamespaceRetained=(-not $completed);cases=$results.ToArray()} | ConvertTo-Json -Depth 6))
    throw
} finally {
    foreach ($entry in $held) { if (-not $entry.handle.IsClosed) { $entry.handle.Dispose() } }
}

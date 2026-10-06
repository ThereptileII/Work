# Windows PowerShell 5.1. No elevation, network access, or writes to OpenCPN/profile.
[CmdletBinding()]
param(
  [ValidateSet('Preflight','Install','Repair','Update','Rollback','Uninstall','Diagnostics')]
  [string]$Action = 'Preflight',
  [string]$OpenCpn = '',
  [string]$PackageDirectory = '',
  [string]$ManifestSha256 = '',
  [string]$Report = '',
  [string]$FailurePoint = '',
  [ValidatePattern('^(|xnav(?:,legacy)?(?:,safe)?)$')]
  [string]$ShortcutModes = '',
  [string]$SummaryPath = '',
  [switch]$SupervisedUpdate,
  [ValidatePattern('^(|[a-f0-9]{32})$')][string]$UpdateTransaction = ''
)
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$Root = Join-Path ([Environment]::GetFolderPath('LocalApplicationData')) 'OpenNavXAlpha1'
$Registry = 'HKCU:\Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenNavXAlpha1'
$Programs = [Environment]::GetFolderPath('Programs')
$Owner = 'OpenNavX.Alpha1.SideBySide.1'
$Utf8 = New-Object System.Text.UTF8Encoding($false)
$SessionLog = New-Object System.Collections.Generic.List[string]
$TransactionLock = $null
$OwnsRoot = $false
$LastStartupHealth = 0
$RecoveredSupervised = $false
. (Join-Path $PSScriptRoot 'UpdateSupervisor.ps1')
if ($SupervisedUpdate -and $Action -ne 'Update') { throw 'Supervised startup is only valid for an update.' }
if ($UpdateTransaction -and $Action -ne 'Rollback') { throw 'Update recovery identity is only valid for rollback.' }

function Log([string]$Message) {
  $line = [DateTime]::UtcNow.ToString('o') + ' ' + $Message
  $SessionLog.Add($line)
  Write-Host $line
}
function Hash([string]$Path) {
  # NSIS can launch Windows PowerShell with an inherited module search path
  # where Get-FileHash is unavailable. Integrity must not depend on that module.
  $algorithm = [Security.Cryptography.SHA256]::Create()
  $stream = $null
  try {
    $stream = [IO.File]::OpenRead($Path)
    return ([BitConverter]::ToString($algorithm.ComputeHash($stream))).Replace('-','').ToLowerInvariant()
  } finally { if ($stream) { $stream.Dispose() }; $algorithm.Dispose() }
}
function PlainPath([string]$Path) {
  if ([string]::IsNullOrWhiteSpace($Path) -or $Path -notmatch '^[A-Za-z]:[\\/]' -or $Path.Contains('"') -or $Path -match '[\x00-\x1f]') {
    throw 'Use an absolute path on a local Windows drive.'
  }
  $full = [IO.Path]::GetFullPath($Path).TrimEnd('\')
  if ($full.Length -lt 4) { throw 'A drive root cannot be an installation location.' }
  $walk = $full
  while ($walk -and $walk.Length -gt 3) {
    if (Test-Path -LiteralPath $walk) {
      if ((Get-Item -LiteralPath $walk -Force).Attributes -band [IO.FileAttributes]::ReparsePoint) {
        throw "Redirected/reparse path refused: $walk"
      }
    }
    $walk = [IO.Path]::GetDirectoryName($walk)
  }
  return $full
}
function RelativePath([string]$Base, [string]$Name) {
  if ($Name -notmatch '^[A-Za-z0-9_ .()&@/+-]+$' -or $Name.Contains('\') -or
      $Name.StartsWith('/') -or $Name -match '(^|/)\.{1,2}(/|$)' -or
      $Name -match '(^|/)(CON|PRN|AUX|NUL|COM[0-9]|LPT[0-9])(\.|/|$)' -or
      $Name -match '[. ](/|$)' -or $Name.Contains('//')) { throw "Unsafe package path: $Name" }
  $result = [IO.Path]::GetFullPath((Join-Path $Base $Name))
  if (-not $result.StartsWith($Base.TrimEnd('\') + '\', [StringComparison]::OrdinalIgnoreCase)) { throw 'Path escaped its owner directory.' }
  return PlainPath $result
}
function ReadJson([string]$Path, [long]$Limit = 4194304) {
  $null = PlainPath $Path
  $f = Get-Item -LiteralPath $Path
  if ($f.Length -gt $Limit -or $f.Length -eq 0) { throw "Invalid JSON record size: $Path" }
  return [IO.File]::ReadAllText($Path, $Utf8) | ConvertFrom-Json
}
function Assert-StatusOnlyOutput($Result) {
  if (-not $Result -or -not $Result.PSObject.Properties['xnav_hardware_output_policy'] -or
      $Result.xnav_hardware_output_policy -isnot [string] -or $Result.xnav_hardware_output_policy -cne 'status-only') {
    throw 'Unqualified SKAGER equipment-output build refused. A status-only product is required.'
  }
}
function Assert-ProductOutputPolicy($Result) {
  if ($Result -and $Result.PSObject.Properties['xnav_hardware_output_policy'] -and
      $Result.xnav_hardware_output_policy -is [string] -and $Result.xnav_hardware_output_policy -ceq 'manual-commissioning') {
    if ($Result.PSObject.Properties['xnav_manual_control_contract'] -and
        $Result.xnav_manual_control_contract -is [int] -and $Result.xnav_manual_control_contract -eq 1) { return }
    throw 'Manual commissioning requires the exact versioned control contract.'
  }
  Assert-StatusOnlyOutput $Result
  if ($Result.PSObject.Properties['xnav_manual_control_contract'] -and
      ($Result.xnav_manual_control_contract -isnot [int] -or $Result.xnav_manual_control_contract -ne 0)) {
    throw 'Status-only product declares a contradictory manual control contract.'
  }
}
function Assert-InstalledProduct($Result) {
  if (-not $Result -or -not $Result.PSObject.Properties['test_fixtures'] -or
      $Result.test_fixtures -isnot [bool] -or $Result.test_fixtures -ne $false -or
      -not $Result.PSObject.Properties['build_purpose'] -or
      $Result.build_purpose -isnot [string] -or $Result.build_purpose -cne 'INSTALLED PRODUCT') {
    throw 'Developer/test-fixture executable refused in the installed Beta 2 product.'
  }
}
function Resolve-OutputPolicy($Result, [bool]$RecordedRecovery = $false) {
  if ($RecordedRecovery -and $Result -and -not $Result.PSObject.Properties['xnav_hardware_output_policy']) {
    return 'historical-unqualified'
  }
  Assert-ProductOutputPolicy $Result
  return $Result.xnav_hardware_output_policy
}
function Test-ExactRepairPackage($Previous, $Package, [string]$ManifestHash) {
  return $Previous -and $Package -and $ManifestHash -cmatch '^[a-f0-9]{64}$' -and
    $Previous.packageSha256 -ceq $ManifestHash -and $Previous.commit -ceq $Package.commit -and
    $Previous.version -ceq $Package.version
}
function AtomicJson([string]$Path, $Value) {
  $null = PlainPath $Path
  $temp = $Path + '.' + [guid]::NewGuid().ToString('N') + '.tmp'
  $bytes = $Utf8.GetBytes(($Value | ConvertTo-Json -Depth 16))
  try {
    $file = New-Object IO.FileStream($temp, [IO.FileMode]::CreateNew, [IO.FileAccess]::Write, [IO.FileShare]::None)
    try { $file.Write($bytes, 0, $bytes.Length); $file.Flush($true) } finally { $file.Dispose() }
    # Windows PowerShell 5.1 converts $null to an empty string for this .NET
    # string parameter; File.Replace rejects that as an invalid backup path.
    if (Test-Path -LiteralPath $Path) { [IO.File]::Replace($temp, $Path, [System.Management.Automation.Language.NullString]::Value) }
    else { [IO.File]::Move($temp, $Path) }
  } finally {
    # Failed replacement (lock/ACL/full disk) leaves the last durable state
    # intact. Remove only this transaction's unique temporary record.
    if ([IO.File]::Exists($temp)) { [IO.File]::Delete($temp) }
  }
}
function Generation([string]$Id) {
  if ($Id -notmatch '^[a-f0-9]{32}$') { throw 'Invalid generation identity.' }
  return Join-Path $Root ('generations\' + $Id)
}
function ReadState {
  if (-not (Test-Path -LiteralPath (Join-Path $Root 'state.json'))) { return $null }
  $state = ReadJson (Join-Path $Root 'state.json')
  if ($state.owner -ne $Owner -or $state.schema -ne 1) { throw 'Unknown installation state; no changes made.' }
  $null = Generation $state.current
  if ($state.previous) { $null = Generation $state.previous }
  $null = PlainPath $state.stock.path
  return $state
}
function PeArchitecture([string]$Path) {
  $f = [IO.File]::OpenRead($Path)
  $r = New-Object IO.BinaryReader($f)
  try {
    if ($f.Length -lt 128 -or $r.ReadUInt16() -ne 0x5a4d) { throw 'Not a Windows executable.' }
    $f.Position = 60; $offset = $r.ReadUInt32()
    if ($offset -gt $f.Length - 24) { throw 'Invalid PE header.' }
    $f.Position = $offset
    if ($r.ReadUInt32() -ne 0x4550) { throw 'Invalid PE signature.' }
    if ($r.ReadUInt16() -ne 0x14c) { throw 'This integration requires the supported x86 OpenCPN plugin ABI.' }
    return 'x86'
  } finally { $r.Dispose(); $f.Dispose() }
}
function StockInfo([string]$Path, $Allowed) {
  $Path = PlainPath $Path
  if ([IO.Path]::GetFileName($Path) -ine 'opencpn.exe') { throw 'Select the original installed opencpn.exe.' }
  $architecture = PeArchitecture $Path
  $hash = Hash $Path
  $version = [Diagnostics.FileVersionInfo]::GetVersionInfo($Path)
  $match = @($Allowed | Where-Object { $_.executableSha256 -ceq $hash -and $_.arch -eq $architecture })
  if ($match.Count -ne 1) { throw "Unsupported OpenCPN executable. SHA-256: $hash. No application or profile files modified." }
  if ($match[0].version -ne '5.12.4' -or $match[0].upstreamCommit -ne '37fd0cddb7334fe489e9f18aa163977a9c5c84f7') { throw 'Unsupported baseline in manifest.' }
  if ($version.FileMajorPart -ne 5 -or $version.FileMinorPart -ne 12 -or $version.FileBuildPart -ne 4) { throw 'Version resource and compatibility manifest disagree.' }
  if ($Utf8.GetByteCount($Path) -gt 4096) { throw 'Stock resource locator exceeds its application bound.' }
  foreach ($relative in @('tcdata/harmonics-dwf-20210110-free.tcd','tcdata/HARMONICS_NO_US.IDX','tcdata/HARMONICS_NO_US','gshhs/poly-c-1.dat','basemap_shp/basemap_low.shp','sounds/2bells.wav')) {
    $resource = RelativePath ([IO.Path]::GetDirectoryName($Path)) $relative
    if (-not [IO.File]::Exists($resource) -or (Get-Item -LiteralPath $resource).Length -eq 0) { throw "Original OpenCPN resource missing; repair OpenCPN first: $relative" }
  }
  return [pscustomobject]@{path=$Path; sha256=$hash; version='5.12.4'; arch=$architecture}
}
function DiscoverStock {
  $paths = New-Object System.Collections.Generic.List[string]
  foreach ($view in @([Microsoft.Win32.RegistryView]::Registry32, [Microsoft.Win32.RegistryView]::Registry64)) {
    $base = [Microsoft.Win32.RegistryKey]::OpenBaseKey([Microsoft.Win32.RegistryHive]::LocalMachine, $view)
    try {
      $uninstall = $base.OpenSubKey('Software\Microsoft\Windows\CurrentVersion\Uninstall')
      if ($uninstall) {
        try {
          # Stock keys can include the full build suffix (5.12.4-0+37fd0cd).
          # These locations are discovery hints; StockInfo still requires the
          # exact accepted executable hash, PE ABI and version resource.
          foreach ($name in $uninstall.GetSubKeyNames()) {
            if ($name -notmatch '^OpenCPN(?: |$)') { continue }
            $key = $uninstall.OpenSubKey($name)
            if ($key) { try { $location = [string]$key.GetValue('InstallLocation'); if ($location) { $paths.Add((Join-Path $location 'opencpn.exe')) } } finally { $key.Dispose() } }
          }
        } finally { $uninstall.Dispose() }
      }
    } finally { $base.Dispose() }
  }
  $paths.Add((Join-Path ${env:ProgramFiles(x86)} 'OpenCPN\opencpn.exe'))
  return @($paths | Select-Object -Unique | Where-Object { Test-Path -LiteralPath $_ })
}
function AssertClosed {
  if (@(Get-Process -Name opencpn -ErrorAction SilentlyContinue).Count) { throw 'Close OpenCPN, SKAGER, Legacy and Safe Mode before changing the installation.' }
}
function FileRecords([string]$Directory) {
  $items = @(Get-ChildItem -LiteralPath $Directory -Recurse -Force)
  foreach ($f in $items) {
    if ($f.Attributes -band [IO.FileAttributes]::ReparsePoint) { throw "Redirected file refused: $($f.FullName)" }
    if (-not $f.PSIsContainer) {
      $relative = $f.FullName.Substring($Directory.Length + 1).Replace('\','/')
      $null = RelativePath $Directory $relative
      [pscustomobject]@{path=$relative; sha256=(Hash $f.FullName)}
    }
  }
}
function VerifyFiles([string]$Directory, $Files) {
  $seen = @{}
  foreach ($f in $Files) {
    $path = RelativePath $Directory $f.path
    if ($f.sha256 -cnotmatch '^[a-f0-9]{64}$' -or $seen.ContainsKey($f.path)) { throw 'Invalid/duplicate owned file record.' }
    $seen[$f.path] = $true
    if (-not [IO.File]::Exists($path) -or (Hash $path) -cne $f.sha256) { throw "Missing or corrupt owned file: $($f.path)" }
  }
}
function ReadGeneration([string]$Id) {
  $directory = Generation $Id
  $manifest = ReadJson (Join-Path $directory 'ownership.json')
  if ($manifest.owner -ne $Owner) { throw 'Unknown generation ownership.' }
  # In particular, reject an unknown rollback target before publishing state.
  $null = ShortcutGroup $manifest
  return $manifest
}
function SelfTest([string]$Directory, [string]$Commit, [string]$Version, [bool]$RecordedRecovery = $false) {
  $reportPath = Join-Path $Directory ('loader-' + [guid]::NewGuid().ToString('N') + '.json')
  $exe = Join-Path $Directory 'app\opencpn.exe'
  $null = PeArchitecture $exe
  if (-not ('OpenNav.InstallerErrorMode' -as [type])) {
    # Windows PowerShell 5.1 resolves its implicit System.dll compiler
    # reference against the current directory. NSIS also has a native plugin
    # with that name. Compile only from the installed framework directory.
    $framework = [Runtime.InteropServices.RuntimeEnvironment]::GetRuntimeDirectory()
    $previousDirectory = [Environment]::CurrentDirectory
    Push-Location -LiteralPath $framework
    try {
      [Environment]::CurrentDirectory = $framework
      Add-Type -TypeDefinition @'
using System.Runtime.InteropServices;
namespace OpenNav {
  public static class InstallerErrorMode {
    [DllImport("kernel32.dll")] public static extern uint GetErrorMode();
    [DllImport("kernel32.dll")] public static extern uint SetErrorMode(uint mode);
  }
}
'@
    } finally {
      [Environment]::CurrentDirectory = $previousDirectory
      Pop-Location
    }
  }
  # A missing import can fail before our executable's self-test code runs.
  # Launch directly with an inherited noninteractive error mode: a shell
  # launch or an outer test runner's error mode is not this child's contract.
  # Scope the process-wide setting to creation in this private installer host.
  $start = New-Object Diagnostics.ProcessStartInfo
  $start.FileName = $exe
  $start.Arguments = '--opennav-self-test "' + $reportPath + '"'
  $start.WorkingDirectory = Join-Path $Directory 'app'
  $start.UseShellExecute = $false
  $start.CreateNoWindow = $true
  $process = $null
  $oldMode = [OpenNav.InstallerErrorMode]::GetErrorMode()
  try {
    $null = [OpenNav.InstallerErrorMode]::SetErrorMode($oldMode -bor 0x8003)
    $process = [Diagnostics.Process]::Start($start)
  } finally { $null = [OpenNav.InstallerErrorMode]::SetErrorMode($oldMode) }
  try {
    if (-not $process.WaitForExit(30000)) {
      $process.Kill()
      $null = $process.WaitForExit(5000)
      throw 'Staged executable loader self-test timed out.'
    }
    if ($process.ExitCode -ne 0) { throw "Staged executable self-test failed: $($process.ExitCode)" }
  } finally { if ($process) { $process.Dispose() } }
  $result = ReadJson $reportPath
  $script:LastStartupHealth = 0
  if ($result.PSObject.Properties['update_startup_health'] -and $result.update_startup_health -is [int] -and $result.update_startup_health -eq 1) { $script:LastStartupHealth = 1 }
  if (-not $result.passed -or $result.commit -cne $Commit -or $result.version -cne $Version -or $result.profile_initialized -or $result.plugins_loaded) { throw 'Executable identity/self-test report mismatch.' }
  # Keep fixture rejection independently observable even when the same test
  # executable also declares a disallowed loopback output policy. Both checks
  # precede profile access and any generation publication.
  if ($Version -match '^0\.4\.') { Assert-InstalledProduct $result }
  # Recovery is allowed only by explicit callers which verified a recorded
  # generation/package. No version string qualifies a new install/update.
  $outputPolicy = Resolve-OutputPolicy $result $RecordedRecovery
  if ($outputPolicy -eq 'historical-unqualified') {
    Log 'Historical recovery only: this generation has no qualified SKAGER equipment-output policy. It is not a public-beta candidate.'
  }
  if ($result.PSObject.Properties['normal_config_directory']) {
    $profile = PlainPath $result.normal_config_directory
    if ([IO.Directory]::Exists($profile)) {
      $null = [IO.Directory]::GetFileSystemEntries($profile)
      $ini = Join-Path $profile 'opencpn.ini'
      if ([IO.File]::Exists($ini)) { $stream = [IO.File]::OpenRead($ini); $stream.Dispose() }
    }
    Log 'Normal OpenCPN profile location accessible; profile content not modified.'
  }
  Remove-Item -LiteralPath $reportPath
  Log "Loader/resource self-test passed for $Commit"
  return $outputPolicy
}
function ShellGroups {
  # The historical group belongs to immutable older maintenance engines,
  # including early 0.4 Beta 2 builds. Version alone does not identify layout.
  # Keep those engines usable after rollback; never rewrite their owned files.
  return @((Join-Path $Programs 'SKAGER'), (Join-Path $Programs 'OpenNav X'),
    (Join-Path $Programs 'OpenNav X Alpha 1'))
}
function ShortcutGroup($Generation) {
  if (-not $Generation.PSObject.Properties['shellLayout']) {
    return (Join-Path $Programs 'OpenNav X Alpha 1')
  }
  if ($Generation.shellLayout -isnot [string]) { throw 'Unknown generation Start-menu layout; preserve and inspect it.' }
  if ($Generation.shellLayout -ceq 'OpenNavX.NeutralStartMenu.1') { return (Join-Path $Programs 'OpenNav X') }
  if ($Generation.shellLayout -ceq 'OpenNavX.SkagerStartMenu.1') { return (Join-Path $Programs 'SKAGER') }
  throw 'Unknown generation Start-menu layout; preserve and inspect it.'
}
function ShortcutNames([string]$Group) {
  if ($Group -ieq (Join-Path $Programs 'SKAGER')) {
    return @('Skager.lnk','OpenCPN Legacy.lnk','Skager Safe Mode.lnk','Maintain Skager.lnk')
  }
  if ($Group -ieq (Join-Path $Programs 'OpenNav X') -or
      $Group -ieq (Join-Path $Programs 'OpenNav X Alpha 1')) {
    return @('OpenNav X.lnk','OpenCPN Legacy.lnk','OpenNav Safe Mode.lnk','Maintain OpenNav.lnk')
  }
  throw 'Unknown shortcut group; preserve and inspect it.'
}
function ShortcutSpec([string]$Name, $GenerationRecord = $null) {
  if ($Name -ceq 'Skager.lnk' -and $GenerationRecord -and $GenerationRecord.PSObject.Properties['updateStartupHealth'] -and $GenerationRecord.updateStartupHealth -eq 1) {
    return @{target='app/skager-start.exe'; arguments='--xnav'; work='app'; mode='xnav'}
  }
  switch -CaseSensitive ($Name) {
    'OpenNav X.lnk'        { return @{target='app/opencpn.exe'; arguments='--xnav'; work='app'; mode='xnav'} }
    'Skager.lnk'           { return @{target='app/opencpn.exe'; arguments='--xnav'; work='app'; mode='xnav'} }
    'OpenCPN Legacy.lnk'   { return @{target='app/opencpn.exe'; arguments='--legacy'; work='app'; mode='legacy'} }
    'OpenNav Safe Mode.lnk' { return @{target='app/opencpn.exe'; arguments='--safe-mode'; work='app'; mode='safe'} }
    'Skager Safe Mode.lnk' { return @{target='app/opencpn.exe'; arguments='--safe-mode'; work='app'; mode='safe'} }
    'Maintain OpenNav.lnk' { return @{target='Maintain.exe'; arguments=''; work=''; mode='maintenance'} }
    'Maintain Skager.lnk'  { return @{target='Maintain.exe'; arguments=''; work=''; mode='maintenance'} }
    default { throw 'Unknown item in SKAGER/retained shortcut folder; preserve and inspect it.' }
  }
}
function AssertShortcut([string]$Path, $Shell) {
  $null = PlainPath $Path
  $item = Get-Item -LiteralPath $Path -Force
  if ($item.PSIsContainer) { throw 'Directory in SKAGER/retained shortcut folder; preserve and inspect it.' }
  if ($item.Name -cnotin @(ShortcutNames $item.DirectoryName)) {
    throw 'Shortcut name does not match its generation layout; preserve and inspect it.'
  }
  $spec = ShortcutSpec $item.Name
  $link = $Shell.CreateShortcut($Path)
  $target = PlainPath ([string]$link.TargetPath)
  $base = (PlainPath (Join-Path $Root 'generations')) + '\'
  if (-not $target.StartsWith($base, [StringComparison]::OrdinalIgnoreCase)) { throw 'Shortcut does not target a SKAGER-owned generation.' }
  $relative = $target.Substring($base.Length).Replace('\','/')
  if ($relative -cnotmatch '^([a-f0-9]{32})/(.+)$') { throw 'Unexpected SKAGER shortcut target.' }
  $id = $Matches[1]; $targetRelative = $Matches[2]
  $record = ReadGeneration $id
  $spec = ShortcutSpec $item.Name $record
  if ($targetRelative -cne $spec.target) { throw 'Unexpected SKAGER shortcut target.' }
  $owned = @($record.managedFiles | Where-Object { $_.path -ceq $spec.target -and $_.sha256 -cmatch '^[a-f0-9]{64}$' })
  if ($owned.Count -ne 1) { throw 'Shortcut target lacks unique generation ownership.' }
  $directory = Generation $id
  $work = $directory; if ($spec.work) { $work = Join-Path $directory $spec.work }
  if ([string]$link.Arguments -cne $spec.arguments -or
      -not [string]::Equals((PlainPath ([string]$link.WorkingDirectory)), $work, [StringComparison]::OrdinalIgnoreCase)) {
    throw 'Modified SKAGER shortcut arguments or working directory; preserve and inspect it.'
  }
  # Missing/corrupt owned binaries remain repairable. The immutable ownership
  # record, exact link target and invocation identify this shortcut, not the
  # current content of a file which Repair is specifically intended to restore.
}
function AssertShellOwnership {
  $shell = New-Object -ComObject WScript.Shell
  foreach ($group in @(ShellGroups)) {
    $null = PlainPath $group
    if (-not (Test-Path -LiteralPath $group)) { continue }
    if (-not [IO.Directory]::Exists($group)) { throw 'SKAGER/retained shortcut group is not a directory.' }
    $ownerPath = Join-Path $Root 'owner.json'
    if (-not [IO.File]::Exists($ownerPath) -or (ReadJson $ownerPath).owner -cne $Owner) {
      throw 'Existing shortcut directory has no verified SKAGER owner; preserve and inspect it.'
    }
    foreach ($file in Get-ChildItem -LiteralPath $group -Force) { AssertShortcut $file.FullName $shell }
  }
  if (Test-Path -LiteralPath $Registry) {
    if ((Get-ItemProperty -LiteralPath $Registry).OpenNavOwner -cne $Owner) { throw 'Unknown uninstall registry ownership.' }
  }
}
function RemoveShortcutGroup([string]$Group) {
  if (-not (Test-Path -LiteralPath $Group)) { return }
  $null = PlainPath $Group
  $shell = New-Object -ComObject WScript.Shell
  $files = @(Get-ChildItem -LiteralPath $Group -Force)
  foreach ($file in $files) { AssertShortcut $file.FullName $shell }
  foreach ($file in $files) {
    AssertShortcut $file.FullName $shell
    Remove-Item -LiteralPath $file.FullName
  }
  if (@(Get-ChildItem -LiteralPath $Group -Force).Count -eq 0) { Remove-Item -LiteralPath $Group }
}
function PublishShell($State) {
  AssertShellOwnership
  $directory = Generation $State.current
  $generation = ReadGeneration $State.current
  $group = ShortcutGroup $generation
  $skager = $group -ieq (Join-Path $Programs 'SKAGER')
  $caption = if ($skager) { 'SKAGER' } else { 'OpenNav X' }
  if (-not $skager) {
    if ($generation.version -match '^0\.4\.') { $caption = 'OpenNav X Beta 2' }
    elseif ($generation.version -match '^0\.3\.') { $caption = 'OpenNav X Beta 1' }
    elseif ($generation.version -match '^0\.2\.') { $caption = 'OpenNav X Alpha 1' }
    # An immutable old maintainer cannot see SKAGER. Do not expose its
    # maintenance shortcut until the new-only group is completely gone.
    RemoveShortcutGroup (Join-Path $Programs 'SKAGER')
  }
  $null = New-Item -ItemType Directory -Path $group -Force
  $shell = New-Object -ComObject WScript.Shell
  $selected = @('xnav','legacy','safe')
  if ($State.PSObject.Properties['shortcutModes']) { $selected = @($State.shortcutModes) }
  elseif ($State -is [Collections.IDictionary] -and $State.Contains('shortcutModes')) { $selected = @($State.shortcutModes) }
  foreach ($name in @(ShortcutNames $group)) {
    $spec = ShortcutSpec $name $generation
    $path = Join-Path $group $name
    if (Test-Path -LiteralPath $path) { AssertShortcut $path $shell }
    if ($spec.mode -ne 'maintenance' -and $spec.mode -notin $selected) {
      if (Test-Path -LiteralPath $path) { Remove-Item -LiteralPath $path }; continue
    }
    $link = $shell.CreateShortcut($path)
    $link.TargetPath = RelativePath $directory $spec.target
    $link.Arguments = $spec.arguments
    $link.WorkingDirectory = $directory
    if ($spec.work) { $link.WorkingDirectory = Join-Path $directory $spec.work }
    $link.Description = if ($skager) { 'SKAGER - shared OpenCPN profile' } else { 'OpenNav X - shared OpenCPN profile' }
    $link.Save()
    AssertShortcut $path $shell
  }
  # Publish a complete usable target group before removing verified old links.
  # State is already durable; Recover can finish either direction after a crash.
  Failure 'after-shortcuts'
  foreach ($other in @(ShellGroups)) { if ($other -cne $group) { RemoveShortcutGroup $other } }
  $null = New-Item -Path $Registry -Force
  foreach ($entry in @{
    DisplayName=$caption; DisplayVersion=$generation.version; Publisher=$(if ($skager) { 'SKAGER project' } else { 'OpenNav X project' });
    InstallLocation=$Root; DisplayIcon=(Join-Path $directory 'app\opencpn.exe');
    UninstallString=('"'+(Join-Path $directory 'Maintain.exe')+'" /ACTION=Uninstall');
    ModifyPath=('"'+(Join-Path $directory 'Maintain.exe')+'"');
    OpenNavOwner=$Owner
  }.GetEnumerator()) { $null = New-ItemProperty -Path $Registry -Name $entry.Key -Value $entry.Value -PropertyType String -Force }
}
function RemoveShell {
  AssertShellOwnership
  foreach ($group in @(ShellGroups)) { RemoveShortcutGroup $group }
  if (Test-Path -LiteralPath $Registry) { Remove-Item -LiteralPath $Registry -Recurse }
}
function RemoveOwnedGenerations {
  $base = Join-Path $Root 'generations'
  if (-not (Test-Path -LiteralPath $base)) { return }
  foreach ($directory in Get-ChildItem -LiteralPath $base -Directory) {
    $path = Generation $directory.Name
    $null = PlainPath $path
    if (-not (Test-Path -LiteralPath (Join-Path $path 'ownership.json'))) {
      Log ('Retained unpublished staging directory: ' + $directory.Name)
      continue
    }
    $record = ReadGeneration $directory.Name
    foreach ($file in $record.managedFiles) {
      $target = RelativePath $path $file.path
      if (-not [IO.File]::Exists($target)) { continue }
      if ((Hash $target) -ceq $file.sha256) {
        try { Remove-Item -LiteralPath $target }
        catch { Log ('Retained locked owned file: ' + $directory.Name + '/' + $file.path) }
      } else { Log ('Retained modified file: ' + $directory.Name + '/' + $file.path) }
    }
    # Imported/custom additions and ownership provenance are intentionally kept.
    # No untrusted recursive directory deletion or delayed system-wide removal.
  }
}
function Failure([string]$Point) {
  if ($FailurePoint -eq $Point) {
    if ($env:GITHUB_ACTIONS -ne 'true') { throw 'Fault injection is limited to disposable CI.' }
    throw "Injected interruption at $Point"
  }
}
function Recover {
  $journalPath = Join-Path $Root 'transaction.json'
  if (-not (Test-Path -LiteralPath $journalPath)) { return }
  $journal = ReadJson $journalPath
  if ($journal.owner -ne $Owner) { throw 'Unknown transaction journal.' }
  $current = ReadState
  if ($current) { PublishShell $current } else { RemoveShell }
  # State is the atomic commit point. Unpublished trees are retained for diagnosis.
  Log ('Recovered transaction at atomic state: ' + $journal.action)
  Remove-Item -LiteralPath $journalPath
}
function ExtractPayload([string]$Zip, [string]$Directory, $Files) {
  Add-Type -AssemblyName System.IO.Compression.FileSystem
  $wanted = @{}
  foreach ($f in $Files) { $null = RelativePath $Directory $f.path; if ($wanted.ContainsKey($f.path)) { throw 'Duplicate payload path.' }; $wanted[$f.path] = $f.sha256 }
  $archive = [IO.Compression.ZipFile]::OpenRead($Zip)
  try {
    if ($archive.Entries.Count -ne $wanted.Count -or $archive.Entries.Count -gt 12000) { throw 'Payload inventory mismatch.' }
    [long]$total = 0
    foreach ($entry in $archive.Entries) {
      $path = RelativePath $Directory $entry.FullName
      if (-not $wanted.ContainsKey($entry.FullName)) { throw 'Unexpected ZIP entry.' }
      $total += $entry.Length
      if ($entry.Length -lt 0 -or $total -gt 2147483648) { throw 'Payload exceeds extraction bounds.' }
      $null = New-Item -ItemType Directory -Path ([IO.Path]::GetDirectoryName($path)) -Force
      [IO.Compression.ZipFileExtensions]::ExtractToFile($entry, $path, $false)
      Failure 'during-extraction'
    }
  } finally { $archive.Dispose() }
  VerifyFiles $Directory $Files
}
function PreserveAdditions([string]$Source, [string]$Destination, $Known) {
  if (-not (Test-Path -LiteralPath $Source)) { return @() }
  $knownMap = @{}; foreach ($f in $Known) { $knownMap[$f.path] = $true }
  $retained = @()
  foreach ($f in @(FileRecords $Source)) {
    if ($f.path -eq 'ownership.json' -or $knownMap.ContainsKey($f.path)) { continue }
    $target = RelativePath $Destination $f.path
    if (Test-Path -LiteralPath $target) {
      if ((Hash $target) -cne $f.sha256) { throw "Custom addition conflicts with new payload: $($f.path)" }
      continue
    }
    $null = New-Item -ItemType Directory -Path ([IO.Path]::GetDirectoryName($target)) -Force
    Copy-Item -LiteralPath (RelativePath $Source $f.path) -Destination $target
    $retained += $f
  }
  return $retained
}
function AssertInstalledContent([string]$Directory,[string]$Version) {
  foreach ($file in Get-ChildItem -LiteralPath $Directory -Recurse -Force -File) {
    $relative=$file.FullName.Substring($Directory.Length+1).Replace('\','/')
    $null=RelativePath $Directory $relative
    # Re-run after preserving user additions: inherited files are not trusted
    # merely because they were absent from the incoming payload inventory.
    if ($file.Name -ieq 'OPENNAV_PORTABLE_PREVIEW') {
      throw 'An installed integration must not contain a portable profile marker.'
    }
    if ($Version -match '^0\.4\.' -and
        ($file.Name -in @('OPENNAV_TEST_PROFILE','OPENNAV_ROUTE_FIXTURE','OPENNAV_OBJECT_FIXTURE','Run-XNav-Demo.cmd','scenarios.json') -or
         $file.Name -like 'opennav-test-*' -or $relative -match '^(?i:demo/|app/demo/)')) {
      throw ('Developer/demo content refused in the installed Beta 2 product: '+$relative)
    }
  }
}
function PeU16([byte[]]$Bytes, [long]$At) {
  if ($At -lt 0 -or $At -gt $Bytes.Length - 2) { throw 'Truncated PE uint16.' }
  return [BitConverter]::ToUInt16($Bytes, [int]$At)
}
function PeU32([byte[]]$Bytes, [long]$At) {
  if ($At -lt 0 -or $At -gt $Bytes.Length - 4) { throw 'Truncated PE uint32.' }
  return [BitConverter]::ToUInt32($Bytes, [int]$At)
}
function PeRvaOffset([byte[]]$Bytes, $Sections, [long]$Rva, [long]$Length) {
  if ($Rva -le 0 -or $Length -le 0 -or $Length -gt 1048576) { throw 'Invalid PE RVA range.' }
  foreach ($section in $Sections) {
    $span = [Math]::Max($section.virtualSize, $section.rawSize)
    if ($Rva -ge $section.rva -and $Rva -lt ([long]$section.rva + $span)) {
      $delta = $Rva - $section.rva
      if ($delta -gt ([long]$section.rawSize - $Length)) { throw 'PE RVA points beyond raw section data.' }
      $offset = [long]$section.raw + $delta
      if ($offset -gt $Bytes.Length - $Length) { throw 'PE RVA points beyond file.' }
      return [int]$offset
    }
  }
  throw 'PE RVA does not map to a section.'
}
function PeImportName([byte[]]$Bytes, $Sections, [long]$Rva) {
  $start = PeRvaOffset $Bytes $Sections $Rva 1
  $end = $start
  while ($end -lt $Bytes.Length -and $end -lt $start + 256 -and $Bytes[$end] -ne 0) { $end++ }
  if ($end -eq $start -or $end -ge $Bytes.Length -or $end -ge $start + 256) { throw 'Invalid PE import name.' }
  $null = PeRvaOffset $Bytes $Sections $Rva ($end - $start + 1)
  $name = [Text.Encoding]::ASCII.GetString($Bytes, $start, $end - $start)
  if ($name -cnotmatch '^[A-Za-z0-9_.+-]+$' -or $name.StartsWith('.') -or
      $name.EndsWith('.') -or $name.Contains('..')) { throw "Invalid PE import module name: $name" }
  return $name.ToLowerInvariant()
}
function GetPeImports([string]$Path) {
  $file = Get-Item -LiteralPath $Path -ErrorAction Stop
  # Cap one allocation to 128 MiB in the 32-bit installer host.
  if ($file.Length -lt 256 -or $file.Length -gt 134217728) { throw "Invalid PE file size: $Path" }
  [byte[]]$bytes = [IO.File]::ReadAllBytes($Path)
  if ($bytes.Length -lt 256 -or $bytes.Length -gt 134217728) { throw "Invalid PE read size: $Path" }
  if ((PeU16 $bytes 0) -ne 0x5a4d) { throw "Not a PE binary: $Path" }
  $pe = [long](PeU32 $bytes 60)
  if ($pe -gt $bytes.Length - 24 -or (PeU32 $bytes $pe) -ne 0x4550) { throw "Invalid PE header: $Path" }
  if ((PeU16 $bytes ($pe + 4)) -ne 0x14c) { throw "Not the supported x86 PE ABI: $Path" }
  $sectionCount = PeU16 $bytes ($pe + 6)
  $optionalSize = PeU16 $bytes ($pe + 20)
  $optional = $pe + 24
  if ($sectionCount -lt 1 -or $sectionCount -gt 96 -or $optionalSize -lt 224 -or
      $optional -gt $bytes.Length - $optionalSize -or (PeU16 $bytes $optional) -ne 0x10b -or
      (PeU32 $bytes ($optional + 92)) -lt 14) {
    throw "Invalid PE32 optional header: $Path"
  }
  $sectionStart = $optional + $optionalSize
  if ($sectionStart -gt $bytes.Length - (40 * $sectionCount)) { throw "Truncated PE sections: $Path" }
  $sections = @()
  for ($i = 0; $i -lt $sectionCount; $i++) {
    $at = $sectionStart + 40 * $i
    $section = [pscustomobject]@{
      virtualSize = [long](PeU32 $bytes ($at + 8))
      rva = [long](PeU32 $bytes ($at + 12))
      rawSize = [long](PeU32 $bytes ($at + 16))
      raw = [long](PeU32 $bytes ($at + 20))
    }
    $span = [Math]::Max($section.virtualSize, $section.rawSize)
    if (($section.rawSize -gt 0 -and $section.raw -gt $bytes.Length - $section.rawSize) -or
        ([long]$section.rva + $span) -gt 4294967296) {
      throw "Invalid PE section range: $Path"
    }
    foreach ($prior in $sections) {
      $priorSpan = [Math]::Max($prior.virtualSize, $prior.rawSize)
      if ($span -gt 0 -and $priorSpan -gt 0 -and
          $section.rva -lt ([long]$prior.rva + $priorSpan) -and
          $prior.rva -lt ([long]$section.rva + $span)) {
        throw "Overlapping PE section RVAs: $Path"
      }
    }
    $sections += $section
  }
  $imports = New-Object 'System.Collections.Generic.HashSet[string]' ([StringComparer]::OrdinalIgnoreCase)
  foreach ($directory in @(@{index=1; size=20}, @{index=13; size=32})) {
    $entry = $optional + 96 + 8 * $directory.index
    $rva = [long](PeU32 $bytes $entry)
    $size = [long](PeU32 $bytes ($entry + 4))
    if ($rva -eq 0 -and $size -eq 0) { continue }
    if ($rva -eq 0 -or $size -lt $directory.size -or $size -gt 1048576) { throw "Invalid PE import directory: $Path" }
    $base = PeRvaOffset $bytes $sections $rva $size
    $terminated = $false
    for ($used = 0; $used -le $size - $directory.size; $used += $directory.size) {
      $at = [long]$base + $used
      $zero = $true
      for ($j = 0; $j -lt $directory.size; $j++) {
        if ($bytes[$at + $j] -ne 0) { $zero = $false; break }
      }
      if ($zero) { $terminated = $true; break }
      if ($directory.index -eq 1) {
        $nameRva = [long](PeU32 $bytes ($at + 12))
      } else {
        $attributes = PeU32 $bytes $at
        if ($attributes -gt 1) { throw "Unsupported PE delay import attributes: $Path" }
        $nameRva = [long](PeU32 $bytes ($at + 4))
        if ($attributes -eq 0) { $nameRva -= [long](PeU32 $bytes ($optional + 28)) }
      }
      if ($nameRva -le 0) { throw "Invalid PE import name RVA: $Path" }
      $null = $imports.Add((PeImportName $bytes $sections $nameRva))
    }
    if (-not $terminated) { throw "Unterminated PE import directory: $Path" }
  }
  return @($imports)
}
function GetCandidateSystemX86 {
  # Use the OS-known x86 system directory, not a caller-controlled environment variable.
  $path = [Environment]::GetFolderPath([Environment+SpecialFolder]::SystemX86)
  if ([string]::IsNullOrWhiteSpace($path)) { throw 'Windows x86 system DLL directory is unavailable.' }
  return $path
}
function AssertCandidateTlsRuntime([string]$Directory) {
  $app = RelativePath $Directory 'app'
  if (-not [IO.Directory]::Exists($app)) { throw 'Candidate app directory is missing.' }
  $system = GetCandidateSystemX86
  if (-not [IO.Directory]::Exists($system)) { throw 'Windows x86 system DLL directory is missing.' }
  $binaries = New-Object 'System.Collections.Generic.Queue[string]'
  $seen = New-Object 'System.Collections.Generic.HashSet[string]' ([StringComparer]::OrdinalIgnoreCase)
  foreach ($file in Get-ChildItem -LiteralPath $app -Recurse -Force -File) {
    if ($file.Extension -ieq '.dll' -or $file.Extension -ieq '.exe') {
      $binaries.Enqueue($file.FullName)
    }
  }
  foreach ($file in Get-ChildItem -LiteralPath $app -Recurse -Force -File) {
    if ($file.Name -ieq 'libeay32.dll' -or $file.Name -ieq 'ssleay32.dll') {
      throw "Unsupported legacy TLS runtime dependency in candidate: $($file.FullName)"
    }
  }
  while ($binaries.Count -gt 0) {
    $binary = $binaries.Dequeue()
    if (-not $seen.Add($binary)) { continue }
    foreach ($name in @(GetPeImports $binary)) {
      if ($name -ieq 'libeay32.dll' -or $name -ieq 'ssleay32.dll') {
        throw "Unsupported legacy TLS runtime dependency in candidate: import $name in $binary"
      }
      $local = @(Get-ChildItem -LiteralPath ([IO.Path]::GetDirectoryName($binary)) -File |
        Where-Object { $_.Name -ieq $name })
      if (-not $local.Count) {
        $local = @(Get-ChildItem -LiteralPath $app -File |
          Where-Object { $_.Name -ieq $name })
      }
      if ($local.Count) {
        $binaries.Enqueue($local[0].FullName)
        continue
      }
      $runtime = $name -ine 'msvcrt.dll' -and
        $name -match '^(?i:msvcp|msvcr|vcruntime|vcomp|concrt|wx|lib|archive|zlib|glew)'
      if (-not $runtime -and ($name -match '^(?i:api-ms-win-|ext-ms-win-)' -or
          [IO.File]::Exists((Join-Path $system $name)))) { continue }
      throw "Missing app-local PE import $name in $binary"
    }
  }
}

try {
  $Root = PlainPath $Root
  foreach ($group in @(ShellGroups)) { $null = PlainPath $group }
  $state = ReadState
  if ((Test-Path -LiteralPath $Root) -and -not $state -and -not (Test-Path -LiteralPath (Join-Path $Root 'owner.json'))) { throw 'Existing directory is not a SKAGER-owned installation.' }
  if (Test-Path -LiteralPath (Join-Path $Root 'owner.json')) {
    if ((ReadJson (Join-Path $Root 'owner.json')).owner -ne $Owner) { throw 'Unknown root ownership.' }
    $OwnsRoot = $true
  }
  AssertShellOwnership
  if ($Action -eq 'Repair' -and -not $PackageDirectory -and $state) {
    $installed = ReadGeneration $state.current
    $PackageDirectory = Join-Path (Generation $state.current) 'maintenance'
    $ManifestSha256 = $installed.packageSha256
  }
  if ($Action -eq 'Diagnostics' -and -not $Report) {
    if (-not $state) { throw 'No installed generation; rerun Setup diagnostics with a report path.' }
    $Report = Join-Path $Root ('logs\diagnostics-' + [DateTime]::UtcNow.ToString('yyyyMMdd-HHmmss') + '.json')
  }
  if ($Action -in @('Install','Update','Repair','Preflight')) {
    $PackageDirectory = PlainPath $PackageDirectory
    if ($ManifestSha256 -cnotmatch '^[a-f0-9]{64}$' -or (Hash (Join-Path $PackageDirectory 'package.json')) -cne $ManifestSha256) { throw 'Package manifest integrity check failed.' }
    $package = ReadJson (Join-Path $PackageDirectory 'package.json')
    if ($package.schema -ne 1 -or $package.commit -cnotmatch '^[a-f0-9]{40}$') { throw 'Unknown integration package contract.' }
    if (-not $OpenCpn -and $state) { $OpenCpn = $state.stock.path }
    if (-not $OpenCpn) {
      $candidates = @(DiscoverStock)
      if ($candidates.Count -ne 1) { throw 'Choose the installed OpenCPN 5.12.4 executable in Setup.' }
      $OpenCpn = $candidates[0]
    }
    $stock = StockInfo $OpenCpn $package.supportedOpenCpn
    if ($state -and ($state.stock.path -ine $stock.path -or $state.stock.sha256 -cne $stock.sha256)) { throw 'Existing integration belongs to a different stock installation.' }
    if ((Hash (Join-Path $PackageDirectory 'payload.zip')) -cne $package.payloadSha256) { throw 'Payload ZIP integrity check failed.' }
    Log "Supported OpenCPN $($stock.version) $($stock.arch) SHA-256 $($stock.sha256)"
  } elseif ($state -and $Action -ne 'Diagnostics') {
    # Never trust a registry hint on maintenance paths either.
    if ((Hash $state.stock.path) -cne $state.stock.sha256) { throw 'Original OpenCPN changed; use diagnostics before maintenance.' }
  }
  if ($Action -eq 'Preflight') {
    if ($SummaryPath) {
      $summary = PlainPath $SummaryPath
      if (Test-Path -LiteralPath $summary) { throw 'Preflight summary must use a new path.' }
      $existing = ''; $suggested = 'Install'
      $selected = @('xnav','legacy','safe')
      if ($state) {
        $existing = (ReadGeneration $state.current).version; $suggested = 'Update'
        if ($state.PSObject.Properties['shortcutModes']) { $selected = @($state.shortcutModes) }
      }
      # The wizard reads only this private temporary INI. Never execute its values.
      [IO.File]::WriteAllLines($summary, @('[Preflight]', ('Stock=' + $stock.path),
        ('StockVersion=' + $stock.version), ('StockHash=' + $stock.sha256), ('Version=' + $package.version),
        ('Existing=' + $existing), ('SuggestedAction=' + $suggested), ('Root=' + $Root),
        ('Recovery=' + (Join-Path $Root 'recovery')),
        ('LegacyShortcut=' + [int]('legacy' -in $selected)), ('SafeShortcut=' + [int]('safe' -in $selected))), [Text.Encoding]::Unicode)
    }
    Log 'Preflight passed; no installed files or profiles changed.'
  }
  elseif ($Action -eq 'Diagnostics') {
    $result = @{owner=$Owner; state=$state; files=@(); stockVerified=$false}
    if ($state) {
      $result.stockVerified = (Hash $state.stock.path) -ceq $state.stock.sha256
      $g = ReadGeneration $state.current
      $result.xnavHardwareOutputPolicy = if ($g.PSObject.Properties['xnavHardwareOutputPolicy']) { $g.xnavHardwareOutputPolicy } else { 'historical-unqualified' }
      foreach ($f in $g.files) {
        $p = RelativePath (Generation $state.current) $f.path
        $result.files += @{path=$f.path; expected=$f.sha256; actual=$(if ([IO.File]::Exists($p)) { Hash $p } else { 'missing' })}
      }
    }
    if (-not $Report) { throw 'Choose a diagnostics JSON report path.' }
    AtomicJson (PlainPath $Report) $result
    Log 'Installation diagnostics written; no navigation coordinates or raw data collected.'
  } else {
    AssertClosed
    if (-not $state -and $Action -notin @('Install','Update')) { throw 'No installed SKAGER generation for this action.' }
    if (Test-Path -LiteralPath $Registry) {
      if ((Get-ItemProperty -LiteralPath $Registry).OpenNavOwner -ne $Owner) { throw 'Unknown registry ownership.' }
    }
    if (-not (Test-Path -LiteralPath $Root)) {
      $null = New-Item -ItemType Directory -Path $Root
      AtomicJson (Join-Path $Root 'owner.json') @{owner=$Owner}
      $OwnsRoot = $true
    }
    $TransactionLock = [IO.File]::Open((Join-Path $Root 'transaction.lock'), [IO.FileMode]::OpenOrCreate, [IO.FileAccess]::ReadWrite, [IO.FileShare]::None)
    # The installed launcher holds this same lock across its current-generation
    # check and process creation. Recheck under the lock before any recovery/write.
    AssertClosed
    Recover
    $state = ReadState
    if (Test-Path -LiteralPath (Join-Path $Root 'update-pending.json')) {
      if ($Action -ne 'Rollback') { throw 'Resolve the pending supervised update before maintenance.' }
      if (-not $UpdateTransaction) { $UpdateTransaction = (Read-UpdatePendingRecord (Join-Path $Root 'update-pending.json')).transaction }
      $pendingContext = Get-ValidatedUpdatePending -InstallationRoot $Root -Transaction $UpdateTransaction
      if ($pendingContext.decision -ceq 'retain-previous') {
        Complete-SupervisedRollback -InstallationRoot $Root -Transaction $UpdateTransaction -Lock $TransactionLock
        $RecoveredSupervised = $true
      } else { $null = Assert-SupervisedRollback -InstallationRoot $Root -Transaction $UpdateTransaction -Lock $TransactionLock }
    } elseif ($UpdateTransaction) { throw 'Pending update recovery is no longer current.' }
    if ($RecoveredSupervised) {
      Log 'Interrupted supervised recovery reconciled the already-selected previous generation.'
    } elseif ($Action -in @('Install','Update','Repair')) {
      if ($Action -eq 'Repair' -and -not $state) { throw 'Repair requires an installed generation.' }
      $modes = @('xnav','legacy','safe')
      if ($ShortcutModes) { $modes = @($ShortcutModes.Split(',')) }
      elseif ($state -and $state.PSObject.Properties['shortcutModes']) { $modes = @($state.shortcutModes) }
      # Immutable previous generations are the application backup. Capture the
      # exact before-state durably before staging; never copy/restore user data.
      $recovery = PlainPath (Join-Path $Root 'recovery')
      $null = New-Item -ItemType Directory -Path $recovery -Force
      AtomicJson (Join-Path $recovery ([DateTime]::UtcNow.ToString('yyyyMMdd-HHmmss-fff') + '-' + [guid]::NewGuid().ToString('N') + '.json')) @{
        schema=1; owner=$Owner; timestamp=[DateTime]::UtcNow.ToString('o'); action=$Action;
        stock=$stock; before=$state; nextVersion=$package.version; nextCommit=$package.commit;
        policy='Original OpenCPN and user profile remain untouched; prior application generation retained.'
      }
      Log 'Recovery record durable; verified original OpenCPN and previous integration remain available.'
      $id = [guid]::NewGuid().ToString('N'); $stage = Generation $id
      $null = New-Item -ItemType Directory -Path $stage -Force
      ExtractPayload (Join-Path $PackageDirectory 'payload.zip') $stage $package.files
      AssertInstalledContent $stage $package.version
      # Normal OpenCPN persists absolute default resource paths. Point new
      # defaults at the untouched stock installation, not a removable generation.
      $locator = Join-Path $stage 'app\OPENNAV_INSTALLED_STOCK'
      [IO.File]::WriteAllText($locator, $stock.path, $Utf8)
      Copy-Item -LiteralPath $PSCommandPath -Destination (Join-Path $stage 'Lifecycle.ps1')
      foreach ($helper in @('UpdateTransaction.ps1','UpdateSupervisor.ps1')) {
        Copy-Item -LiteralPath (Join-Path $PSScriptRoot $helper) -Destination (Join-Path $stage $helper)
      }
      Copy-Item -LiteralPath (Join-Path $PackageDirectory 'Maintain.exe') -Destination (Join-Path $stage 'Maintain.exe')
      $maintenance = Join-Path $stage 'maintenance'
      $null = New-Item -ItemType Directory -Path $maintenance
      foreach ($name in @('package.json','payload.zip','Maintain.exe')) {
        Copy-Item -LiteralPath (Join-Path $PackageDirectory $name) -Destination (Join-Path $maintenance $name)
      }

      foreach ($name in @('UpdateTransaction.ps1','UpdateSupervisor.ps1')) {
        Copy-Item -LiteralPath (Join-Path $PSScriptRoot $name) -Destination (Join-Path $maintenance $name)
      }

      # Unbundled installed plugins stay beside the integrated executable, retaining names/resources.
      $pluginRoot = Join-Path ([IO.Path]::GetDirectoryName($stock.path)) 'plugins'
      $bundledPluginFiles = @($package.files | Where-Object { $_.path.StartsWith('app/plugins/') } | ForEach-Object { [pscustomobject]@{path=$_.path.Substring(12);sha256=$_.sha256} })
      $retained = @(PreserveAdditions $pluginRoot (Join-Path $stage 'app\plugins') $bundledPluginFiles)
      if ($state) {
        $old = ReadGeneration $state.current
        $null = PreserveAdditions (Generation $state.current) $stage $old.managedFiles
      }
      AssertInstalledContent $stage $package.version
      AssertCandidateTlsRuntime $stage
      $recordedRepair = $Action -eq 'Repair' -and $state -and (Test-ExactRepairPackage (ReadGeneration $state.current) $package $ManifestSha256)
      $outputPolicy = SelfTest $stage $package.commit $package.version $recordedRepair
      $startupHealth = $LastStartupHealth
      if ($startupHealth -eq 1) {
        foreach ($helper in @('app/skager-start.exe','app/skager-update-prompt.exe')) {
          $matches = @($package.files | Where-Object { $_.path -ceq $helper })
          if ($matches.Count -ne 1 -or (Hash (RelativePath $stage $helper)) -cne $matches[0].sha256) { throw 'Startup update helper is not part of the exact package.' }
        }
      } elseif ($SupervisedUpdate) { throw 'Candidate does not support authenticated startup.' }
      AtomicJson (Join-Path $stage 'ownership.json') @{owner=$Owner; version=$package.version; commit=$package.commit; packageSha256=$ManifestSha256; xnavHardwareOutputPolicy=$outputPolicy; xnavManualControlContract=$(if ($outputPolicy -ceq 'manual-commissioning') { 1 } else { 0 }); shellLayout='OpenNavX.SkagerStartMenu.1'; updateStartupHealth=$startupHealth; shortcutModes=$modes; files=@(FileRecords $stage); managedFiles=@(FileRecords $maintenance | ForEach-Object { [pscustomobject]@{path=('maintenance/'+$_.path);sha256=$_.sha256} }) + @($package.files) + @([pscustomobject]@{path='UpdateTransaction.ps1';sha256=(Hash (Join-Path $stage 'UpdateTransaction.ps1'))}, [pscustomobject]@{path='UpdateSupervisor.ps1';sha256=(Hash (Join-Path $stage 'UpdateSupervisor.ps1'))}, [pscustomobject]@{path='Lifecycle.ps1';sha256=(Hash (Join-Path $stage 'Lifecycle.ps1'))}, [pscustomobject]@{path='Maintain.exe';sha256=(Hash (Join-Path $stage 'Maintain.exe'))}, [pscustomobject]@{path='app/OPENNAV_INSTALLED_STOCK';sha256=(Hash $locator)}); importedPlugins=$retained}
      $previous = ''; if ($state) { $previous = $state.current }
      $next = @{owner=$Owner;schema=1;stock=$stock;current=$id;previous=$previous;shortcutModes=$modes}
      if ($SupervisedUpdate) {
        if (-not $state) { throw 'Supervised update requires a known-good installed version.' }
        $null = New-SupervisedUpdatePending -InstallationRoot $Root -CandidateGeneration $id -PreviousGeneration $previous -Lock $TransactionLock
      }
      AtomicJson (Join-Path $Root 'transaction.json') @{owner=$Owner;action=$Action;before=$state;after=$next}
      Failure 'before-commit'
      AssertShellOwnership
      AtomicJson (Join-Path $Root 'state.json') $next
      Failure 'after-commit'
      PublishShell $next
      Remove-Item -LiteralPath (Join-Path $Root 'transaction.json')
      if ($startupHealth -eq 1) {
        $provision = New-Object Diagnostics.ProcessStartInfo
        $provision.FileName = Join-Path $stage 'app\skager-start.exe'
        $provision.Arguments = '--initialize-trust'
        $provision.UseShellExecute = $false; $provision.CreateNoWindow = $true
        $init = [Diagnostics.Process]::Start($provision)
        try {
          if (-not $init.WaitForExit(15000)) { throw 'Update trust provisioning did not finish; current installed app remains recoverable.' }
          if ($init.ExitCode -ne 0) { throw 'Update trust requires explicit recovery; no network update is permitted.' }
        } finally { $init.Dispose() }
      }
      Log "$Action committed generation $id; original OpenCPN and shared profile untouched."
    } elseif ($Action -eq 'Rollback' -and $state.previous) {
      $old = ReadGeneration $state.previous
      VerifyFiles (Generation $state.previous) $old.files
      AssertInstalledContent (Generation $state.previous) $old.version
      $null = SelfTest (Generation $state.previous) $old.commit $old.version $true
      $next = @{owner=$Owner;schema=1;stock=$state.stock;current=$state.previous;previous='';shortcutModes=$(if ($old.PSObject.Properties['shortcutModes']) { @($old.shortcutModes) } else { @('xnav','legacy','safe') })}
      AssertShellOwnership
      AtomicJson (Join-Path $Root 'transaction.json') @{owner=$Owner;action=$Action;before=$state;after=$next}
      # The old maintainer reads the committed state at startup. Eliminate
      # the group or newer launcher target it cannot recognize before committing an old generation,
      # including a crash immediately after that commit.
      if ((ShortcutGroup $old) -ine (Join-Path $Programs 'SKAGER') -or
          -not $old.PSObject.Properties['updateStartupHealth'] -or $old.updateStartupHealth -ne 1) {
        RemoveShortcutGroup (Join-Path $Programs 'SKAGER')
      }
      AtomicJson (Join-Path $Root 'state.json') $next
      Failure 'after-commit'
      PublishShell $next
      Remove-Item -LiteralPath (Join-Path $Root 'transaction.json')
      if ($UpdateTransaction) { Complete-SupervisedRollback -InstallationRoot $Root -Transaction $UpdateTransaction -Lock $TransactionLock }
      Log 'Rollback restored prior exact application generation; newer navigation data retained.'
    } elseif ($Action -in @('Uninstall','Rollback')) {
      AtomicJson (Join-Path $Root 'transaction.json') @{owner=$Owner;action='Uninstall';before=$state;after=$null}
      RemoveShell
      Remove-Item -LiteralPath (Join-Path $Root 'state.json')
      Remove-Item -LiteralPath (Join-Path $Root 'transaction.json')
      RemoveOwnedGenerations
      Log 'Integration unregistered and verified owned files removed. Modified/custom additions and diagnostics retained; original OpenCPN unchanged.'
    }
    if ($state -and (Hash $state.stock.path) -cne $state.stock.sha256) { throw 'Unexpected stock hash change during transaction.' }
  }
  if ($Report -and $Action -ne 'Diagnostics') { AtomicJson (PlainPath $Report) @{status='passed';action=$Action;log=@($SessionLog)} }
  exit 0
} catch {
  Log ('FAILED: ' + $_.Exception.Message)
  if ($Report) { try { AtomicJson (PlainPath $Report) @{status='failed';action=$Action;error=$_.Exception.Message;log=@($SessionLog)} } catch {} }
  exit 1
} finally {
  if ($TransactionLock) { $TransactionLock.Dispose() }
  if ($OwnsRoot -and (Test-Path -LiteralPath (Join-Path $Root 'owner.json'))) {
    try {
      $logs = PlainPath (Join-Path $Root 'logs'); $null = New-Item -ItemType Directory -Path $logs -Force
      [IO.File]::WriteAllLines((Join-Path $logs ([DateTime]::UtcNow.ToString('yyyyMMdd-HHmmss-fff')+'-'+$Action+'.log')), $SessionLog, $Utf8)
    } catch {}
  }
}

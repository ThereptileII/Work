# Windows PowerShell 5.1. No elevation, network access, or writes to OpenCPN/profile.
[CmdletBinding()]
param(
  [ValidateSet('Preflight','Install','Repair','Update','Rollback','Uninstall','Diagnostics')]
  [string]$Action = 'Preflight',
  [string]$OpenCpn = '',
  [string]$PackageDirectory = '',
  [string]$ManifestSha256 = '',
  [string]$Report = '',
  [string]$FailurePoint = ''
)
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$Root = Join-Path ([Environment]::GetFolderPath('LocalApplicationData')) 'OpenNavXAlpha1'
$Registry = 'HKCU:\Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenNavXAlpha1'
$Shortcuts = Join-Path ([Environment]::GetFolderPath('Programs')) 'OpenNav X Alpha 1'
$Owner = 'OpenNavX.Alpha1.SideBySide.1'
$Utf8 = New-Object System.Text.UTF8Encoding($false)
$SessionLog = New-Object System.Collections.Generic.List[string]
$TransactionLock = $null

function Log([string]$Message) {
  $line = [DateTime]::UtcNow.ToString('o') + ' ' + $Message
  $SessionLog.Add($line)
  Write-Host $line
}
function Hash([string]$Path) { (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash.ToLowerInvariant() }
function PlainPath([string]$Path) {
  if ([string]::IsNullOrWhiteSpace($Path) -or $Path -notmatch '^[A-Za-z]:[\\/]' -or $Path.Contains('"')) {
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
  if ($Name -notmatch '^[A-Za-z0-9_ .()&/+-]+$' -or $Name.Contains('\') -or
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
function AtomicJson([string]$Path, $Value) {
  $null = PlainPath $Path
  $temp = $Path + '.' + [guid]::NewGuid().ToString('N') + '.tmp'
  $bytes = $Utf8.GetBytes(($Value | ConvertTo-Json -Depth 16))
  $file = New-Object IO.FileStream($temp, [IO.FileMode]::CreateNew, [IO.FileAccess]::Write, [IO.FileShare]::None)
  try { $file.Write($bytes, 0, $bytes.Length); $file.Flush($true) } finally { $file.Dispose() }
  if (Test-Path -LiteralPath $Path) { [IO.File]::Replace($temp, $Path, $null) }
  else { [IO.File]::Move($temp, $Path) }
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
    if ($r.ReadUInt16() -ne 0x14c) { throw 'This Alpha requires the supported x86 OpenCPN plugin ABI.' }
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
  return [pscustomobject]@{path=$Path; sha256=$hash; version='5.12.4'; arch=$architecture}
}
function DiscoverStock {
  $paths = New-Object System.Collections.Generic.List[string]
  foreach ($view in @([Microsoft.Win32.RegistryView]::Registry32, [Microsoft.Win32.RegistryView]::Registry64)) {
    $base = [Microsoft.Win32.RegistryKey]::OpenBaseKey([Microsoft.Win32.RegistryHive]::LocalMachine, $view)
    try {
      $key = $base.OpenSubKey('Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenCPN 5.12.4')
      if ($key) { try { $location = [string]$key.GetValue('InstallLocation'); if ($location) { $paths.Add((Join-Path $location 'opencpn.exe')) } } finally { $key.Dispose() } }
    } finally { $base.Dispose() }
  }
  $paths.Add((Join-Path ${env:ProgramFiles(x86)} 'OpenCPN\opencpn.exe'))
  return @($paths | Select-Object -Unique | Where-Object { Test-Path -LiteralPath $_ })
}
function AssertClosed {
  if (@(Get-Process -Name opencpn -ErrorAction SilentlyContinue).Count) { throw 'Close OpenCPN, XNav, Legacy and Safe Mode before changing the installation.' }
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
  return $manifest
}
function SelfTest([string]$Directory, [string]$Commit, [string]$Version) {
  $reportPath = Join-Path $Directory ('loader-' + [guid]::NewGuid().ToString('N') + '.json')
  $exe = Join-Path $Directory 'app\opencpn.exe'
  $null = PeArchitecture $exe
  $process = Start-Process -FilePath $exe -ArgumentList @('--opennav-self-test', ('"' + $reportPath + '"')) -PassThru
  if (-not $process.WaitForExit(30000)) { $process.Kill(); throw 'Staged executable loader self-test timed out.' }
  if ($process.ExitCode -ne 0) { throw "Staged executable self-test failed: $($process.ExitCode)" }
  $result = ReadJson $reportPath
  if (-not $result.passed -or $result.commit -cne $Commit -or $result.version -cne $Version -or $result.profile_initialized -or $result.plugins_loaded) { throw 'Executable identity/self-test report mismatch.' }
  Remove-Item -LiteralPath $reportPath
  Log "Loader/resource self-test passed for $Commit"
}
function PublishShell($State) {
  if (Test-Path -LiteralPath $Shortcuts) {
    $null = PlainPath $Shortcuts
    foreach ($f in Get-ChildItem -LiteralPath $Shortcuts -Force) {
      if ($f.Name -notin @('OpenNav X.lnk','OpenCPN Legacy.lnk','OpenNav Safe Mode.lnk','Maintain OpenNav.lnk')) { throw 'Unknown item in OpenNav shortcut folder; preserve and inspect it.' }
    }
  }
  $directory = Generation $State.current
  $generation = ReadGeneration $State.current
  $null = New-Item -ItemType Directory -Path $Shortcuts -Force
  $shell = New-Object -ComObject WScript.Shell
  foreach ($pair in @(@('OpenNav X','--xnav'),@('OpenCPN Legacy','--legacy'),@('OpenNav Safe Mode','--safe-mode'))) {
    $link = $shell.CreateShortcut((Join-Path $Shortcuts ($pair[0]+'.lnk')))
    $link.TargetPath = Join-Path $directory 'app\opencpn.exe'; $link.Arguments = $pair[1]
    $link.WorkingDirectory = Join-Path $directory 'app'; $link.Description = 'OpenNav X Alpha 1 - shared OpenCPN profile'; $link.Save()
  }
  $link = $shell.CreateShortcut((Join-Path $Shortcuts 'Maintain OpenNav.lnk'))
  $link.TargetPath = Join-Path $directory 'Maintain.exe'; $link.WorkingDirectory = $directory; $link.Save()
  $null = New-Item -Path $Registry -Force
  foreach ($entry in @{
    DisplayName='OpenNav X Alpha 1'; DisplayVersion=$generation.version; Publisher='OpenNav X project';
    InstallLocation=$Root; DisplayIcon=(Join-Path $directory 'app\opencpn.exe');
    UninstallString=('"'+(Join-Path $directory 'Maintain.exe')+'" /ACTION=Uninstall');
    ModifyPath=('"'+(Join-Path $directory 'Maintain.exe')+'"');
    OpenNavOwner=$Owner
  }.GetEnumerator()) { $null = New-ItemProperty -Path $Registry -Name $entry.Key -Value $entry.Value -PropertyType String -Force }
}
function RemoveShell {
  if (Test-Path -LiteralPath $Registry) {
    if ((Get-ItemProperty -LiteralPath $Registry).OpenNavOwner -ne $Owner) { throw 'Unknown uninstall registry ownership.' }
    Remove-Item -LiteralPath $Registry -Recurse
  }
  if (Test-Path -LiteralPath $Shortcuts) {
    foreach ($name in @('OpenNav X.lnk','OpenCPN Legacy.lnk','OpenNav Safe Mode.lnk','Maintain OpenNav.lnk')) {
      $p = Join-Path $Shortcuts $name
      if (Test-Path -LiteralPath $p) { Remove-Item -LiteralPath (PlainPath $p) }
    }
    if (@(Get-ChildItem -LiteralPath $Shortcuts -Force).Count -eq 0) { Remove-Item -LiteralPath $Shortcuts }
  }
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

try {
  $Root = PlainPath $Root; $Shortcuts = PlainPath $Shortcuts
  $state = ReadState
  if ((Test-Path -LiteralPath $Root) -and -not $state -and -not (Test-Path -LiteralPath (Join-Path $Root 'owner.json'))) { throw 'Existing directory is not an OpenNav-owned installation.' }
  if (Test-Path -LiteralPath (Join-Path $Root 'owner.json')) {
    if ((ReadJson (Join-Path $Root 'owner.json')).owner -ne $Owner) { throw 'Unknown root ownership.' }
  }
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
  if ($Action -eq 'Preflight') { Log 'Preflight passed; no installed files or profiles changed.' }
  elseif ($Action -eq 'Diagnostics') {
    $result = @{owner=$Owner; state=$state; files=@(); stockVerified=$false}
    if ($state) {
      $result.stockVerified = (Hash $state.stock.path) -ceq $state.stock.sha256
      $g = ReadGeneration $state.current
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
    if (-not $state -and $Action -notin @('Install','Update')) { throw 'No installed Alpha generation for this action.' }
    if (Test-Path -LiteralPath $Registry) {
      if ((Get-ItemProperty -LiteralPath $Registry).OpenNavOwner -ne $Owner) { throw 'Unknown registry ownership.' }
    }
    if (-not (Test-Path -LiteralPath $Root)) {
      $null = New-Item -ItemType Directory -Path $Root
      AtomicJson (Join-Path $Root 'owner.json') @{owner=$Owner}
    }
    $TransactionLock = [IO.File]::Open((Join-Path $Root 'transaction.lock'), [IO.FileMode]::OpenOrCreate, [IO.FileAccess]::ReadWrite, [IO.FileShare]::None)
    Recover
    $state = ReadState
    if ($Action -in @('Install','Update','Repair')) {
      if ($Action -eq 'Repair' -and -not $state) { throw 'Repair requires an installed generation.' }
      $id = [guid]::NewGuid().ToString('N'); $stage = Generation $id
      $null = New-Item -ItemType Directory -Path $stage -Force
      ExtractPayload (Join-Path $PackageDirectory 'payload.zip') $stage $package.files
      if (Test-Path -LiteralPath (Join-Path $stage 'app\OPENNAV_PORTABLE_PREVIEW')) { throw 'An installed integration must not contain a portable profile marker.' }
      Copy-Item -LiteralPath $PSCommandPath -Destination (Join-Path $stage 'Lifecycle.ps1')
      Copy-Item -LiteralPath (Join-Path $PackageDirectory 'Maintain.exe') -Destination (Join-Path $stage 'Maintain.exe')
      $maintenance = Join-Path $stage 'maintenance'
      $null = New-Item -ItemType Directory -Path $maintenance
      foreach ($name in @('package.json','payload.zip','Maintain.exe')) {
        Copy-Item -LiteralPath (Join-Path $PackageDirectory $name) -Destination (Join-Path $maintenance $name)
      }

      # Unbundled installed plugins stay beside the integrated executable, retaining names/resources.
      $pluginRoot = Join-Path ([IO.Path]::GetDirectoryName($stock.path)) 'plugins'
      $bundledPluginFiles = @($package.files | Where-Object { $_.path.StartsWith('app/plugins/') } | ForEach-Object { [pscustomobject]@{path=$_.path.Substring(12);sha256=$_.sha256} })
      $retained = @(PreserveAdditions $pluginRoot (Join-Path $stage 'app\plugins') $bundledPluginFiles)
      if ($state) {
        $old = ReadGeneration $state.current
        $null = PreserveAdditions (Generation $state.current) $stage $old.managedFiles
      }
      SelfTest $stage $package.commit $package.version
      AtomicJson (Join-Path $stage 'ownership.json') @{owner=$Owner; version=$package.version; commit=$package.commit; packageSha256=$ManifestSha256; files=@(FileRecords $stage); managedFiles=@(FileRecords $maintenance | ForEach-Object { [pscustomobject]@{path=('maintenance/'+$_.path);sha256=$_.sha256} }) + @($package.files) + @([pscustomobject]@{path='Lifecycle.ps1';sha256=(Hash (Join-Path $stage 'Lifecycle.ps1'))}, [pscustomobject]@{path='Maintain.exe';sha256=(Hash (Join-Path $stage 'Maintain.exe'))}); importedPlugins=$retained}
      $previous = ''; if ($state) { $previous = $state.current }
      $next = @{owner=$Owner;schema=1;stock=$stock;current=$id;previous=$previous}
      AtomicJson (Join-Path $Root 'transaction.json') @{owner=$Owner;action=$Action;before=$state;after=$next}
      Failure 'before-commit'
      AtomicJson (Join-Path $Root 'state.json') $next
      Failure 'after-commit'
      PublishShell $next
      Remove-Item -LiteralPath (Join-Path $Root 'transaction.json')
      Log "$Action committed generation $id; original OpenCPN and shared profile untouched."
    } elseif ($Action -eq 'Rollback' -and $state.previous) {
      $old = ReadGeneration $state.previous
      VerifyFiles (Generation $state.previous) $old.files
      SelfTest (Generation $state.previous) $old.commit $old.version
      $next = @{owner=$Owner;schema=1;stock=$state.stock;current=$state.previous;previous=''}
      AtomicJson (Join-Path $Root 'transaction.json') @{owner=$Owner;action=$Action;before=$state;after=$next}
      AtomicJson (Join-Path $Root 'state.json') $next
      PublishShell $next
      Remove-Item -LiteralPath (Join-Path $Root 'transaction.json')
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
  if ((Test-Path -LiteralPath (Join-Path $Root 'owner.json'))) {
    try {
      $logs = PlainPath (Join-Path $Root 'logs'); $null = New-Item -ItemType Directory -Path $logs -Force
      [IO.File]::WriteAllLines((Join-Path $logs ([DateTime]::UtcNow.ToString('yyyyMMdd-HHmmss-fff')+'-'+$Action+'.log')), $SessionLog, $Utf8)
    } catch {}
  }
}

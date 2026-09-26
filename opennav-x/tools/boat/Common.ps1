# Shared boat-PC deployment helpers. No remote-access service changes, bus output,
# synthetic input, process termination, or automatic privilege elevation.
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'
function Assert-LocalPath([string]$Path) {
  if ($Path -notmatch '^[A-Za-z]:\\' -or $Path -match '[\x00-\x1f"]') { throw 'Use an absolute local Windows path.' }
  $full=[IO.Path]::GetFullPath($Path).TrimEnd('\')
  if ($full.Length -lt 4) { throw 'Drive-root operations are forbidden.' }
  $walk=$full
  while ($walk.Length -gt 3) {
    if ((Test-Path -LiteralPath $walk) -and ((Get-Item -LiteralPath $walk -Force).Attributes -band [IO.FileAttributes]::ReparsePoint)) { throw 'Reparse paths are forbidden.' }
    $walk=[IO.Path]::GetDirectoryName($walk)
  }
  return $full
}
function Get-Digest([string]$Path) {
  $algorithm=[Security.Cryptography.SHA256]::Create();$stream=[IO.File]::OpenRead((Assert-LocalPath $Path))
  try { return ([BitConverter]::ToString($algorithm.ComputeHash($stream))).Replace('-','').ToLowerInvariant() }
  finally { $stream.Dispose();$algorithm.Dispose() }
}
function Read-Record([string]$Path) {
  $file=Get-Item -LiteralPath (Assert-LocalPath $Path)
  if ($file.Length -le 0 -or $file.Length -gt 4194304) { throw 'JSON record size outside bounds.' }
  return [IO.File]::ReadAllText($file.FullName) | ConvertFrom-Json
}
function Write-Record([string]$Path,$Record) {
  $path=Assert-LocalPath $Path
  if (Test-Path -LiteralPath $path) { throw 'A new evidence path is required.' }
  $bytes=(New-Object Text.UTF8Encoding($false)).GetBytes(($Record | ConvertTo-Json -Depth 16))
  $temporary=$path+'.'+[guid]::NewGuid().ToString('N')+'.partial'
  $file=New-Object IO.FileStream($temporary,[IO.FileMode]::CreateNew,[IO.FileAccess]::Write,[IO.FileShare]::None)
  try {$file.Write($bytes,0,$bytes.Length);$file.Flush($true)} finally {$file.Dispose()}
  [IO.File]::Move($temporary,$path)
}
function Get-Target([string]$Workspace) {
  $root=Assert-LocalPath $Workspace
  $config=Read-Record (Join-Path $root 'boat-target.json')
  if ($config.schema -ne 1 -or $config.owner -cne 'OpenNavX.BoatTarget.1') { throw 'Unknown boat-target configuration.' }
  $null=Assert-LocalPath $config.stockExecutable; $null=Assert-LocalPath $config.profileDirectory
  # This value is the exact previously qualified official 5.12.4 x86 binary.
  # A boat-specific config cannot extend the supported binary allowlist.
  $supported='7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c'
  if ((Get-Digest $config.stockExecutable) -cne $supported) { throw 'Unsupported original OpenCPN executable. Stop; no install or launch performed.' }
  return $config
}
function Get-Installed {
  $root=Assert-LocalPath (Join-Path ([Environment]::GetFolderPath('LocalApplicationData')) 'OpenNavXAlpha1')
  $state=Read-Record (Join-Path $root 'state.json')
  if ($state.owner -cne 'OpenNavX.Alpha1.SideBySide.1' -or $state.current -cnotmatch '^[a-f0-9]{32}$') { throw 'Unrecognized installed integration ownership.' }
  $generation=Assert-LocalPath (Join-Path $root ('generations\'+$state.current))
  $ownership=Read-Record (Join-Path $generation 'ownership.json')
  if ($ownership.owner -cne $state.owner) { throw 'Installed generation ownership mismatch.' }
  $exe=Join-Path $generation 'app\opencpn.exe'
  $record=@($ownership.managedFiles | Where-Object { $_.path -ceq 'app/opencpn.exe' })
  if ($record.Count -ne 1 -or (Get-Digest $exe) -cne $record[0].sha256) { throw 'Installed executable hash mismatch; use Repair.' }
  return [pscustomobject]@{root=$root;state=$state;generation=$generation;ownership=$ownership;executable=$exe}
}
function New-RunDirectory([string]$Workspace,[string]$Purpose) {
  if ($Purpose -cnotmatch '^[a-z0-9-]+$') { throw 'Invalid run label.' }
  $root=Assert-LocalPath (Join-Path $Workspace 'runs')
  $null=New-Item -ItemType Directory -Path $root -Force
  $path=Join-Path $root ([DateTime]::UtcNow.ToString('yyyyMMdd-HHmmss')+'-'+$Purpose+'-'+[guid]::NewGuid().ToString('N').Substring(0,8))
  $null=New-Item -ItemType Directory -Path $path
  return $path
}
function Assert-TrueBoolean($Value,[string]$Label) {
  if ($Value -isnot [bool] -or $Value -ne $true) { throw ('Explicit boolean true required: '+$Label) }
}
function Read-ProfileForAudit([string]$Path) {
  $path=Assert-LocalPath $Path
  $size=(Get-Item -LiteralPath $path).Length
  if ($size -le 0 -or $size -gt 4194304) { throw 'Profile size outside read-only audit bounds.' }
  $encoding=New-Object Text.UTF8Encoding($false,$true)
  $text=$encoding.GetString([IO.File]::ReadAllBytes($path)).TrimStart([char]0xfeff)
  if ($text -match '[\x00-\x08\x0b\x0c\x0e-\x1f]') { throw 'Corrupt/non-text profile; no application launch.' }
  $values=@{};$section='';$sections=@{}
  foreach ($raw in ($text -split '\r?\n')) {
    $line=$raw.Trim()
    if (-not $line -or $line.StartsWith('#') -or $line.StartsWith(';')) { continue }
    if ($line -match '^\[([^\[\]]+)\]$') {
      $section=$Matches[1]
      if ($sections.ContainsKey($section)) { throw 'Duplicate profile section requires manual review.' }
      $sections[$section]=$true;continue
    }
    $separator=$line.IndexOf('=')
    if (-not $section -or $separator -le 0) { throw 'Unrecognized profile syntax; no application launch.' }
    $key=$section+'/'+$line.Substring(0,$separator).Trim()
    if ($values.ContainsKey($key)) { throw 'Duplicate profile key requires manual review.' }
    $values[$key]=$line.Substring($separator+1)
  }
  if (-not $sections.ContainsKey('Settings')) { throw 'Missing normal OpenCPN settings; no application launch.' }
  return $values
}
function Assert-InputOnlyProfile($Values) {
  # Pinned ConnectionParams::Deserialize: IOSelect index 8; enabled index 17.
  # Reject malformed/ambiguous values instead of reproducing wxAtoi fallback.
  $connections=$Values['Settings/NMEADataSource/DataConnections']
  if ($connections) {
    foreach ($connection in ($connections -split '\|')) {
      if (-not $connection) { continue }
      $fields=$connection.Split(';')
      if ($fields.Count -lt 18 -or $fields[8] -cnotmatch '^[012]$' -or $fields[17] -cnotmatch '^[01]$') { throw 'Connection direction cannot be verified as read-only.' }
      if ($fields[17] -ceq '1' -and $fields[8] -cne '0') { throw 'Enabled OpenCPN output connection; no remote launch.' }
    }
  }
  $persist=$Values['Settings/PersistActiveRoute']
  if ($null -ne $persist -and $persist -cnotmatch '^[01]$') { throw 'Ambiguous persisted route state.' }
  if ($persist -ceq '1' -and $Values['Settings/ActiveRoute']) { throw 'Persisted active route requires a separate safe review; no remote launch.' }
  if ($Values['Directories/pluginInstallDir']) { throw 'Custom plugin search directory requires a separately implemented path audit; no remote launch.' }
}
function Get-AuditPluginCandidates([string[]]$Roots) {
  $seen=@{}
  foreach ($root in $Roots) {
    $root=Assert-LocalPath $root
    if (-not [IO.Directory]::Exists($root)) { continue }
    # Inspect before recursing, so redirected subdirectories are never followed.
    $pending=New-Object 'Collections.Generic.Queue[string]';$pending.Enqueue($root)
    while ($pending.Count) {
      foreach ($entry in Get-ChildItem -LiteralPath $pending.Dequeue() -Force) {
        if ($entry.Attributes -band [IO.FileAttributes]::ReparsePoint) { throw 'Redirected plugin path; no remote launch.' }
        if ($entry.PSIsContainer) {$pending.Enqueue($entry.FullName)}
        elseif ($entry.Name -like '*_pi.dll') {
          if ($seen.Count -ge 256) { throw 'Plugin inventory exceeds audit bound.' }
          if (-not $seen.ContainsKey($entry.FullName)) {$seen[$entry.FullName]=$true;$entry.FullName}
        }
      }
    }
  }
}
function Assert-PluginAudit([string[]]$Candidates,$Records) {
  $seen=@{}
  foreach ($plugin in @($Records)) {
    $path=Assert-LocalPath $plugin.path
    Assert-TrueBoolean $plugin.startupAndIdleReadOnly 'Plugin startup and idle behavior reviewed'
    if ($seen.ContainsKey($path) -or $plugin.sha256 -cnotmatch '^[a-f0-9]{64}$') { throw 'Invalid/duplicate plugin audit entry.' }
    if ($path -notin $Candidates -or (Get-Digest $path) -cne $plugin.sha256) { throw 'Plugin changed or outside complete current candidate inventory.' }
    $seen[$path]=$true
  }
  if ($seen.Count -ne @($Candidates).Count) { throw 'Plugin inventory incomplete; disabled plugins also require startup review.' }
}
function Assert-ReadOnlyAudit($Config,$Installed) {
  # An operator/code inspection creates this short-lived attestation only after
  # examining real connection directions, plugins and active-route state.
  # Never edit the live profile to make a test pass or silently disable sensors.
  $audit=$Config.readOnlyAudit
  Assert-TrueBoolean $audit.connectionsOutputDisabled 'Connection outputs disabled'
  Assert-TrueBoolean $audit.pluginOutputsReviewed 'Plugin outputs reviewed'
  Assert-TrueBoolean $audit.noActiveRouteOutput 'No active route output'
  $at=[DateTime]::Parse($audit.reviewedUtc).ToUniversalTime()
  if ($at -gt [DateTime]::UtcNow -or ([DateTime]::UtcNow-$at).TotalHours -gt 24) { throw 'Boat read-only audit expired; inspect current profile and plugin configuration.' }
  $normalProfile=Assert-LocalPath (Join-Path ([Environment]::GetFolderPath('CommonApplicationData')) 'opencpn')
  if ((Assert-LocalPath $Config.profileDirectory) -ine $normalProfile) { throw 'Audit must cover the actual normal OpenCPN profile.' }
  if ((Assert-LocalPath $Config.stockExecutable) -ine (Assert-LocalPath $Installed.state.stock.path)) { throw 'Audit stock installation differs from installed integration.' }
  $ini=Join-Path $normalProfile 'opencpn.ini'
  if ((Get-Digest $ini) -cne $audit.profileIniSha256) { throw 'Profile changed since read-only audit; review before launch.' }
  if ($audit.buildCommit -cne $Installed.ownership.commit) { throw 'Read-only audit belongs to another build.' }
  $values=Read-ProfileForAudit $ini;Assert-InputOnlyProfile $values
  $local=[Environment]::GetFolderPath('LocalApplicationData')
  if (-not $env:LOCALAPPDATA -or (Assert-LocalPath $env:LOCALAPPDATA) -ine (Assert-LocalPath $local)) { throw 'Plugin environment differs from audited interactive user.' }
  $roots=@((Join-Path $local 'opencpn\plugins'),(Join-Path ([IO.Path]::GetDirectoryName($Installed.executable)) 'plugins'))
  # OpenCPN loads/constructs candidates before checking bEnabled. Review ALL
  # candidate DLLs, including disabled plugins, rather than trusting that flag.
  $candidates=@(Get-AuditPluginCandidates $roots)
  Assert-PluginAudit $candidates $audit.pluginFiles
  if ((Get-Digest $ini) -cne $audit.profileIniSha256) { throw 'Profile changed during read-only audit.' }
}
function Invoke-InteractiveJob([string]$Workspace,$Job,[int]$TimeoutSeconds=90) {
  $directory=New-RunDirectory $Workspace $Job.action.ToLowerInvariant()
  $request=Join-Path $directory 'request.json';$result=Join-Path $directory 'result.json'
  $Job | Add-Member -NotePropertyName resultPath -NotePropertyValue $result
  if ($Job.action -ceq 'Launch') { $Job | Add-Member -NotePropertyName workspace -NotePropertyValue (Assert-LocalPath $Workspace) }
  Write-Record $request $Job
  $script=Assert-LocalPath (Join-Path $PSScriptRoot 'InteractiveJob.ps1')
  $sid=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value
  $explorer=@(Get-CimInstance Win32_Process -Filter "Name='explorer.exe'" | Where-Object {
    (Invoke-CimMethod -InputObject $_ -MethodName GetOwnerSid).Sid -eq $sid
  })
  if ($explorer.Count -ne 1) { throw 'One unlocked interactive desktop for this SSH account is required; no task started.' }
  $name='OpenNavX-Boat-'+[guid]::NewGuid().ToString('N')
  $action=New-ScheduledTaskAction -Execute (Join-Path $env:WINDIR 'System32\WindowsPowerShell\v1.0\powershell.exe') -Argument ('-NoProfile -NonInteractive -ExecutionPolicy Bypass -File "'+$script+'" -Request "'+$request+'"')
  $principal=New-ScheduledTaskPrincipal -UserId $sid -LogonType Interactive -RunLevel Limited
  $settings=New-ScheduledTaskSettingsSet -ExecutionTimeLimit (New-TimeSpan -Minutes 5) -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries
  $task=$null
  try {
    $task=Register-ScheduledTask -TaskName $name -Action $action -Principal $principal -Settings $settings
    Start-ScheduledTask -TaskName $name
    $deadline=[DateTime]::UtcNow.AddSeconds($TimeoutSeconds)
    while (-not [IO.File]::Exists($result) -and [DateTime]::UtcNow -lt $deadline) { Start-Sleep -Milliseconds 250 }
    if (-not [IO.File]::Exists($result)) { throw 'Interactive job timed out. Inspect the desktop; no application was force-killed.' }
    $value=Read-Record $result
    if ($value.status -cne 'passed') { throw ('Interactive job failed: '+$value.error) }
    return $value
  } finally {
    if ($task) { Unregister-ScheduledTask -TaskName $name -Confirm:$false }
  }
}

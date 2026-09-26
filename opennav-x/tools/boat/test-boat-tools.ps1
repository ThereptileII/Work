# Native disposable filesystem tests. Does not use real boat config or hardware.
[CmdletBinding()]
param()
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'
if ($env:GITHUB_ACTIONS -ne 'true' -or [Environment]::OSVersion.Platform -ne 'Win32NT') {throw 'Run only on disposable native Windows CI.'}
. (Join-Path $PSScriptRoot 'Common.ps1')
$root=Join-Path ([IO.Path]::GetTempPath()) ('OpenNav boat tools '+[guid]::NewGuid().ToString('N'))
$null=New-Item -ItemType Directory -Path $root
$checks=New-Object 'Collections.Generic.List[string]'
try {
  $stock=Join-Path $root 'stock';$profile=Join-Path $root 'profile';$workspace=Join-Path $root 'workspace'
  $null=New-Item -ItemType Directory -Path $stock,$profile
  [IO.File]::WriteAllBytes((Join-Path $stock 'opencpn.exe'),[byte[]]@(1,2,3,4))
  [IO.File]::WriteAllBytes((Join-Path $profile 'opencpn.ini'),(New-Object byte[] 21380))
  [IO.File]::WriteAllText((Join-Path $profile 'navigation & chart.txt'),'unchanged fixture')
  [IO.File]::WriteAllText((Join-Path $stock 'empty.dat'),'')
  $backup=& (Join-Path $PSScriptRoot 'backup-environment.ps1') -Workspace $workspace -OpenCpnDirectory $stock -ProfileDirectory $profile | ConvertFrom-Json
  if ($backup.status -ne 'verified' -or $backup.applicationFiles -ne 2 -or $backup.profileFiles -ne 2) {throw 'Cold backup inventory failed.'}
  foreach ($pair in @(@($stock,'opencpn'),@($profile,'profile'))) {
    foreach ($file in Get-ChildItem -LiteralPath $pair[0] -File) {
      if ((Get-Digest $file.FullName) -cne (Get-Digest (Join-Path $backup.backup ($pair[1]+'\'+$file.Name)))) {throw 'Backup content mismatch.'}
    }
  }
  $checks.Add('Cold copy verifies multiple files, empty files, zero-filled 21380-byte profile and literal ampersand filename')
  $rejected=$false
  try {& (Join-Path $PSScriptRoot 'backup-environment.ps1') -Workspace (Join-Path $profile 'nested') -OpenCpnDirectory $stock -ProfileDirectory $profile} catch {$rejected=$true}
  if (-not $rejected) {throw 'Nested backup accepted.'}
  $checks.Add('Backup cannot recurse into original application/profile')
  $portable=Join-Path $root 'old portable'; $null=New-Item -ItemType Directory -Path (Join-Path $portable 'app'),(Join-Path $portable 'profile')
  [IO.File]::WriteAllText((Join-Path $portable 'app\OPENNAV_PORTABLE_PREVIEW'),'old explicitly owned fixture')
  [IO.File]::WriteAllText((Join-Path $portable 'FILE_SHA256.json'),'{}')
  [IO.File]::WriteAllText((Join-Path $portable 'profile\keep.txt'),'private navigation must survive')
  $manifestHash=Get-Digest (Join-Path $portable 'FILE_SHA256.json')
  $retired=& (Join-Path $PSScriptRoot 'retire-portable.ps1') -Workspace $workspace -Directory $portable -ExpectedManifestSha256 $manifestHash | ConvertFrom-Json
  if ($retired.status -cne 'retired' -or [IO.Directory]::Exists($portable) -or (Read-Record $retired.completionRecord).manifestSha256 -cne $manifestHash) {throw 'Retirement publication failed.'}
  if ([IO.File]::ReadAllText((Join-Path $retired.recoveryDirectory 'profile\keep.txt')) -cne 'private navigation must survive') {throw 'Retirement lost user data.'}
  $plan=Read-Record ($retired.completionRecord.Replace('.json','.planned.json'))
  if ($plan.status -cne 'planned' -or $plan.originalDirectory -ine $portable -or $plan.recoveryDirectory -ine $retired.recoveryDirectory) {throw 'Durable before-move recovery locator missing.'}
  $checks.Add('Portable retirement atomically archives user data with durable before-move locator and completion evidence')
  $path=Join-Path $root 'evidence.json';Write-Record $path @{status='passed';sentinel=7}
  if ((Read-Record $path).sentinel -ne 7 -or @(Get-ChildItem $root -Filter '*.partial').Count) {throw 'Atomic report publication failed.'}
  $rejected=$false;try {Write-Record $path @{sentinel=8}} catch {$rejected=$true}
  if (-not $rejected -or (Read-Record $path).sentinel -ne 7) {throw 'Existing evidence was overwritten.'}
  $checks.Add('Atomic JSON publishes complete records and preserves existing evidence')
  foreach ($path in @('\\server\share\app','C:\',('C:\bad'+[char]10+'path'))) {
    $rejected=$false;try {$null=Assert-LocalPath $path} catch {$rejected=$true}
    if (-not $rejected) {throw 'Unsafe path accepted.'}
  }
  $checks.Add('UNC, drive root and control-character paths refused')
  foreach ($value in @($false,'false','true',1,0,$null)) {
    $rejected=$false;try {Assert-TrueBoolean $value 'test'} catch {$rejected=$true}
    if (-not $rejected) {throw 'Non-boolean audit accepted.'}
  }
  Assert-TrueBoolean $true 'test'
  $checks.Add('Only explicit JSON boolean true can attest a safety review')
  $auditIni=Join-Path $root 'audit.ini'
  foreach ($contents in @('',"[Settings]`nBad", "[Settings]`nKey=1`nKey=2", "[Settings]`n[Settings]", ('[Settings]'+[char]0+'abc'))) {
    [IO.File]::WriteAllText($auditIni,$contents)
    $rejected=$false;try {$null=Read-ProfileForAudit $auditIni} catch {$rejected=$true}
    if (-not $rejected) {throw 'Malformed profile accepted for remote launch.'}
  }
  $rejected=$false;try {$null=Read-ProfileForAudit (Join-Path $profile 'opencpn.ini')} catch {$rejected=$true}
  if (-not $rejected) {throw 'Zero-filled profile accepted for remote launch.'}
  $checks.Add('Empty, zero-filled, control-character, malformed and ambiguous profiles cannot launch')
  $fields=@('0','0','127.0.0.1','10110','0','COM1','4800','1','0','0','','0','','','0','0','0','1')
  [IO.File]::WriteAllText($auditIni,("[Settings]`nPersistActiveRoute=0`n[Settings/NMEADataSource]`nDataConnections="+($fields -join ';')))
  $values=Read-ProfileForAudit $auditIni;Assert-InputOnlyProfile $values
  foreach ($direction in @('1','2','wrong')) {
    $fields[8]=$direction;$values['Settings/NMEADataSource/DataConnections']=$fields -join ';'
    $rejected=$false;try {Assert-InputOnlyProfile $values} catch {$rejected=$true}
    if (-not $rejected) {throw 'Output or ambiguous connection accepted.'}
  }
  $fields[8]='1';$fields[17]='0';$values['Settings/NMEADataSource/DataConnections']=$fields -join ';';Assert-InputOnlyProfile $values
  $checks.Add('Pinned connection direction fields permit enabled input and reject enabled output or malformed direction')
  $values['Settings/PersistActiveRoute']='1';$values['Settings/ActiveRoute']='active-fixture'
  $rejected=$false;try {Assert-InputOnlyProfile $values} catch {$rejected=$true}
  if (-not $rejected) {throw 'Persisted active route accepted.'}
  $values['Settings/PersistActiveRoute']='0';$values['Directories/pluginInstallDir']='custom'
  $rejected=$false;try {Assert-InputOnlyProfile $values} catch {$rejected=$true}
  if (-not $rejected) {throw 'Custom plugin path accepted without audited normalization.'}
  $checks.Add('Persisted active route and unsupported custom plugin path refuse launch')
  $plugins=Join-Path $root 'plugins';$nested=Join-Path $plugins 'nested';$null=New-Item -ItemType Directory -Path $nested
  $plugin=Join-Path $nested 'disabled_pi.dll';[IO.File]::WriteAllText($plugin,'candidate constructor must still be reviewed')
  $candidates=@(Get-AuditPluginCandidates @($plugins))
  if ($candidates.Count -ne 1) {throw 'Recursive plugin candidate missed.'}
  $rejected=$false;try {Assert-PluginAudit $candidates @()} catch {$rejected=$true}
  if (-not $rejected) {throw 'Incomplete plugin audit accepted.'}
  $record=[pscustomobject]@{path=$plugin;sha256=(Get-Digest $plugin);startupAndIdleReadOnly=$true}
  Assert-PluginAudit $candidates @($record)
  $record.startupAndIdleReadOnly='false';$rejected=$false;try {Assert-PluginAudit $candidates @($record)} catch {$rejected=$true}
  if (-not $rejected) {throw 'String false plugin audit accepted.'}
  $record.startupAndIdleReadOnly=$true;[IO.File]::AppendAllText($plugin,' changed')
  $rejected=$false;try {Assert-PluginAudit $candidates @($record)} catch {$rejected=$true}
  if (-not $rejected) {throw 'Changed plugin accepted.'}
  $checks.Add('Complete recursive plugin inventory includes disabled DLLs and rejects missing, changed or non-boolean reviews')
  [pscustomobject]@{status='passed';checks=@($checks);count=$checks.Count} | ConvertTo-Json -Depth 5
} finally {Remove-Item -LiteralPath $root -Recurse -Force}

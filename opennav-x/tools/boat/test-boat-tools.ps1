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
  . (Join-Path $PSScriptRoot 'PortableReview.ps1')
  foreach ($relative in @('../bad','/root/file','app\file','app/C:stream','app/CON.txt','app/file.','app//file')) {
    $rejected=$false;try {Assert-ReviewRelativePath $relative} catch {$rejected=$true}
    if (-not $rejected) {throw 'Unsafe recovery archive path accepted.'}
  }
  Assert-ReviewRelativePath 'app/ordinary file.dll'
  $checks.Add('Portable extraction refuses traversal, ADS, drive paths, duplicate separators and Windows reserved names')
  Add-Type -AssemblyName System.IO.Compression
  Add-Type -AssemblyName System.IO.Compression.FileSystem
  foreach ($archiveNames in @(@('OpenNavX-Beta2-Portable-Recovery/../outside'),@('wrong-root/file'),@('OpenNavX-Beta2-Portable-Recovery/app/A','OpenNavX-Beta2-Portable-Recovery/app/a'))) {
    $zipPath=Join-Path $root ([guid]::NewGuid().ToString('N')+'.zip')
    $zip=[IO.Compression.ZipFile]::Open($zipPath,[IO.Compression.ZipArchiveMode]::Create)
    try {foreach ($name in $archiveNames) {$null=$zip.CreateEntry($name)}} finally {$zip.Dispose()}
    $rejected=$false;try {$null=Expand-ReviewArchive $zipPath (Join-Path $root ([guid]::NewGuid().ToString('N')))} catch {$rejected=$true}
    if (-not $rejected) {throw 'Unsafe recovery ZIP extracted.'}
  }
  $checks.Add('Portable ZIP rejects traversal, wrong product root and case-colliding entries before extracting files')
  $package=Join-Path $root 'OpenNavX-Beta2-Portable-Recovery'
  foreach ($directory in @('app/plugins','profile/plugins','logs','docs')) {$null=New-Item -ItemType Directory -Path (Join-Path $package $directory) -Force}
  foreach ($directory in @('app/plugins','profile/plugins')) {
    foreach ($name in @('chartdldr_pi.dll','dashboard_pi.dll','grib_pi.dll','wmm_pi.dll')) {[IO.File]::WriteAllText((Join-Path $package ($directory+'/'+$name)),'owned fixture DLL; never executed')}
  }
  [IO.File]::WriteAllText((Join-Path $package 'app/OPENNAV_PORTABLE_PREVIEW'),'isolated marker')
  [IO.File]::WriteAllText((Join-Path $package 'app/opencpn.exe'),'fixture; never executed')
  [IO.File]::WriteAllText((Join-Path $package 'logs/README.txt'),'isolated logs')
  $commit='a'*40
  $build=@{version='0.4.0-beta2';commit=$commit;test_fixtures=$false;build_purpose='INSTALLED PRODUCT';executable_sha256=(Get-Digest (Join-Path $package 'app/opencpn.exe'))}
  $buildPath=Join-Path $package 'docs/PRODUCT_BUILD.json';Write-Record $buildPath $build
  $reviewProfile=Join-Path $package 'profile';$reviewIni=Join-Path $reviewProfile 'opencpn.conf'
  $iniText="[Settings]`nPersistActiveRoute=0`n"
  foreach ($name in @('chartdldr_pi.dll','dashboard_pi.dll','grib_pi.dll','wmm_pi.dll')) {$iniText+='[PlugIns/'+$name+"]`nbEnabled=0`n"}
  [IO.File]::WriteAllText($reviewIni,$iniText)
  function SaveFixtureManifest {
    $entries=@{}
    foreach ($file in @(Get-ReviewFiles $package)) {
      if ($file.Name -ne 'FILE_SHA256.json') {$entries[$file.FullName.Substring($package.Length+1).Replace('\','/')]=Get-Digest $file.FullName}
    }
    [IO.File]::WriteAllText((Join-Path $package 'FILE_SHA256.json'),($entries | ConvertTo-Json))
    return Get-Digest (Join-Path $package 'FILE_SHA256.json')
  }
  $hash=SaveFixtureManifest
  $null=Assert-ReviewPackage $package $hash $commit $true;Assert-ReviewProfile $reviewProfile
  $validZip=Join-Path $root 'accepted-recovery.zip'
  [IO.Compression.ZipFile]::CreateFromDirectory($package,$validZip,[IO.Compression.CompressionLevel]::Optimal,$true)
  $extracted=Expand-ReviewArchive $validZip (Join-Path $root 'accepted-extraction')
  $null=Assert-ReviewPackage $extracted $hash $commit $true
  [IO.File]::WriteAllText((Join-Path $package 'app/extra.dll'),'not in accepted package')
  $rejected=$false;try {$null=Assert-ReviewPackage $package $hash $commit} catch {$rejected=$true}
  if (-not $rejected) {throw 'Extra recovery DLL accepted.'}
  Remove-Item -LiteralPath (Join-Path $package 'app/extra.dll')
  foreach ($fixtures in @($true,'false')) {
    $build.test_fixtures=$fixtures;[IO.File]::WriteAllText($buildPath,($build | ConvertTo-Json));$hash=SaveFixtureManifest
    $rejected=$false;try {$null=Assert-ReviewPackage $package $hash $commit} catch {$rejected=$true}
    if (-not $rejected) {throw 'Fixture-enabled or ambiguous product identity accepted.'}
  }
  $build.test_fixtures=$false;[IO.File]::WriteAllText($buildPath,($build | ConvertTo-Json));$hash=SaveFixtureManifest
  $null=Assert-ReviewPackage $package $hash $commit
  $checks.Add('Portable manifest enforces exact OFF product, unchanged executable and no unowned DLLs')
  $inputFields=@('0','0','127.0.0.1','10110','0','COM1','4800','1','0','0','','0','','','0','0','0','1')
  foreach ($addition in @(("[Settings/NMEADataSource]`nDataConnections="+($inputFields -join ';')), "[OpenNav]`nAlphaSettings=imported vessel configuration", "[Directories]`npluginInstallDir=custom")) {
    [IO.File]::WriteAllText($reviewIni,($iniText+$addition))
    $rejected=$false;try {Assert-ReviewProfile $reviewProfile} catch {$rejected=$true}
    if (-not $rejected) {throw 'Recovery profile accepted marine input or imported configuration.'}
  }
  [IO.File]::WriteAllText($reviewIni,($iniText.Replace('bEnabled=0','bEnabled=1')))
  $rejected=$false;try {Assert-ReviewProfile $reviewProfile} catch {$rejected=$true}
  if (-not $rejected) {throw 'Recovery plugin initialization unexpectedly enabled.'}
  [IO.File]::WriteAllText($reviewIni,$iniText)
  [IO.File]::WriteAllText((Join-Path $reviewProfile 'navobj.xml'),'<gpx><wpt lat="1" lon="2"/></gpx>')
  $rejected=$false;try {Assert-ReviewProfile $reviewProfile} catch {$rejected=$true}
  if (-not $rejected) {throw 'Recovery profile imported navigation objects.'}
  [IO.File]::WriteAllText((Join-Path $reviewProfile 'navobj.xml'),'<gpx/>');Assert-ReviewProfile $reviewProfile
  $checks.Add('Display-only profile rejects even input connections, imported settings/navigation, enabled plugins and custom paths')
  $reviewRecord=Join-Path $root 'portable-review.json'
  Write-Record $reviewRecord @{owner='OpenNavX.PortableDisplayReview.1';purpose='DISPLAY ONLY; NO INSTALLED, CHART OR HARDWARE ACCEPTANCE';package=$package;manifestSha256=$hash;commit=$commit;protectedRoots=@(Get-NormalOpenCpnRoots);protectedFiles=@()}
  $recordHash=Get-Digest $reviewRecord
  $null=Read-PortableReview $reviewRecord $recordHash
  [IO.File]::WriteAllText($reviewIni,($iniText+"[OpenNav]`nAlphaSettings=unexpected configuration"))
  $rejected=$false;try {$null=Read-PortableReview $reviewRecord $recordHash} catch {$rejected=$true}
  if (-not $rejected) {throw 'Changed profile authorized a new display launch.'}
  $null=Read-PortableReview $reviewRecord $recordHash $false
  $rejected=$false;try {$null=Read-PortableReview $reviewRecord ('0'*64) $false} catch {$rejected=$true}
  if (-not $rejected) {throw 'Changed review ownership record authorized process actions.'}
  [IO.File]::WriteAllText($reviewIni,$iniText)
  $checks.Add('Changed display profile blocks relaunch while retaining identified capture/close; changed ownership record refuses all actions')
  $protected=Join-Path $root 'protected';$null=New-Item -ItemType Directory -Path $protected
  [IO.File]::WriteAllText((Join-Path $protected 'normal.ini'),'must survive')
  $before=@(Get-ProtectedInventory @($protected));Assert-ProtectedInventory @($protected) $before
  [IO.File]::WriteAllText((Join-Path $protected 'normal.ini'),'changed')
  $rejected=$false;try {Assert-ProtectedInventory @($protected) $before} catch {$rejected=$true}
  if (-not $rejected) {throw 'Protected original file change accepted.'}
  $checks.Add('Normal-installation/profile hash inventory detects modifications without changing originals')
  $download=Join-Path $root 'OpenNavX-Beta1 (2).zip'
  [IO.File]::WriteAllText($download,'explicitly identified obsolete download')
  $downloadHash=Get-Digest $download
  $rejected=$false;try {$null=& (Join-Path $PSScriptRoot 'retire-download.ps1') -Workspace $workspace -File $download -ExpectedSha256 ('0'*64)} catch {$rejected=$true}
  if (-not $rejected -or (Get-Digest $download) -cne $downloadHash) {throw 'Wrong-hash download retirement changed its source.'}
  $retired=& (Join-Path $PSScriptRoot 'retire-download.ps1') -Workspace $workspace -File $download -ExpectedSha256 $downloadHash | ConvertFrom-Json
  if ($retired.status -cne 'retired' -or [IO.File]::Exists($download) -or (Get-Digest $retired.recoveryFile) -cne $downloadHash) {throw 'Download retirement did not preserve exact bytes.'}
  $journal=Read-Record ([IO.Path]::ChangeExtension($retired.recoveryFile,'.planned.json'))
  if ($journal.status -cne 'planned' -or $journal.originalFile -ine $download -or $journal.recoveryFile -ine $retired.recoveryFile) {throw 'Download before-move recovery locator missing.'}
  $unrelated=Join-Path $root 'personal.zip';[IO.File]::WriteAllText($unrelated,'unrelated data')
  $rejected=$false;try {$null=& (Join-Path $PSScriptRoot 'retire-download.ps1') -Workspace $workspace -File $unrelated -ExpectedSha256 (Get-Digest $unrelated)} catch {$rejected=$true}
  if (-not $rejected -or -not [IO.File]::Exists($unrelated)) {throw 'Unrelated download was retired.'}
  $checks.Add('Obsolete download archive preserves exact bytes and durable locator; wrong hash and unrelated names cannot move')
  [pscustomobject]@{status='passed';checks=@($checks);count=$checks.Count} | ConvertTo-Json -Depth 5
} finally {Remove-Item -LiteralPath $root -Recurse -Force}

# Isolated display review only. This never qualifies or patches installed OpenCPN.
. (Join-Path $PSScriptRoot 'Common.ps1')
function Get-ReviewFiles([string]$Root) {
  $root=Assert-LocalPath $Root
  if (-not [IO.Directory]::Exists($root)) { return }
  $pending=New-Object 'Collections.Generic.Queue[string]';$pending.Enqueue($root)
  $count=0
  while ($pending.Count) {
    foreach ($entry in Get-ChildItem -LiteralPath $pending.Dequeue() -Force) {
      if ($entry.Attributes -band [IO.FileAttributes]::ReparsePoint) { throw 'Redirected review/protected path refused.' }
      if ($entry.PSIsContainer) {$pending.Enqueue($entry.FullName)} else {
        $count++;if ($count -gt 20000) { throw 'Review file inventory exceeds bound.' }
        $entry
      }
    }
  }
}
function Assert-ReviewRelativePath([string]$Path) {
  if (-not $Path -or $Path -match '[\\:\x00-\x1f]' -or $Path.StartsWith('/')) { throw 'Unsafe archive/manifest path.' }
  foreach ($part in $Path.Split('/')) {
    if (-not $part -or $part -in @('.','..') -or $part -match '[ .]$' -or $part -match '^(?i:CON|PRN|AUX|NUL|COM[1-9]|LPT[1-9])(?:\.|$)') { throw 'Unsafe Windows archive path component.' }
  }
}
function Expand-ReviewArchive([string]$Archive,[string]$Destination) {
  Add-Type -AssemblyName System.IO.Compression
  Add-Type -AssemblyName System.IO.Compression.FileSystem
  $archive=Assert-LocalPath $Archive;$destination=Assert-LocalPath $Destination
  if (Test-Path -LiteralPath $destination) { throw 'Use a new review extraction directory.' }
  $null=New-Item -ItemType Directory -Path $destination
  $zip=[IO.Compression.ZipFile]::OpenRead($archive)
  try {
    $seen=@{};$total=[long]0
    if ($zip.Entries.Count -lt 1 -or $zip.Entries.Count -gt 20000) { throw 'Archive entry count outside bounds.' }
    foreach ($entry in $zip.Entries) {
      Assert-ReviewRelativePath $entry.FullName
      if (-not $entry.FullName.StartsWith('OpenNavX-Beta2-Portable-Recovery/',[StringComparison]::Ordinal) -or $seen.ContainsKey($entry.FullName)) { throw 'Wrong recovery archive root or duplicate entry.' }
      if (($entry.ExternalAttributes -shr 16 -band 61440) -eq 40960) { throw 'Archive links are forbidden.' }
      $total+=$entry.Length
      if ($entry.Length -gt 1073741824 -or $total -gt 2147483648) { throw 'Archive expanded size outside bounds.' }
      $seen[$entry.FullName]=$true
    }
    foreach ($entry in $zip.Entries) {
      $target=Assert-LocalPath (Join-Path $destination $entry.FullName.Replace('/','\'))
      $null=New-Item -ItemType Directory -Path ([IO.Path]::GetDirectoryName($target)) -Force
      [IO.Compression.ZipFileExtensions]::ExtractToFile($entry,$target,$false)
    }
  } finally {$zip.Dispose()}
  return Join-Path $destination 'OpenNavX-Beta2-Portable-Recovery'
}
function Assert-ReviewPackage([string]$Package,[string]$ManifestSha256,[string]$Commit,[bool]$Pristine=$false) {
  $package=Assert-LocalPath $Package
  if ($ManifestSha256 -cnotmatch '^[a-f0-9]{64}$' -or $Commit -cnotmatch '^[a-f0-9]{40}$') { throw 'Exact accepted package manifest and commit are required.' }
  $manifestPath=Join-Path $package 'FILE_SHA256.json'
  if ((Get-Digest $manifestPath) -cne $ManifestSha256) { throw 'Recovery manifest changed.' }
  $manifest=Read-Record $manifestPath;$known=@{}
  foreach ($entry in $manifest.PSObject.Properties) {
    Assert-ReviewRelativePath $entry.Name
    if ($known.ContainsKey($entry.Name) -or $entry.Value -isnot [string] -or $entry.Value -cnotmatch '^[a-f0-9]{64}$') { throw 'Invalid recovery manifest entry.' }
    $known[$entry.Name]=$entry.Value
    # Only logs and the explicitly isolated profile may evolve after first run.
    if ($Pristine -or $entry.Name -notlike 'profile/*' -or $entry.Name -like 'profile/plugins/*') {
      if ((Get-Digest (Join-Path $package $entry.Name.Replace('/','\'))) -cne $entry.Value) { throw ('Recovery file changed: '+$entry.Name) }
    }
  }
  foreach ($file in @(Get-ReviewFiles $package)) {
    $name=$file.FullName.Substring($package.Length+1).Replace('\','/')
    if ($name -eq 'FILE_SHA256.json') {continue}
    if ($name -match '(^|/)(demo|OPENNAV_TEST_PROFILE|OPENNAV_ROUTE_FIXTURE|OPENNAV_OBJECT_FIXTURE|scenarios\.json)(/|$)' -or $name -like '*Run-XNav-Demo*' -or $file.Name -eq 'OPENNAV_INSTALLED_STOCK') { throw 'Synthetic/installed-profile marker in portable review.' }
    if (-not $known.ContainsKey($name) -and ($Pristine -or ($name -notlike 'logs/*' -and $name -notlike 'profile/*') -or $name -like 'profile/plugins/*')) { throw 'Unowned file in immutable recovery package.' }
  }
  if (-not $known.ContainsKey('app/OPENNAV_PORTABLE_PREVIEW') -or -not $known.ContainsKey('app/opencpn.exe')) { throw 'Portable isolation marker/executable absent from accepted inventory.' }
  $build=Read-Record (Join-Path $package 'docs\PRODUCT_BUILD.json')
  if ($build.test_fixtures -isnot [bool] -or $build.test_fixtures -ne $false -or $build.build_purpose -cne 'INSTALLED PRODUCT' -or $build.version -cne '0.4.0-beta2' -or $build.commit -cne $Commit -or $build.executable_sha256 -cne $known['app/opencpn.exe']) { throw 'Recovery package is not the exact fixture-free Beta 2 product.' }
  $approved=@('chartdldr_pi.dll','dashboard_pi.dll','grib_pi.dll','wmm_pi.dll')
  foreach ($relative in @('app\plugins','profile\plugins')) {
    $directory=Join-Path $package $relative
    $candidates=@(Get-AuditPluginCandidates @($directory))
    if ($candidates.Count -ne $approved.Count) { throw 'Recovery plugin candidate set differs from reviewed bundled plugins.' }
    foreach ($candidate in $candidates) {
      if ([IO.Path]::GetDirectoryName($candidate) -ine $directory -or [IO.Path]::GetFileName($candidate) -notin $approved) { throw 'Unreviewed/nested recovery plugin candidate.' }
    }
  }
  return [pscustomobject]@{package=$package;executable=(Join-Path $package 'app\opencpn.exe');executableSha256=$known['app/opencpn.exe'];profile=(Join-Path $package 'profile');logs=(Join-Path $package 'logs');commit=$Commit}
}
function Assert-ReviewProfile([string]$Profile) {
  $profile=Assert-LocalPath $Profile
  $values=Read-ProfileForAudit (Join-Path $profile 'opencpn.conf')
  Assert-InputOnlyProfile $values
  # This preliminary screen review has NO marine connection, including inputs.
  if ($values['Settings/NMEADataSource/DataConnections'] -or $values['Settings/ActiveRoute'] -or $values['OpenNav/AlphaSettings']) { throw 'Display review requires an unconfigured isolated vessel profile.' }
  foreach ($plugin in @('chartdldr_pi.dll','dashboard_pi.dll','grib_pi.dll','wmm_pi.dll')) {
    if ($values['PlugIns/'+$plugin+'/bEnabled'] -cne '0') { throw 'Review bundled plugin initialization must stay disabled.' }
  }
  foreach ($file in @(Get-ReviewFiles $profile)) {
    if ($file.Name -ieq 'opencpn.ini') { throw 'A second imported INI is not permitted in the display review profile.' }
    if ($file.Name -like 'navobj*') {
      if ($file.Length -gt 1048576) { throw 'Display review cannot import navigation objects.' }
      $text=[IO.File]::ReadAllText($file.FullName)
      if ($text -match '<(?:[\w.-]+:)?(?:wpt|rte|trk)\b' -or $text -match '<!DOCTYPE|<!ENTITY') { throw 'Display review cannot retain navigation objects or external XML entities.' }
    }
  }
}
function Get-NormalOpenCpnRoots {
  $paths=@()
  foreach ($base in @($env:ProgramFiles,${env:ProgramFiles(x86)},$env:ProgramData,$env:LOCALAPPDATA,$env:APPDATA)) {
    if ($base) {$paths+=Assert-LocalPath (Join-Path $base 'opencpn')}
  }
  return @($paths | Sort-Object -Unique)
}
function Get-ProtectedInventory([string[]]$Roots) {
  $records=New-Object 'Collections.Generic.List[object]'
  foreach ($root in $Roots) {
    foreach ($file in @(Get-ReviewFiles $root)) {
      if ($records.Count -ge 20000) { throw 'Protected file inventory exceeds bound.' }
      $records.Add([pscustomobject]@{path=$file.FullName;sha256=(Get-Digest $file.FullName)})
    }
  }
  return @($records | Sort-Object path)
}
function Assert-ProtectedInventory([string[]]$Roots,$Before) {
  $now=@(Get-ProtectedInventory $Roots)
  if (($now | ConvertTo-Json -Depth 4 -Compress) -cne (@($Before) | ConvertTo-Json -Depth 4 -Compress)) { throw 'Normal OpenCPN files changed; stop the display review and retain evidence.' }
}
function Read-PortableReview([string]$RecordPath,[string]$ExpectedRecordSha256,[bool]$ValidateProfile=$true) {
  if ($ExpectedRecordSha256 -cnotmatch '^[a-f0-9]{64}$' -or (Get-Digest $RecordPath) -cne $ExpectedRecordSha256) { throw 'Portable review record changed.' }
  $record=Read-Record $RecordPath
  if ($record.owner -cne 'OpenNavX.PortableDisplayReview.1' -or $record.purpose -cne 'DISPLAY ONLY; NO INSTALLED, CHART OR HARDWARE ACCEPTANCE') { throw 'Not an isolated display-review record.' }
  $package=Assert-ReviewPackage $record.package $record.manifestSha256 $record.commit
  # Always allow an identified owned process to close normally, even if its
  # profile changed to a state which now prevents another launch.
  if ($ValidateProfile) {Assert-ReviewProfile $package.profile}
  $expectedRoots=@(Get-NormalOpenCpnRoots)
  if (($expectedRoots | ConvertTo-Json -Compress) -cne (@($record.protectedRoots) | ConvertTo-Json -Compress)) { throw 'Interactive user/environment changed since review preparation.' }
  foreach ($root in $expectedRoots) {
    if ($package.package -ieq $root -or $package.package.StartsWith($root+'\',[StringComparison]::OrdinalIgnoreCase)) { throw 'Recovery package overlaps a normal OpenCPN directory.' }
  }
  return [pscustomobject]@{record=$record;product=$package}
}

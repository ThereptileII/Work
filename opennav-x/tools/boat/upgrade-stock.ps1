# Exact, authorized 5.12.2 -> official 5.12.4 pre/postflight. Never launches an
# installer or OpenCPN: the reviewed interactive wizard has a separate driver.
[CmdletBinding()]
param(
  [ValidateSet('Preflight','Verify')][string]$Action='Preflight',
  [string]$Workspace='C:\XNav',
  [string]$Setup,
  [string]$BackupRecord,
  [string]$ExpectedBackupRecordSha256,
  [string]$Record,
  [string]$ExpectedRecordSha256,
  [string]$WizardReport,
  [string]$ExpectedWizardReportSha256,
  [switch]$RestoreMissingPluginFiles
)
. (Join-Path $PSScriptRoot 'Common.ps1')
. (Join-Path $PSScriptRoot 'OfficialUpgradePolicy.ps1')
$SourceHash='2fdcd6a2cdef7f730aa4c094fcd21302ed2a5d531a611ee180c06533f3a2cb48'
$SetupHash='e949f55de57611afe2fc0dad5a8ac33795c46ba488cb40ca07b65f639a07b8aa'
$TargetHash='7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c'
$PluginUninstallerHash='0da3cfb79b1cf2085f6f53c095abe60300f76f95a9503c09ce467a9c833912fb'
function Assert-UpgradeClosed {
  if (@(Get-Process -Name opencpn -ErrorAction SilentlyContinue).Count) {throw 'Close every OpenCPN/XNav instance normally before stock maintenance.'}
}
function Upgrade-Relative([string]$Root,[string]$Relative) {
  if (-not $Relative -or $Relative -match '[:\x00-\x1f]' -or $Relative.StartsWith('/') -or $Relative.StartsWith('\')) {throw 'Unsafe recovery relative path.'}
  foreach ($part in ($Relative -split '[\\/]')) {
    if (-not $part -or $part -in @('.','..') -or $part -match '[ .]$' -or $part -match '^(?i:CON|PRN|AUX|NUL|COM[1-9]|LPT[1-9])(?:\.|$)') {throw 'Unsafe recovery path component.'}
  }
  return Assert-LocalPath (Join-Path (Assert-LocalPath $Root) $Relative)
}
function Get-UpgradeFiles([string]$Root) {
  $root=Assert-LocalPath $Root
  if (-not [IO.Directory]::Exists($root)) {throw 'Required application/profile/recovery tree missing.'}
  $pending=New-Object 'Collections.Generic.Queue[string]';$pending.Enqueue($root)
  $files=New-Object 'Collections.Generic.List[object]'
  while ($pending.Count) {
    foreach ($entry in Get-ChildItem -LiteralPath $pending.Dequeue() -Force) {
      if ($entry.Attributes -band [IO.FileAttributes]::ReparsePoint) {throw 'Redirected maintenance tree refused.'}
      if ($entry.PSIsContainer) {$pending.Enqueue($entry.FullName)} else {
        if ($files.Count -ge 20000) {throw 'Maintenance inventory exceeds bound.'}
        $files.Add([pscustomobject]@{path=$entry.FullName.Substring($root.Length+1);bytes=$entry.Length;sha256=(Get-Digest $entry.FullName)})
      }
    }
  }
  return @($files | Sort-Object path)
}
function Assert-UpgradeFiles([string]$Root,$Expected) {
  $map=@{}
  foreach ($entry in @($Expected)) {
    $null=Upgrade-Relative $Root $entry.path
    if ($map.ContainsKey($entry.path) -or $entry.sha256 -cnotmatch '^[a-f0-9]{64}$' -or $entry.bytes -lt 0) {throw 'Invalid recovery inventory record.'}
    $map[$entry.path]=$entry
  }
  $actual=@(Get-UpgradeFiles $Root)
  if ($actual.Count -ne $map.Count) {throw 'Maintenance inventory file count changed.'}
  foreach ($entry in $actual) {
    $prior=$map[$entry.path]
    if (-not $prior -or $prior.sha256 -cne $entry.sha256 -or $prior.bytes -ne $entry.bytes) {throw ('Maintenance inventory mismatch: '+$entry.path)}
  }
}
function Get-UnbundledPluginFiles($Files) {
  foreach ($entry in @($Files)) {
    $name=$entry.path.Replace('\','/')
    if ($name -ceq 'Uninstall rtlsdr_pi.exe') {$entry;continue}
    if ($name -notmatch '^plugins/') {continue}
    # Preserve complete third-party trees, including ancillary executables,
    # libraries, scripts and licenses. Only these four stock bundles may change.
    if ($name -match '^plugins/(chartdldr_pi|dashboard_pi|grib_pi|wmm_pi)(\.dll$|/)') {continue}
    $entry
  }
}
function Restore-UpgradePlugins([string]$Application,[string]$Backup,$Files,[bool]$Restore) {
  $restored=New-Object 'Collections.Generic.List[string]'
  foreach ($entry in @(Get-UnbundledPluginFiles $Files)) {
    Assert-UpgradeClosed
    $target=Upgrade-Relative $Application $entry.path
    $source=Upgrade-Relative $Backup $entry.path
    if ((Get-Digest $source) -cne $entry.sha256) {throw 'Plugin recovery file changed.'}
    if ([IO.File]::Exists($target)) {
      if ((Get-Digest $target) -cne $entry.sha256) {throw ('Changed third-party plugin file; no overwrite permitted: '+$entry.path)}
    } elseif ($Restore) {
      $null=New-Item -ItemType Directory -Path ([IO.Path]::GetDirectoryName($target)) -Force
      $temporary=$target+'.opennav-recover-'+[guid]::NewGuid().ToString('N')+'.partial'
      Copy-Item -LiteralPath $source -Destination $temporary
      if ((Get-Digest $temporary) -cne $entry.sha256) {throw 'Plugin recovery copy failed verification.'}
      [IO.File]::Move($temporary,$target) # Never replace an existing file.
      $restored.Add($entry.path)
    } else {throw ('Missing third-party plugin file; verified recovery required: '+$entry.path)}
  }
  return @($restored)
}
function Get-UpgradeRegistration {
  $base=[Microsoft.Win32.RegistryKey]::OpenBaseKey([Microsoft.Win32.RegistryHive]::LocalMachine,[Microsoft.Win32.RegistryView]::Registry32)
  $uninstall=$null
  try {
    $uninstall=$base.OpenSubKey('SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall')
    if (-not $uninstall) {throw 'Original OpenCPN uninstall registration unavailable.'}
    foreach ($name in @($uninstall.GetSubKeyNames() | Sort-Object)) {
      if ($name -notmatch '^OpenCPN(?: |$)') {continue}
      $key=$uninstall.OpenSubKey($name)
      try {
        $values=@{}
        foreach ($value in $key.GetValueNames()) {$values[$value]=@{kind=$key.GetValueKind($value).ToString();value=$key.GetValue($value,$null,[Microsoft.Win32.RegistryValueOptions]::DoNotExpandEnvironmentNames)}}
        [pscustomobject]@{key=$name;location=$key.GetValue('InstallLocation');version=$key.GetValue('DisplayVersion');uninstaller=$key.GetValue('UninstallString');values=$values}
      } finally {$key.Dispose()}
    }
  } finally {if ($uninstall) {$uninstall.Dispose()};$base.Dispose()}
}
function Assert-UpgradePe([string]$Path,[int]$Patch) {
  $stream=[IO.File]::OpenRead($Path);$reader=New-Object IO.BinaryReader($stream)
  try {
    if ($stream.Length -lt 128 -or $reader.ReadUInt16() -ne 0x5a4d) {throw 'Expected native PE executable.'}
    $stream.Position=60;$offset=$reader.ReadUInt32()
    if ($offset -gt $stream.Length-24) {throw 'Invalid PE offset.'}
    $stream.Position=$offset
    if ($reader.ReadUInt32() -ne 0x4550 -or $reader.ReadUInt16() -ne 0x14c) {throw 'Expected supported x86 OpenCPN ABI.'}
  } finally {$reader.Dispose();$stream.Dispose()}
  $version=[Diagnostics.FileVersionInfo]::GetVersionInfo($Path)
  if ($version.FileMajorPart -ne 5 -or $version.FileMinorPart -ne 12 -or $version.FileBuildPart -ne $Patch) {throw 'OpenCPN PE version differs from its expected release.'}
}
function Read-UpgradeRecord([string]$Path,[string]$ExpectedHash) {
  if ($ExpectedHash -cnotmatch '^[a-f0-9]{64}$' -or (Get-Digest $Path) -cne $ExpectedHash) {throw 'Exact maintenance record hash required.'}
  return Read-Record $Path
}

if ([Environment]::OSVersion.Platform -ne 'Win32NT') {throw 'Native Windows stock maintenance only.'}
Assert-UpgradeClosed
$Workspace=Assert-LocalPath $Workspace
$application=Assert-LocalPath (Join-Path ${env:ProgramFiles(x86)} 'OpenCPN')
$profile=Assert-LocalPath (Join-Path ([Environment]::GetFolderPath('CommonApplicationData')) 'opencpn')
$executable=Join-Path $application 'opencpn.exe'
foreach ($original in @($application,$profile)) {
  if ($Workspace -ieq $original -or $Workspace.StartsWith($original+'\',[StringComparison]::OrdinalIgnoreCase) -or $original.StartsWith($Workspace+'\',[StringComparison]::OrdinalIgnoreCase)) {throw 'Workspace must be separate from original OpenCPN/profile.'}
}
$directory=$null;$result=$null
try {
  if ($Action -eq 'Preflight') {
    if ($RestoreMissingPluginFiles) {throw 'Plugin recovery is available only during postflight.'}
    $Setup=Assert-LocalPath $Setup;$BackupRecord=Assert-LocalPath $BackupRecord
    if ((Get-Digest $Setup) -cne $SetupHash) {throw 'Not the exact approved official 5.12.4 installer.'}
    if ((Get-Digest $executable) -cne $SourceHash) {throw 'This operation authorizes only the inspected original OpenCPN 5.12.2 executable.'}
    Assert-UpgradePe $executable 2
    $backup=Read-UpgradeRecord $BackupRecord $ExpectedBackupRecordSha256
    if ($backup.schema -ne 1 -or $backup.owner -cne 'OpenNavX.BoatRecovery.1' -or $backup.status -cne 'verified' -or $backup.sourceApplication -ine $application -or $backup.sourceProfile -ine $profile) {throw 'Recovery set does not belong to these exact original application/profile trees.'}
    $backupDirectory=Assert-LocalPath ([IO.Path]::GetDirectoryName($BackupRecord))
    if ($backupDirectory.EndsWith('.partial',[StringComparison]::OrdinalIgnoreCase)) {throw 'Incomplete recovery set refused.'}
    foreach ($pair in @(@('opencpn',$application,$backup.application),@('profile',$profile,$backup.profile))) {
      Assert-UpgradeFiles (Join-Path $backupDirectory $pair[0]) $pair[2]
      Assert-UpgradeFiles $pair[1] $pair[2]
    }
    $registered=Split-OfficialRegistration @(Get-UpgradeRegistration) $application
    $registration=@($registered.core);$pluginRegistration=@($registered.plugins)
    if ($registration[0].version -notmatch '^5\.12\.2(?:-|$)') {throw 'Unexpected original OpenCPN registration version.'}
    $pluginUninstallers=@()
    foreach ($plugin in $pluginRegistration) {
      $pluginUninstaller=Assert-LocalPath $plugin.uninstaller
      if ((Get-Digest $pluginUninstaller) -cne $PluginUninstallerHash) {throw 'Observed RTL-SDR plugin uninstaller hash changed.'}
      $pluginUninstallers+=@([pscustomobject]@{path=$pluginUninstaller;sha256=$PluginUninstallerHash})
    }
    $uninstaller=Assert-LocalPath ($registration[0].uninstaller.Trim('"'))
    if ([IO.Path]::GetDirectoryName($uninstaller) -ine $application -or -not [IO.File]::Exists($uninstaller)) {throw 'Original program-files-only uninstaller not inside the backed-up application.'}
    $drive=New-Object IO.DriveInfo([IO.Path]::GetPathRoot($application))
    if ($drive.AvailableFreeSpace -lt 2147483648) {throw 'At least 2 GiB free installation/recovery margin required.'}
    Assert-UpgradeClosed
    $directory=New-RunDirectory $Workspace 'stock-upgrade'
    $recordPath=Join-Path $directory 'prepared.json'
    Write-Record $recordPath @{schema=1;owner='OpenNavX.StockUpgrade.1';status='prepared';createdUtc=[DateTime]::UtcNow.ToString('o');setup=$Setup;setupSha256=$SetupHash;sourceExecutable=$executable;sourceExecutableSha256=$SourceHash;targetExecutableSha256=$TargetHash;sourceApplication=$application;sourceProfile=$profile;backupRecord=$BackupRecord;backupRecordSha256=$ExpectedBackupRecordSha256;applicationFiles=@($backup.application);profileFiles=@($backup.profile);registrationBefore=$registration;pluginRegistrationBefore=$pluginRegistration;pluginUninstallerBefore=$pluginUninstallers;profileAclBefore=(Get-Acl -LiteralPath $profile).Sddl;originalUninstaller=$uninstaller;originalUninstallerSha256=(Get-Digest $uninstaller);policy='Visible official Upgrade only. No /S substitute, application launch, INI repair or profile mutation. Local private recovery record.'}
    $result=@{status='prepared';record=$recordPath;recordSha256=(Get-Digest $recordPath);applicationFiles=@($backup.application).Count;profileFiles=@($backup.profile).Count;installerStarted=$false;next='Use the separate reviewed interactive wizard; Upgrade, preserve all configuration, Run/Show unchecked. Then Verify.'}
  } else {
    $prepared=Read-UpgradeRecord $Record $ExpectedRecordSha256
    if ($prepared.owner -cne 'OpenNavX.StockUpgrade.1' -or $prepared.status -cne 'prepared' -or $prepared.sourceApplication -ine $application -or $prepared.sourceProfile -ine $profile -or $prepared.sourceExecutableSha256 -cne $SourceHash -or $prepared.setupSha256 -cne $SetupHash -or $prepared.targetExecutableSha256 -cne $TargetHash) {throw 'Unexpected stock upgrade preparation.'}
    $wizard=Read-UpgradeRecord $WizardReport $ExpectedWizardReportSha256
    if ($wizard.owner -cne 'OpenNavX.StockUpgradeWizard.1' -or $wizard.status -cne 'completed' -or $wizard.recordSha256 -cne $ExpectedRecordSha256 -or $wizard.setupSha256 -cne $SetupHash -or $wizard.targetDirectory -ine $application -or $wizard.exitCode -notin @(0,1223)) {throw 'Exact completed visible wizard evidence required.'}
    foreach ($flag in @('upgradeSelected','configResetUnchecked','runUnchecked','showUnchecked','finishObserved')) {Assert-TrueBoolean $wizard.$flag $flag}
    $directory=New-RunDirectory $Workspace 'stock-postflight'
    if ((Get-Digest $executable) -cne $TargetHash) {throw 'Official 5.12.4 executable postcondition failed; do not launch.'}
    Assert-UpgradePe $executable 4
    $backup=Read-UpgradeRecord $prepared.backupRecord $prepared.backupRecordSha256
    $backupDirectory=[IO.Path]::GetDirectoryName($prepared.backupRecord)
    Assert-UpgradeFiles (Join-Path $backupDirectory 'opencpn') $prepared.applicationFiles
    Assert-UpgradeFiles (Join-Path $backupDirectory 'profile') $prepared.profileFiles
    Assert-UpgradeFiles $profile $prepared.profileFiles
    $registered=Split-OfficialRegistration @(Get-UpgradeRegistration) $application
    $registration=@($registered.core);$pluginRegistration=@($registered.plugins)
    if ($registration[0].version -notmatch '^5\.12\.4(?:-|$)') {throw 'Official registration upgrade postcondition failed.'}
    Assert-OfficialPluginRegistration $pluginRegistration @($prepared.pluginRegistrationBefore)
    $pluginUninstallers=@($prepared.pluginUninstallerBefore)
    if ($pluginUninstallers.Count -ne $pluginRegistration.Count) {throw 'Prepared plugin uninstaller inventory mismatch.'}
    foreach ($plugin in $pluginUninstallers) {
      if ($plugin.path -cne (Join-Path $application 'Uninstall rtlsdr_pi.exe') -or $plugin.sha256 -cne $PluginUninstallerHash) {throw 'Unrecognized prepared plugin uninstaller.'}
    }
    $uninstaller=Assert-LocalPath ($registration[0].uninstaller.Trim('"'))
    if ([IO.Path]::GetDirectoryName($uninstaller) -ine $application -or -not [IO.File]::Exists($uninstaller)) {throw 'Official target uninstaller unavailable.'}
    foreach ($resource in @('uidata\styles.xml','s57data\s57objectclasses.csv','gshhs\poly-c-1.dat')) {
      if ((Get-Item -LiteralPath (Join-Path $application $resource)).Length -le 0) {throw 'Required stock resource missing.'}
    }
    if (-not @(Get-ChildItem -LiteralPath (Join-Path $application 'basemap_shp') -Filter '*.shp' -File).Count) {throw 'Official coastline resources missing.'}
    $restored=@(Restore-UpgradePlugins $application (Join-Path $backupDirectory 'opencpn') $prepared.applicationFiles ([bool]$RestoreMissingPluginFiles))
    foreach ($plugin in $pluginUninstallers) {
      if ((Get-Digest $plugin.path) -cne $plugin.sha256) {throw 'Plugin uninstall registration would point to a missing or changed file.'}
    }
    Assert-UpgradeFiles $profile $prepared.profileFiles
    Assert-UpgradeClosed
    $after=@(Get-UpgradeFiles $application)
    Write-Record (Join-Path $directory 'verified.json') @{owner='OpenNavX.StockUpgradePostflight.1';status='verified';createdUtc=[DateTime]::UtcNow.ToString('o');recordSha256=$ExpectedRecordSha256;wizardReportSha256=$ExpectedWizardReportSha256;executableSha256=(Get-Digest $executable);sourceApplication=$application;sourceProfile=$profile;applicationFiles=$after;profileFiles=@($prepared.profileFiles);registrationAfter=$registration;pluginRegistrationAfter=$pluginRegistration;pluginUninstallerAfter=$pluginUninstallers;profileAclAfter=(Get-Acl -LiteralPath $profile).Sddl;restoredPluginFiles=$restored;applicationLaunched=$false;profileRepaired=$false;note='Profile preserved byte-for-byte, including any pre-existing corruption. Read-only launch audit remains mandatory. Official installer may refresh profile ACLs.'}
    $result=@{status='verified';report=(Join-Path $directory 'verified.json');reportSha256=(Get-Digest (Join-Path $directory 'verified.json'));version='5.12.4';architecture='x86';executableSha256=$TargetHash;profileFilesUnchanged=@($prepared.profileFiles).Count;restoredPluginFiles=$restored.Count;applicationLaunched=$false;profileRepaired=$false}
  }
} catch {
  $failureMessage=$_.Exception.Message
  if ($directory) {
    $observed=@{};$collectionErrors=@()
    foreach ($pair in @(@('application',$application),@('profile',$profile))) {
      try {$observed[$pair[0]]=@(Get-UpgradeFiles $pair[1])} catch {$collectionErrors+=$_.Exception.Message}
    }
    Write-Record (Join-Path $directory 'failure.json') @{status='failed';action=$Action;error=$failureMessage;observedFiles=$observed;inventoryErrors=$collectionErrors;applicationLaunched=$false;automaticRollback=$false;note='Retain original recovery set and inspect. No profile/INI restoration or process termination performed.'}
  }
  throw
}
$result | ConvertTo-Json -Depth 4

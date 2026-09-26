# Interactive, elevated, exact-stock upgrade only. No app launch, forced kill,
# reboot, silent setup, independent uninstaller, or remote-access changes.
[CmdletBinding()]
param(
  [Parameter(Mandatory=$true)][string]$Record,
  [Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{64}$')][string]$ExpectedRecordSha256,
  [Parameter(Mandatory=$true)][string]$Output,
  [ValidateRange(60,1200)][int]$TimeoutSeconds=600
)
. (Join-Path $PSScriptRoot 'Common.ps1')
. (Join-Path $PSScriptRoot 'OfficialUpgradePolicy.ps1')
$setupHash='e949f55de57611afe2fc0dad5a8ac33795c46ba488cb40ca07b65f639a07b8aa'
$oldHash='2fdcd6a2cdef7f730aa4c094fcd21302ed2a5d531a611ee180c06533f3a2cb48'
$newHash='7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c'
$destination='C:\Program Files (x86)\OpenCPN'
$result=@{owner='OpenNavX.StockUpgradeWizard.1';status='failed';recordSha256=$ExpectedRecordSha256;
  setupSha256=$setupHash;targetDirectory=$destination;upgradeSelected=$false;configResetUnchecked=$false;
  runUnchecked=$false;showUnchecked=$false;finishObserved=$false;installStarted=$false;exitCode=$null;
  pages=@();startedUtc=[DateTime]::UtcNow.ToString('o');applicationLaunched=$false;forcedTermination=$false}
$process=$null;$setupLock=$null;$lastPage='Start';$lastObserved='';$unknownSince=$null
$Output=Assert-LocalPath $Output
if (Test-Path -LiteralPath $Output) { throw 'A fresh evidence output path is required.' }
if (-not [IO.Directory]::Exists([IO.Path]::GetDirectoryName($Output))) { throw 'Evidence directory must exist.' }
foreach ($protected in @($destination,'C:\ProgramData\opencpn')) {
  if ($Output.StartsWith($protected+'\',[StringComparison]::OrdinalIgnoreCase)) { throw 'Evidence must remain outside the live application/profile.' }
}
function Assert-Closed {
  if (@(Get-Process -Name opencpn -ErrorAction SilentlyContinue).Count) { throw 'OpenCPN/XNav is running; no installer input sent.' }
}
function Read-OriginalRegistration {
  $base=[Microsoft.Win32.RegistryKey]::OpenBaseKey([Microsoft.Win32.RegistryHive]::LocalMachine,[Microsoft.Win32.RegistryView]::Registry32)
  $uninstall=$null
  try {
    $uninstall=$base.OpenSubKey('SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall')
    if (-not $uninstall) { throw 'Original uninstall registry unavailable.' }
    foreach ($name in @($uninstall.GetSubKeyNames() | Sort-Object)) {
      if ($name -notmatch '^OpenCPN(?: |$)') {continue}
      $key=$uninstall.OpenSubKey($name)
      try {
        $values=@{}
        foreach ($value in $key.GetValueNames()) {$values[$value]=[pscustomobject]@{kind=$key.GetValueKind($value).ToString();value=$key.GetValue($value,$null,[Microsoft.Win32.RegistryValueOptions]::DoNotExpandEnvironmentNames)}}
        [pscustomobject]@{key=$name;location=$key.GetValue('InstallLocation');version=$key.GetValue('DisplayVersion');uninstaller=$key.GetValue('UninstallString');values=[pscustomobject]$values}
      } finally {$key.Dispose()}
    }
  } finally {if($uninstall){$uninstall.Dispose()};$base.Dispose()}
}
function Assert-PreparedOriginal {
  Assert-Closed
  if ((Get-Digest $Record) -cne $ExpectedRecordSha256 -or
      (Get-Digest $setup) -cne $setupHash -or
      (Get-Digest $prepared.sourceExecutable) -cne $oldHash -or
      (Get-Digest $prepared.originalUninstaller) -cne $prepared.originalUninstallerSha256 -or
      (Get-Digest $prepared.backupRecord) -cne $prepared.backupRecordSha256) { throw 'Prepared source/setup/uninstaller/recovery identity changed.' }
  if (([DateTime]::UtcNow-$created).TotalMinutes -gt 60) { throw 'Prepared record expired.' }
  $registration=Split-OfficialRegistration @(Read-OriginalRegistration) $destination
  Assert-OfficialRegistration @($registration.core) @($prepared.registrationBefore)
  Assert-OfficialPluginRegistration @($registration.plugins) @($prepared.pluginRegistrationBefore)
  if(@($prepared.pluginUninstallerBefore).Count -ne @($prepared.pluginRegistrationBefore).Count){throw 'Plugin recovery identity is incomplete.'}
  foreach($plugin in @($prepared.pluginUninstallerBefore)) {
    if($plugin.path -cne ($destination+'\Uninstall rtlsdr_pi.exe') -or $plugin.sha256 -cne '0da3cfb79b1cf2085f6f53c095abe60300f76f95a9503c09ce467a9c833912fb' -or
        (Get-Digest $plugin.path) -cne $plugin.sha256){throw 'Prepared RTL-SDR uninstaller identity changed.'}
  }
}
function Assert-SetupIdentity {
  $process.Refresh()
  if ($process.HasExited -or (Assert-LocalPath $process.MainModule.FileName) -ine $setup) { throw 'Launched official setup PID no longer has the verified image.' }
}
function Select-Button($Controls,[string]$Label) {
  $matches=@($Controls | Where-Object { $_.Class -ceq 'Button' -and $_.Enabled -and $_.Text.Replace('&','').Trim() -ceq $Label })
  if ($matches.Count -ne 1) { throw "Expected one enabled owned button: $Label" }
  return $matches[0]
}
function Capture-Page($Window,$Controls,[string]$Page,[string]$Action) {
  Assert-SetupIdentity
  [OpenNavOfficialWizard]::Foreground($Window.Handle,$process.Id)
  $rect=[OpenNavOfficialWizard]::CaptureBounds($Window.Handle,$process.Id)
  $width=$rect.Right-$rect.Left;$height=$rect.Bottom-$rect.Top
  if ($width -le 0 -or $height -le 0 -or $width -gt 4096 -or $height -gt 2160) { throw 'Setup capture bounds invalid.' }
  $name='official-upgrade-'+$result.pages.Count+'.png';$path=Join-Path ([IO.Path]::GetDirectoryName($Output)) $name
  if (Test-Path -LiteralPath $path) { throw 'Screenshot would overwrite prior evidence.' }
  $bitmap=New-Object Drawing.Bitmap($width,$height);$graphics=[Drawing.Graphics]::FromImage($bitmap)
  try {
    [OpenNavOfficialWizard]::AssertForeground($Window.Handle,$process.Id)
    $graphics.CopyFromScreen($rect.Left,$rect.Top,0,0,$bitmap.Size)
    # Do not reacquire foreground here: that would hide an unrelated-window
    # interruption and could save its private pixels as installer evidence.
    $after=[OpenNavOfficialWizard]::CaptureBounds($Window.Handle,$process.Id)
    if($after.Left -ne $rect.Left -or $after.Top -ne $rect.Top -or $after.Right -ne $rect.Right -or $after.Bottom -ne $rect.Bottom){throw 'Setup moved during capture; pixels discarded.'}
    $bitmap.Save($path,[Drawing.Imaging.ImageFormat]::Png)
  } finally {$graphics.Dispose();$bitmap.Dispose()}
  $summary=@($Controls | Where-Object { $_.Class -notlike 'RichEdit*' } | ForEach-Object {
    @{class=$_.Class;text=$_.Text.Substring(0,[Math]::Min(2048,$_.Text.Length));enabled=$_.Enabled}
  })
  $result.pages+=@{page=$Page;action=$Action;title=$Window.Text;controls=$summary;screenshot=$name;utc=[DateTime]::UtcNow.ToString('o')}
}
try {
  if (-not [Environment]::Is64BitProcess -or [Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT) { throw 'Native 64-bit Windows PowerShell with interactive desktop required.' }
  $identity=[Security.Principal.WindowsIdentity]::GetCurrent()
  if (-not (New-Object Security.Principal.WindowsPrincipal($identity)).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) { throw 'Run as the same interactive user in an already-elevated task; no UAC automation.' }
  $Record=Assert-LocalPath $Record
  if ((Get-Digest $Record) -cne $ExpectedRecordSha256) { throw 'Prepared upgrade record hash mismatch.' }
  $prepared=Read-Record $Record
  if ($prepared.owner -cne 'OpenNavX.StockUpgrade.1' -or $prepared.status -cne 'prepared') { throw 'Unrecognized prepared upgrade record.' }
  $created=[DateTime]::Parse($prepared.createdUtc).ToUniversalTime()
  if ($created -gt [DateTime]::UtcNow -or ([DateTime]::UtcNow-$created).TotalMinutes -gt 60) { throw 'Prepared record expired; recheck cold recovery first.' }
  $setup=Assert-LocalPath $prepared.setup
  if ($prepared.setupSha256 -cne $setupHash -or $prepared.sourceExecutableSha256 -cne $oldHash -or $prepared.targetExecutableSha256 -cne $newHash -or
      (Assert-LocalPath $prepared.sourceApplication) -ine $destination -or (Assert-LocalPath $prepared.sourceExecutable) -ine (Join-Path $destination 'opencpn.exe') -or
      (Assert-LocalPath $prepared.sourceProfile) -ine 'C:\ProgramData\opencpn') { throw 'Prepared identities are outside the one approved boat-stock upgrade.' }
  $uninstaller=Assert-LocalPath $prepared.originalUninstaller
  if ([IO.Path]::GetDirectoryName($uninstaller) -ine $destination -or $prepared.originalUninstallerSha256 -cnotmatch '^[a-f0-9]{64}$' -or
      @($prepared.registrationBefore).Count -ne 1 -or @($prepared.registrationBefore)[0].location -ine $destination -or
      @($prepared.registrationBefore)[0].version -cnotmatch '^5\.12\.2(?:-|$)' -or
      (Assert-LocalPath (@($prepared.registrationBefore)[0].uninstaller.Trim('"'))) -ine $uninstaller) { throw 'Unexpected original uninstaller/registration boundary.' }
  Assert-PreparedOriginal
  $setupLock=New-Object IO.FileStream($setup,[IO.FileMode]::Open,[IO.FileAccess]::Read,[IO.FileShare]::Read)
  if ((Get-Digest $setup) -cne $setupHash) { throw 'Official installer hash mismatch.' }
  Add-Type -Path (Join-Path $PSScriptRoot 'OfficialWizardNative.cs')
  [OpenNavOfficialWizard]::Initialize()
  Add-Type -AssemblyName System.Drawing
  $start=New-Object Diagnostics.ProcessStartInfo
  $start.FileName=$setup;$start.Arguments='';$start.WorkingDirectory=[IO.Path]::GetDirectoryName($setup);$start.UseShellExecute=$false
  $process=[Diagnostics.Process]::Start($start);$result.processId=$process.Id
  $deadline=[DateTime]::UtcNow.AddSeconds($TimeoutSeconds)
  while (-not $process.HasExited -and [DateTime]::UtcNow -lt $deadline) {
    Assert-Closed;Assert-SetupIdentity
    $windows=@([OpenNavOfficialWizard]::Windows($process.Id))
    if ($windows.Count -gt 1) { throw 'Unexpected additional official setup dialog; no input sent.' }
    if ($windows.Count -eq 0) { Start-Sleep -Milliseconds 200;$process.Refresh();continue }
    $window=$windows[0];$controls=@([OpenNavOfficialWizard]::Children($window.Handle,$process.Id))
    Start-Sleep -Milliseconds 200
    $settled=@([OpenNavOfficialWizard]::Children($window.Handle,$process.Id))
    if (($controls | ConvertTo-Json -Compress) -cne ($settled | ConvertTo-Json -Compress)) { continue }
    $page=Get-OfficialUpgradePage $controls
    if ($page -eq 'Unknown') {
      if ($null -eq $unknownSince) {$unknownSince=[DateTime]::UtcNow}
      if (([DateTime]::UtcNow-$unknownSince).TotalSeconds -gt 4) {Capture-Page $window $controls 'Unknown' 'No action; unrecognized page';throw 'Unrecognized stable setup page; installer remains open for inspection.'}
      continue
    }
    $unknownSince=$null
    if ($page -eq $lastObserved) { Start-Sleep -Milliseconds 200;$process.Refresh();continue }
    Assert-OfficialUpgradeTransition $lastPage $page
    $button=$null;$action='Observe installation only'
    switch ($page) {
      Language {
        $combos=@($controls | Where-Object { $_.Class -ceq 'ComboBox' });if($combos.Count -ne 1){throw 'Unexpected language controls.'}
        [OpenNavOfficialWizard]::English($combos[0].Handle,$process.Id);$button=Select-Button $controls 'OK';$action='Choose English'
      }
      Welcome {$button=Select-Button $controls 'Next >';$action='Continue welcome'}
      License {$button=Select-Button $controls 'Next >';$action='Continue official license'}
      Upgrade {
        $upgrade=Select-Button $controls 'Upgrade OpenCPN using previous settings (recommended)'
        $parallel=Select-Button $controls 'Parallel Installation (advanced users)'
        [OpenNavOfficialWizard]::Check($parallel.Handle,$process.Id,$false);[OpenNavOfficialWizard]::Check($upgrade.Handle,$process.Id,$true)
        $result.upgradeSelected=$true;$button=Select-Button $controls 'Next >';$action='Select Upgrade using previous settings only'
      }
      Components {
        $trees=@($controls | Where-Object { $_.Class -ceq 'SysTreeView32' });if($trees.Count -ne 1){throw 'Expected one native component tree.'}
        $tree=$trees[0].Handle;$names=Get-OfficialResetNames
        # Clear individual dangerous leaves through the normal NSIS Space-key
        # handler. Parent checkbox then reflects all children, including mixed.
        for($pass=0;$pass -lt 3;$pass++) {
          $items=@([OpenNavOfficialWizard]::Tree($tree,$process.Id));$changed=$false
          foreach($item in $items) {
            if($item.Text -match '^(Reset|Delete)' -and $item.Text -cnotin $names){throw 'Unknown destructive component.'}
            if($item.Text -cin $names -and $item.Text -cne $names[0] -and $item.State -eq 2) {
              [OpenNavOfficialWizard]::ToggleTree($tree,$item.Handle,$process.Id);$changed=$true
            }
          }
          if(-not $changed){break}
        }
        $items=@([OpenNavOfficialWizard]::Tree($tree,$process.Id));Assert-OfficialResetItems $items
        $result.components=@($items | ForEach-Object {@{text=$_.Text;state=$_.State}})
        $result.configResetUnchecked=$true;$button=Select-Button $controls 'Next >';$action='Verified every reset/delete option unchecked'
      }
      StartMenu {
        $edits=@($controls | Where-Object { $_.Class -ceq 'Edit' });if($edits.Count -ne 1 -or $edits[0].Text -cne 'OpenCPN'){throw 'Unexpected Start Menu folder.'}
        $button=Select-Button $controls 'Next >';$action='Retain normal OpenCPN Start Menu folder'
      }
      Ready {
        if(-not $result.upgradeSelected -or -not $result.configResetUnchecked){throw 'Upgrade/component gates incomplete.'}
        $options=@($controls | Where-Object { $_.Class -ceq 'Edit' -and $_.Text -like 'Setup type:*' })
        if($options.Count -ne 1){throw 'Expected exact final options summary.'}
        Assert-OfficialUpgradeSummary $options[0].Text $destination
        Assert-PreparedOriginal
        $button=Select-Button $controls 'Install';$action='Install exact official Upgrade; preserve all existing configuration'
        $result.installStarted=$true
      }
      Installing {if(-not $result.installStarted){throw 'Unexpected installation progress.'}}
      Finish {
        if(-not $result.installStarted -or (Get-Digest (Join-Path $destination 'opencpn.exe')) -cne $newHash){throw 'Finish reached without exact target executable.'}
        $run=Select-Button $controls 'Run OpenCPN Version 5.12.4-0+37fd0cd';$show=Select-Button $controls 'Show Install Log file'
        [OpenNavOfficialWizard]::Check($run.Handle,$process.Id,$false);[OpenNavOfficialWizard]::Check($show.Handle,$process.Id,$false)
        $result.runUnchecked=$true;$result.showUnchecked=$true;$result.finishObserved=$true
        $button=Select-Button $controls 'Finish';$action='Finish with Run and Show disabled; leave vessel application closed'
      }
    }
    $controls=@([OpenNavOfficialWizard]::Children($window.Handle,$process.Id))
    Capture-Page $window $controls $page $action
    if($button){Assert-SetupIdentity;[OpenNavOfficialWizard]::Click($button.Handle,$process.Id)}
    $lastPage=$page;$lastObserved=$page
    Start-Sleep -Milliseconds 300;$process.Refresh()
  }
  $process.Refresh()
  if(-not $process.HasExited){throw 'Official wizard timed out; no forced termination or reboot attempted.'}
  $result.exitCode=$process.ExitCode
  Assert-Closed
  if(-not $result.finishObserved -or -not $result.runUnchecked -or -not $result.showUnchecked -or (Get-Digest (Join-Path $destination 'opencpn.exe')) -cne $newHash){throw 'Official upgrade completion not established.'}
  if($result.exitCode -notin @(0,1223)){throw 'Unexpected official installer exit status.'}
  $result.status='completed';$result.postVerification='Required: upgrade-stock.ps1 Verify checks complete profile/plugin/resource/registry postconditions before acceptance.'
} catch {
  $result.error=$_.Exception.Message
  if($process){$process.Refresh();$result.installerStillRunning=(-not $process.HasExited)}
} finally {
  if($setupLock){$setupLock.Dispose()}
  if($process){$process.Dispose()}
  $result.completedUtc=[DateTime]::UtcNow.ToString('o')
  Write-Record $Output $result
}
if($result.status -cne 'completed'){throw $result.error}
$result | ConvertTo-Json -Depth 8

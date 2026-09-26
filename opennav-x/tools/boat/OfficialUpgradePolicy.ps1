# Pure, testable page/option policy for the exact official 5.12.4 upgrade.
Set-StrictMode -Version Latest
function Get-OfficialUpgradePage($Controls) {
  $labels=@($Controls | ForEach-Object { $_.Text.Replace('&','').Trim() })
  if ($labels -contains 'Choose Install Location' -or $labels -contains 'Default configuration settings') { throw 'Upgrade unexpectedly offered a new directory or configuration change.' }
  if ($labels -contains 'Please select a language:') { return 'Language' }
  if ($labels -contains 'Welcome to OpenCPN Version 5.12.4-0+37fd0cd Setup') { return 'Welcome' }
  if ($labels -contains 'License Agreement') { return 'License' }
  if ($labels -contains 'Already Installed' -and $labels -contains 'Upgrade OpenCPN using previous settings (recommended)' -and $labels -contains 'Parallel Installation (advanced users)') { return 'Upgrade' }
  if ($labels -contains 'Installation Settings') { return 'Components' }
  if ($labels -contains 'Choose Start Menu Folder') { return 'StartMenu' }
  if ($labels -contains 'Ready to Install') { return 'Ready' }
  if ($labels -contains 'Completing OpenCPN Version 5.12.4-0+37fd0cd Setup' -and
      $labels -contains 'Run OpenCPN Version 5.12.4-0+37fd0cd' -and $labels -contains 'Show Install Log file' -and
      @($labels | Where-Object { $_ -like 'OpenCPN Version 5.12.4-0+37fd0cd has been installed on your computer.*' }).Count -eq 1) { return 'Finish' }
  if ($labels -contains 'Installing' -and @($labels | Where-Object { $_ -like '*OpenCPN Version 5.12.4-0+37fd0cd*' }).Count) { return 'Installing' }
  return 'Unknown'
}
function Assert-OfficialUpgradeTransition([string]$Previous,[string]$Page) {
  $allowed=@{
    Language=@('Start');Welcome=@('Start','Language');License=@('Welcome');
    Upgrade=@('License');Components=@('Upgrade');StartMenu=@('Components');
    Ready=@('Components','StartMenu');Installing=@('Ready');Finish=@('Ready','Installing')
  }
  if (-not $allowed.ContainsKey($Page) -or $Previous -notin $allowed[$Page]) { throw "Unexpected official wizard transition: $Previous -> $Page" }
}
function Assert-OfficialUpgradeSummary([string]$Text,[string]$Destination) {
  # Some upstream resources contain escaped line separators; normalize only
  # these fixed control-text separators before checking exact option sections.
  $text=$Text.Replace('\r\n',"`n").Replace('\t',"`t").Replace("`r`n","`n").Trim()
  $sections=@([regex]::Split($text,'\n\s*\n') | ForEach-Object { $_.Trim() })
  $type=@($sections | Where-Object { $_ -match '^Setup type:' })
  $dest=@($sections | Where-Object { $_ -match '^Destination location:' })
  $remove=@($sections | Where-Object { $_ -match '^Delete \(existing\) Config Subdirectories/ Files:' })
  $shortcuts=@($sections | Where-Object { $_ -match '^Create shortcuts:' })
  if ($sections.Count -ne 4 -or $type.Count -ne 1 -or $dest.Count -ne 1 -or $remove.Count -ne 1 -or $shortcuts.Count -ne 1) { throw 'Unrecognized or incomplete final installation options.' }
  if ($type[0] -cnotmatch '^Setup type:\s+Upgrade$') { throw 'Only an explicit Upgrade is allowed.' }
  if ($dest[0] -cnotmatch ('^Destination location:\s+'+[regex]::Escape($Destination)+'$')) { throw 'Unexpected upgrade destination.' }
  if ($remove[0] -cnotmatch '^Delete \(existing\) Config Subdirectories/ Files:\s+none$') { throw 'Existing configuration/chart/navigation deletion was selected.' }
  if ($shortcuts[0] -notmatch '^Create shortcuts:\s+') { throw 'Invalid shortcut summary.' }
}
function Get-OfficialResetNames {
  return @('Reset ALL OpenCPN configuration files','Delete config file (opencpn.ini)',
    'Delete Chart Database file (CHRTLIST.DAT)','Delete SENC directory',
    'Delete CM93 Directory','Delete Log file (opencpn.log)','Delete Navobject file (navobj.xml)')
}
function Assert-OfficialResetItems($Items) {
  $names=Get-OfficialResetNames
  foreach ($item in @($Items)) {
    if ($item.Text -match '^(Reset|Delete)' -and $item.Text -cnotin $names) { throw 'Unknown destructive component; no installation allowed.' }
    if ($item.Text -cin $names -and $item.State -notin @(1,4)) { throw 'Config reset/delete component remains selected or ambiguous.' }
    if ($item.Text -ceq 'OpenCPN Configuration Settings' -and $item.State -notin @(1,4)) { throw 'Default configuration writer remains selected.' }
  }
  if (@($Items | Where-Object { $_.Text -ceq 'Reset ALL OpenCPN configuration files' }).Count -ne 1) { throw 'Existing-profile reset group was not found.' }
  foreach ($name in @($names)+@('OpenCPN Configuration Settings')) {
    if (@($Items | Where-Object { $_.Text -ceq $name }).Count -gt 1) { throw 'Duplicate configuration component.' }
  }
}
function Assert-OfficialRegistration($Current,$Prepared) {
  if (@($Current).Count -ne 1 -or @($Prepared).Count -ne 1) { throw 'Ambiguous original registration.' }
  $current=@($Current)[0];$expected=@($Prepared)[0]
  foreach ($field in @('key','location','version','uninstaller')) {
    if ($current.$field -cne $expected.$field) { throw 'Original registration changed after preparation.' }
  }
  $actualNames=@($(if($current.values -is [Collections.IDictionary]){$current.values.Keys}else{$current.values.PSObject.Properties.Name}) | Sort-Object)
  $expectedNames=@($(if($expected.values -is [Collections.IDictionary]){$expected.values.Keys}else{$expected.values.PSObject.Properties.Name}) | Sort-Object)
  if (($actualNames -join "`n") -cne ($expectedNames -join "`n")) { throw 'Original registration values changed.' }
  foreach ($name in $expectedNames) {
    if ($current.values.$name.kind -cne $expected.values.$name.kind -or
        ($current.values.$name.value | ConvertTo-Json -Compress -Depth 8) -cne ($expected.values.$name.value | ConvertTo-Json -Compress -Depth 8)) { throw 'Original registry value or type changed.' }
  }
}
function Split-OfficialRegistration($Entries,[string]$Destination) {
  # Pinned NSIS Check_Prev_Installs counts only OpenCPN-prefixed keys longer
  # than seven characters with a CompareVersion. The observed bare OpenCPN
  # key instead belongs to this exact old RTL-SDR plugin and must be retained.
  # Do not generalize this exception to arbitrary plugins or bare registrations.
  $core=@();$plugins=@()
  foreach($entry in @($Entries)) {
    $valueNames=if($entry.values -is [Collections.IDictionary]){@($entry.values.Keys)}else{@($entry.values.PSObject.Properties.Name)}
    $comparison=''
    if($valueNames -ccontains 'CompareVersion') {$comparison=[string]$entry.values.CompareVersion.value}
    if($entry.key -ceq 'OpenCPN') {
      if($entry.version -cne '1.3.1' -or -not [string]::IsNullOrEmpty($entry.location) -or
          $entry.uninstaller -cne ($Destination+'\Uninstall rtlsdr_pi.exe') -or $comparison -cne '' -or
          $valueNames -cnotcontains 'DisplayName' -or $entry.values.DisplayName.kind -cne 'String' -or $entry.values.DisplayName.value -cne 'OpenCPN rtlsdr_pi' -or
          $valueNames -cnotcontains 'Publisher' -or $entry.values.Publisher.kind -cne 'String' -or $entry.values.Publisher.value -cne 'opencpn.org') {
        throw 'Unrecognized bare OpenCPN registration; no maintenance allowed.'
      }
      $plugins+=@($entry)
    } elseif($entry.key -ceq ('OpenCPN '+$entry.version) -and $entry.version -cmatch '^5\.12\.(2|4)(?:-|$)' -and
        $entry.location -ieq $Destination -and -not [string]::IsNullOrWhiteSpace($comparison)) {
      $core+=@($entry)
    } else {throw 'Unrecognized OpenCPN installation registration.'}
  }
  if($core.Count -ne 1 -or $plugins.Count -gt 1){throw 'Ambiguous stock or plugin registration.'}
  return [pscustomobject]@{core=$core;plugins=$plugins}
}
function Assert-OfficialPluginRegistration($Current,$Prepared) {
  if(@($Current).Count -ne @($Prepared).Count -or @($Current).Count -gt 1){throw 'Plugin registration inventory changed.'}
  if(@($Current).Count -eq 1){Assert-OfficialRegistration $Current $Prepared}
}

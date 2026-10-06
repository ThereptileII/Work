# Explicit manual-only UI policy. No serial access, raw keys, arbitrary targets,
# automatic command sequence, retry, plugin restoration or profile publication.
. (Join-Path $PSScriptRoot 'ManualPilotCommissioning.ps1')
function Initialize-ManualPilotUiNative {
  if(-not ('OpenNavX.ManualPilotUiNative' -as [type])) {
    # Compile together so the new resolver reuses only public read-only frame
    # validation/signatures, without widening ReviewWindowNative.ActionLabels.
    if('OpenNavX.ReviewWindowNative' -as [type]){throw 'Use a fresh authenticated helper process for manual UI.'}
    Add-Type -Path @((Join-Path $PSScriptRoot 'ReviewWindowNative.cs'),(Join-Path $PSScriptRoot 'ManualPilotUiNative.cs'))
  }
}
function Assert-ManualPilotUiAction([string]$Action,[string]$Value,[string]$Nonce) {
  if($Nonce -cnotmatch '^[a-f0-9]{32}$'){throw 'A new explicit 32-digit lowercase action nonce is required.'}
  if($Action -cne 'Observe'){$null=[OpenNavX.ManualPilotUiNative]::Target($Action)}
  if($Action -ceq 'SetInterface') {if($Value -cne 'COM8'){throw 'Only the selected existing COM8 interface is admitted.'}}
  elseif($Action -ceq 'SetName') {if(-not [OpenNavX.ManualPilotUiNative]::CompatibleName($Value)){throw 'Actual compatible NAME required.'}}
  elseif($Value){throw 'This action accepts no value.'}
}
function Assert-ManualPilotUiDiagnostics($Data,[string]$Commit,[datetime]$Written,[datetime]$Started,[datetime]$Now) {
  if($Written -lt $Started -or $Written -gt $Now -or ($Now-$Written).TotalSeconds -gt 5 -or
     $Data.build_commit -cne $Commit -or $Data.build_purpose -cne 'INSTALLED PRODUCT' -or
     $Data.data_mode -cne 'OPENCPN selected navigation' -or $Data.xnav_hardware_output_policy -cne 'manual-commissioning' -or
     $Data.xnav_manual_control_contract -isnot [int] -or $Data.xnav_manual_control_contract -ne 1){throw 'Fresh exact manual product diagnostics required.'}
  foreach($value in @($Data.test_fixtures,$Data.runtime.test_fixtures,$Data.runtime.pilot.simulated,$Data.runtime.pilot.track_capability,
      $Data.runtime.pilot.wind_capability,$Data.runtime.pilot.output_unavailable,$Data.runtime.display.route_creation_active,$Data.runtime.replay.active)) {
    if($value -isnot [bool] -or $value){throw 'Unexpected physical-product runtime state.'}
  }
  foreach($value in @($Data.runtime.pilot.enabled,$Data.runtime.pilot.serial_session_enabled,$Data.runtime.pilot.configured_permission,$Data.runtime.pilot.fresh,$Data.runtime.pilot.control_capability)) {
    if($value -isnot [bool]){throw 'Missing real pilot state.'}
  }
}
function Assert-ManualPilotUiState([string]$Action,[string]$Value,$Pilot,$Binding,$Snapshot,[string]$Epoch,[string]$CommandId) {
  if($Action -ceq 'SaveIdentity') {
    if($Snapshot.InterfaceValue -cne 'COM8' -or $null -eq $Snapshot.NameValue){throw 'Save requires exact COM8 identity fields.'}
    $Value=$Snapshot.NameValue
    if($Value -and -not [OpenNavX.ManualPilotUiNative]::CompatibleName($Value)){throw 'Only a compatible observed NAME can be saved.'}
  }
  if($Action -ceq 'SetName' -or ($Action -ceq 'SaveIdentity' -and $Value)) {
    $identities=@($Snapshot.Identities|Where-Object {$_.Name -ceq $Value})
    if($identities.Count -ne 1 -or $Pilot.discovery.verified_identities -lt 1 -or
       $Pilot.discovery.identity_conflicts -ne 0 -or $Pilot.discovery.traffic_limit_exceeded){throw 'Unique actual compatible identity required.'}
    $claims=@($Pilot.receive_diagnostics.sources|Where-Object {$_.interface -ceq 'COM8' -and $_.pgn -eq 60928 -and $_.address -eq $identities[0].Address -and
      [string]$_.age_ms -cmatch '^[0-9]+$' -and [uint64]$_.age_ms -le 30000})
    if($claims.Count -ne 1){throw 'NAME needs a recent actual COM8 address claim; refresh and observe first.'}
  }
  $commands=@('Auto','AcceptAuto','Standby','Minus1','Plus1','Minus10','Plus10')
  $physical=$Action -cin ($commands+@('Enable','AcceptEnable','Permit','AcceptPermission'))
  if($physical) {
    if($Binding.interface -cne 'COM8' -or -not $Binding.name -or -not $Pilot.fresh -or
       $Pilot.source -cnotmatch ('^ST4000 / NMEA2000 / COM8/NAME-'+[regex]::Escape($Binding.name)+'/source-([0-9]{1,3})$') -or
       [int]$Matches[1] -ge 254 -or $Pilot.mode -cnotin @('STANDBY','AUTO') -or
       [string]$Pilot.feedback_sequence -cnotmatch '^[1-9][0-9]*$' -or [string]$Pilot.connection_epoch -cnotmatch '^[1-9][0-9]*$') {
      throw 'Fresh actual compatible physical pilot feedback and exact configured binding required.'
    }
  }
  if($Action -cin ($commands+@('Enable','AcceptEnable'))) {
    if($Epoch -cnotmatch '^[1-9][0-9]*$' -or $Epoch -cne [string]$Pilot.connection_epoch){throw 'Caller must bind the observed current connection epoch.'}
    if($Binding.permission -cne 'manual' -or -not $Pilot.configured_permission){throw 'Explicit saved manual permission required.'}
  }
  if($Action -cin $commands) {
    if(-not $Pilot.enabled -or -not $Pilot.serial_session_enabled -or -not $Pilot.control_capability){throw 'Explicit live session enablement required.'}
    if($CommandId -cnotmatch '^[0-9]+$' -or $CommandId -cne [string]$Pilot.command_id){throw 'Caller must bind the last observed command id; inspect uncertain outcomes before further actions.'}
    if($Action -cne 'Standby' -and $Pilot.command_state -cnotin @('None','Confirmed','Disabled')){throw 'Previous command outcome is unresolved; inspect physical feedback.'}
    if($Action -cin @('Minus1','Plus1','Minus10','Plus10') -and $Pilot.mode -cne 'AUTO'){throw 'Course adjustment requires actual AUTO feedback.'}
  }
  if($Action -cin @('Enable','AcceptEnable') -and ($Pilot.enabled -or $Pilot.serial_session_enabled)){throw 'Control is already enabled; Enable is never a toggle-to-disable retry.'}
  if($Action -ceq 'Disable' -and -not $Pilot.enabled){throw 'Control is already disabled; Disable must never enable it.'}
  if($Action -cin @('SetInterface','SetName','SaveIdentity','OpenIdentity','RefreshIdentity','Permit','AcceptPermission') -and ($Pilot.enabled -or $Pilot.serial_session_enabled)){throw 'Disable this session before editing identity/permission or requesting discovery.'}
}
function Assert-ManualPilotUiProfile($V) {
  $path=Join-Path $V.context.profile 'opencpn.ini';$values=Read-ProfileForAudit $path
  Assert-ManualPilotProfile $values $true
  $binding=Get-ManualPilotBinding $values['OpenNav/AlphaSettings']
  $before=Join-Path $V.paths.directory 'output.ini'
  # Reuse the existing exact pilot delta + ordinary session-preservation policy.
  # This private validation copy removes only the deliberately live permission;
  # the actual profile is never edited or restored by the UI helper.
  $temporary=Join-Path $V.paths.directory ('ui-profile-check-'+[guid]::NewGuid().ToString('N')+'.ini')
  try {
    $encoding=New-Object Text.UTF8Encoding($false,$true)
    $text=$encoding.GetString([IO.File]::ReadAllBytes($path))
    if($binding.permission -ceq 'manual') {
      $alpha=$values['OpenNav/AlphaSettings'];$safe=$alpha.Replace('"pilot.permission" "manual"\n','"pilot.permission" "display-only"\n')
      $pattern='(?m)^AlphaSettings='+[regex]::Escape($alpha)+'(?=\r?$)'
      if([regex]::Matches($text,$pattern).Count -ne 1){throw 'Unique exact settings line required.'}
      $text=[regex]::Replace($text,$pattern,[Text.RegularExpressions.MatchEvaluator]{param($m)return 'AlphaSettings='+$safe})
    }
    [IO.File]::WriteAllBytes($temporary,$encoding.GetBytes($text))
    $null=Assert-ManualPilotProfileDelta $before $temporary
  }finally{if(Test-Path $temporary){Remove-Item -LiteralPath $temporary}}
  return $binding
}
function Get-ManualPilotUiObservation($V,$Process,$Snapshot) {
  $path=Join-Path $V.context.profile 'opennav-logs/opennav-diagnostics.json'
  $data=Read-Record $path
  Assert-ManualPilotUiDiagnostics $data $V.record.candidate.commit (Get-Item $path).LastWriteTimeUtc $Process.StartTime.ToUniversalTime() ([datetime]::UtcNow)
  $p=$data.runtime.pilot
  # Never return full diagnostics, chart/route positions, logs, or arbitrary text.
  return [pscustomobject]@{pilot=$p;public=[pscustomobject]@{utc=[datetime]::UtcNow.ToString('o');modal=$Snapshot.Modal;controls=$Snapshot.Controls;identities=$Snapshot.Identities;identityInterface=$(if($Snapshot.InterfaceValue -ceq 'COM8'){'COM8'}else{'unset-or-unexpected'});identityName=$(if([OpenNavX.ManualPilotUiNative]::CompatibleName($Snapshot.NameValue)){$Snapshot.NameValue}else{'unset-or-unexpected'});
    pilot=[pscustomobject]@{fresh=$p.fresh;mode=$p.mode;source=$p.source;feedbackSequence=$p.feedback_sequence;connectionEpoch=$p.connection_epoch;
      enabled=$p.enabled;serialSessionEnabled=$p.serial_session_enabled;configuredPermission=$p.configured_permission;
      commandId=$p.command_id;commandState=$p.command_state;
      actualHeadingQuality=$p.actual_heading_quality;lockedHeadingQuality=$p.locked_heading_quality;
      actualHeadingMagnetic=$(if($p.PSObject.Properties['actual_heading_magnetic_deg']){$p.actual_heading_magnetic_deg}else{$null});
      lockedHeadingMagnetic=$(if($p.PSObject.Properties['locked_heading_magnetic_deg']){$p.locked_heading_magnetic_deg}else{$null});
      modeAgeMs=$(if($p.PSObject.Properties['mode_age_ms']){$p.mode_age_ms}else{$null})}}}
}
function Invoke-ManualPilotUi($Job) {
  Initialize-ManualPilotUiNative
  Assert-ManualPilotUiAction $Job.uiAction $Job.value $Job.nonce
  $v=Read-ManualPilot $Job.workspace $Job.manualRecord $Job.manualRecordSha256 -Active
  Assert-ManualPilotInteractive $v
  if($Job.executable -ine $v.context.installation.executable -or $Job.executableSha256 -cne $v.record.candidate.executableSha256){throw 'Manual UI executable changed.'}
  $parentLock=Open-CommissioningRestoreLock $v.record.parentDirectory;$lock=$null;$process=$null;$oldDpi=[IntPtr]::Zero
  try {
    $lock=Open-CommissioningRestoreLock $v.paths.directory
    $v=Read-ManualPilot $Job.workspace $Job.manualRecord $Job.manualRecordSha256 -Active
    if(Test-Path (Join-Path $v.context.installation.root 'update-pending.json')){throw 'Pending update prevents manual UI input.'}
    if(-not (Test-Path (Join-Path $v.paths.directory 'startup-off.json')) -or (Test-Path (Join-Path $v.paths.directory 'close-intent.json'))){throw 'Verified startup and still-active process required.'}
    $receipt=Read-Record (Join-Path $v.paths.directory 'launched.json');$process=Get-ManualPilotProcess $v $receipt
    $binding=Assert-ManualPilotUiProfile $v
    $oldDpi=[OpenNavX.ReviewWindowNative]::SetThreadDpiAwarenessContext([IntPtr](-4))
    if($oldDpi -eq [IntPtr]::Zero){throw 'Per-monitor native coordinates unavailable.'}
    $frame=$process.MainWindowHandle
    $snapshot=[OpenNavX.ManualPilotUiNative]::Observe($frame,$process.Id)
    $observation=Get-ManualPilotUiObservation $v $process $snapshot
    Assert-ManualPilotUiState $Job.uiAction $Job.value $observation.pilot $binding $snapshot $Job.expectedEpoch $Job.expectedCommandId
    $intent=Join-Path $v.paths.directory ('ui-'+$Job.nonce+'-intent.json')
    if(Test-Path $intent){throw 'This action nonce is already consumed; inspect its evidence, never automatically retry.'}
    $record=@{owner='OpenNavX.ManualPilotUi.1';recordSha256=$Job.manualRecordSha256;nonce=$Job.nonce;action=$Job.uiAction;value=$Job.value;pid=$receipt.pid;startedTicks=$receipt.startedTicks;commit=$v.record.candidate.commit;
      before=$observation.public;retryAllowed=$false;physicalOutcome='Not inferred from UI input'}
    # Durable intent before any input. An interrupted dispatch cannot be replayed.
    Write-Record $intent $record
    if($Job.uiAction -cne 'Observe') {
      $again=Get-ManualPilotProcess $v $receipt;$again.Dispose()
      $binding=Assert-ManualPilotUiProfile $v
      $snapshot=[OpenNavX.ManualPilotUiNative]::Observe($frame,$process.Id)
      $latest=Get-ManualPilotUiObservation $v $process $snapshot
      Assert-ManualPilotUiState $Job.uiAction $Job.value $latest.pilot $binding $snapshot $Job.expectedEpoch $Job.expectedCommandId
      [OpenNavX.ManualPilotUiNative]::Act($frame,$process.Id,$Job.uiAction,$Job.value)
    }
    $result=[pscustomobject]@{status='passed';action='ManualPilotUi';uiAction=$Job.uiAction;nonce=$Job.nonce;pid=$receipt.pid;startedTicks=$receipt.startedTicks;commit=$v.record.candidate.commit;
      inputDispatched=($Job.uiAction -cne 'Observe');physicalOutcome='Not inferred; invoke Observe and inspect fresh pilot feedback';observation=$observation.public}
    Write-Record (Join-Path $v.paths.directory ('ui-'+$Job.nonce+'-result.json')) $result
    return $result
  }finally {
    if($oldDpi -ne [IntPtr]::Zero){$null=[OpenNavX.ReviewWindowNative]::SetThreadDpiAwarenessContext($oldDpi)}
    if($process){$process.Dispose()};if($lock){$lock.Dispose()};$parentLock.Dispose()
  }
}
function Invoke-ManualPilotUiJob([string]$Workspace,[string]$Record,[string]$Hash,[string]$Action,[string]$Value,[string]$Nonce,[string]$Epoch,[string]$CommandId) {
  Initialize-ManualPilotUiNative;Assert-ManualPilotUiAction $Action $Value $Nonce
  $v=Read-ManualPilot $Workspace $Record $Hash -Active
  $job=[pscustomobject]@{action='ManualPilotUi';executable=$v.context.installation.executable;executableSha256=$v.record.candidate.executableSha256;
    manualRecord=$Record;manualRecordSha256=$Hash;uiAction=$Action;value=$Value;nonce=$Nonce;expectedEpoch=$Epoch;expectedCommandId=$CommandId}
  return Invoke-InteractiveJob $Workspace $job -TimeoutSeconds 60
}

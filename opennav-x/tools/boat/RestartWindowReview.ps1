# One explicit UI mode request or display-only review of a broker-proven child.
# Never arms a broker, starts OpenCPN, renews an audit, or operates equipment.
. (Join-Path $PSScriptRoot 'RestartCommissioning.ps1')
. (Join-Path $PSScriptRoot 'ReviewWindow.ps1')
$script:RestartWindowFiles=@('RestartWindowReview.ps1','RestartWindowNative.cs','ReviewWindow.ps1','ReviewWindowNative.cs')
function Assert-RestartReady($Ready,$Arm,$Session,[datetime]$Now) {
  if($Ready.owner -cne $script:RestartOwner -or $Arm.owner -cne $script:RestartOwner -or
     $Ready.session -cne $Session.session -or $Arm.session -cne $Session.session -or
     $Ready.recordSha256 -cne $Session.recordSha256 -or $Arm.recordSha256 -cne $Session.recordSha256 -or
     $Ready.parent.pid -cne $Arm.parentPid -or $Ready.parent.createdFiletime -cne $Arm.parentCreatedFiletime -or
     $Ready.mode -cne $Arm.mode -or $Arm.mode -cnotin @('--xnav','--legacy','--safe-mode')) {throw 'Exact immutable Arm and listening readiness identities required.'}
  foreach($value in @($Arm.parentPid,$Arm.parentCreatedFiletime,$Ready.broker.pid,$Ready.broker.createdFiletime)){Assert-RestartDecimal $value 'ready identity'}
  $at=[datetime]::Parse($Ready.createdUtc).ToUniversalTime();$armed=[datetime]::Parse($Arm.createdUtc).ToUniversalTime()
  if($at -gt $Now -or $armed -gt $at -or ($at-$armed).TotalSeconds -gt 30 -or ($Now-$at).TotalSeconds -ge 90){throw 'Listening readiness expired; no UI command may be retried.'}
}
function Assert-RestartWindowAction([string]$Action,[string]$Mode,[string]$ReviewAction) {
  if($Mode -cnotin @('--xnav','--legacy','--safe-mode')){throw 'Only an exact installed mode may be reviewed.'}
  if($Action -ceq 'RequestGuardedMode') {
    if($ReviewAction -cne 'RequestMode'){throw 'One explicit mode action required.'}
  } elseif($Action -ceq 'ReviewRestartChild') {
    $allowed=if($Mode -ceq '--xnav'){Get-WindowReviewActions}else{@('Capture','Resize1280x800')}
    if($ReviewAction -cnotin $allowed){throw 'Only fixed display actions for this proven child mode are permitted.'}
  } else {throw 'Unknown guarded review action.'}
}
function Read-RestartWindowReview($Job,[switch]$IntentWritten) {
  if($Job.action -cnotin @('RequestGuardedMode','ReviewRestartChild')){throw 'Unknown guarded review action.'}
  if(@($Job.helperFiles).Count -ne $script:RestartWindowFiles.Count){throw 'Exact reviewed helper inventory required.'}
  foreach($name in $script:RestartWindowFiles) {
    $entry=@($Job.helperFiles|Where-Object {$_.name -ceq $name})
    if($entry.Count -ne 1 -or $entry[0].sha256 -cnotmatch '^[a-f0-9]{64}$' -or (Get-Digest (Join-Path $PSScriptRoot $name)) -cne $entry[0].sha256){throw 'Guarded review helper changed after dispatch.'}
  }
  Initialize-RestartNative
  $session=Read-RestartSession $Job.sessionRecord $Job.sessionRecordSha256
  if($session.workspace -ine (Assert-LocalPath $Job.workspace) -or $session.buildCommit -cne $Job.buildCommit -or
     $session.generation -cne $Job.generation -or $session.executable -cne $Job.executable -or
     $session.executableSha256 -cne $Job.executableSha256){throw 'Guarded review must use the exact installed session.'}
  $root=[IO.Path]::GetDirectoryName((Assert-LocalPath $Job.sessionRecord))
  $process=Get-RestartProcess ([uint32]$Job.processId) $session $session.executable
  if($process.createdFiletime -cne $Job.processCreatedFiletime){throw 'Exact reviewed PID creation changed.'}
  if($Job.action -ceq 'RequestGuardedMode') {
    $armPath=Join-Path $root ('arm-'+$process.pid+'-'+$process.createdFiletime+'.json')
    if($Job.armFile -cne $armPath -or $Job.armSha256 -cnotmatch '^[a-f0-9]{64}$' -or (Get-Digest $armPath) -cne $Job.armSha256){throw 'Immutable exact-parent Arm record required.'}
    $arm=Read-Record $armPath;$transition=Assert-LocalPath $arm.transition
    if([IO.Path]::GetDirectoryName($transition) -cne $root -or [IO.Path]::GetFileName($transition) -cnotmatch '^transition-\d{4}$'){throw 'Arm transition escaped its cold session.'}
    $readyPath=Join-Path $transition 'ready.json'
    if($Job.readySha256 -cnotmatch '^[a-f0-9]{64}$' -or (Get-Digest $readyPath) -cne $Job.readySha256){throw 'Immutable broker readiness required.'}
    $ready=Read-Record $readyPath
    Assert-RestartReady $ready $arm $session ([datetime]::UtcNow)
    if($arm.parentPid -cne $process.pid -or $arm.parentCreatedFiletime -cne $process.createdFiletime -or $arm.mode -cne $Job.mode){throw 'Requested mode or parent differs from Arm.'}
    $execute=Join-Path ([Environment]::GetFolderPath('System')) 'WindowsPowerShell\v1.0\powershell.exe'
    $arguments='-NoProfile -NonInteractive -ExecutionPolicy Bypass -File "'+(Join-Path $PSScriptRoot 'RestartCommissioningBroker.ps1')+'" -SessionRecord "'+$Job.sessionRecord+'" -ExpectedSha256 '+$Job.sessionRecordSha256+' -ParentProcessId '+$process.pid+' -ParentCreatedFiletime '+$process.createdFiletime+' -Mode '+$Job.mode
    if($arm.execute -ine $execute -or $arm.arguments -cne $arguments -or $arm.taskName -cnotmatch '^OpenNavX-RestartReview-[a-f0-9]{32}$'){throw 'Arm task differs from the fixed broker invocation.'}
    $task=Get-ScheduledTask -TaskName $arm.taskName -ErrorAction Stop
    Assert-RestartTaskIdentity $task $arm $session.sid -ExpectedState Running
    $broker=Get-RestartProcess ([uint32]$ready.broker.pid) $session $execute
    if($broker.createdFiletime -cne $ready.broker.createdFiletime -or -not $broker.commandLine.EndsWith((' '+$arguments),[StringComparison]::Ordinal) -or
       $broker.commandLine.Substring(0,$broker.commandLine.Length-$arguments.Length-1).Trim('"') -ine $execute){throw 'Listening broker process/command line changed.'}
    $chain=Get-RestartBaseline $session $Job.sessionRecord $process -ReviewOnly -PendingDirectory $transition
    if($chain.sha256 -cne $ready.beforeSha256){throw 'Listening broker baseline differs from the verified chain.'}
    $intent=Join-Path $transition 'ui-intent-consumed.json'
    if($IntentWritten) {
      if((Get-Digest $intent) -cne $Job.intentSha256){throw 'Consumed one-action intent changed.'}
    } elseif(Test-Path -LiteralPath $intent){throw 'A UI mode action was already consumed; inspect outcome without retrying.'}
    Assert-RestartReady $ready $arm $session ([datetime]::UtcNow)
    if([datetime]::UtcNow -ge [datetime]::Parse($session.expiresUtc).ToUniversalTime()){throw 'Cold restart session expired during review.'}
    Assert-RestartWindowAction $Job.action $chain.mode $Job.reviewAction
    return [pscustomobject]@{session=$session;process=$process;mode=$chain.mode;chain=$chain;transition=$transition;intent=$intent;arm=$arm;ready=$ready}
  }
  $chain=Get-RestartBaseline $session $Job.sessionRecord $process -ReviewOnly
  if(-not $chain.completionPath -or $Job.completionFile -cne $chain.completionPath -or $Job.completionSha256 -cnotmatch '^[a-f0-9]{64}$' -or
     $Job.completionSha256 -cne $chain.completionSha256){throw 'The last consumed transition and exact completed-child proof are required; cold-launch fiction is refused.'}
  Assert-RestartWindowAction $Job.action $chain.mode $Job.reviewAction
  if($Job.mode -cne $chain.mode){throw 'Child review mode differs from native request/permit.'}
  return [pscustomobject]@{session=$session;process=$process;mode=$chain.mode;chain=$chain}
}
function Initialize-RestartWindowNative {
  if(-not ('OpenNavX.RestartWindowNative' -as [type])){Add-Type -Path (Join-Path $PSScriptRoot 'RestartWindowNative.cs')}
}
function Save-RestartWindowImage([IntPtr]$Frame,[int]$ProcessId,[string]$Mode,[string]$Path) {
  $path=Assert-LocalPath $Path;if(Test-Path -LiteralPath $path){throw 'Fresh private image path required.'}
  $info=[OpenNavX.RestartWindowNative]::AssertFrame($Frame,$ProcessId,$Mode)
  Add-Type -AssemblyName System.Drawing
  $bitmap=New-Object Drawing.Bitmap($info.Bounds.Width,$info.Bounds.Height);$graphics=[Drawing.Graphics]::FromImage($bitmap)
  try {
    [OpenNavX.RestartWindowNative]::AssertCapture($Frame,$ProcessId,$info)
    $graphics.CopyFromScreen($info.Bounds.Left,$info.Bounds.Top,0,0,$bitmap.Size)
    [OpenNavX.RestartWindowNative]::AssertCapture($Frame,$ProcessId,$info)
    $bitmap.Save($path,[Drawing.Imaging.ImageFormat]::Png)
  } finally {$graphics.Dispose();$bitmap.Dispose()}
  return @{path=$path;sha256=(Get-Digest $path);window=$info}
}
function Invoke-RestartWindowReview($Job) {
  $proof=Read-RestartWindowReview $Job
  if($proof.session.windowsSessionId -cne [Diagnostics.Process]::GetCurrentProcess().SessionId.ToString()){throw 'Guarded review must run in the exact interactive desktop session.'}
  $directory=Assert-LocalPath $Job.evidenceDirectory
  if([IO.Path]::GetDirectoryName($directory) -ine (Join-Path $Job.workspace 'runs') -or
     [IO.Path]::GetFileName($directory) -cnotmatch '^\d{8}-\d{6}-restart-window-[a-f0-9]{8}$' -or
     -not [IO.Directory]::Exists($directory) -or @(Get-ChildItem -LiteralPath $directory -Force).Count){throw 'New private restart-window evidence directory required.'}
  Assert-RestartPrivateDirectory $directory $proof.session.sid
  $process=Get-Process -Id $Job.processId -ErrorAction Stop
  try {
    $null=$process.Handle
    if($process.HasExited -or $process.StartTime.ToUniversalTime().ToFileTimeUtc().ToString() -cne $Job.processCreatedFiletime){throw 'Reviewed process exited or changed.'}
    Initialize-RestartWindowNative
    $oldDpi=[OpenNavX.RestartWindowNative]::SetThreadDpiAwarenessContext([IntPtr](-4))
    if($oldDpi -eq [IntPtr]::Zero){throw 'Physical pixel DPI context unavailable.'}
    try {
      $frame=$process.MainWindowHandle
      [OpenNavX.RestartWindowNative]::Foreground($frame,$process.Id,$proof.mode)
      $before=Save-RestartWindowImage $frame $process.Id $proof.mode (Join-Path $directory 'before.png')
      $proof=Read-RestartWindowReview $Job;$process.Refresh()
      if($process.HasExited -or $process.MainWindowHandle -ne $frame){throw 'Reviewed main frame changed.'}
      $result=@{status='passed';action=$Job.action;reviewAction=$Job.reviewAction;processId=$process.Id;processCreatedFiletime=$Job.processCreatedFiletime;
        mode=$proof.mode;buildCommit=$Job.buildCommit;generation=$Job.generation;sessionRecordSha256=$Job.sessionRecordSha256;
        utc=[datetime]::UtcNow.ToString('o');before=$before;actuatorCommandsIssuedByTool=$false;physicalBusSilenceNotClaimed=$true;launchPerformedByTool=$false}
      if($Job.action -ceq 'RequestGuardedMode') {
        $command=[OpenNavX.RestartWindowNative]::InspectModeCommand($frame,$process.Id,$proof.mode,$Job.mode)
        Write-Record $proof.intent @{owner='OpenNavX.GuardedModeIntent.1';session=$proof.session.session;recordSha256=$Job.sessionRecordSha256;
          armSha256=$Job.armSha256;readySha256=$Job.readySha256;parent=$proof.process;mode=$Job.mode;fromMode=$proof.mode;
          command=$command;beforeImageSha256=$before.sha256;status='consumed-before-ui-action';utc=[datetime]::UtcNow.ToString('o')}
        $Job|Add-Member -NotePropertyName intentSha256 -NotePropertyValue (Get-Digest $proof.intent)
        # A disappearing listener, changed file or failed UI send consumes this
        # request forever. It can never fall back to a cold/unarmed launch.
        $null=Read-RestartWindowReview $Job -IntentWritten
        [OpenNavX.RestartWindowNative]::RequestMode($frame,$process.Id,$command)
        $result.outcome='UI request sent once; child success requires broker completion and receipt';$result.requestedMode=$Job.mode
        $result.intentFile=$proof.intent;$result.intentSha256=$Job.intentSha256;$result.command=$command;$result.childSuccessClaimed=$false
      } else {
        switch($Job.reviewAction) {
          'Capture' {}
          'Resize1280x800' {[OpenNavX.RestartWindowNative]::Resize1280x800($frame,$process.Id,$proof.mode)}
          default {
            Initialize-WindowReviewNative
            if($Job.reviewAction -ceq 'Escape'){[OpenNavX.ReviewWindowNative]::Escape($frame,$process.Id)}
            elseif($Job.reviewAction -cin @('SelectFirstVisibleWaypoint','SelectFirstVisibleAis')){$result.selection=[OpenNavX.ReviewWindowNative]::SelectRow($frame,$process.Id,$Job.reviewAction)}
            else{[OpenNavX.ReviewWindowNative]::Click($frame,$process.Id,$Job.reviewAction)}
          }
        }
        $null=Read-RestartWindowReview $Job
        $result.after=Save-RestartWindowImage $frame $process.Id $proof.mode (Join-Path $directory 'after.png')
        $result.proofKind='completed-guarded-restart';$result.completionFile=$proof.chain.completionPath;$result.completionSha256=$proof.chain.completionSha256
      }
      return $result
    } finally {$null=[OpenNavX.RestartWindowNative]::SetThreadDpiAwarenessContext($oldDpi)}
  } finally {$process.Dispose()}
}

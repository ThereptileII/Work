# Separate failed-bootstrap observation authority. Never creates startup health,
# launches an application, or converts failed bootstrap evidence to success.
[CmdletBinding()]
param(
 [string]$ReviewedHelperDirectory,[string]$Workspace='C:\XNav',
 [ValidateSet('Inspect','Acknowledge')][string]$Phase='Inspect',
 [string]$FailureResult,[string]$FailureResultSha256,[string]$FailureRequestSha256,
 [string]$Observation,[string]$ObservationSha256,[string]$ExpectedTargetSha256,
 [string]$Inspection,[string]$InspectionSha256,[string]$ReviewedImageSha256,
 [string]$InternalRequest,[string]$InternalRequestSha256,[string]$SelfSha256,
 [switch]$PolicySelfTest
)
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'
function Assert-BootstrapWelcomePolicy($Failed,$Request,$Observed,[string]$App,[string]$Root,[datetime]$Now) {
 if($Failed.status -cne 'failed' -or $Failed.action -cne 'LaunchStartup' -or
    $Failed.error -cne 'Installed launcher did not complete successfully; inspect desktop without retry.' -or
    $Request.action -cne 'LaunchStartup' -or $Request.mode -cne '--xnav' -or $Request.executable -ine $App -or $Request.workspace -ine $Root -or
    $Request.executableSha256 -cne '8ed5cc1fad45bfc9cda13f98ec0b55cf18270b45a348764673aa7702c12c192c'){throw 'Exact retained failed bootstrap required.'}
 if($Observed.schema -ne 1 -or $Observed.status -cne 'passed' -or $Observed.scope -cne 'read-only exact startup processes' -or
    $Observed.sid -cnotmatch '^S-1-5-[0-9-]+$' -or $Observed.session -le 0 -or @($Observed.targets).Count -ne 2){throw 'Exact independent interactive observation required.'}
 $at=[datetime]::Parse($Observed.utc).ToUniversalTime()
 if($at -gt $Now -or ($Now-$at).TotalHours -gt 1){throw 'Startup observation expired.'}
 $a=@($Observed.targets|Where-Object {$_.pid -eq 7196});$l=@($Observed.targets|Where-Object {$_.pid -eq 1228})
 if($a.Count -ne 1 -or $l.Count -ne 1 -or $a[0].image -ine $App -or
    $a[0].sha256 -cne $Request.executableSha256 -or $a[0].startedUtc -cne '2026-10-05T20:01:57.2931063Z' -or
    $l[0].startedUtc -cne '2026-10-05T20:01:22.9834203Z'){throw 'Original observed process pair changed.'}
 $swedish='V'+[char]0x00e4+'lkommen till OpenCPN'
 $modal=@($Observed.windows|Where-Object {$_.pid -eq 7196 -and -not $_.child -and $_.className -ceq '#32770' -and $_.visible -eq $true -and $_.enabled -eq $true -and $_.title -cin @('Welcome to OpenCPN',$swedish)})
 if($modal.Count -ne 1){throw 'Observed standard navigation warning required.'}
}
if($PolicySelfTest) {
 $now=[datetime]::UtcNow
 $failed=[pscustomobject]@{status='failed';action='LaunchStartup';error='Installed launcher did not complete successfully; inspect desktop without retry.'}
 $request=[pscustomobject]@{action='LaunchStartup';mode='--xnav';executable='C:\owned\app\opencpn.exe';workspace='C:\XNav';executableSha256='8ed5cc1fad45bfc9cda13f98ec0b55cf18270b45a348764673aa7702c12c192c'}
 $observed=[pscustomobject]@{schema=1;status='passed';scope='read-only exact startup processes';sid='S-1-5-21-42';session=1;utc=$now.ToString('o');targets=@([pscustomobject]@{pid=7196;image=$request.executable;sha256=$request.executableSha256;startedUtc='2026-10-05T20:01:57.2931063Z'},[pscustomobject]@{pid=1228;startedUtc='2026-10-05T20:01:22.9834203Z'});windows=@([pscustomobject]@{pid=7196;child=$false;className='#32770';visible=$true;enabled=$true;title='Welcome to OpenCPN'})}
 Assert-BootstrapWelcomePolicy $failed $request $observed $request.executable $request.workspace $now
 $checks=1
 foreach($change in @(@('failure','status','passed'),@('failure','action','Launch'),@('failure','error','Other failure'),@('request','mode','--legacy'),@('request','executableSha256',('0'*64)),@('observation','session',0),@('observation','utc',$now.AddHours(-2).ToString('o')), @('observation','status','failed'))) {
  $f=$failed|ConvertTo-Json -Depth 10|ConvertFrom-Json;$r=$request|ConvertTo-Json -Depth 10|ConvertFrom-Json;$o=$observed|ConvertTo-Json -Depth 10|ConvertFrom-Json
  $obj=@{failure=$f;request=$r;observation=$o}[$change[0]];$obj.($change[1])=$change[2];$rejected=$false
  try{Assert-BootstrapWelcomePolicy $f $r $o $request.executable $request.workspace $now}catch{$rejected=$true}
  if(-not $rejected){throw 'Invalid observation authority accepted.'};$checks++
 }
 # PowerShell variables are case-insensitive: keep the deserialized object
 # separate from the public, string-constrained $Inspection parameter.
 $collisionRegression=& {
  param([string]$Inspection)
  $inspectionRecord='{"owner":"SKAGER.FailedBootstrapWelcome.1","phase":"Inspect"}'|ConvertFrom-Json
  if($inspectionRecord.owner -cne 'SKAGER.FailedBootstrapWelcome.1' -or $Inspection -cne 'C:\fixture\review.json'){throw 'Inspection object collided with typed path parameter.'}
  return $true
 } 'C:\fixture\review.json'
 if(-not $collisionRegression){throw 'Inspection collision regression failed.'};$checks++
 @{status='passed';checks=$checks;scope='pure policy and typed inspection regression only; no boat/native/window actions'}|ConvertTo-Json;return
}
function PrivateHash([string]$Path) {$h=[Security.Cryptography.SHA256]::Create();$f=[IO.File]::OpenRead($Path);try{return ([BitConverter]::ToString($h.ComputeHash($f))).Replace('-','').ToLowerInvariant()}finally{$f.Dispose();$h.Dispose()}}
if($InternalRequest) {
 if($SelfSha256 -cnotmatch '^[a-f0-9]{64}$' -or $InternalRequestSha256 -cnotmatch '^[a-f0-9]{64}$' -or (PrivateHash $PSCommandPath) -cne $SelfSha256 -or (PrivateHash $InternalRequest) -cne $InternalRequestSha256){throw 'Recovery dispatch changed.'}
 $job=[IO.File]::ReadAllText($InternalRequest)|ConvertFrom-Json
 $ReviewedHelperDirectory=$job.helperDirectory;$Workspace=$job.workspace
 foreach($entry in $job.helpers){if((PrivateHash (Join-Path $ReviewedHelperDirectory $entry.name)) -cne $entry.sha256){throw 'Qualified helper changed.'}}
}
. (Join-Path $ReviewedHelperDirectory 'InstalledWelcome.ps1')
$Workspace=Assert-LocalPath $Workspace;$ReviewedHelperDirectory=Assert-LocalPath $ReviewedHelperDirectory
$held=New-Object 'Collections.Generic.List[IDisposable]'
function Hold-Proof([string]$Path,[string]$Hash) {
 $path=Assert-LocalPath $Path
 if($Hash -cnotmatch '^[a-f0-9]{64}$'){throw 'Explicit proof hash required.'}
 $held.Add([IO.File]::Open($path,[IO.FileMode]::Open,[IO.FileAccess]::Read,[IO.FileShare]::Read))
 if((Get-Digest $path) -cne $Hash){throw 'Pinned proof changed.'}
 return Read-Record $path
}
function Read-BootstrapAuthority($Job) {
 $failed=Hold-Proof $Job.failureResult $Job.failureResultSha256
 $requestPath=Join-Path ([IO.Path]::GetDirectoryName($Job.failureResult)) 'request.json'
 $request=Hold-Proof $requestPath $Job.failureRequestSha256
 if($request.resultPath -ine $Job.failureResult){throw 'Failed request/result path mismatch.'}
 $observed=Hold-Proof $Job.observation $Job.observationSha256
 $config=Hold-Proof (Join-Path $Job.workspace 'boat-target.json') $Job.targetSha256
 $installed=Get-Installed
 Assert-BootstrapWelcomePolicy $failed $request $observed $installed.executable $Job.workspace ([datetime]::UtcNow)
 if($installed.ownership.commit -cne 'c0d8d85fb602e86d40e2f3f1be32307919702408' -or $installed.ownership.version -cne '0.4.0-beta2'){throw 'Exact candidate required.'}
 $entry=@($installed.ownership.managedFiles|Where-Object {$_.path -ceq 'docs/PRODUCT_BUILD.json'})
 if($entry.Count -ne 1){throw 'Owned product build report required.'}
 $build=Hold-Proof (Join-Path $installed.generation 'docs/PRODUCT_BUILD.json') $entry[0].sha256
 if($build.test_fixtures -isnot [bool] -or $build.test_fixtures -or $build.build_purpose -cne 'INSTALLED PRODUCT' -or $build.commit -cne $installed.ownership.commit -or $build.executable_sha256 -cne $request.executableSha256){throw 'Fixture-free exact product required.'}
 $launcher=@($observed.targets|Where-Object {$_.pid -eq 1228})[0]
 $entry=@($installed.ownership.managedFiles|Where-Object {$_.path -ceq 'app/skager-start.exe'})
 if($entry.Count -ne 1 -or $launcher.image -ine (Join-Path $installed.generation 'app/skager-start.exe') -or $launcher.sha256 -cne $entry[0].sha256){throw 'Observed launcher is not owned by candidate.'}
 # This object is explicitly observation authority, never a fabricated Launch receipt.
 $authority=[pscustomobject]@{owner='SKAGER.FailedBootstrapObservation.1';sid=$observed.sid;sessionId=$observed.session;commissioning=$config.readOnlyAudit.commissioning}
 Assert-InstalledWelcomeRuntime $config $installed $authority $Job.workspace
 return [pscustomobject]@{observed=$observed;installed=$installed;authority=$authority}
}
function Assert-LiveObservation($Proof) {
 $sid=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value;$session=[Diagnostics.Process]::GetCurrentProcess().SessionId
 if($sid -cne $Proof.observed.sid -or $session -ne $Proof.observed.session -or $session -le 0){throw 'Original interactive user/session required.'}
 foreach($target in $Proof.observed.targets) {
  $process=Get-Process -Id $target.pid -ErrorAction Stop
  try {
   $null=$process.get_Handle();$native=Get-CimInstance Win32_Process -Filter ('ProcessId='+[int]$target.pid) -OperationTimeoutSec 3
   $owner=Invoke-CimMethod -InputObject $native -MethodName GetOwnerSid -OperationTimeoutSec 3
   if($process.HasExited -or $process.StartTime.ToUniversalTime().Ticks -ne [DateTimeOffset]::Parse($target.startedUtc).UtcDateTime.Ticks -or $process.SessionId -ne $session -or $owner.ReturnValue -ne 0 -or $owner.Sid -cne $sid -or (Assert-LocalPath $process.Path) -ine $target.image -or (Get-Digest $target.image) -cne $target.sha256){throw 'Original observed process identity changed.'}
  } finally {$process.Dispose()}
 }
}
try {
 if(-not $InternalRequest) {
  $job=[pscustomobject]@{schema=1;phase=$Phase;workspace=$Workspace;helperDirectory=$ReviewedHelperDirectory;failureResult=(Assert-LocalPath $FailureResult);failureResultSha256=$FailureResultSha256;failureRequestSha256=$FailureRequestSha256;observation=(Assert-LocalPath $Observation);observationSha256=$ObservationSha256;targetSha256=$ExpectedTargetSha256;inspection=$Inspection;inspectionSha256=$InspectionSha256;reviewedImageSha256=$ReviewedImageSha256}
  if($Phase -eq 'Acknowledge' -and (-not $Inspection -or $InspectionSha256 -cnotmatch '^[a-f0-9]{64}$' -or $ReviewedImageSha256 -cnotmatch '^[a-f0-9]{64}$')){throw 'Independent exact inspection and reviewed image hashes required.'}
  if($Phase -eq 'Inspect' -and ($Inspection -or $InspectionSha256 -or $ReviewedImageSha256)){throw 'Inspect cannot accept acknowledgement evidence.'}
  $proof=Read-BootstrapAuthority $job
  $sid=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value
  if($sid -cne $proof.observed.sid){throw 'Observation belongs to another account.'}
  $directory=New-PreparationDirectory ([pscustomobject]@{workspace=$Workspace;sid=$sid}) 'bootstrap-welcome'
  $job|Add-Member directory $directory
  $job|Add-Member helpers @((Get-ChildItem -LiteralPath $ReviewedHelperDirectory -File|Where-Object {$_.Extension -cin @('.ps1','.cs')}|Sort-Object Name)|ForEach-Object {@{name=$_.Name;sha256=(Get-Digest $_.FullName)}})
  foreach($entry in $job.helpers){$held.Add([IO.File]::Open((Join-Path $ReviewedHelperDirectory $entry.name),[IO.FileMode]::Open,[IO.FileAccess]::Read,[IO.FileShare]::Read))}
  $self=Assert-LocalPath $PSCommandPath;$held.Add([IO.File]::Open($self,[IO.FileMode]::Open,[IO.FileAccess]::Read,[IO.FileShare]::Read));$selfHash=Get-Digest $self
  $requestPath=Join-Path $directory 'request.json';Write-Record $requestPath $job
  $held.Add([IO.File]::Open($requestPath,[IO.FileMode]::Open,[IO.FileAccess]::Read,[IO.FileShare]::Read));$requestHash=Get-Digest $requestPath
  $action=New-ScheduledTaskAction -Execute (Join-Path $env:WINDIR 'System32\WindowsPowerShell\v1.0\powershell.exe') -Argument ('-NoProfile -NonInteractive -ExecutionPolicy Bypass -File "'+$self+'" -InternalRequest "'+$requestPath+'" -InternalRequestSha256 '+$requestHash+' -SelfSha256 '+$selfHash)
  $principal=New-ScheduledTaskPrincipal -UserId $sid -LogonType Interactive -RunLevel Limited
  $settings=New-ScheduledTaskSettingsSet -ExecutionTimeLimit (New-TimeSpan -Seconds 90) -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries
  $name='OpenNavX-BootstrapWelcome-'+[guid]::NewGuid().ToString('N');$task=$null
  try {
   $task=Register-ScheduledTask -TaskName $name -Action $action -Principal $principal -Settings $settings;Start-ScheduledTask -TaskName $name
   $resultPath=Join-Path $directory 'review.json';$deadline=[datetime]::UtcNow.AddSeconds(90)
   while(-not (Test-Path -LiteralPath $resultPath) -and [datetime]::UtcNow -lt $deadline){Start-Sleep -Milliseconds 200}
   if(-not (Test-Path -LiteralPath $resultPath)){throw ('Recovery observation timed out; inspect one-use intents without retry: '+$directory)}
   Read-Record $resultPath|ConvertTo-Json -Depth 16
  } finally {if($task){Unregister-ScheduledTask -TaskName $name -Confirm:$false}}
  return
 }
 $directory=Assert-LocalPath $job.directory
 if([IO.Path]::GetDirectoryName($directory) -ine (Join-Path $Workspace 'runs') -or [IO.Path]::GetDirectoryName($InternalRequest) -ine $directory){throw 'Private recovery directory required.'}
 $acl=Get-Acl -LiteralPath $directory
 if(-not $acl.AreAccessRulesProtected){throw 'Recovery evidence must be private.'}
 foreach($rule in $acl.GetAccessRules($true,$true,[Security.Principal.SecurityIdentifier])){if($rule.IdentityReference.Value -cnotin @([Security.Principal.WindowsIdentity]::GetCurrent().User.Value,'S-1-5-18','S-1-5-32-544')){throw 'Unexpected recovery evidence principal.'}}
 $result=@{schema=1;owner='SKAGER.FailedBootstrapWelcome.1';status='failed';phase=$job.phase;utc=[datetime]::UtcNow.ToString('o');failureResultSha256=$job.failureResultSha256;failureRequestSha256=$job.failureRequestSha256;observationSha256=$job.observationSha256;targetSha256=$job.targetSha256;selfSha256=$SelfSha256;helpers=$job.helpers;bootstrapAccepted=$false;startupHealthAccepted=$false;acknowledgementSent=$false;launcherDismissed=$false;scope='diagnostic navigation-caution review only; no launch or design acceptance'}
 try {
  $proof=Read-BootstrapAuthority $job;Assert-LiveObservation $proof
  Initialize-StockWelcomeNative
  if(-not ('OpenNavX.RestartWindowNative' -as [type])){Add-Type -Path (Join-Path $ReviewedHelperDirectory 'RestartWindowNative.cs')}
  $dpi=[OpenNavX.RestartWindowNative]::SetThreadDpiAwarenessContext([IntPtr](-4));if($dpi -eq [IntPtr]::Zero){throw 'Physical DPI context unavailable.'}
  try {
   if($job.phase -ceq 'Inspect') {
    $ticks=[DateTimeOffset]::Parse('2026-10-05T20:01:57.2931063Z').UtcDateTime.Ticks
    $info=Invoke-StockWelcomeFocus 7196 $ticks (Join-Path $directory 'focus-intent.json')
    Assert-StockWelcomeWindow $info 7196
    $proof=Read-BootstrapAuthority $job;Assert-LiveObservation $proof
    $image=Join-Path $directory 'welcome.png';$result.imageSha256=Save-StockWelcomeCapture 7196 $info $image
    $result.image=$image;$result.nativeWindow=$info;$result.status='passed'
   } elseif($job.phase -ceq 'Acknowledge') {
    $inspectionRecord=Hold-Proof $job.inspection $job.inspectionSha256
    if($inspectionRecord.owner -cne 'SKAGER.FailedBootstrapWelcome.1' -or $inspectionRecord.status -cne 'passed' -or $inspectionRecord.phase -cne 'Inspect' -or $inspectionRecord.acknowledgementSent -ne $false -or $inspectionRecord.bootstrapAccepted -ne $false -or $inspectionRecord.startupHealthAccepted -ne $false){throw 'Separate successful diagnostic inspection required.'}
    foreach($field in @('failureResultSha256','failureRequestSha256','observationSha256','targetSha256')){if($inspectionRecord.$field -cne $job.$field){throw 'Inspection authority differs.'}}
    if($inspectionRecord.selfSha256 -cne $SelfSha256 -or ($inspectionRecord.helpers|ConvertTo-Json -Depth 4 -Compress) -cne ($job.helpers|ConvertTo-Json -Depth 4 -Compress)){throw 'Inspection helper bytes differ.'}
    $at=[datetime]::Parse($inspectionRecord.utc).ToUniversalTime();if($at -gt [datetime]::UtcNow -or ([datetime]::UtcNow-$at).TotalMinutes -gt 30){throw 'Warning inspection expired.'}
    $inspectionDir=[IO.Path]::GetDirectoryName((Assert-LocalPath $job.inspection))
    if([IO.Path]::GetDirectoryName($inspectionDir) -ine (Join-Path $Workspace 'runs') -or $inspectionDir -ieq $directory -or [IO.Path]::GetFileName($job.inspection) -cne 'review.json' -or $inspectionRecord.image -ine (Join-Path $inspectionDir 'welcome.png') -or $inspectionRecord.imageSha256 -cne $job.reviewedImageSha256 -or (Get-Digest $inspectionRecord.image) -cne $job.reviewedImageSha256){throw 'Exact independently reviewed warning image required.'}
    Assert-StockWelcomeWindow $inspectionRecord.nativeWindow 7196
    $intent=Join-Path $inspectionDir 'agree-intent.json';if(Test-Path -LiteralPath $intent){throw 'Acknowledgement already consumed; no retry.'}
    $proof=Read-BootstrapAuthority $job;Assert-LiveObservation $proof
    Invoke-StockWelcomeAgreement 7196 $inspectionRecord.nativeWindow $job.reviewedImageSha256 (Join-Path $directory 'before-agree.png') $intent
    $result.acknowledgementSent=$true;$result.status='passed';$result.inspectionSha256=$job.inspectionSha256;$result.reviewedImageSha256=$job.reviewedImageSha256
    $result.note='Agree delivery attempted once. Startup health remains failed/unaccepted; fresh observation required. No frame capture, app close or relaunch performed.'
   } else {throw 'Unsupported recovery phase.'}
  } finally {$null=[OpenNavX.RestartWindowNative]::SetThreadDpiAwarenessContext($dpi)}
 } catch {$result.error=$_.Exception.Message;$result.errorType=$_.Exception.GetType().FullName;$result.scriptStackTrace=$_.ScriptStackTrace;$result.positionMessage=$_.InvocationInfo.PositionMessage}
 Write-Record (Join-Path $directory 'review.json') $result
} finally {foreach($handle in $held){$handle.Dispose()}}

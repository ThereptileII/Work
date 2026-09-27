# Evidence policy and isolated runtime-proof fixtures; no real app or user data.
[CmdletBinding()]
param([switch]$PortableContracts,[switch]$IsolatedLocal)
. (Join-Path $PSScriptRoot 'InstalledWelcome.ps1')
$checks=New-Object 'Collections.Generic.List[string]'
function Pass([string]$Name,[scriptblock]$Body){& $Body;$checks.Add($Name)}
function Refuse([string]$Name,[scriptblock]$Body){$bad=$false;try{$null=& $Body}catch{$bad=$true};if(-not $bad){throw ('Unsafe installed warning accepted: '+$Name)};$checks.Add($Name)}
function CopyValue($Value){return ($Value | ConvertTo-Json -Depth 12 | ConvertFrom-Json)}
$now=[datetime]::UtcNow
$job=[pscustomobject]@{action='ReviewInstalledWelcome';reviewAction='InspectWelcome';processId=42;executable='C:\owned\app\opencpn.exe';executableSha256=('a'*64);workspace='C:\XNav';buildCommit=('b'*40);generation=('c'*32);launchResultSha256=('d'*64);launchRequestSha256=('e'*64);helperFiles=@(@{name='InstalledWelcome.ps1';sha256=('f'*64)})}
$request=[pscustomobject]@{action='Launch';mode='--xnav';executable=$job.executable;executableSha256=$job.executableSha256;workspace=$job.workspace}
$launch=[pscustomobject]@{status='passed';action='Launch';mode='--xnav';pid=42;executableSha256=$job.executableSha256;generation=$job.generation;buildCommit=$job.buildCommit;targetSha256=('1'*64);sid='S-1-5-21-1';sessionId=1;utc=$now.AddMinutes(-1).ToString('o');processStartedUtc=$now.AddSeconds(-58).ToString('o')}
$installed=[pscustomobject]@{state=[pscustomobject]@{current=$job.generation};ownership=[pscustomobject]@{commit=$job.buildCommit;version='0.4.0-beta2'};executable=$job.executable}
$build=[pscustomobject]@{test_fixtures=$false;build_purpose='INSTALLED PRODUCT';version='0.4.0-beta2';commit=$job.buildCommit;executable_sha256=$job.executableSha256}
foreach($mode in @('--xnav','--legacy','--safe-mode')){Pass "Exact separately launched installed mode: $mode" {$r=CopyValue $request;$r.mode=$mode;$l=CopyValue $launch;$l.mode=$mode;Assert-InstalledWelcomePolicy $job $l $r $installed $build $now}}
foreach($action in @('InspectWelcome','FocusWelcome','AcknowledgeWelcome')){Pass "Fixed warning action $action" {$v=CopyValue $job;$v.reviewAction=$action;Assert-InstalledWelcomePolicy $v $launch $request $installed $build $now}}
foreach($action in @('LaunchStock','LaunchPortableReview','ReviewRestartChild','Launch;Other')){Refuse 'Non-normal launch identity cannot authorize installed warning' {$v=CopyValue $launch;$v.action=$action;Assert-InstalledWelcomePolicy $job $v $request $installed $build $now}}
foreach($field in @('action','mode','executable','executableSha256','workspace')){Refuse "Changed request $field" {$v=CopyValue $request;$v.$field='wrong';Assert-InstalledWelcomePolicy $job $launch $v $installed $build $now}}
foreach($field in @('status','action','mode','executableSha256','generation','buildCommit','targetSha256','sid')){Refuse "Changed launch $field" {$v=CopyValue $launch;$v.$field='wrong';Assert-InstalledWelcomePolicy $job $v $request $installed $build $now}}
foreach($field in @('processStartedUtc','sid','sessionId','targetSha256','generation','buildCommit')){Refuse "Older receipt missing $field is not upgraded by inference" {$v=CopyValue $launch;$v.PSObject.Properties.Remove($field);Assert-InstalledWelcomePolicy $job $v $request $installed $build $now}}
foreach($value in @($true,'false',0,$null)){Refuse 'Only literal fixture-disabled product accepted' {$v=CopyValue $build;$v.test_fixtures=$value;Assert-InstalledWelcomePolicy $job $launch $request $installed $v $now}}
foreach($field in @('build_purpose','version','commit','executable_sha256')){Refuse "Changed owned build $field" {$v=CopyValue $build;$v.$field='wrong';Assert-InstalledWelcomePolicy $job $launch $request $installed $v $now}}
foreach($action in @('Capture','Cancel','Agree','Click','AUTO','STANDBY','Restart','')){Refuse 'No arbitrary installed warning action' {$v=CopyValue $job;$v.reviewAction=$action;Assert-InstalledWelcomePolicy $v $launch $request $installed $build $now}}
foreach($time in @($now.AddHours(-5),$now.AddMinutes(1))){Refuse 'Expired/future launch refused' {$v=CopyValue $launch;$v.utc=$time.ToString('o');Assert-InstalledWelcomePolicy $job $v $request $installed $build $now}}
$process=[pscustomobject]@{Id=42;Path=$job.executable;SessionId=1;MainWindowHandle=123;HasExited=$false;StartTime=[datetime]::Parse($launch.processStartedUtc)}
Pass 'Exact native process creation tick/SID/session policy' {Assert-InstalledWelcomeProcess $process $job $launch 'S-1-5-21-1' 1}
foreach($field in @('Id','Path','SessionId','MainWindowHandle','HasExited','StartTime')){Refuse "PID reuse or changed process $field" {$v=CopyValue $process;$v.$field=switch($field){'Id'{43};'Path'{'C:\other.exe'};'SessionId'{2};'MainWindowHandle'{0};'HasExited'{$true};'StartTime'{$process.StartTime.AddTicks(1)}};Assert-InstalledWelcomeProcess $v $job $launch 'S-1-5-21-1' 1}}
Refuse 'Different SID refused' {Assert-InstalledWelcomeProcess $process $job $launch 'S-1-5-21-2' 1}
$inspection=[pscustomobject]@{status='passed';action='ReviewInstalledWelcome';reviewAction='InspectWelcome';utc=$now.AddMinutes(-1).ToString('o');mode=$launch.mode;processId=42;buildCommit=$job.buildCommit;generation=$job.generation;executableSha256=$job.executableSha256;launchResultSha256=$job.launchResultSha256;launchRequestSha256=$job.launchRequestSha256;helperFiles=$job.helperFiles;imageSha256=('2'*64);
  nativeWindow=[pscustomobject]@{Frame=123;Modal=456;Agree=789;Cancel=790;Html=791;Dpi=96;Bounds=[pscustomobject]@{Left=100;Top=100;Right=700;Bottom=500;Width=600;Height=400};ProcessId=42;Title='Welcome to OpenCPN';ModalClass='#32770';AgreeText='Agree';CancelText='Cancel';AgreeId=5100;CancelId=5101;HtmlClass='wxWindowNR';HtmlName='htmlWindow'}}
Pass 'Inspection belongs to this exact installed launch and warning' {Assert-InstalledWelcomeInspection $inspection $job $launch $now}
foreach($field in @('action','reviewAction','mode','buildCommit','generation','executableSha256','launchResultSha256','launchRequestSha256','imageSha256')){Refuse "Cross-stock/cross-build/changed captured proof $field" {$v=CopyValue $inspection;$v.$field='changed';Assert-InstalledWelcomeInspection $v $job $launch $now}}
Refuse 'Changed helper after inspection' {$v=CopyValue $inspection;$v.helperFiles[0].sha256='3'*64;Assert-InstalledWelcomeInspection $v $job $launch $now}
Refuse 'Old inspection refused' {$v=CopyValue $inspection;$v.utc=$now.AddMinutes(-31).ToString('o');Assert-InstalledWelcomeInspection $v $job $launch $now}
foreach($field in @('Title','ModalClass','AgreeText','CancelText','HtmlClass','HtmlName')){Refuse "Different native caution $field" {$v=CopyValue $inspection;$v.nativeWindow.$field='changed';Assert-InstalledWelcomeInspection $v $job $launch $now}}
Pass 'Installed inspection preserves actual C# serialized rectangle dimensions' {
 $typed=Convert-StockWelcomeWindow $inspection.nativeWindow
 $parsed=$typed | ConvertTo-Json -Depth 8 | ConvertFrom-Json
 $restored=Convert-StockWelcomeWindow $parsed
 if($parsed.Bounds.Width -ne 600 -or $restored.Bounds.Width -ne 600 -or $restored.Modal -ne 456){throw 'Installed warning JSON conversion differs'}
}
Refuse 'Installed warning cannot ignore inconsistent serialized derived bounds' {$v=CopyValue $inspection;$v.nativeWindow.Bounds.Width++;Convert-StockWelcomeWindow $v.nativeWindow}
foreach($fileName in @('InstalledWelcome.ps1','review-installed-welcome.ps1','Common.ps1','InteractiveJob.ps1')){Pass "Parses $fileName" {$tokens=$null;$errors=$null;$null=[Management.Automation.Language.Parser]::ParseFile((Join-Path $PSScriptRoot $fileName),[ref]$tokens,[ref]$errors);if($errors.Count){throw ($errors|Out-String)}}}
$arguments=@{InstalledWelcomeFixture=$true};if($PortableContracts){$arguments.PortableContracts=$true};if($IsolatedLocal){$arguments.IsolatedLocal=$true}
$runtime=& (Join-Path $PSScriptRoot 'test-commissioning-launch.ps1') @arguments | ConvertFrom-Json
if($runtime.status -cne 'passed' -or $runtime.identity -cne 'installed-warning-runtime'){throw 'Installed runtime proof fixture failed'}
Refuse 'An installed FocusWelcome result is never acknowledgement evidence' {$v=CopyValue $inspection;$v.reviewAction='FocusWelcome';Assert-InstalledWelcomeInspection $v $job $launch $now}
# Policy ordering with fake operations only. Full unmocked installed runtime
# mutation/refusal fixture above remains mandatory and runs before this scope.
& {
  $focusJob=CopyValue $job;$focusJob.reviewAction='FocusWelcome'
  $focusProof=[pscustomobject]@{launch=$launch}
  $focusProcess=CopyValue $process
  $focusProcess|Add-Member -MemberType ScriptMethod -Name Refresh -Value {if($script:reuseFocusPid){$this.StartTime=$this.StartTime.AddTicks(1)}}
  function Read-InstalledWelcome($Unused){$script:focusAudits++;if($script:focusAudits -eq $script:rejectFocusAudit){throw 'TEST full installed proof rejected'};return $focusProof}
  function Invoke-StockWelcomeFocus([int]$ProcessId,[long]$StartedUtcTicks,[string]$Intent){
    $script:focusCalls++
    if($ProcessId -ne 42 -or $StartedUtcTicks -ne $process.StartTime.ToUniversalTime().Ticks -or [IO.Path]::GetFileName($Intent) -cne 'focus-intent.json'){throw 'Bound fixed focus identity lost'}
    if($script:uncertainFocus){throw 'TEST caption input uncertain'}
    return [pscustomobject]@{ProcessId=$ProcessId;Modal=456}
  }
  function Save-StockWelcomeCapture([int]$ProcessId,$Info,[string]$Path){$script:focusCaptures++;return ('9'*64)}
  function ResetFocus { $script:focusAudits=0;$script:focusCalls=0;$script:focusCaptures=0;$script:rejectFocusAudit=0;$script:reuseFocusPid=$false;$script:uncertainFocus=$false;$focusProcess.StartTime=$process.StartTime }
  $directory=[IO.Path]::GetTempPath()
  Pass 'Installed focus verifies complete proof before and after and retains no-ack result' {
    ResetFocus;$value=Invoke-InstalledWelcomeFocus $focusJob $focusProof $focusProcess 'S-1-5-21-1' 1 $directory
    if($focusAudits -ne 2 -or $focusCalls -ne 1 -or $focusCaptures -ne 1 -or $value.acknowledgementSent -ne $false -or $value.focusVerified -ne $true){throw 'Installed focus proof/result boundary differs'}
  }
  Pass 'Installed proof failure before input cannot reach caption primitive' {
    ResetFocus;$script:rejectFocusAudit=1;$failed=$false
    try{$null=Invoke-InstalledWelcomeFocus $focusJob $focusProof $focusProcess 'S-1-5-21-1' 1 $directory}catch{$failed=$true}
    if(-not $failed -or $focusCalls -ne 0 -or $focusCaptures -ne 0){throw 'Proof rejection reached input or did not refuse'}
  }
  Pass 'Source/generation change after input refuses capture and success' {
    ResetFocus;$script:rejectFocusAudit=2;$failed=$false
    try{$null=Invoke-InstalledWelcomeFocus $focusJob $focusProof $focusProcess 'S-1-5-21-1' 1 $directory}catch{$failed=$true}
    if(-not $failed -or $focusCalls -ne 1 -or $focusAudits -ne 2 -or $focusCaptures -ne 0){throw 'Changed proof became successful focus/capture'}
  }
  Pass 'PID reuse on refresh refuses before caption input' {
    ResetFocus;$script:reuseFocusPid=$true;$failed=$false
    try{$null=Invoke-InstalledWelcomeFocus $focusJob $focusProof $focusProcess 'S-1-5-21-1' 1 $directory}catch{$failed=$true}
    if(-not $failed -or $focusCalls -ne 0 -or $focusCaptures -ne 0){throw 'Reused process reached focus'}
  }
  Pass 'Uncertain native focus never retries or proceeds to capture' {
    ResetFocus;$script:uncertainFocus=$true;$failed=$false
    try{$null=Invoke-InstalledWelcomeFocus $focusJob $focusProof $focusProcess 'S-1-5-21-1' 1 $directory}catch{$failed=$true}
    if(-not $failed -or $focusCalls -ne 1 -or $focusAudits -ne 1 -or $focusCaptures -ne 0){throw 'Uncertain input was retried or claimed successful'}
  }
}
[pscustomobject]@{status='passed';count=$checks.Count+$runtime.count;policyCount=$checks.Count;runtimeCount=$runtime.count;checks=@($checks);runtimeChecks=$runtime.checks;windowsApisInvoked=$false;applicationLaunched=$false;boatAccess=$false;hardwareCommands=$false;actualInstalledModalAcceptance=$false} | ConvertTo-Json -Depth 6

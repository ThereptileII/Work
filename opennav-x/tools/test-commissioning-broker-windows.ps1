# Native CI only: actual fixed broker + marker-only application/companion.
# Identity substitutions are confined to copied dependencies in random TEMP.
# No installed application, real profile, marine input or physical output used.
[CmdletBinding()]
param([Parameter(Mandatory=$true)][string]$Binaries,[Parameter(Mandatory=$true)][string]$Evidence)
Set-StrictMode -Version Latest;$ErrorActionPreference='Stop'
if($env:GITHUB_ACTIONS -cne 'true' -or [Environment]::OSVersion.Platform -ne 'Win32NT'){throw 'Disposable native Windows CI only.'}
if($PSVersionTable.PSEdition -ne 'Desktop') {
 & (Join-Path $env:WINDIR 'System32\WindowsPowerShell\v1.0\powershell.exe') -NoProfile -NonInteractive -ExecutionPolicy Bypass -File $PSCommandPath -Binaries $Binaries -Evidence $Evidence
 exit $LASTEXITCODE
}
if(@(Get-Process -Name opencpn -ErrorAction SilentlyContinue).Count){throw 'No existing OpenCPN process permitted during isolated marker tests.'}
$sourceTools=Join-Path $PSScriptRoot 'boat';$fixtureSources=Join-Path (Split-Path $PSScriptRoot -Parent) 'tests/commissioning-restart'
. (Join-Path $sourceTools 'RestartCommissioning.ps1')
. (Join-Path $sourceTools 'Commissioning.ps1')
. (Join-Path $fixtureSources 'New-BrokerFixture.ps1')
$Binaries=Assert-LocalPath ([IO.Path]::GetFullPath($Binaries));$Evidence=Assert-LocalPath ([IO.Path]::GetFullPath($Evidence))
$marker=Join-Path $Binaries 'opencpn.exe';$helper=Join-Path $Binaries 'opennav-restart.exe'
$signature='OpenNavX.NativeRestart.MarkerOnly.1'
# Check inert compiled identity before executing ANY argument. Passing an actual
# navigation executable as Binaries cannot accidentally launch real software.
if(-not [Text.Encoding]::ASCII.GetString([IO.File]::ReadAllBytes($marker)).Contains($signature)){throw 'This is not the marker-only test application.'}
$identity=& $marker --marker-self-test
if($LASTEXITCODE -ne 0){throw 'Marker-only identity probe failed.'}
$identity=$identity|ConvertFrom-Json
if($identity.contract -cne $signature -or $identity.marine_code -isnot [bool] -or $identity.marine_code -or
   $identity.child_started -isnot [bool] -or $identity.child_started -or $identity.profile_accessed -isnot [bool] -or $identity.profile_accessed){throw 'Wrong marker-only capabilities.'}
$root=Join-Path ([IO.Path]::GetTempPath()) ('OpenNav broker fixture '+[guid]::NewGuid().ToString('N'))
$null=New-Item -ItemType Directory -Path $root
$null=New-Item -ItemType Directory -Path $Evidence -Force
$checks=New-Object 'Collections.Generic.List[string]';$cases=New-Object 'Collections.Generic.List[object]'
function Require($Condition,[string]$Label){if(-not $Condition){throw $Label};$checks.Add($Label)}
function WaitMarker([string]$Path,[int]$Seconds=15){$until=[DateTime]::UtcNow.AddSeconds($Seconds);while(-not [IO.File]::Exists($Path)){if([DateTime]::UtcNow -ge $until){throw ('Missing fixture marker: '+[IO.Path]::GetFileName($Path))};Start-Sleep -Milliseconds 20}}
$status='failed';$failure=$null;$taskName=$null
try {
 foreach($case in @('success','output-connection','plugin-bytes','expired','consumed','receipt-recording-failure')) {
  $fixture=New-BrokerFixture (Join-Path $root $case) $Binaries $sourceTools $fixtureSources $case
  [IO.File]::WriteAllText((Join-Path $fixture.app 'target-mode.txt'),'--legacy')
  [IO.File]::WriteAllText((Join-Path $fixture.app 'hold-child.txt'),'explicit marker-only PID verification window')
  $start=New-Object Diagnostics.ProcessStartInfo;$start.FileName=$fixture.executable;$start.Arguments='--parent';$start.WorkingDirectory=$fixture.app;$start.UseShellExecute=$false
  $start.EnvironmentVariables['PATH']=$fixture.session.path
  $start.EnvironmentVariables['LOCALAPPDATA']=(Join-Path $fixture.root 'local')
  $start.EnvironmentVariables['APPDATA']=(Join-Path $fixture.root 'roaming')
  $start.EnvironmentVariables['OPENNAV_COMMISSIONING_RESTART_SESSION']=$fixture.session.session
  $start.EnvironmentVariables['OPENNAV_COMMISSIONING_RESTART_RECORD_SHA256']=$fixture.recordSha256
  $parent=$null;$broker=$null;$companion=$null;$outTask=$null;$errTask=$null
  try {
   $parent=[Diagnostics.Process]::Start($start);$null=$parent.Handle
   $created=$parent.StartTime.ToUniversalTime().ToFileTimeUtc().ToString()
   WaitMarker (Join-Path $fixture.app 'parent-armed.txt')
   Require ([IO.File]::ReadAllLines((Join-Path $fixture.app 'parent-armed.txt'))[0] -ceq 'yes') ($case+': actual marker armed actual guarded helper')
   $companions=@(Get-CimInstance Win32_Process -Filter ('ParentProcessId='+$parent.Id)|Where-Object {$_.ExecutablePath -and $_.ExecutablePath -ieq $fixture.helper})
   Require ($companions.Count -eq 1) ($case+': exactly one owned fixture helper')
   $companion=Get-Process -Id $companions[0].ProcessId;$null=$companion.Handle
   $cold=Join-Path $fixture.sessionDirectory 'cold-launch-consumed.json'
   Write-Record $cold @{owner=$script:RestartOwner;session=$fixture.session.session;recordSha256=$fixture.recordSha256;status='consumed-before-start';mode='--xnav'}
   Write-Record (Join-Path $fixture.sessionDirectory 'cold-child.json') @{owner=$script:RestartOwner;session=$fixture.session.session;recordSha256=$fixture.recordSha256;pid=$parent.Id.ToString();createdFiletime=$created;launchSha256=(Get-Digest $cold)}
   if($case -ceq 'consumed'){$null=New-Item -ItemType Directory -Path $fixture.transition;Write-Record (Join-Path $fixture.transition 'permit-consumed.json') @{testOnly='interrupted consumed transition'}}
   $run=New-Object Diagnostics.ProcessStartInfo
   $run.FileName=Join-Path $env:WINDIR 'System32\WindowsPowerShell\v1.0\powershell.exe'
   $run.Arguments='-NoProfile -NonInteractive -ExecutionPolicy Bypass -File "'+(Join-Path $fixture.scripts 'RestartCommissioningBroker.ps1')+'" -SessionRecord "'+$fixture.record+'" -ExpectedSha256 '+$fixture.recordSha256+' -ParentProcessId '+$parent.Id+' -ParentCreatedFiletime '+$created+' -Mode --legacy'
   $run.WorkingDirectory=$fixture.scripts;$run.UseShellExecute=$false;$run.RedirectStandardOutput=$true;$run.RedirectStandardError=$true
   $run.EnvironmentVariables['LOCALAPPDATA']=(Join-Path $fixture.root 'local')
   $run.EnvironmentVariables['OPENNAV_BROKER_FIXTURE_IDENTITY_SHA256']=$fixture.identityHash
   $broker=[Diagnostics.Process]::Start($run);$outTask=$broker.StandardOutput.ReadToEndAsync();$errTask=$broker.StandardError.ReadToEndAsync()
   if($case -cin @('expired','consumed')) {
    Require ($broker.WaitForExit(15000) -and $broker.ExitCode -ne 0) ($case+': broker refuses before opening authorization pipe')
   } else {
    $ready=Join-Path $fixture.transition 'ready.json';$until=[DateTime]::UtcNow.AddSeconds(20)
    while(-not [IO.File]::Exists($ready) -and -not $broker.HasExited -and [DateTime]::UtcNow -lt $until){Start-Sleep -Milliseconds 20}
    if(-not [IO.File]::Exists($ready)){$detail=if($errTask.IsCompleted){$errTask.GetAwaiter().GetResult()}else{'broker stderr still pending; no blocking read'};throw ('Actual broker failed to publish readiness: '+$detail)}
    Require (-not $parent.HasExited) ($case+': parent held alive until broker ready')
    $text=[IO.File]::ReadAllText($fixture.profile).Replace('InterfaceMode=xnav','InterfaceMode=legacy')
    if($case -ceq 'output-connection'){$text=$text.Replace('COM8;115200;0;0;','COM8;115200;0;1;')}
    [IO.File]::WriteAllText($fixture.profile,$text,(New-Object Text.UTF8Encoding($false)))
    if($case -ceq 'plugin-bytes'){[IO.File]::AppendAllText($fixture.plugin,'unreviewed change')}
    if($case -ceq 'receipt-recording-failure'){[IO.File]::WriteAllText((Join-Path $fixture.transition 'receipt.json'),'explicit create-new failure injection')}
   }
   [IO.File]::WriteAllText((Join-Path $fixture.app 'parent-release.txt'),'one deliberate fixture close')
   Require ($parent.WaitForExit(15000) -and $parent.ExitCode -eq 0) ($case+': genuine parent exits normally')
   Require ($broker.WaitForExit(120000)) ($case+': bounded broker completion')
   Require ($companion.WaitForExit(15000)) ($case+': actual helper exits after permit or refusal')
   $children=@(Get-ChildItem -LiteralPath $fixture.app -Filter 'child--*.txt')
   $permit=Join-Path $fixture.transition 'permit-consumed.json';$completion=Join-Path $fixture.transition 'completion.json'
   if($case -ceq 'success') {
    Require ($broker.ExitCode -eq 0 -and $companion.ExitCode -eq 0 -and $children.Count -eq 1) 'success: exact broker/audit/helper produces one marker child'
    Require ([IO.File]::Exists($completion) -and [IO.File]::Exists($permit)) 'success: consumed-before-allow and verified child completion retained'
    $record=Read-Record $completion;$child=Get-Process -Id ([int]$record.child.pid)
    try{Require ($child.Path -ieq $fixture.executable -and $child.StartTime.ToUniversalTime().ToFileTimeUtc().ToString() -ceq $record.child.createdFiletime) 'success: actual retained child identity matches durable broker receipt'}finally{$child.Dispose()}
   } elseif($case -ceq 'receipt-recording-failure') {
    Require ($broker.ExitCode -ne 0 -and $children.Count -eq 1 -and [IO.File]::Exists($permit) -and -not [IO.File]::Exists($completion)) 'receipt failure: actual child exists but permit remains consumed without accepted completion'
    $record=Read-Record (Join-Path $fixture.transition 'failure.json')
    Require ($record.status -ceq 'uncertain-or-failed-consumed-permit') 'receipt failure: uncertainty is explicit and never retried'
   } else {
    Require ($broker.ExitCode -ne 0 -and $companion.ExitCode -ne 0 -and $children.Count -eq 0 -and -not [IO.File]::Exists($completion)) ($case+': refusal starts no child')
    if($case -cne 'consumed'){Require (-not [IO.File]::Exists($permit)) ($case+': no allow permit issued')}
   }
   $stdout=$outTask.GetAwaiter().GetResult();$stderr=$errTask.GetAwaiter().GetResult()
   [IO.File]::WriteAllText((Join-Path $Evidence ($case+'-stdout.txt')),$stdout)
   [IO.File]::WriteAllText((Join-Path $Evidence ($case+'-stderr.txt')),$stderr)
   $cases.Add(@{name=$case;brokerExit=$broker.ExitCode;helperExit=$companion.ExitCode;markerChildren=$children.Count;realApplication=$false})
  } finally {
   # Release only our marker protocol; never Stop-Process/Kill any application.
   [IO.File]::WriteAllText((Join-Path $fixture.app 'parent-release.txt'),'fixture cleanup release')
   [IO.File]::WriteAllText((Join-Path $fixture.app 'child-release.txt'),'fixture cleanup release')
   foreach($process in @($parent,$companion,$broker)){if($process){$null=$process.WaitForExit(15000);$process.Dispose()}}
   if($outTask -and $outTask.IsCompleted){[IO.File]::WriteAllText((Join-Path $Evidence ($case+'-stdout.txt')),$outTask.GetAwaiter().GetResult())}
   if($errTask -and $errTask.IsCompleted){[IO.File]::WriteAllText((Join-Path $Evidence ($case+'-stderr.txt')),$errTask.GetAwaiter().GetResult())}
  }
 }
 # Actual ScheduledTasks/CIM representation gate: one inert owned action, no
 # product broker/app launch. Exercise exact policy used by Arm/Collect.
 $taskScript=Join-Path $root 'task-marker.ps1';$taskMarker=Join-Path $root 'task-complete.txt'
 [IO.File]::WriteAllText($taskScript,"[IO.File]::WriteAllText('"+$taskMarker.Replace("'","''")+"','inert disposable task completed')")
 $execute=Join-Path $env:WINDIR 'System32\WindowsPowerShell\v1.0\powershell.exe';$arguments='-NoProfile -NonInteractive -ExecutionPolicy Bypass -File "'+$taskScript+'"'
 $taskName='OpenNavX-RestartReview-'+[guid]::NewGuid().ToString('N');$sid=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value
 $action=New-ScheduledTaskAction -Execute $execute -Argument $arguments;$principal=New-ScheduledTaskPrincipal -UserId $sid -LogonType Interactive -RunLevel Limited
 $settings=New-ScheduledTaskSettingsSet -ExecutionTimeLimit (New-TimeSpan -Seconds 30) -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries
 $null=Register-ScheduledTask -TaskName $taskName -Action $action -Principal $principal -Settings $settings
 Start-ScheduledTask -TaskName $taskName;WaitMarker $taskMarker 30
 $until=[DateTime]::UtcNow.AddSeconds(10)
 do{$nativeTask=Get-ScheduledTask -TaskName $taskName;if($nativeTask.State.ToString() -ceq 'Ready'){break};Start-Sleep -Milliseconds 100}while([DateTime]::UtcNow -lt $until)
 Write-Record (Join-Path $Evidence 'native-task-identity.json') @{state=$nativeTask.State.ToString();actions=@($nativeTask.Actions|Select-Object Execute,Arguments,WorkingDirectory);principal=@{userId=$nativeTask.Principal.UserId;runLevel=$nativeTask.Principal.RunLevel.ToString();logonType=$nativeTask.Principal.LogonType.ToString()};triggersNull=($null -eq $nativeTask.Triggers);triggerCount=@($nativeTask.Triggers).Count;expected=@{execute=$execute;arguments=$arguments;sid=$sid}}
 Assert-RestartTaskIdentity $nativeTask ([pscustomobject]@{execute=$execute;arguments=$arguments}) $sid
 Require $true 'native Arm/Collect CIM policy matches actual limited interactive trigger-free task'
 Unregister-ScheduledTask -TaskName $taskName -Confirm:$false;$taskName=$null
 $status='passed'
} catch {$failure=$_.Exception.Message;[IO.File]::WriteAllText((Join-Path $Evidence 'failure.txt'),($_|Out-String)+"`r`n"+$_.ScriptStackTrace);throw}
finally {
 if($taskName){$owned=Get-ScheduledTask -TaskName $taskName -ErrorAction SilentlyContinue;if($owned -and $owned.State.ToString() -ceq 'Ready' -and $owned.Actions[0].Execute -ceq $execute -and $owned.Actions[0].Arguments -ceq $arguments){Unregister-ScheduledTask -TaskName $taskName -Confirm:$false}}
 @{status=$status;checks=$checks.Count;checkDetails=$checks.ToArray();cases=$cases.ToArray();failure=$failure;actualBroker=$true;identitySubstitutions='Copied dependencies only; synthetic TEMP installation/profile/known folders';realApplication=$false;boatAccess=$false;physicalOutput=$false;productAcceptance=$false}|ConvertTo-Json -Depth 8|Set-Content -LiteralPath (Join-Path $Evidence 'broker-result.json') -Encoding UTF8
 if($status -ceq 'passed'){Remove-Item -LiteralPath $root -Recurse -Force}
}

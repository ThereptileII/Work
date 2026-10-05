# Native CI: actual unchanged Prepare + Arm/Collect with TEMP identity adapters.
# Marker-only processes and inert plugin bytes; no boat/app/profile/hardware use.
[CmdletBinding()]
param([Parameter(Mandatory=$true)][string]$Binaries,[Parameter(Mandatory=$true)][string]$Evidence)
Set-StrictMode -Version Latest;$ErrorActionPreference='Stop'
if($env:GITHUB_ACTIONS -cne 'true' -or [Environment]::OSVersion.Platform -ne 'Win32NT'){throw 'Disposable native Windows CI only.'}
if($PSVersionTable.PSEdition -ne 'Desktop') {
 & (Join-Path $env:WINDIR 'System32\WindowsPowerShell\v1.0\powershell.exe') -NoProfile -NonInteractive -ExecutionPolicy Bypass -File $PSCommandPath -Binaries $Binaries -Evidence $Evidence
 exit $LASTEXITCODE
}
if(@(Get-Process -Name opencpn -ErrorAction SilentlyContinue).Count){throw 'No existing OpenCPN process permitted.'}
$sourceTools=Join-Path $PSScriptRoot 'boat';$fixtureSources=Join-Path (Split-Path $PSScriptRoot -Parent) 'tests/commissioning-restart'
. (Join-Path $sourceTools 'RestartCommissioning.ps1')
. (Join-Path $sourceTools 'Commissioning.ps1')
. (Join-Path $fixtureSources 'New-BrokerFixture.ps1')
. (Join-Path $fixtureSources 'BrokerMarkerCleanup.ps1')
. (Join-Path $fixtureSources 'Enable-ScheduledBrokerFixture.ps1')
$Binaries=Assert-LocalPath ([IO.Path]::GetFullPath($Binaries));$Evidence=Assert-LocalPath ([IO.Path]::GetFullPath($Evidence))
$marker=Join-Path $Binaries 'opencpn.exe';$signature='OpenNavX.NativeRestart.MarkerOnly.1'
if(-not [Text.Encoding]::ASCII.GetString([IO.File]::ReadAllBytes($marker)).Contains($signature)){throw 'Not the inert marker executable; refusing execution.'}
$probe=& $marker --marker-self-test;if($LASTEXITCODE -ne 0){throw 'Marker capability probe failed.'};$probe=$probe|ConvertFrom-Json
if($probe.contract -cne $signature -or $probe.marine_code -isnot [bool] -or $probe.marine_code -or $probe.child_started -isnot [bool] -or $probe.child_started -or $probe.profile_accessed -isnot [bool] -or $probe.profile_accessed){throw 'Invalid marker-only capabilities.'}
$root=Join-Path ([IO.Path]::GetTempPath()) ('OpenNav broker fixture '+[guid]::NewGuid().ToString('N'))
$null=New-Item -ItemType Directory -Path $root;$null=New-Item -ItemType Directory -Path $Evidence -Force
$checks=New-Object 'Collections.Generic.List[string]';$cases=New-Object 'Collections.Generic.List[object]'
$cleanupErrors=New-Object 'Collections.Generic.List[string]'
$status='failed';$failure=$null
function Require($Condition,[string]$Label){if(-not $Condition){throw $Label};$checks.Add($Label)}
function Invoke-FixtureScript($Fixture,[string]$Name,[string]$Arguments,[string]$Label,[int]$Seconds=45) {
 if($Name -cnotin @('RestartCommissioningPrepare.ps1','RestartCommissioningArm.ps1')){throw 'No arbitrary fixture command permitted.'}
 $run=New-Object Diagnostics.ProcessStartInfo
 $run.FileName=Join-Path $env:WINDIR 'System32\WindowsPowerShell\v1.0\powershell.exe'
 $run.Arguments='-NoProfile -NonInteractive -ExecutionPolicy Bypass -File "'+(Join-Path $Fixture.scripts $Name)+'" '+$Arguments
 $run.WorkingDirectory=$Fixture.scripts;$run.UseShellExecute=$false;$run.RedirectStandardOutput=$true;$run.RedirectStandardError=$true
 $process=[Diagnostics.Process]::Start($run);$stdout=$process.StandardOutput.ReadToEndAsync();$stderr=$process.StandardError.ReadToEndAsync()
 try {
  if(-not $process.WaitForExit($Seconds*1000)){throw ('Bounded fixture command timed out: '+$Label+'; no force termination was performed.')}
  $out=$stdout.GetAwaiter().GetResult();$err=$stderr.GetAwaiter().GetResult()
  [IO.File]::WriteAllText((Join-Path $Evidence ($Label+'-stdout.txt')),$out);[IO.File]::WriteAllText((Join-Path $Evidence ($Label+'-stderr.txt')),$err)
  return [pscustomobject]@{exit=$process.ExitCode;stdout=$out;stderr=$err}
 } finally {$process.Dispose()}
}
function Wait-FixtureFile([string]$Path,[int]$Seconds=20) {
 $until=[DateTime]::UtcNow.AddSeconds($Seconds)
 while(-not [IO.File]::Exists($Path)){if([DateTime]::UtcNow -ge $until){throw ('Missing fixture record: '+[IO.Path]::GetFileName($Path))};Start-Sleep -Milliseconds 50}
}
try {
 foreach($case in @('corrupt-capability','shutdown-copy-hash','success','output-connection','palette-standard')) {
  $palette=$case -ceq 'palette-standard';$mode=$(if($palette){'--xnav'}else{'--legacy'});$expectedSuccess=$case -cin @('success','palette-standard')
  $fixture=New-BrokerFixture (Join-Path $root $case) $Binaries $sourceTools $fixtureSources $case -WithoutRestartSession
  $proof=Enable-ScheduledBrokerFixture $fixture $sourceTools
  Require ($proof.Count -eq 3) ($case+': Prepare/Arm/Collect/Broker entrypoints exactly match production bytes')
  $iniBefore=Get-Digest $fixture.profile;$targetBefore=Get-Digest (Join-Path $fixture.workspace 'boat-target.json')
  Require (@(Get-ChildItem -LiteralPath (Join-Path $fixture.workspace 'runs') -Directory -Filter '*-restart-session-*').Count -eq 0) ($case+': no constructed restart session exists')
  if($case -ceq 'corrupt-capability') {
   $build=Read-Record $fixture.productBuild;$build.commissioning_restart_protocol=0
   [IO.File]::WriteAllText($fixture.productBuild,($build|ConvertTo-Json -Depth 8),(New-Object Text.UTF8Encoding($false)))
   $ownerPath=Join-Path $fixture.generation 'ownership.json';$owner=Read-Record $ownerPath
   @($owner.managedFiles|Where-Object {$_.path -ceq 'docs/PRODUCT_BUILD.json'})[0].sha256=Get-Digest $fixture.productBuild
   [IO.File]::WriteAllText($ownerPath,($owner|ConvertTo-Json -Depth 8),(New-Object Text.UTF8Encoding($false)))
  }
  if($case -ceq 'shutdown-copy-hash') {
   $review=Read-Record $fixture.shutdown
   Require ($review.plugins.Count -eq 3 -and @($review.plugins.sha256|Sort-Object -Unique).Count -eq 3) 'Distinct stock, managed and bundled copies are present in the actual preparation fixture'
   $review.plugins[1].sha256=$review.plugins[0].sha256
   [IO.File]::WriteAllText($fixture.shutdown,($review|ConvertTo-Json -Depth 8),(New-Object Text.UTF8Encoding($false)))
   $fixture.shutdownSha256=Get-Digest $fixture.shutdown
  }
  $prepareArgs='-Workspace "'+$fixture.workspace+'" -ShutdownReview "'+$fixture.shutdown+'" -ShutdownReviewSha256 '+$fixture.shutdownSha256
  $preparedRun=Invoke-FixtureScript $fixture 'RestartCommissioningPrepare.ps1' $prepareArgs ($case+'-prepare')
  Require ((Get-Digest $fixture.profile) -ceq $iniBefore -and (Get-Digest (Join-Path $fixture.workspace 'boat-target.json')) -ceq $targetBefore) ($case+': Prepare preserves profile and independent audit bytes')
  if($case -ceq 'corrupt-capability') {
   Require ($preparedRun.exit -ne 0 -and $preparedRun.stderr.Contains('not qualified for commissioning restart protocol 1')) 'corrupt capability: actual Prepare refuses owned but incapable payload'
   Require (@(Get-ChildItem -LiteralPath (Join-Path $fixture.workspace 'runs') -Directory -Filter '*-restart-session-*').Count -eq 0) 'corrupt capability: refused before creating a cold session'
   Require (@(Get-Process -Name opencpn -ErrorAction SilentlyContinue).Count -eq 0) 'corrupt capability: no marker/application launched'
   $cases.Add(@{name=$case;actualPrepare=$true;status='refused-before-session'});continue
  }
  if($case -ceq 'shutdown-copy-hash') {
   Require ($preparedRun.exit -ne 0 -and $preparedRun.stderr.Contains('Retained shutdown DLL/source identity/review differs')) 'Actual Prepare refuses a review borrowing the other copy DLL hash'
   Require (@(Get-ChildItem -LiteralPath (Join-Path $fixture.workspace 'runs') -Recurse -Filter 'session.json').Count -eq 0) 'Mismatched DLL review creates no accepted cold session'
   Require (@(Get-Process -Name opencpn -ErrorAction SilentlyContinue).Count -eq 0) 'Mismatched DLL review launches no marker/application'
   $cases.Add(@{name=$case;actualPrepare=$true;status='refused-before-session'});continue
  }
  Require ($preparedRun.exit -eq 0) ($case+': actual Prepare succeeds: '+$preparedRun.stderr)
  $prepared=$preparedRun.stdout|ConvertFrom-Json;$session=Read-Record $prepared.record
  $shutdownReview=Read-Record $session.shutdownReview
  Require ($shutdownReview.schema -eq 2 -and $shutdownReview.plugins.Count -eq 3 -and @($shutdownReview.plugins.path|Sort-Object -Unique).Count -eq 3) ($case+': same-basename copies retain all three separate shutdown identities')
  $directory=[IO.Path]::GetDirectoryName($prepared.record);$transition=Join-Path $directory 'transition-0001'
  Require ($prepared.status -ceq 'prepared-only' -and -not $prepared.applicationLaunched -and -not $prepared.profileChanged -and (Get-Digest $prepared.record) -ceq $prepared.recordSha256) ($case+': actual private immutable session produced without application launch')
  Require ($session.sid -ceq [Security.Principal.WindowsIdentity]::GetCurrent().User.Value -and $session.windowsSessionId -ceq [Diagnostics.Process]::GetCurrentProcess().SessionId.ToString()) ($case+': actual OS account/session retained')
  Require ((Get-Digest $session.beforeIni) -ceq $iniBefore -and $session.scripts.Count -eq ($script:RestartDependencies.Count+1)) ($case+': cold bytes and all production/fixture dependencies pinned')
  $coldDependency=@($session.scripts|Where-Object {$_.name -ceq 'ColdBaseline.ps1'})
  Require ($coldDependency.Count -eq 1 -and $coldDependency[0].sha256 -ceq (Get-Digest (Join-Path $fixture.scripts 'ColdBaseline.ps1'))) ($case+': actual fresh Prepare pins the exact composed cold-baseline reader')
  if($case -ceq 'success') {
   # Exercise actual Arm's immutable-session reader before it reaches any
   # parent-process lookup or task creation. Only disposable fixture bytes change.
   $sessionBytes=[IO.File]::ReadAllBytes($prepared.record)
   $coldPath=Join-Path $fixture.scripts 'ColdBaseline.ps1';$coldBytes=[IO.File]::ReadAllBytes($coldPath)
   foreach($mutation in @('cold-byte','old-closure','other-directory')) {
    try {
     $changedSession=$session|ConvertTo-Json -Depth 24|ConvertFrom-Json
     if($mutation -ceq 'cold-byte') {[IO.File]::AppendAllText($coldPath,"`n# altered disposable dependency`n")}
     elseif($mutation -ceq 'old-closure') {$changedSession.scripts=@($changedSession.scripts|Where-Object {$_.name -cne 'ColdBaseline.ps1'})}
     else {$changedSession.toolDirectory=$fixture.root}
     if($mutation -cne 'cold-byte'){[IO.File]::WriteAllText($prepared.record,($changedSession|ConvertTo-Json -Depth 24))}
     $badArgs='-SessionRecord "'+$prepared.record+'" -ExpectedSha256 '+(Get-Digest $prepared.record)+' -ParentProcessId 42 -ParentCreatedFiletime 133000000000000000 -Mode --legacy'
     $rejected=Invoke-FixtureScript $fixture 'RestartCommissioningArm.ps1' $badArgs ('composed-'+$mutation)
     $reason=if($mutation -ceq 'cold-byte'){'Commissioning verifier changed after cold review.'}else{'Cold restart tool inventory differs.'}
     Require ($rejected.exit -ne 0 -and $rejected.stderr.Contains($reason)) ('Composed session refuses '+$mutation+' through actual Arm identity guard')
     Require (@(Get-ChildItem -LiteralPath $directory -Filter 'arm-*.json').Count -eq 0) ('Composed '+$mutation+' refusal creates no Arm intent')
    } finally {
     [IO.File]::WriteAllBytes($prepared.record,$sessionBytes);[IO.File]::WriteAllBytes($coldPath,$coldBytes)
    }
   }
   Require ((Get-Digest $prepared.record) -ceq $prepared.recordSha256 -and (Get-Digest $coldPath) -ceq $coldDependency[0].sha256) 'Composed negative fixtures restore exact bytes before actual successful Arm/Collect'
  }
  Assert-RestartPrivateDirectory $directory $session.sid;Require $true ($case+': actual private evidence ACL enforced')
  [IO.File]::WriteAllText((Join-Path $fixture.app 'target-mode.txt'),$mode);[IO.File]::WriteAllText((Join-Path $fixture.app 'hold-child.txt'),'marker-only child identity hold');[IO.File]::WriteAllText((Join-Path $fixture.app 'hold-parent-for-scheduler.txt'),'bounded native scheduler fixture')
  $start=New-Object Diagnostics.ProcessStartInfo;$start.FileName=$fixture.executable;$start.Arguments='--parent';$start.WorkingDirectory=$fixture.app;$start.UseShellExecute=$false
  $start.EnvironmentVariables['PATH']=$session.path;$start.EnvironmentVariables['LOCALAPPDATA']=(Join-Path $fixture.root 'local');$start.EnvironmentVariables['APPDATA']=(Join-Path $fixture.root 'roaming')
  $start.EnvironmentVariables['OPENNAV_COMMISSIONING_RESTART_SESSION']=$session.session;$start.EnvironmentVariables['OPENNAV_COMMISSIONING_RESTART_RECORD_SHA256']=$prepared.recordSha256
  $parent=$null;$companion=$null;$arm=$null;$armFile=$null;$caseCompleted=$false
  try {
   $parent=[Diagnostics.Process]::Start($start);$null=$parent.Handle;$created=$parent.StartTime.ToUniversalTime().ToFileTimeUtc().ToString()
   Wait-FixtureFile (Join-Path $fixture.app 'parent-armed.txt')
   $helpers=@(Get-CimInstance Win32_Process -Filter ('ParentProcessId='+$parent.Id)|Where-Object {$_.ExecutablePath -and $_.ExecutablePath -ieq $fixture.helper})
   Require ($helpers.Count -eq 1) ($case+': marker creates exactly one actual companion')
   $companion=Get-Process -Id $helpers[0].ProcessId;$null=$companion.Handle
   # This journal is the sole launch-fixture boundary: the marker parent uses
   # --parent, not a real product cold-launch UI. Prepare and Arm remain actual.
   $cold=Join-Path $directory 'cold-launch-consumed.json'
   Write-Record $cold @{owner=$script:RestartOwner;session=$session.session;recordSha256=$prepared.recordSha256;status='consumed-before-start';mode='--xnav'}
   Write-Record (Join-Path $directory 'cold-child.json') @{owner=$script:RestartOwner;session=$session.session;recordSha256=$prepared.recordSha256;pid=$parent.Id.ToString();createdFiletime=$created;launchSha256=(Get-Digest $cold)}
   $base='-SessionRecord "'+$prepared.record+'" -ExpectedSha256 '+$prepared.recordSha256+' -ParentProcessId '+$parent.Id+' -ParentCreatedFiletime '
   if($case -ceq 'success') {
    $wrong=Invoke-FixtureScript $fixture 'RestartCommissioningArm.ps1' ($base+([uint64]$created+1).ToString()+' -Mode --legacy') 'wrong-parent'
    Require ($wrong.exit -ne 0 -and $wrong.stderr.Contains('Parent creation identity differs')) 'actual Arm refuses replaced parent identity before registering a task'
    Require (@(Get-ChildItem -LiteralPath $directory -Filter 'arm-*.json').Count -eq 0) 'wrong parent creates no arm journal'
   }
   $armArguments=$base+$created+' -Mode '+$mode
   if($palette){$armArguments+=' -ChartPalette Standard'}
   $armed=Invoke-FixtureScript $fixture 'RestartCommissioningArm.ps1' $armArguments ($case+'-arm')
   Require ($armed.exit -eq 0) ($case+': actual Arm starts fixed native limited task: '+$armed.stderr)
   $ready=$armed.stdout|ConvertFrom-Json;$armFile=Join-Path $directory ('arm-'+$parent.Id+'-'+$created+'.json');$arm=Read-Record $armFile
   Require ($ready.status -ceq 'listening-for-one-explicit-restart' -and $ready.taskName -ceq $arm.taskName -and -not $parent.HasExited) ($case+': scheduled broker ready before parent close')
   $task=Get-ScheduledTask -TaskName $arm.taskName
   Require ((Resolve-RestartTaskSid $task.Principal.UserId) -ceq $session.sid -and $task.Actions[0].Execute -ceq (Join-Path ([Environment]::GetFolderPath('Windows')) 'System32\WindowsPowerShell\v1.0\powershell.exe')) ($case+': real scheduled task resolves to actual SID and fixed native PowerShell')
   if($case -ceq 'success') {
    $armHash=Get-Digest $armFile
    $duplicate=Invoke-FixtureScript $fixture 'RestartCommissioningArm.ps1' $armArguments 'duplicate-arm'
    Require ($duplicate.exit -ne 0 -and (Get-Digest $armFile) -ceq $armHash) 'actual Arm cannot overwrite/re-arm an existing transition'
    $early=Invoke-FixtureScript $fixture 'RestartCommissioningArm.ps1' ($armArguments+' -Action Collect') 'early-collect'
    Require ($early.exit -ne 0 -and (Get-ScheduledTask -TaskName $arm.taskName).State.ToString() -ceq 'Running') 'actual Collect refuses to remove a live verifier task'
   }
   if($palette) {
    $readyPath=Join-Path $transition 'ready.json';$listening=Read-Record $readyPath
    Require ($arm.chartPalette -ceq 'Standard' -and $listening.chartPalette -ceq 'Standard') 'actual scheduled Arm and listening broker bind the selected palette'
    Write-Record (Join-Path $transition 'ui-intent-consumed.json') @{owner='OpenNavX.GuardedModeIntent.1';session=$session.session;recordSha256=$prepared.recordSha256;mode=$mode;chartPalette='Standard';fromMode='--xnav';
      parent=@{pid=$parent.Id.ToString();createdFiletime=$created};command=@{Palette='Standard'};status='consumed-before-ui-action';armSha256=(Get-Digest $armFile);readySha256=(Get-Digest $readyPath);testOnly='Synthetic consumed intent; separate native HWND fixture tests UI'}
   }
   $changed=[IO.File]::ReadAllText($fixture.profile)
   if($palette){$changed=$changed.Replace('ChartPresentationV1=XNav','ChartPresentationV1=Standard')}else{$changed=$changed.Replace('InterfaceMode=xnav','InterfaceMode=legacy')}
   if($case -ceq 'output-connection'){$changed=$changed.Replace('COM8;115200;0;0;','COM8;115200;0;1;')}
   [IO.File]::WriteAllText($fixture.profile,$changed,(New-Object Text.UTF8Encoding($false)))
   [IO.File]::WriteAllText((Join-Path $fixture.app 'parent-release.txt'),'one deliberate marker close')
   Require ($parent.WaitForExit(15000) -and $parent.ExitCode -eq 0) ($case+': genuine parent exits normally')
   $outcome=Join-Path $transition $(if($expectedSuccess){'completion.json'}else{'failure.json'})
   Wait-FixtureFile $outcome 120
   Require ($companion.WaitForExit(15000)) ($case+': actual helper terminates after authorization decision')
   $until=[DateTime]::UtcNow.AddSeconds(15)
   do{$task=Get-ScheduledTask -TaskName $arm.taskName;if($task.State.ToString() -ceq 'Ready'){break};Start-Sleep -Milliseconds 100}while([DateTime]::UtcNow -lt $until)
   Require ($task.State.ToString() -ceq 'Ready') ($case+': broker scheduled task has exited before collection')
   $taskExit=(Get-ScheduledTaskInfo -TaskName $arm.taskName).LastTaskResult
   if($expectedSuccess){Require ($taskExit -eq 0) ($case+': actual scheduled broker exited normally after completion')}
   $children=@(Get-ChildItem -LiteralPath $fixture.app -Filter 'child--*.txt')
   if($expectedSuccess) {
    $completion=Read-Record $outcome
    Require ($completion.status -ceq 'child-identity-verified' -and $children.Count -eq 1 -and $companion.ExitCode -eq 0) 'actual Prepare and scheduled Arm chain produces one verified marker child'
   } else {
    Require ($children.Count -eq 0 -and $companion.ExitCode -ne 0 -and -not [IO.File]::Exists((Join-Path $transition 'permit-consumed.json'))) 'actual scheduled broker refuses changed output connection without child or permit'
   }
   if($palette) {
    $wrongCollect=Invoke-FixtureScript $fixture 'RestartCommissioningArm.ps1' ($armArguments.Replace('-ChartPalette Standard','-ChartPalette XNav')+' -Action Collect') 'opposite-palette-collect'
    Require ($wrongCollect.exit -ne 0 -and $wrongCollect.stderr.Contains('Unknown broker task ownership')) 'actual Collect refuses a different palette intent'
   }
   $collected=Invoke-FixtureScript $fixture 'RestartCommissioningArm.ps1' ($armArguments+' -Action Collect') ($case+'-collect')
   Require ($collected.exit -eq 0 -and ($collected.stdout|ConvertFrom-Json).status -ceq 'broker-task-collected') ($case+': actual Collect succeeds after reviewed task outcome: '+$collected.stderr)
   Require ($null -eq (Get-ScheduledTask -TaskName $arm.taskName -ErrorAction SilentlyContinue) -and [IO.File]::Exists(($armFile+'.collected.json'))) ($case+': only owned task removed and durable collection retained')
   $cases.Add(@{name=$case;scheduledTaskExit=$taskExit;chartPalette=$(if($palette){'Standard'}else{''});syntheticUiIntent=$palette;actualPrepare=$true;actualArm=$true;actualCollect=$true;markerChildren=$children.Count;entrypointHashes=$proof;status='passed'})
   $caseCompleted=$true
  } finally {
   try {
    [IO.File]::WriteAllText((Join-Path $fixture.app 'parent-release.txt'),'fixture cleanup release')
    foreach($process in @($parent,$companion)){if($process){try{if(-not $process.WaitForExit(15000)){throw 'Owned fixture process still running; temporary tree retained.'}}finally{$process.Dispose()}}}
    $closed=Wait-BrokerMarkerChildren $fixture.app $fixture.executable (Get-Digest $marker) $session.session $prepared.recordSha256
    if($caseCompleted -and $children.Count -gt 0){Require ($closed -eq $children.Count) ($case+': exact held marker child exits normally before fixture removal')}
   } catch {$cleanupErrors.Add($case+': '+$_.Exception.Message);if($caseCompleted){throw}}
   # Never stop/kill a live scheduled broker. Retain failed TEMP evidence and
   # owned task for inspection; its own fixed five-minute limit still applies.
   if($arm) {
    try {
     $task=Get-ScheduledTask -TaskName $arm.taskName -ErrorAction SilentlyContinue
     if($task -and $task.State.ToString() -ceq 'Ready') {Assert-RestartTaskIdentity $task $arm $session.sid;Unregister-ScheduledTask -TaskName $arm.taskName -Confirm:$false}
    } catch {
     $cleanupErrors.Add($case+': '+$_.Exception.Message)
     # Preserve an earlier test failure. A cleanup failure after an otherwise
     # successful case is itself a failure and never authorizes task removal.
     if($caseCompleted){throw}
    }
   }
  }
 }
 $status='passed'
} catch {$failure=$_.Exception.Message;[IO.File]::WriteAllText((Join-Path $Evidence 'failure.txt'),($_|Out-String)+"`r`n"+$_.ScriptStackTrace);throw}
finally {
 $cleanupFailure=$null
 if($status -ceq 'passed'){try{Remove-Item -LiteralPath $root -Recurse -Force}catch{$status='failed';$failure=$_.Exception.Message;$cleanupErrors.Add('temporary tree: '+$failure);$cleanupFailure=$_}}
 @{status=$status;checks=$checks.Count;checkDetails=$checks.ToArray();cases=$cases.ToArray();failure=$failure;cleanupErrors=$cleanupErrors.ToArray();actualPrepare=$true;actualArmCollect=$true;identitySubstitutions='Copied TEMP known-folder/installation identity; fixed test-only adapter seal for native scheduler';coldLaunch='Synthetic marker parent journal, not product UI cold launch';realApplication=$false;boatAccess=$false;physicalOutput=$false;productAcceptance=$false}|ConvertTo-Json -Depth 10|Set-Content -LiteralPath (Join-Path $Evidence 'prepare-arm-result.json') -Encoding UTF8
 if($cleanupFailure){throw $cleanupFailure}
}

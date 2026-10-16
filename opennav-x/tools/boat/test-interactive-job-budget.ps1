# Execute the actual dispatcher parameter/settings/wait/finally statements with
# an inert scheduler. No task, application, window, profile or boat access.
[CmdletBinding()]
param()
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'
$checks=New-Object 'Collections.Generic.List[string]'
function Read-Ast([string]$Name) {
  $tokens=$null;$errors=$null
  $ast=[Management.Automation.Language.Parser]::ParseFile((Join-Path $PSScriptRoot $Name),[ref]$tokens,[ref]$errors)
  if($errors.Count){throw ('Fixture source does not parse: '+$Name)}
  return $ast
}
$ast=Read-Ast 'Common.ps1'
$dispatch=@($ast.FindAll({param($n) $n -is [Management.Automation.Language.FunctionDefinitionAst] -and $n.Name -ceq 'Invoke-InteractiveJob'},$true))
if($dispatch.Count -ne 1){throw 'Expected exact dispatcher.'}
$statements=$dispatch[0].Body.EndBlock.Statements
$budget=@($statements|Where-Object {$_ -is [Management.Automation.Language.AssignmentStatementAst] -and $_.Left.Extent.Text -ceq '$longProof'})
$settings=@($statements|Where-Object {$_ -is [Management.Automation.Language.AssignmentStatementAst] -and $_.Left.Extent.Text -ceq '$settings'})
$transaction=@($statements|Where-Object {$_ -is [Management.Automation.Language.TryStatementAst]})
if($budget.Count -ne 1 -or $settings.Count -ne 1 -or $transaction.Count -ne 1){throw 'Dispatcher boundary changed; review fixture extraction.'}
$parameters=($dispatch[0].Parameters|ForEach-Object {$_.Extent.Text}) -join ','
$probe=[scriptblock]::Create(('param('+ $parameters + ")`n" + @'
$script:observedWait=$TimeoutSeconds
$manual=$false;$name='inert-owned-task';$request='inert-request.json'
$result=$script:resultPath;$action=$null;$principal=$null
$task=$null;$retainPendingTask=$false
'@ + "`n"+$budget[0].Extent.Text+"`n"+$settings[0].Extent.Text+"`n"+$transaction[0].Extent.Text))
function New-ScheduledTaskSettingsSet([timespan]$ExecutionTimeLimit,[switch]$AllowStartIfOnBatteries,[switch]$DontStopIfGoingOnBatteries) {
  $script:observedTaskMinutes=$ExecutionTimeLimit.TotalMinutes
  if(-not $AllowStartIfOnBatteries -or -not $DontStopIfGoingOnBatteries){throw 'Existing battery policy changed.'}
  return [pscustomobject]@{limit=$ExecutionTimeLimit}
}
function Register-ScheduledTask($TaskName,$Action,$Principal,$Settings) {$script:registered++;return [pscustomobject]@{name=$TaskName}}
function Start-ScheduledTask($TaskName) {
  $script:started++
  if($script:mode -ceq 'passed'){[IO.File]::WriteAllText($script:resultPath,'{"status":"passed"}')}
  elseif($script:mode -ceq 'failed'){[IO.File]::WriteAllText($script:resultPath,'{"status":"failed","error":"inert guard refusal"}')}
}
function Unregister-ScheduledTask($TaskName,[switch]$Confirm) {$script:unregistered++}
function Read-Record([string]$Path) {return ([IO.File]::ReadAllText($Path)|ConvertFrom-Json)}
function Start-Sleep {throw 'Unexpected wait: explicit zero timeout was overridden.'}
$root=Join-Path ([IO.Path]::GetTempPath()) ('interactive-budget-'+[guid]::NewGuid().ToString('N'))
$null=New-Item -ItemType Directory -Path $root
function Invoke-Case([string]$Action,[string]$Mode,$Timeout=$null) {
  $script:mode=$Mode;$script:registered=0;$script:started=0;$script:unregistered=0
  $script:resultPath=Join-Path $root ([guid]::NewGuid().ToString('N')+'.json')
  $args=@{Workspace='inert';Job=[pscustomobject]@{action=$Action}}
  if($null -ne $Timeout){$args.TimeoutSeconds=[int]$Timeout}
  $script:failure=$null;$script:value=$null
  try{$script:value=& $probe @args}catch{$script:failure=$_.Exception.Message}
  if($script:registered -ne 1 -or $script:started -ne 1){throw 'Dispatcher retried or omitted its one task.'}
}
try {
  $long=@('Launch','LaunchStartup','ReviewInstalledWelcome')
  $ordinary=@('LaunchPortableReview','Close','Capture','ReviewWindow','LaunchStock','ReviewStock','RequestGuardedMode','ReviewRestartChild','LaunchManualPilot','CloseManualPilot','ManualPilotUi','launch','Unknown')
  foreach($action in $long+$ordinary) {
    Invoke-Case $action 'passed'
    $expectedWait=if($action -cin $long){720}else{90};$expectedTask=if($action -cin $long){15}else{5}
    if($failure -or $value.status -cne 'passed' -or $observedWait -ne $expectedWait -or $observedTaskMinutes -ne $expectedTask -or $unregistered -ne 1){throw ('Wrong completed-job budget/custody: '+$action+' '+$failure)}
    $checks.Add('Actual default wait/task mapping and completed cleanup: '+$action)
  }
  foreach($action in $long+@('Capture','ManualPilotUi')) {
    Invoke-Case $action 'passed' 7
    if($failure -or $observedWait -ne 7 -or $unregistered -ne 1){throw 'Explicit caller timeout was ignored.'}
    $checks.Add('Explicit caller timeout retained: '+$action)
    Invoke-Case $action 'pending' 0
    if($observedWait -ne 0 -or -not $failure){throw 'Missing result was accepted or explicit deadline changed.'}
    if($action -cin $long) {
      if($unregistered -ne 0 -or $failure -notlike '*result is pending*' -or $failure -notlike '*task=inert-owned-task*' -or $failure -notlike '*request=inert-request.json*' -or -not $failure.Contains('result='+$resultPath)){throw 'Pending long-proof task custody was lost.'}
    }elseif($unregistered -ne 1 -or $failure -cne 'Interactive job timed out. Inspect the desktop; no application was force-killed.'){throw 'Unrelated timeout behavior changed.'}
    $checks.Add('Timeout refuses without retry and retains only selected task custody: '+$action)
  }
  foreach($action in $long) {
    Invoke-Case $action 'failed'
    if($failure -notlike '*inert guard refusal*' -or -not $failure.Contains($resultPath) -or $unregistered -ne 1){throw 'Guard refusal or completed result evidence was lost.'}
    $checks.Add('Actual completed guard refusal remains failure: '+$action)
  }
  function Get-Digest([string]$Path){return 'a'*64}
  function Invoke-InteractiveJob($Workspace,$Job,[int]$TimeoutSeconds) {return [pscustomobject]@{action=$Job.action;timeout=$TimeoutSeconds}}
  foreach($name in @('run-mode.ps1','smoke-test.ps1')) {
    $caller=Read-Ast $name
    $command=@($caller.FindAll({param($n) $n -is [Management.Automation.Language.CommandAst] -and $n.GetCommandName() -ceq 'Invoke-InteractiveJob' -and @($n.CommandElements|Where-Object {$_ -is [Management.Automation.Language.CommandParameterAst] -and $_.ParameterName -ceq 'TimeoutSeconds'}).Count -eq 1},$true))
    if($command.Count -ne 1){throw 'Expected one actual explicit launch callsite.'}
    foreach($UseStartupLauncher in @($false,$true)) {
      $Workspace='inert';$installed=[pscustomobject]@{executable='inert.exe'}
      $job=[pscustomobject]@{action=$(if($UseStartupLauncher){'LaunchStartup'}else{'Launch'})}
      $result=& ([scriptblock]::Create($command[0].Extent.Text))
      if($result.timeout -ne 720 -or $result.action -cne $job.action){throw 'Launch wrapper still selects its former short budget.'}
      $checks.Add('Actual wrapper call uses 720 seconds: '+$name+' startup='+$UseStartupLauncher)
    }
  }
  [pscustomobject]@{schema=1;status='passed';count=$checks.Count;checks=$checks.ToArray();actualScheduler=$false;applicationLaunched=$false;boatAccess=$false;nativeTimingAcceptance=$false}|ConvertTo-Json -Depth 4
}finally{Remove-Item -LiteralPath $root -Recurse -Force}

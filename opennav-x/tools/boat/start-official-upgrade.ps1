# Dispatch only the reviewed official-stock wizard as this already logged-in
# administrator. Never changes UAC, services, credentials, or application state.
[CmdletBinding()]
param(
  [ValidateSet('Start','Collect')][string]$Action='Start',
  [string]$Workspace='C:\XNav',
  [string]$Record,
  [string]$ExpectedRecordSha256,
  [string]$DispatchRecord,
  [string]$ExpectedDispatchRecordSha256
)
. (Join-Path $PSScriptRoot 'Common.ps1')
if ([Environment]::OSVersion.Platform -ne 'Win32NT' -or -not [Environment]::Is64BitProcess) {throw 'Native 64-bit Windows PowerShell required.'}
$identity=[Security.Principal.WindowsIdentity]::GetCurrent()
if (-not (New-Object Security.Principal.WindowsPrincipal($identity)).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {throw 'An already elevated administrator session is required; no UAC automation.'}
$sid=$identity.User.Value
function Assert-DispatchTask($Task,$Request) {
  $principalSid=[string]$Task.Principal.UserId
  if ($principalSid -cne $Request.userSid) {
    # Task Scheduler resolves an input SID to the local account name on this
    # Windows build. Compare the resolved identity, never its display spelling.
    $account=New-Object Security.Principal.NTAccount($principalSid)
    $principalSid=$account.Translate([Security.Principal.SecurityIdentifier]).Value
  }
  if ($Task.TaskPath -cne '\' -or $Task.TaskName -cne $Request.taskName -or $principalSid -cne $Request.userSid -or
      $Task.Principal.LogonType -ne 'Interactive' -or $Task.Principal.RunLevel -ne 'Highest' -or @($Task.Actions).Count -ne 1 -or
      @($Task.Actions)[0].Execute -ine $Request.execute -or @($Task.Actions)[0].Arguments -cne $Request.arguments -or
      $Task.Settings.ExecutionTimeLimit -cne 'PT0S' -or $Task.Settings.StopIfGoingOnBatteries -ne $false -or
      $Task.Settings.DisallowStartIfOnBatteries -ne $false -or $Task.Settings.AllowHardTerminate -ne $false) {throw 'Scheduled task identity or no-termination policy changed; no task removed.'}
}
if ($Action -eq 'Collect') {
  $DispatchRecord=Assert-LocalPath $DispatchRecord
  if ($ExpectedDispatchRecordSha256 -cnotmatch '^[a-f0-9]{64}$' -or (Get-Digest $DispatchRecord) -cne $ExpectedDispatchRecordSha256) {throw 'Dispatch record hash mismatch.'}
  $request=Read-Record $DispatchRecord
  if ($request.owner -cne 'OpenNavX.OfficialUpgradeDispatch.1' -or $request.userSid -cne $sid -or $request.taskName -cnotmatch '^OpenNavX-OfficialUpgrade-[a-f0-9]{32}$') {throw 'Unknown dispatch ownership.'}
  $task=Get-ScheduledTask -TaskName $request.taskName -TaskPath '\'
  Assert-DispatchTask $task $request
  $info=Get-ScheduledTaskInfo -TaskName $request.taskName -TaskPath '\'
  if ($task.State -in @('Running','Queued')) {
    @{status='pending';taskState=$task.State.ToString();lastTaskResult=$info.LastTaskResult;output=$request.output} | ConvertTo-Json
    return
  }
  if (-not [IO.File]::Exists($request.output)) {
    $attempted=$info.LastRunTime.ToUniversalTime() -ge [DateTime]::Parse($request.createdUtc).ToUniversalTime().AddSeconds(-1)
    $status=if ($attempted) {'failed-without-result'} else {'pending'}
    @{status=$status;taskState=$task.State.ToString();lastTaskResult=$info.LastTaskResult;output=$request.output;taskRetained=$true} | ConvertTo-Json
    return
  }
  $result=Read-Record $request.output
  if ($result.owner -cne 'OpenNavX.StockUpgradeWizard.1' -or $result.recordSha256 -cne $request.preparedSha256) {throw 'Wizard result identity mismatch.'}
  # Removing an already completed one-shot task never terminates the installer.
  # A failed wizard may intentionally leave setup open for careful inspection.
  Unregister-ScheduledTask -TaskName $request.taskName -TaskPath '\' -Confirm:$false
  @{status=$result.status;output=$request.output;sha256=(Get-Digest $request.output);lastTaskResult=$info.LastTaskResult;taskRemoved=$true;postflightRequired=$true} | ConvertTo-Json
  return
}
if (@(Get-Process -Name opencpn -ErrorAction SilentlyContinue).Count) {throw 'Close OpenCPN/XNav normally before stock upgrade.'}
$Record=Assert-LocalPath $Record
if ($ExpectedRecordSha256 -cnotmatch '^[a-f0-9]{64}$' -or (Get-Digest $Record) -cne $ExpectedRecordSha256) {throw 'Prepared record hash mismatch.'}
$prepared=Read-Record $Record
if ($prepared.owner -cne 'OpenNavX.StockUpgrade.1' -or $prepared.status -cne 'prepared') {throw 'A fresh reviewed stock-upgrade preflight is required.'}
$explorer=@(Get-CimInstance Win32_Process -Filter "Name='explorer.exe'" | Where-Object {(Invoke-CimMethod -InputObject $_ -MethodName GetOwnerSid).Sid -eq $sid})
if ($explorer.Count -ne 1) {throw 'Exactly one interactive desktop for this account is required.'}
$Workspace=Assert-LocalPath $Workspace
foreach ($protected in @((Join-Path ${env:ProgramFiles(x86)} 'OpenCPN'),(Join-Path ([Environment]::GetFolderPath('CommonApplicationData')) 'opencpn'))) {
  $protected=Assert-LocalPath $protected
  if ($Workspace -ieq $protected -or $Workspace.StartsWith($protected+'\',[StringComparison]::OrdinalIgnoreCase) -or $protected.StartsWith($Workspace+'\',[StringComparison]::OrdinalIgnoreCase)) {throw 'Dispatch workspace must be separate from application and profile.'}
}
$directory=New-RunDirectory $Workspace 'official-stock-upgrade'
$output=Join-Path $directory 'wizard.json'
$script=Assert-LocalPath (Join-Path $PSScriptRoot 'upgrade-official-opencpn.ps1')
$hashes=@{}
foreach ($name in @('upgrade-official-opencpn.ps1','OfficialUpgradePolicy.ps1','OfficialWizardNative.cs','Common.ps1')) {$hashes[$name]=Get-Digest (Join-Path $PSScriptRoot $name)}
$name='OpenNavX-OfficialUpgrade-'+[guid]::NewGuid().ToString('N')
$execute=Assert-LocalPath (Join-Path $env:WINDIR 'System32\WindowsPowerShell\v1.0\powershell.exe')
$arguments='-NoProfile -NonInteractive -ExecutionPolicy Bypass -File "'+$script+'" -Record "'+$Record+'" -ExpectedRecordSha256 '+$ExpectedRecordSha256+' -Output "'+$output+'"'
$request=@{owner='OpenNavX.OfficialUpgradeDispatch.1';createdUtc=[DateTime]::UtcNow.ToString('o');taskName=$name;userSid=$sid;interactiveSession=$explorer[0].SessionId;execute=$execute;arguments=$arguments;preparedRecord=$Record;preparedSha256=$ExpectedRecordSha256;output=$output;driverHashes=$hashes}
$dispatch=Join-Path $directory 'dispatch.json'
Write-Record $dispatch $request
$scheduledAction=New-ScheduledTaskAction -Execute $execute -Argument $arguments
$principal=New-ScheduledTaskPrincipal -UserId $sid -LogonType Interactive -RunLevel Highest
# The reviewed driver has its own bounded observation deadline. Scheduler must
# never force-terminate setup or its previous-version uninstaller on timeout.
$settings=New-ScheduledTaskSettingsSet -ExecutionTimeLimit ([TimeSpan]::Zero) -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries -DisallowHardTerminate
$task=Register-ScheduledTask -TaskName $name -Action $scheduledAction -Principal $principal -Settings $settings
Assert-DispatchTask $task $request
foreach ($entry in $hashes.GetEnumerator()) {if ((Get-Digest (Join-Path $PSScriptRoot $entry.Key)) -cne $entry.Value) {throw 'Driver changed before dispatch; task not started.'}}
Start-ScheduledTask -TaskName $name
@{status='started';dispatch=$dispatch;dispatchSha256=(Get-Digest $dispatch);output=$output;taskName=$name;postflightRequired=$true} | ConvertTo-Json

# Interactive-session dispatcher for one explicitly requested restart verifier.
# Does not click a control, launch OpenCPN, change a profile or operate equipment.
[CmdletBinding()]
param(
 [Parameter(Mandatory=$true)][string]$SessionRecord,
 [Parameter(Mandatory=$true)][string]$ExpectedSha256,
 [Parameter(Mandatory=$true)][uint32]$ParentProcessId,
 [Parameter(Mandatory=$true)][string]$ParentCreatedFiletime,
 [Parameter(Mandatory=$true)][ValidateSet('--xnav','--legacy','--safe-mode')][string]$Mode,
 [ValidateSet('Arm','Collect')][string]$Action='Arm',
 [string]$ChartPalette=''
)
. (Join-Path $PSScriptRoot 'RestartCommissioning.ps1')
Assert-RestartChartPalette $Mode $ChartPalette
Initialize-RestartNative
$session=Read-RestartSession $SessionRecord $ExpectedSha256
Assert-RestartDecimal $ParentCreatedFiletime 'parent creation'
if(-not $ParentProcessId){throw 'Exact parent PID required.'}
$record=Assert-LocalPath $SessionRecord;$directory=[IO.Path]::GetDirectoryName($record)
$armFile=Join-Path $directory ('arm-'+$ParentProcessId+'-'+$ParentCreatedFiletime+'.json')
if($Action -ceq 'Collect') {
 $arm=Read-Record $armFile
 if($arm.owner -cne $script:RestartOwner -or $arm.session -cne $session.session -or $arm.recordSha256 -cne $ExpectedSha256 -or
    $arm.parentPid -cne $ParentProcessId.ToString() -or $arm.parentCreatedFiletime -cne $ParentCreatedFiletime -or $arm.mode -cne $Mode -or
    (Get-RestartChartPalette $arm) -cne $ChartPalette -or $arm.taskName -cnotmatch '^OpenNavX-RestartReview-[a-f0-9]{32}$'){throw 'Unknown broker task ownership.'}
 $task=Get-ScheduledTask -TaskName $arm.taskName -ErrorAction Stop
 Assert-RestartTaskIdentity $task $arm $session.sid
 $transition=Assert-LocalPath $arm.transition
 if(-not (Test-Path -LiteralPath (Join-Path $transition 'completion.json')) -and -not (Test-Path -LiteralPath (Join-Path $transition 'failure.json'))){throw 'No completed broker outcome; inspect without retrying.'}
 Unregister-ScheduledTask -TaskName $arm.taskName -Confirm:$false
 Write-Record ($armFile+'.collected.json') @{owner=$script:RestartOwner;armSha256=(Get-Digest $armFile);taskName=$arm.taskName;at=[DateTime]::UtcNow.ToString('o')}
 [pscustomobject]@{status='broker-task-collected';outcomeDirectory=$transition;applicationChanged=$false}|ConvertTo-Json -Compress
 return
}
$parent=Get-RestartProcess $ParentProcessId $session $session.executable
if($parent.createdFiletime -cne $ParentCreatedFiletime){throw 'Parent creation identity differs.'}
$baseline=Get-RestartBaseline $session $record $parent
if($ChartPalette -and $baseline.mode -cne '--xnav'){throw 'Chart palette must be selected in the current SKAGER interface.'}
$name='OpenNavX-RestartReview-'+[guid]::NewGuid().ToString('N')
$execute=Join-Path ([Environment]::GetFolderPath('Windows')) 'System32\WindowsPowerShell\v1.0\powershell.exe'
$script=Assert-LocalPath (Join-Path $PSScriptRoot 'RestartCommissioningBroker.ps1')
# Record/path validation forbids quotes and control characters; all other
# arguments are bounded hashes, canonical decimals or a fixed mode flag.
$arguments='-NoProfile -NonInteractive -ExecutionPolicy Bypass -File "'+$script+'" -SessionRecord "'+$record+'" -ExpectedSha256 '+$ExpectedSha256+' -ParentProcessId '+$ParentProcessId+' -ParentCreatedFiletime '+$ParentCreatedFiletime+' -Mode '+$Mode
if($ChartPalette){$arguments+=' -ChartPalette '+$ChartPalette}
Write-Record $armFile @{owner=$script:RestartOwner;session=$session.session;recordSha256=$ExpectedSha256;parentPid=$parent.pid;parentCreatedFiletime=$parent.createdFiletime;mode=$Mode;chartPalette=$ChartPalette;taskName=$name;execute=$execute;arguments=$arguments;transition=$baseline.nextDirectory;createdUtc=[DateTime]::UtcNow.ToString('o')}
$taskAction=New-ScheduledTaskAction -Execute $execute -Argument $arguments
$principal=New-ScheduledTaskPrincipal -UserId $session.sid -LogonType Interactive -RunLevel Limited
# Only the verifier is subject to this task timeout. It never starts OpenCPN,
# so terminating an expired broker cannot kill a navigation application.
$settings=New-ScheduledTaskSettingsSet -ExecutionTimeLimit (New-TimeSpan -Minutes 5) -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries
$null=Register-ScheduledTask -TaskName $name -Action $taskAction -Principal $principal -Settings $settings
Start-ScheduledTask -TaskName $name
$ready=Join-Path $baseline.nextDirectory 'ready.json';$until=[DateTime]::UtcNow.AddSeconds(30)
while(-not (Test-Path -LiteralPath $ready) -and [DateTime]::UtcNow -lt $until){Start-Sleep -Milliseconds 100}
if(-not (Test-Path -LiteralPath $ready)){throw 'Broker did not publish readiness; inspect the owned task. Do not click/retry a mode switch.'}
$observed=Read-Record $ready
if($observed.owner -cne $script:RestartOwner -or $observed.session -cne $session.session -or $observed.recordSha256 -cne $ExpectedSha256 -or
   $observed.parent.pid -cne $parent.pid -or $observed.parent.createdFiletime -cne $parent.createdFiletime -or $observed.mode -cne $Mode -or (Get-RestartChartPalette $observed) -cne $ChartPalette){throw 'Broker readiness does not match this one explicit request.'}
$broker=Get-RestartProcess ([uint32]$observed.broker.pid) $session $execute
if($broker.createdFiletime -cne $observed.broker.createdFiletime){throw 'Broker readiness process was replaced/exited.'}
[pscustomobject]@{status='listening-for-one-explicit-restart';parentPid=$parent.pid;mode=$Mode;chartPalette=$ChartPalette;taskName=$name;outcomeDirectory=$baseline.nextDirectory;applicationChanged=$false}|ConvertTo-Json -Compress

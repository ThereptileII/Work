# SCRUM-311 inert component fixture dispatch, never an application launcher.
# SSH runs in session 0; clientIdentity deliberately refuses that session.
[CmdletBinding()]
param(
    [Parameter(Mandatory=$true)][string]$FixtureDirectory,
    [Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{64}$')][string]$ExecutableSha256,
    [Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{64}$')][string]$RunnerSha256,
    [switch]$Interactive
)
$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest
if([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT){throw 'Native Windows required.'}
# Fixed scratch namespace and filenames only; no arbitrary command/script input.
if($FixtureDirectory -cnotmatch '^C:\\XNav\\ComponentChecks\\peer-capability-[a-f0-9]{8}(-interactive)?$'){
    throw 'Exact disposable peer-fixture directory required.'
}
$current=$FixtureDirectory
while($current){
    $item=Get-Item -LiteralPath $current -Force
    if(-not $item.PSIsContainer -or ($item.Attributes -band [IO.FileAttributes]::ReparsePoint)){
        throw 'Existing plain scratch ancestry required.'
    }
    $parent=[IO.Path]::GetDirectoryName($current.TrimEnd('\'))
    if(-not $parent -or $parent -eq $current){break}
    $current=$parent
}
$binary=Join-Path $FixtureDirectory 'brokerpipe.test.exe'
$runner=Join-Path $FixtureDirectory 'runner.ps1'
function Assert-FixtureBytes {
    foreach($entry in @(@($binary,$ExecutableSha256),@($runner,$RunnerSha256))){
        $file=Get-Item -LiteralPath $entry[0] -Force
        if($file.PSIsContainer -or ($file.Attributes -band [IO.FileAttributes]::ReparsePoint) -or
           (Get-FileHash -LiteralPath $file.FullName -Algorithm SHA256).Hash.ToLowerInvariant() -cne $entry[1]){
            throw 'Original reviewed fixture/runner hash required.'
        }
    }
}
Assert-FixtureBytes
$proofPath=Join-Path $FixtureDirectory 'interactive-result.json'
$evidence=Join-Path $FixtureDirectory 'PeerCapabilityInteractive'
$utf8=New-Object Text.UTF8Encoding($false)
if($Interactive){
    $proof=[ordered]@{schema=1;status='failed';sessionId=[Diagnostics.Process]::GetCurrentProcess().SessionId;
        executableSha256=$ExecutableSha256;runnerSha256=$RunnerSha256;failure=$null}
    try{
        if($proof.sessionId -eq 0){throw 'Interactive native pipe fixture cannot run in session zero.'}
        if(Test-Path -LiteralPath $proofPath){throw 'Fresh interactive result required.'}
        Assert-FixtureBytes
        & $runner -TestExecutable $binary -ExpectedSha256 $ExecutableSha256 -EvidenceDirectory $evidence -Group PeerCapability
        $proof.status='passed'
    }catch{$proof.failure=$_.Exception.Message}
    [IO.File]::WriteAllText($proofPath,($proof|ConvertTo-Json -Depth 4),$utf8)
    if($proof.status -cne 'passed'){exit 1}
    exit 0
}
if((Test-Path -LiteralPath $proofPath) -or (Test-Path -LiteralPath $evidence)){
    throw 'Fresh interactive evidence required; no automatic retry.'
}
$sid=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value
$desktops=@(Get-CimInstance Win32_Process -Filter "Name='explorer.exe'" | Where-Object{
    (Invoke-CimMethod -InputObject $_ -MethodName GetOwnerSid).Sid -ceq $sid
})
if($desktops.Count -ne 1 -or $desktops[0].SessionId -eq 0){throw 'One existing same-account interactive desktop required.'}
$name='SKAGER-InertPeerFixture-'+[guid]::NewGuid().ToString('N')
$hostImage=Join-Path $env:WINDIR 'System32\WindowsPowerShell\v1.0\powershell.exe'
$arguments='-NoProfile -NonInteractive -WindowStyle Hidden -ExecutionPolicy Bypass -File "'+$PSCommandPath+
    '" -Interactive -FixtureDirectory "'+$FixtureDirectory+'" -ExecutableSha256 '+$ExecutableSha256+' -RunnerSha256 '+$RunnerSha256
$action=New-ScheduledTaskAction -Execute $hostImage -Argument $arguments
$principal=New-ScheduledTaskPrincipal -UserId $sid -LogonType Interactive -RunLevel Limited
$settings=New-ScheduledTaskSettingsSet -ExecutionTimeLimit (New-TimeSpan -Minutes 3) -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries
$created=$false
$taskProof=[ordered]@{schema=1;task=$name;status='not-completed';removed=$false;sessionId=$desktops[0].SessionId;
    executableSha256=$ExecutableSha256;runnerSha256=$RunnerSha256;lastTaskResult=$null;failure=$null}
try{
    $null=Register-ScheduledTask -TaskName $name -Action $action -Principal $principal -Settings $settings
    $created=$true
    Start-ScheduledTask -TaskName $name
    $deadline=[DateTime]::UtcNow.AddSeconds(125)
    do{
        Start-Sleep -Milliseconds 250
        $task=Get-ScheduledTask -TaskName $name
        $info=Get-ScheduledTaskInfo -TaskName $name
        $finished=(Test-Path -LiteralPath $proofPath) -and $task.State -ne 'Running'
    }while(-not $finished -and [DateTime]::UtcNow -lt $deadline)
    $taskProof.lastTaskResult=$info.LastTaskResult
    if(-not $finished){throw 'Original inert task remains uncompleted; inspect retained task/evidence, do not retry.'}
    $result=Get-Content -LiteralPath $proofPath -Raw|ConvertFrom-Json
    if($info.LastTaskResult -ne 0 -or $result.status -cne 'passed' -or $result.sessionId -ne $desktops[0].SessionId){
        throw 'Interactive fixture did not pass in the expected desktop session.'
    }
    Assert-FixtureBytes
    $taskProof.status='passed'
}catch{$taskProof.failure=$_.Exception.Message;throw}
finally{
    # Remove only the task this invocation created and only after it has stopped.
    # Never stop a navigation process, replace a task, or erase failed evidence.
    if($created -and (Get-ScheduledTask -TaskName $name).State -ne 'Running'){
        Unregister-ScheduledTask -TaskName $name -Confirm:$false
        $taskProof.removed=$true
    }
    [IO.File]::WriteAllText((Join-Path $FixtureDirectory 'interactive-task.json'),($taskProof|ConvertTo-Json -Depth 4),$utf8)
}

# Protected stdin -> same user's existing desktop -> Credential Manager.
# No key in arguments, environment, result files or logs. No OpenCPN launch.
[CmdletBinding()]
param([switch]$TestOnly,[switch]$Interactive,[string]$PipeName,
      [int]$ServerProcessId,[int]$ExpectedDesktopSession,
      [string]$ResultPath)
$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest
if([Environment]::OSVersion.Platform -ne 'Win32NT' -or $PSVersionTable.PSVersion.Major -ne 5){throw 'Windows PowerShell 5.1 required.'}
Add-Type -Path (Join-Path $PSScriptRoot 'AisCredentialImport.cs')
Add-Type -Path (Join-Path $PSScriptRoot 'AisCredentialPipe.cs')
if($Interactive) {
  if($PipeName -cnotmatch '^OpenNavX-AisCredential-[a-f0-9]{32}$' -or [Diagnostics.Process]::GetCurrentProcess().SessionId -ne $ExpectedDesktopSession){throw 'Invalid credential handoff.'}
  $pipe=$null;$result='handoff-failed';$id=$null;$phase='connect';$nativeChecks=0
  try {
    $pipe=New-Object IO.Pipes.NamedPipeClientStream('.',$PipeName,[IO.Pipes.PipeDirection]::In,[IO.Pipes.PipeOptions]::None,[Security.Principal.TokenImpersonationLevel]::Identification)
    $pipe.Connect(20000)
    $phase='verify-sender'
    if(-not [XNavAisCredentialPipe]::Server($pipe,$ServerProcessId)){$phase+='-'+[XNavAisCredentialPipe]::PeerFailure;throw 'Unexpected credential sender.'}
    $phase='protected-store'
    if($TestOnly) {
      $id=[guid]::NewGuid().ToString('N')
      $result=[XNavAisCredentialImport]::StoreForTest($pipe,$id)
      if($result -ceq 'stored-and-verified') {
        $validation=& (Join-Path $PSScriptRoot 'test-ais-credential-import.ps1') -IsolatedLocal | ConvertFrom-Json
        if(-not $validation.passed -or -not $validation.native){throw 'Native isolated store validation failed.'}
        $nativeChecks=$validation.checks
      }
    } else {$result=[XNavAisCredentialImport]::StoreProduction($pipe)}
  } catch {$result='handoff-failed-'+$phase}
  finally {
    if($pipe){$pipe.Dispose()}
    if($id){try{[XNavAisCredentialImport]::RemoveForTest($id)}catch{$result='test-cleanup-failed'}}
  }
  [IO.File]::WriteAllText($ResultPath+'.tmp',([ordered]@{status=$result;testOnly=[bool]$TestOnly;nativeChecks=$nativeChecks;interactiveSession=$ExpectedDesktopSession;store='Windows Credential Manager';keyPrinted=$false;applicationLaunched=$false}|ConvertTo-Json -Compress))
  [IO.File]::Move($ResultPath+'.tmp',$ResultPath)
  exit
}
$key=$null;$pipe=$null;$task=$null;$jobName=$null
try {
  $key=[XNavAisCredentialImport]::ReadPayload([Console]::OpenStandardInput())
  if($null -eq $key){throw 'Invalid credential input frame.'}
  $sid=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value
  $desktops=@(Get-CimInstance Win32_Process -Filter "Name='explorer.exe'" | Where-Object {(Invoke-CimMethod -InputObject $_ -MethodName GetOwnerSid).Sid -ceq $sid} | Select-Object -ExpandProperty SessionId -Unique)
  if($desktops.Count -ne 1){throw 'Exactly one existing desktop for this account is required.'}
  $nonce=[guid]::NewGuid().ToString('N');$PipeName='OpenNavX-AisCredential-'+$nonce
  $jobName='OpenNavX-AisCredential-'+$nonce
  $ResultPath=Join-Path $PSScriptRoot ('credential-result-'+$nonce+'.json')
  $pipe=[XNavAisCredentialPipe]::Create($PipeName)
  $wait=$pipe.BeginWaitForConnection($null,$null)
  $arguments='-NoProfile -NonInteractive -WindowStyle Hidden -ExecutionPolicy Bypass -File "'+$PSCommandPath+'" -Interactive -PipeName '+$PipeName+' -ServerProcessId '+$PID+' -ExpectedDesktopSession '+$desktops[0]+' -ResultPath "'+$ResultPath+'"'
  if($TestOnly){$arguments+=' -TestOnly'}
  $action=New-ScheduledTaskAction -Execute (Join-Path $env:WINDIR 'System32\WindowsPowerShell\v1.0\powershell.exe') -Argument $arguments
  $principal=New-ScheduledTaskPrincipal -UserId $sid -LogonType Interactive -RunLevel Limited
  $settings=New-ScheduledTaskSettingsSet -ExecutionTimeLimit (New-TimeSpan -Seconds 55) -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries
  $task=Register-ScheduledTask -TaskName $jobName -Action $action -Principal $principal -Settings $settings
  Start-ScheduledTask -TaskName $jobName
  if(-not $wait.AsyncWaitHandle.WaitOne(25000)){throw 'Desktop credential receiver timed out.'}
  $pipe.EndWaitForConnection($wait)
  if(-not [XNavAisCredentialPipe]::Client($pipe,$desktops[0])){throw 'Unexpected credential receiver.'}
  $length=[BitConverter]::GetBytes([uint32]$key.Length)
  $pipe.Write($length,0,4);$pipe.Write($key,0,$key.Length);$pipe.Flush();$pipe.WaitForPipeDrain();$pipe.Dispose();$pipe=$null
  [Array]::Clear($key,0,$key.Length);$key=$null
  $deadline=[DateTime]::UtcNow.AddSeconds(20)
  while(-not [IO.File]::Exists($ResultPath) -and [DateTime]::UtcNow -lt $deadline){Start-Sleep -Milliseconds 100}
  if(-not [IO.File]::Exists($ResultPath)){throw 'Desktop import result unavailable.'}
  $result=[IO.File]::ReadAllText($ResultPath)|ConvertFrom-Json
  if($result.status -cnotin @('stored-and-verified','already-stored-and-verified')){throw ('Desktop credential import: '+$result.status)}
  $result|ConvertTo-Json -Compress
} finally {
  if($pipe){$pipe.Dispose()}
  if($key){[Array]::Clear($key,0,$key.Length)}
  if($task){Unregister-ScheduledTask -TaskName $jobName -Confirm:$false}
}

# Native .NET Framework Get-Process lifecycle proof with inert owned windows.
[CmdletBinding()]
param([Parameter(Mandatory=$true)][string]$Evidence)
Set-StrictMode -Version Latest;$ErrorActionPreference='Stop'
if([Environment]::OSVersion.Platform -ne 'Win32NT' -or $env:GITHUB_ACTIONS -cne 'true'){throw 'Disposable native CI only.'}
if(@(Get-Process -Name opencpn -ErrorAction SilentlyContinue).Count){throw 'No navigation application may coexist with marker tests.'}
. (Join-Path $PSScriptRoot 'boat\StockReview.ps1')
$destination=Assert-LocalPath ([IO.Path]::GetFullPath($Evidence));$null=New-Item -ItemType Directory -Path $destination -Force
$fixture=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../tests/stock-close/window-fixture.ps1'))
$checks=New-Object 'Collections.Generic.List[string]';$cases=New-Object 'Collections.Generic.List[object]';$cleanup=New-Object 'Collections.Generic.List[string]';$failure=$null
function Check([bool]$Okay,[string]$Name){if(-not $Okay){throw $Name};$checks.Add($Name)}
try {
 foreach($entry in @('SharedPrimitive','InteractiveJob')) { foreach($exitCode in @(0,17)) {
  $directory=Join-Path ([IO.Path]::GetTempPath()) ('opennav-stock-close-'+[guid]::NewGuid().ToString('N'));$null=New-Item -ItemType Directory -Path $directory
  [IO.File]::WriteAllText((Join-Path $directory 'fixture.json'),(@{owner='OpenNavX.StockClose.Fixture.1';exitCode=$exitCode}|ConvertTo-Json -Compress))
  $start=New-Object Diagnostics.ProcessStartInfo;$start.FileName=Join-Path ([Environment]::GetFolderPath('System')) 'WindowsPowerShell\v1.0\powershell.exe'
  $start.Arguments='-NoProfile -STA -ExecutionPolicy Bypass -File "'+$fixture+'" -Directory "'+$directory+'"';$start.UseShellExecute=$false;$start.CreateNoWindow=$true
  $started=[Diagnostics.Process]::Start($start);$null=$started.Handle;$process=$null;$unheld=$null
  try {
   $ready=Join-Path $directory 'ready.json';$deadline=[datetime]::UtcNow.AddSeconds(10)
   while(-not (Test-Path -LiteralPath $ready) -and -not $started.HasExited -and [datetime]::UtcNow -lt $deadline){Start-Sleep -Milliseconds 50}
   if(-not (Test-Path -LiteralPath $ready)){throw 'Owned close marker failed to expose its normal window.'}
   $record=Get-Content -LiteralPath $ready -Raw|ConvertFrom-Json
   Check ($record.pid -eq $started.Id -and $record.startedTicks -eq $started.StartTime.ToUniversalTime().Ticks) 'Owned marker PID and creation identity proven'
   $process=Get-Process -Id $started.Id;$unheld=Get-Process -Id $started.Id
   Check ($process.MainWindowHandle.ToInt64() -eq $record.handle) 'Fresh Get-Process object targets the owned normal window'
   $bad=$null;try{$null=Invoke-ReviewedNormalClose $process $started.Id ($record.startedTicks+1)}catch{$bad=$_.Exception.Message}
   Check ($bad -and -not $process.HasExited -and $bad.Contains('"closeRequested":false')) 'Creation mismatch refuses before requesting close'
   $proof=$null;$errorText=$null
   if($entry -ceq 'SharedPrimitive') {
    try{$proof=Invoke-ReviewedNormalClose $process $started.Id $record.startedTicks}catch{$errorText=$_.Exception.GetBaseException().Message}
   } else {
    $requestPath=Join-Path $directory 'request.json';$resultPath=Join-Path $directory 'result.json'
    Write-Record $requestPath @{action='Close';executable=$started.Path;executableSha256=(Get-Digest $started.Path);processId=$started.Id;resultPath=$resultPath}
    & (Join-Path $PSScriptRoot 'boat\InteractiveJob.ps1') -Request $requestPath
    $dispatch=Read-Record $resultPath
    if($dispatch.status -ceq 'passed'){$proof=$dispatch.close}else{$errorText=$dispatch.error}
    Check ($dispatch.action -ceq 'Close' -and $dispatch.status -ceq $(if($exitCode -eq 0){'passed'}else{'failed'})) 'Actual installed Close dispatcher preserves success/nonzero classification'
   }
   if($exitCode -eq 0){Check ($null -eq $errorText -and $proof.handleRetained -and $proof.exitCodeKnown -and $proof.exitCode -eq 0) 'Fresh Get-Process normal close records measured zero exit'}
   else {
    Check ($errorText -and $errorText.Contains('closeEvidence=')) 'Measured nonzero exit refuses with explicit evidence'
    $proof=$errorText.Substring($errorText.IndexOf('closeEvidence=')+'closeEvidence='.Length)|ConvertFrom-Json
    Check ($proof.handleRetained -and $proof.exitCodeKnown -and $proof.exitCode -eq 17) 'Known nonzero is retained as numeric seventeen, never null or success'
   }
   Check ($proof.closeRequested -and $proof.waitCompleted -and $started.WaitForExit(1000) -and $started.ExitCode -eq $exitCode) 'One normal close and completion agree with independently retained owner handle'
   $oldGetterRefused=$false;try{$null=$unheld.get_ExitCode()}catch{$oldGetterRefused=$true}
   Check $oldGetterRefused 'Original unretained Get-Process getter fails after exit in native .NET Framework'
   $cases.Add(@{entry=$entry;requestedExit=$exitCode;proof=$proof;oldUnretainedGetterRefused=$oldGetterRefused})
  } finally {
   try {if(-not $started.HasExited){[IO.File]::WriteAllText((Join-Path $directory 'release'),'normal marker close');if(-not $started.WaitForExit(30000)){throw 'Marker did not close normally; no force termination.'}};if($process){$process.Dispose()};if($unheld){$unheld.Dispose()};$started.Dispose();Remove-Item -LiteralPath $directory -Recurse -Force}catch{$cleanup.Add($_.Exception.Message)}
  }
 }}
 if($cleanup.Count){throw 'Marker cleanup failed.'}
} catch {$failure=$_.Exception.Message;throw}
finally {
 Write-Record (Join-Path $destination 'stock-close-result.json') @{status=$(if($failure -or $cleanup.Count){'failed'}else{'passed'});error=$failure;count=$checks.Count;checks=$checks.ToArray();cases=$cases.ToArray();cleanupErrors=$cleanup.ToArray();sourceSha256=(Get-Digest (Join-Path $PSScriptRoot 'boat\Common.ps1'));dispatcherSha256=(Get-Digest (Join-Path $PSScriptRoot 'boat\InteractiveJob.ps1'));actualApplication=$false;boatAccess=$false;forcedExit=$false}
}

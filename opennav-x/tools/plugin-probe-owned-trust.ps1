param(
  [Parameter(Mandatory)][ValidateSet('Import','Remove')][string]$Operation,
  [Parameter(Mandatory)][string]$Certificate,
  [Parameter(Mandatory)][string]$Receipt
)
$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest
if(-not $IsWindows -or $env:GITHUB_ACTIONS -cne 'true' -or $env:RUNNER_OS -cne 'Windows' -or $env:RUNNER_ENVIRONMENT -cne 'github-hosted' -or $env:GITHUB_REPOSITORY -cne 'ThereptileII/Work') { throw 'Disposable GitHub-hosted Windows in ThereptileII/Work only' }
$Candidate=[Security.Cryptography.X509Certificates.X509Certificate2]::new($Certificate)
try {
  $Thumbprint=$Candidate.Thumbprint
  $Path="Cert:\LocalMachine\Root\$Thumbprint"
  if($Operation -ceq 'Import') {
    if((Test-Path -LiteralPath $Receipt) -or (Test-Path -LiteralPath $Path)) { throw 'CA identity or ownership receipt already exists' }
    # Persist the previously absent exact identity before any possible partial import.
    # CreateNew also rejects a competing receipt creation; never overwrite ownership.
    $OwnedFile=[IO.File]::Open($Receipt,[IO.FileMode]::CreateNew,[IO.FileAccess]::Write,[IO.FileShare]::None)
    try { $Bytes=[Text.Encoding]::UTF8.GetBytes($Thumbprint);$OwnedFile.Write($Bytes,0,$Bytes.Length) } finally { $OwnedFile.Dispose() }
    # CurrentUser imports block on a modal Security Warning. Only this disposable
    # runner's machine root is used; no policy or existing certificate is changed.
    $Start=[Diagnostics.ProcessStartInfo]::new((Join-Path $env:SystemRoot 'System32/certutil.exe'))
    $Start.UseShellExecute=$false;$Start.RedirectStandardOutput=$true;$Start.RedirectStandardError=$true
    foreach($Argument in @('-f','-addstore','Root',(Resolve-Path -LiteralPath $Certificate).Path)) { $Start.ArgumentList.Add($Argument) }
    $Process=$null;$Watch=[Diagnostics.Stopwatch]::StartNew()
    try {
      Write-Output "Import starting: $Thumbprint LocalMachine/Root"
      $Process=[Diagnostics.Process]::Start($Start)
      $Stdout=$Process.StandardOutput.ReadToEndAsync();$Stderr=$Process.StandardError.ReadToEndAsync()
      $TimedOut=-not $Process.WaitForExit(30000)
      if($TimedOut) {
        if(-not $Process.HasExited) { $Process.Kill($true) }
        if(-not $Process.WaitForExit(5000)) { throw 'Owned certutil process tree did not terminate' }
      }
      if(-not $Stdout.Wait(5000) -or -not $Stderr.Wait(5000)) { throw 'Owned certutil output pipes did not close' }
      Write-Output $Stdout.Result
      Write-Output $Stderr.Result
      [ordered]@{processId=$Process.Id;exitCode=$Process.ExitCode;timedOut=$TimedOut;timeoutSeconds=30;elapsedSeconds=$Watch.Elapsed.TotalSeconds;thumbprint=$Thumbprint;store='LocalMachine/Root'} | ConvertTo-Json -Compress | Write-Output
      if($TimedOut -or $Process.ExitCode -ne 0) { throw 'Owned CA import exceeded 30 seconds or certutil failed' }
    } finally {
      if($Process) {
        try {
          if(-not $Process.HasExited) { $Process.Kill($true) }
          if(-not $Process.WaitForExit(5000)) { throw 'Owned certutil process remains after cleanup' }
        } finally { $Process.Dispose() }
      }
    }
    if(-not(Test-Path -LiteralPath $Path)) { throw 'Owned CA import thumbprint is absent' }
    $Imported=Get-Item -LiteralPath $Path
    if($Imported.Thumbprint -cne $Thumbprint -or [Convert]::ToBase64String($Imported.RawData) -cne [Convert]::ToBase64String($Candidate.RawData)) { throw 'Owned CA import differs from exact generated certificate' }
  } else {
    if(-not(Test-Path -LiteralPath $Receipt) -or [IO.File]::ReadAllText($Receipt) -cne $Thumbprint) { throw 'Missing exact owned CA receipt' }
    if(Test-Path -LiteralPath $Path) {
      $Imported=Get-Item -LiteralPath $Path
      if([Convert]::ToBase64String($Imported.RawData) -cne [Convert]::ToBase64String($Candidate.RawData)) { throw 'Refusing removal of a different certificate' }
      Remove-Item -LiteralPath $Path -Force
    }
    if(Test-Path -LiteralPath $Path) { throw 'Owned CA remains after cleanup' }
  }
  Write-Output "$Operation verified: $Thumbprint"
} finally { $Candidate.Dispose() }

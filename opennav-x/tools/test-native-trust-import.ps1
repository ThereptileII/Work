# Bounded diagnostic only: no application build and no TLS acceptance claim.
param([ValidateRange(5,30)][int]$TimeoutSeconds = 30)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
if (-not $IsWindows -or $env:GITHUB_ACTIONS -cne 'true' -or $env:RUNNER_OS -cne 'Windows' -or $env:RUNNER_ENVIRONMENT -cne 'github-hosted' -or $env:GITHUB_REPOSITORY -cne 'ThereptileII/Work') {
  throw 'Restricted to a disposable GitHub-hosted Windows runner in ThereptileII/Work'
}
$Root = Split-Path $PSScriptRoot -Parent
$Evidence = Join-Path $Root 'evidence/local/native-trust-import'
if (Test-Path -LiteralPath $Evidence) { throw 'Diagnostic evidence destination must be new' }
New-Item -ItemType Directory -Path $Evidence | Out-Null
$Work = Join-Path $env:RUNNER_TEMP ('skager-trust-import-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $Work | Out-Null
$OwnedThumbprints = [Collections.Generic.List[string]]::new()
$Cases = [Collections.Generic.List[object]]::new()
$Cleanup = [Collections.Generic.List[object]]::new()
$Children = [Collections.Generic.List[Diagnostics.Process]]::new()
$Failure = $null
function Write-Json([string]$Path, $Value) { $Value | ConvertTo-Json -Depth 12 | Set-Content -LiteralPath $Path -Encoding utf8 }
function Hash([string]$Path) { (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash.ToLowerInvariant() }
function Event([string]$Stage) {
  $Record = @{utc=[DateTime]::UtcNow.ToString('o');stage=$Stage}
  $Record | ConvertTo-Json -Compress | Add-Content -LiteralPath (Join-Path $Evidence 'parent-events.jsonl') -Encoding utf8
  Write-Host "$($Record.utc) $Stage"
}
# Enumerate all top-level windows, including modal dialogs that MainWindowTitle can miss.
# Capture only windows owned by the timed-out process tree. Never interact with them.
Add-Type -TypeDefinition @'
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;
public static class TrustDiagnosticWindows {
  public sealed class Window { public uint process_id; public string title; public bool visible; }
  private delegate bool Callback(IntPtr hwnd, IntPtr param);
  [DllImport("user32.dll")] private static extern bool EnumWindows(Callback cb, IntPtr p);
  [DllImport("user32.dll")] private static extern uint GetWindowThreadProcessId(IntPtr h, out uint id);
  [DllImport("user32.dll", CharSet=CharSet.Unicode)] private static extern int GetWindowText(IntPtr h, StringBuilder s, int n);
  [DllImport("user32.dll")] private static extern bool IsWindowVisible(IntPtr h);
  public static Window[] Read() {
    var result = new List<Window>();
    EnumWindows((h,p) => { uint id; GetWindowThreadProcessId(h, out id); var text=new StringBuilder(2048);
      GetWindowText(h,text,text.Capacity); result.Add(new Window {process_id=id,title=text.ToString(),visible=IsWindowVisible(h)}); return true; },IntPtr.Zero);
    return result.ToArray();
  }
}
'@
function Bounded([string]$Name,[string]$Program,[string[]]$Arguments) {
  Event "$Name.start"
  $Info = [Diagnostics.ProcessStartInfo]::new($Program)
  $Info.UseShellExecute = $false
  $Info.RedirectStandardOutput = $true
  $Info.RedirectStandardError = $true
  foreach ($Argument in $Arguments) { $Info.ArgumentList.Add($Argument) }
  $Process = [Diagnostics.Process]::Start($Info)
  $Children.Add($Process)
  $Stdout = $Process.StandardOutput.ReadToEndAsync()
  $Stderr = $Process.StandardError.ReadToEndAsync()
  $Watch = [Diagnostics.Stopwatch]::StartNew()
  $TimedOut = -not $Process.WaitForExit($TimeoutSeconds * 1000)
  if ($TimedOut) {
    Event "$Name.timeout"
    # CIM inventory is observational and limited to the owned process tree.
    try {
      $Inventory = @(Get-CimInstance Win32_Process -OperationTimeoutSec 3 | Select-Object ProcessId,ParentProcessId,Name)
      $Ids = [Collections.Generic.HashSet[uint32]]::new()
      [void]$Ids.Add([uint32]$Process.Id)
      do {
        $Added = $false
        foreach ($Item in $Inventory) {
          if ($Ids.Contains([uint32]$Item.ParentProcessId) -and $Ids.Add([uint32]$Item.ProcessId)) { $Added = $true }
        }
      } while ($Added)
      Write-Json (Join-Path $Evidence "$Name-timeout-windows.json") @{
        processes=@($Inventory | Where-Object { $Ids.Contains([uint32]$_.ProcessId) })
        windows=@([TrustDiagnosticWindows]::Read() | Where-Object { $Ids.Contains($_.process_id) })
      }
    } catch { $_.ToString() | Set-Content -LiteralPath (Join-Path $Evidence "$Name-window-capture-error.txt") }
    if (-not $Process.HasExited) { $Process.Kill($true) }
    if (-not $Process.WaitForExit(5000)) { throw "$Name process tree did not terminate" }
  }
  $Watch.Stop()
  # Child-tree termination closes redirected handles; bounded task waits avoid pipe hangs.
  if (-not $Stdout.Wait(5000) -or -not $Stderr.Wait(5000)) { throw "$Name output pipes did not close" }
  $Stdout.Result | Set-Content -LiteralPath (Join-Path $Evidence "$Name.stdout.txt") -Encoding utf8
  $Stderr.Result | Set-Content -LiteralPath (Join-Path $Evidence "$Name.stderr.txt") -Encoding utf8
  $Result = @{name=$Name;timed_out=$TimedOut;exit_code=$Process.ExitCode;elapsed_seconds=$Watch.Elapsed.TotalSeconds;process_id=$Process.Id;timeout_seconds=$TimeoutSeconds}
  Write-Json (Join-Path $Evidence "$Name-process.json") $Result
  Event "$Name.finished"
  return $Result
}
function Remove-Owned([string]$Thumbprint) {
  $Path = "Cert:\CurrentUser\Root\$Thumbprint"
  $Present = Test-Path -LiteralPath $Path
  if ($Present) { Remove-Item -LiteralPath $Path -Force }
  $Absent = -not (Test-Path -LiteralPath $Path)
  $Cleanup.Add(@{thumbprint=$Thumbprint;present_before_cleanup=$Present;absent_after_cleanup=$Absent})
  if (-not $Absent) { throw "Owned certificate remains after cleanup: $Thumbprint" }
}
try {
  $Source = Join-Path $Root 'tools/test-downloader-trust-windows.ps1'
  $Tokens = $null; $ParseErrors = $null
  $Ast = [Management.Automation.Language.Parser]::ParseFile($Source,[ref]$Tokens,[ref]$ParseErrors)
  if (@($ParseErrors).Count) { throw 'Original trust test did not parse' }
  $Functions = @($Ast.FindAll({param($Node) $Node -is [Management.Automation.Language.FunctionDefinitionAst] -and $Node.Name -ceq 'Import-OwnedTrust'},$true))
  if ($Functions.Count -ne 1) { throw 'Expected exactly one original Import-OwnedTrust function' }
  $Function = $Functions[0].Extent.Text
  $ImportLine = '  $Imported = Import-Certificate -FilePath $Certificate -CertStoreLocation ''Cert:\CurrentUser\Root'''
  if (($Function.Split(@($ImportLine),[StringSplitOptions]::None)).Count -ne 2) { throw 'Exact import statement changed' }
  # Only surround the actual statement with durable events; its arguments and checks are unchanged.
  $Instrumented = $Function.Replace($ImportLine, "  Stage 'before-Import-Certificate'`n$ImportLine`n  Stage 'after-Import-Certificate'")
  $Child = Join-Path $Work 'import-child.ps1'
  $ChildText = @'
param([string]$Certificate,[string]$Mode,[string]$Events)
$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest
function Stage([string]$Name) {
  @{utc=[DateTime]::UtcNow.ToString('o');stage=$Name;process_id=$PID} | ConvertTo-Json -Compress | Add-Content -LiteralPath $Events -Encoding utf8
}
'@ + "`n" + $Instrumented + "`n" + @'
Stage 'child-start'
if ($Mode -ceq 'original') {
  Import-OwnedTrust $Certificate
} elseif ($Mode -ceq 'certutil-user-force') {
  Stage 'before-certutil-user-force'
  & "$env:SystemRoot/System32/certutil.exe" -user -f -addstore Root $Certificate
  if ($LASTEXITCODE -ne 0) { throw "certutil failed: $LASTEXITCODE" }
  Stage 'after-certutil-user-force'
} else { throw 'Unknown diagnostic mode' }
Stage 'child-success'
'@
  Set-Content -LiteralPath $Child -Value $ChildText -Encoding utf8
  Copy-Item -LiteralPath $Child -Destination (Join-Path $Evidence 'instrumented-import-child.ps1')
  $OpenSsl = (Get-Command openssl.exe -CommandType Application -ErrorAction Stop | Select-Object -First 1).Source
  Write-Json (Join-Path $Evidence 'identity.json') @{
    repository=$env:GITHUB_REPOSITORY;commit=$env:GITHUB_SHA;run_id=$env:GITHUB_RUN_ID;run_attempt=$env:GITHUB_RUN_ATTEMPT;job=$env:GITHUB_JOB
    powershell=$PSVersionTable.PSVersion.ToString();original_trust_script_sha256=(Hash $Source);diagnostic_sha256=(Hash $PSCommandPath)
    instrumented_child_sha256=(Hash $Child);openssl_path=$OpenSsl;openssl_sha256=(Hash $OpenSsl)
    scope='Import-only diagnosis; no application build, TLS runtime, package, or boat acceptance'
  }
  foreach ($Mode in @('original','certutil-user-force')) {
    $Cert = Join-Path $Work "$Mode.pem"; $Key = Join-Path $Work "$Mode.key"
    $Generate = Bounded "$Mode-generate" $OpenSsl @('req','-x509','-newkey','rsa:2048','-nodes','-days','2','-subj',"/CN=SKAGER-diagnostic-$([guid]::NewGuid().ToString('N'))",'-keyout',$Key,'-out',$Cert)
    if ($Generate.timed_out -or $Generate.exit_code -ne 0) { throw 'Fresh test certificate generation failed' }
    $Certificate = [Security.Cryptography.X509Certificates.X509Certificate2]::new($Cert)
    $Thumbprint = $Certificate.Thumbprint
    $Certificate.Dispose()
    if (Test-Path -LiteralPath "Cert:\CurrentUser\Root\$Thumbprint") { throw 'Fresh certificate already exists; refusing ownership' }
    $OwnedThumbprints.Add($Thumbprint)
    Copy-Item -LiteralPath $Cert -Destination (Join-Path $Evidence "$Mode-public.pem")
    Write-Json (Join-Path $Evidence "$Mode-owned.json") @{thumbprint=$Thumbprint;store='CurrentUser/Root';absent_before_import=$true;certificate_sha256=(Hash $Cert)}
    $Result = Bounded $Mode (Join-Path $PSHOME 'pwsh.exe') @('-NoLogo','-NoProfile','-File',$Child,'-Certificate',$Cert,'-Mode',$Mode,'-Events',(Join-Path $Evidence "$Mode-events.jsonl"))
    $Result.thumbprint = $Thumbprint
    $Result.exact_certificate_present = Test-Path -LiteralPath "Cert:\CurrentUser\Root\$Thumbprint"
    $Result.verified_import = -not $Result.timed_out -and $Result.exit_code -eq 0 -and $Result.exact_certificate_present
    $Cases.Add($Result)
    Write-Json (Join-Path $Evidence "$Mode-result.json") $Result
    Remove-Owned $Thumbprint
  }
} catch {
  $Failure = $_.ToString()
  $Failure | Set-Content -LiteralPath (Join-Path $Evidence 'error.txt') -Encoding utf8
} finally {
  foreach ($Process in $Children) {
    if (-not $Process.HasExited) {
      $Process.Kill($true)
      if (-not $Process.WaitForExit(5000)) { $Failure = 'Owned child could not be terminated' }
    }
  }
  foreach ($Thumbprint in $OwnedThumbprints) {
    try { Remove-Owned $Thumbprint } catch { $Failure = $_.ToString() }
  }
  Remove-Item -LiteralPath $Work -Recurse -Force
  Write-Json (Join-Path $Evidence 'summary.json') @{
    status=$(if($Failure){'error'}else{'diagnostic-complete'});cases=@($Cases.ToArray());cleanup=@($Cleanup.ToArray());error=$Failure
    acceptance='Diagnostic only; timeout or import success does not qualify TLS, application, package, or boat behavior'
  }
}
if ($Failure) { throw $Failure }

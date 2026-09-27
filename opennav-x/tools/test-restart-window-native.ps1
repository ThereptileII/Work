# Native fixed-control exercise only. No product/profile/marine process runs.
[CmdletBinding()]
param([Parameter(Mandatory=$true)][string]$Evidence,[switch]$IsolatedLocal)
Set-StrictMode -Version Latest;$ErrorActionPreference='Stop'
if([Environment]::OSVersion.Platform -ne 'Win32NT' -or ($env:GITHUB_ACTIONS -cne 'true' -and -not $IsolatedLocal)){throw 'Native CI or explicit disposable local desktop required.'}
if(@(Get-Process -Name opencpn -ErrorAction SilentlyContinue).Count){throw 'Native UI fixture refuses coexistence with any navigation application.'}
$root=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$fixture=Join-Path $root 'tests\commissioning-restart\window-review-fixture.ps1'
. (Join-Path $PSScriptRoot 'boat\RestartWindowReview.ps1')
Initialize-RestartWindowNative
$evidencePath=[IO.Path]::GetFullPath($Evidence);$null=New-Item -ItemType Directory -Path $evidencePath -Force
$results=New-Object 'Collections.Generic.List[object]';$errorText=$null
$oldDpi=[OpenNavX.RestartWindowNative]::SetThreadDpiAwarenessContext([IntPtr](-4))
if($oldDpi -eq [IntPtr]::Zero){throw 'Native physical-pixel DPI unavailable.'}
try {
 foreach($spec in @(
  @('--xnav','--legacy','normal','Open Legacy OpenCPN'),@('--xnav','--xnav','normal','Restart XNav'),@('--xnav','--safe-mode','normal','Safe Mode'),
  @('--legacy','--xnav','normal','Switch to XNav'),@('--safe-mode','--xnav','normal','Switch to XNav'),
  @('--xnav','--legacy','ambiguous',''),@('--xnav','--legacy','replace-on-down',''),@('--legacy','--xnav','hidden-menu',''),@('--xnav','--legacy','modal',''))) {
  $directory=Join-Path ([IO.Path]::GetTempPath()) ('opennav-mode-window-'+[guid]::NewGuid().ToString('N'));$null=New-Item -ItemType Directory -Path $directory
  [IO.File]::WriteAllText((Join-Path $directory 'fixture.json'),(@{owner='OpenNavX.NativeModeWindow.Fixture.1';mode=$spec[0];case=$spec[2]}|ConvertTo-Json -Compress))
  $start=New-Object Diagnostics.ProcessStartInfo
  $start.FileName=Join-Path ([Environment]::GetFolderPath('System')) 'WindowsPowerShell\v1.0\powershell.exe'
  $start.Arguments='-NoProfile -STA -ExecutionPolicy Bypass -File "'+$fixture+'" -Directory "'+$directory+'"'
  $start.UseShellExecute=$false;$start.CreateNoWindow=$true
  $process=[Diagnostics.Process]::Start($start);$null=$process.Handle
  try {
   $deadline=[datetime]::UtcNow.AddSeconds(10);$readyPath=Join-Path $directory 'ready.json'
   while(-not (Test-Path -LiteralPath $readyPath) -and -not $process.HasExited -and [datetime]::UtcNow -lt $deadline){Start-Sleep -Milliseconds 50}
   if(-not (Test-Path -LiteralPath $readyPath)){throw 'Native marker window did not become ready.'}
   $ready=Get-Content -LiteralPath $readyPath -Raw|ConvertFrom-Json
   if($ready.pid -ne $process.Id -or $ready.createdFiletime -cne $process.StartTime.ToUniversalTime().ToFileTimeUtc().ToString()){throw 'Fixture creation identity differs.'}
   $refused=$false;$reason=$null;$command=$null
   try {
    [OpenNavX.RestartWindowNative]::Foreground([IntPtr]$ready.handle,$process.Id,$spec[0])
    $command=[OpenNavX.RestartWindowNative]::InspectModeCommand([IntPtr]$ready.handle,$process.Id,$spec[0],$spec[1])
    [OpenNavX.RestartWindowNative]::RequestMode([IntPtr]$ready.handle,$process.Id,$command)
   } catch {$refused=$true;$reason=$_.Exception.Message}
   $clickPath=Join-Path $directory 'clicks.txt';[string[]]$clicks=@()
   if(Test-Path -LiteralPath $clickPath){$clicks=@(Get-Content -LiteralPath $clickPath)}
   if($spec[3]){if($refused -or @($clicks).Count -ne 1 -or $clicks[0] -cne $spec[3]){throw ('Actual native mode action failed: '+$reason)}}
   elseif(-not $refused -or @($clicks).Count){throw 'Unsafe/ambiguous native fixture received an action.'}
   $results.Add(@{from=$spec[0];to=$spec[1];case=$spec[2];refused=$refused;refusal=$reason;clicks=@($clicks);command=$command;pid=$process.Id;createdFiletime=$ready.createdFiletime})
  } finally {
   [IO.File]::WriteAllText((Join-Path $directory 'release'),'release fixed marker window')
   if(-not $process.WaitForExit(30000)){throw 'Native marker did not exit within its bound; no force termination.'}
   $process.Dispose();Remove-Item -LiteralPath $directory -Recurse -Force
  }
 }
} catch {$errorText=$_.Exception.Message;throw}
finally {
 $null=[OpenNavX.RestartWindowNative]::SetThreadDpiAwarenessContext($oldDpi)
 [IO.File]::WriteAllText((Join-Path $evidencePath 'native-window-results.json'),(@{status=$(if($errorText){'failed'}else{'passed'});error=$errorText;cases=$results.ToArray();
  nativeHelperSha256=(Get-Digest (Join-Path $PSScriptRoot 'boat\RestartWindowNative.cs'));fixtureSha256=(Get-Digest $fixture);productLaunched=$false;physicalOutput=$false}|ConvertTo-Json -Depth 10))
}

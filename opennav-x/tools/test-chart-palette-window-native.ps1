# Native harmless HWND boundary exercise; no installed product or vessel access.
[CmdletBinding()]
param([Parameter(Mandatory=$true)][string]$Evidence,[switch]$IsolatedLocal)
Set-StrictMode -Version Latest;$ErrorActionPreference='Stop'
if([Environment]::OSVersion.Platform -ne 'Win32NT' -or ($env:GITHUB_ACTIONS -cne 'true' -and -not $IsolatedLocal)){throw 'Native CI or explicit disposable desktop required.'}
if(@(Get-Process -Name opencpn -ErrorAction SilentlyContinue).Count){throw 'Fixture refuses coexistence with navigation application.'}
$root=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'));$fixture=Join-Path $root 'tests\commissioning-restart\palette-window-fixture.ps1'
. (Join-Path $PSScriptRoot 'boat\RestartWindowReview.ps1')
Initialize-RestartWindowNative;Initialize-WindowReviewNative
$evidencePath=[IO.Path]::GetFullPath($Evidence);$null=New-Item -ItemType Directory -Path $evidencePath -Force
$results=New-Object 'Collections.Generic.List[object]';$failure=$null
$oldDpi=[OpenNavX.RestartWindowNative]::SetThreadDpiAwarenessContext([IntPtr](-4));if($oldDpi -eq [IntPtr]::Zero){throw 'Physical-pixel DPI unavailable.'}
try {
 foreach($spec in @(@('XNav','normal'),@('Standard','normal'),@('Standard','duplicate-choice'),@('Standard','hidden-choice'),@('Standard','replace-choice'),@('Standard','wrong-sheet'),@('Standard','wrong-detail'),@('Standard','unowned-sheet'),@('Standard','duplicate-confirm'),@('Standard','replace-confirm'),@('Standard','obscured-sheet'),@('Standard','reveal'),@('Standard','hidden-target'),@('Standard','duplicate-target'),@('Standard','changed-body'),@('Standard','no-progress'))) {
  $directory=Join-Path ([IO.Path]::GetTempPath()) ('opennav-palette-window-'+[guid]::NewGuid().ToString('N'));$null=New-Item -ItemType Directory -Path $directory
  [IO.File]::WriteAllText((Join-Path $directory 'fixture.json'),(@{owner='OpenNavX.PaletteWindow.Fixture.1';palette=$spec[0];case=$spec[1]}|ConvertTo-Json -Compress))
  $start=New-Object Diagnostics.ProcessStartInfo;$start.FileName=Join-Path ([Environment]::GetFolderPath('System')) 'WindowsPowerShell\v1.0\powershell.exe';$start.Arguments='-NoProfile -STA -ExecutionPolicy Bypass -File "'+$fixture+'" -Directory "'+$directory+'"';$start.UseShellExecute=$false;$start.CreateNoWindow=$true
  $process=[Diagnostics.Process]::Start($start);$null=$process.Handle;$caseResult=$null
  try {
   $readyPath=Join-Path $directory 'ready.json';$deadline=[datetime]::UtcNow.AddSeconds(10)
   while(-not (Test-Path -LiteralPath $readyPath) -and -not $process.HasExited -and [datetime]::UtcNow -lt $deadline){Start-Sleep -Milliseconds 50}
   if(-not (Test-Path -LiteralPath $readyPath)){throw 'Palette fixture did not become ready.'}
   $ready=Get-Content -LiteralPath $readyPath -Raw|ConvertFrom-Json
   if($ready.pid -ne $process.Id -or $ready.createdFiletime -cne $process.StartTime.ToUniversalTime().ToFileTimeUtc().ToString()){throw 'Fixture identity changed.'}
   $frame=[IntPtr]$ready.handle;$refused=$false;$reason=$null;$command=$null;$sheet=$null
   $navigation=$spec[1] -cin @('reveal','hidden-target','duplicate-target','changed-body','no-progress')
   try {
    if($navigation) {
     [OpenNavX.ReviewWindowNative]::Foreground($frame,$process.Id)
     [OpenNavX.ReviewWindowNative]::Click($frame,$process.Id,'Layers')
     [OpenNavX.ReviewWindowNative]::Click($frame,$process.Id,'RevealChartPalettePreference')
     [OpenNavX.ReviewWindowNative]::Click($frame,$process.Id,'ChartPalettePreferences')
    } else {
     [OpenNavX.RestartWindowNative]::Foreground($frame,$process.Id,'--xnav')
     $command=[OpenNavX.RestartWindowNative]::InspectPaletteCommand($frame,$process.Id,$spec[0])
     $sheet=[OpenNavX.RestartWindowNative]::OpenPaletteConfirmation($frame,$process.Id,$command)
     [OpenNavX.RestartWindowNative]::ConfirmPalette($frame,$process.Id,$command,$sheet)
    }
   } catch {$refused=$true;$reason=$_.Exception.Message}
   # Final release is deliberately queued once because real wx modal handlers
   # remain on the event stack; observe this inert marker rather than retrying.
   Start-Sleep -Milliseconds 200
   [string[]]$clicks=@();$clickPath=Join-Path $directory 'clicks.txt';if(Test-Path -LiteralPath $clickPath){$clicks=@(Get-Content -LiteralPath $clickPath)}
   if($spec[1] -ceq 'normal'){$expected=@(('selected:'+$(if($spec[0] -ceq 'XNav'){'SKAGER'}else{'Standard'})),'confirmed');if($refused -or ($clicks -join '|') -cne ($expected -join '|')){throw ('Actual palette action failed: '+$reason)}}
   elseif($spec[1] -ceq 'reveal'){if($refused -or ($clicks -join '|') -cne 'layers|preferences'){throw ('Actual source-body reveal failed: '+$reason)}}
   elseif(-not $refused -or @($clicks|Where-Object {$_ -ceq 'confirmed' -or $_ -ceq 'preferences' -or $_ -like 'UNSAFE*'}).Count){throw ('Unsafe palette fixture was accepted: '+$spec[1])}
   if($spec[1] -cin @('duplicate-choice','hidden-choice','replace-choice') -and $clicks.Count){throw 'Refused selection emitted a click.'}
   if($spec[1] -cin @('wrong-sheet','wrong-detail','unowned-sheet','duplicate-confirm','replace-confirm','obscured-sheet') -and ($clicks -join '|') -cne 'selected:Standard'){throw 'Refused confirmation repeated selection or emitted a release.'}
   $caseResult=@{palette=$spec[0];case=$spec[1];refused=$refused;reason=$reason;clicks=$clicks;command=$command;sheet=$sheet};$results.Add($caseResult)
  } finally {
   [IO.File]::WriteAllText((Join-Path $directory 'release'),'release inert palette fixture');if(-not $process.WaitForExit(30000)){throw 'Palette fixture did not exit; no force termination.'};$exitCode=$process.ExitCode;if($null -ne $caseResult){$caseResult.fixtureExitCode=$exitCode};$process.Dispose()
   if($exitCode -ne 0){throw ('Palette fixture did not exit normally: '+$exitCode)}
   Remove-Item -LiteralPath $directory -Recurse -Force
  }
 }
} catch {$failure=$_.Exception.Message;throw}
finally {
 $null=[OpenNavX.RestartWindowNative]::SetThreadDpiAwarenessContext($oldDpi)
 [IO.File]::WriteAllText((Join-Path $evidencePath 'native-palette-results.json'),(@{status=$(if($failure){'failed'}else{'passed'});error=$failure;cases=$results.ToArray();nativeHelperSha256=(Get-Digest (Join-Path $PSScriptRoot 'boat\RestartWindowNative.cs'));navigationHelperSha256=(Get-Digest (Join-Path $PSScriptRoot 'boat\ReviewWindowNative.cs'));fixtureSha256=(Get-Digest $fixture);productLaunched=$false;boatTouched=$false;physicalOutput=$false}|ConvertTo-Json -Depth 12))
}

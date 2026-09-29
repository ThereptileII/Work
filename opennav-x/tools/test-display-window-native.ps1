# Disposable native display controls only. Never loads a navigation application.
[CmdletBinding()]
param([Parameter(Mandatory=$true)][string]$Evidence,[switch]$IsolatedLocal)
Set-StrictMode -Version Latest;$ErrorActionPreference='Stop'
if([Environment]::OSVersion.Platform -ne 'Win32NT' -or ($env:GITHUB_ACTIONS -cne 'true' -and -not $IsolatedLocal)){throw 'Native CI or explicit disposable local desktop required.'}
if(@(Get-Process -Name opencpn -ErrorAction SilentlyContinue).Count){throw 'Display fixtures refuse coexistence with any navigation application.'}
$root=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$fixture=Join-Path $root 'tests\display-review\window-fixture.ps1'
. (Join-Path $PSScriptRoot 'boat\ReviewWindow.ps1')
Initialize-WindowReviewNative
# Only target discovery is substituted for these disposable windows. The actual
# diagnostic file reader, freshness/identity checks and HWND pan path execute.
function Get-Target([string]$Workspace) {
 if($Workspace -cne $script:displayProfile){throw 'Unexpected fixture workspace.'}
 return [pscustomobject]@{profileDirectory=$script:displayProfile}
}
$evidencePath=[IO.Path]::GetFullPath($Evidence);$null=New-Item -ItemType Directory -Path $evidencePath -Force
$results=New-Object 'Collections.Generic.List[object]';$errorText=$null;$cleanupErrors=New-Object 'Collections.Generic.List[string]'
$oldDpi=[OpenNavX.ReviewWindowNative]::SetThreadDpiAwarenessContext([IntPtr](-4))
if($oldDpi -eq [IntPtr]::Zero){throw 'Physical-pixel DPI context unavailable.'}
try {
 foreach($spec in @(
  @('Display','normal','DISPLAY'),@('ToggleFullscreen','normal','Fullscreen / window'),@('ToggleFullscreen','return','Fullscreen / window'),
  @('ToggleOrientation','normal','North'),@('ToggleOrientation','course','Course'),@('CyclePalette','normal','Status Day'),
  @('Display','wrong-page',''),@('ToggleFullscreen','wrong-page',''),@('ToggleOrientation','wrong-page',''),
  @('ToggleOrientation','ambiguous',''),@('Display','replace-on-down',''),@('Display','rename-on-down',''),
  @('Display','move-on-down',''),@('Display','duplicate-on-down',''),@('ToggleFullscreen','modal',''),
  @('PanRight','normal','PAN_RIGHT_DOWN,PAN_RIGHT_UP'),@('PanRight','canvas-child','PAN_RIGHT_DOWN,PAN_RIGHT_UP'),
  @('PanRight','wrong-page',''),@('PanRight','wrong-geometry',''),@('PanRight','modal',''),
  @('PanRight','missing-diagnostics',''),@('PanRight','stale-diagnostics',''),@('PanRight','wrong-commit',''),
  @('Resize1280x800','normal','RESIZED'),@('Resize1280x800','maximized-offscreen','RESIZED'),
  @('Resize1280x800','partial-offscreen','RESIZED'),@('Resize1280x800','entirely-offscreen',''),
  @('Resize1280x800','minimized',''),@('Resize1280x800','demo',''),
  @('Resize1280x800','wrong-pid',''),@('Resize1280x800','modal',''),
  @('Capture','prototype-normal','CAPTURE'),@('Capture','prototype-preferences','CAPTURE'),@('Capture','prototype-passage','CAPTURE'),
  @('Capture','prototype-traffic','CAPTURE'),@('Capture','prototype-back','CAPTURE'),
  @('Capture','prototype-two-sheets',''),@('Capture','prototype-unknown',''),@('Capture','prototype-wrong-owner',''),
  @('Capture','prototype-duplicate',''),@('Capture','prototype-clipped',''),
  @('Capture','prototype-signature',''),@('Capture','prototype-moved',''),
  @('Capture','prototype-rail-duplicate',''),@('Capture','prototype-modal',''),
  @('Navigation','prototype-normal','Chart'),@('Route','prototype-normal','Passage'),
  @('AIS','prototype-normal','Traffic'),@('Instruments','prototype-normal','Instruments'),
  @('CyclePalette','prototype-normal','Status Day'))) {
  $directory=Join-Path ([IO.Path]::GetTempPath()) ('opennav-display-window-'+[guid]::NewGuid().ToString('N'));$null=New-Item -ItemType Directory -Path $directory
  $fixtureCase=if($spec[1] -cin @('missing-diagnostics','stale-diagnostics','wrong-commit')){'normal'}else{$spec[1]}
  [IO.File]::WriteAllText((Join-Path $directory 'fixture.json'),(@{owner='OpenNavX.NativeDisplayWindow.Fixture.1';action=$spec[0];case=$fixtureCase}|ConvertTo-Json -Compress))
  $start=New-Object Diagnostics.ProcessStartInfo
  $start.FileName=Join-Path ([Environment]::GetFolderPath('System')) 'WindowsPowerShell\v1.0\powershell.exe'
  $start.Arguments='-NoProfile -STA -ExecutionPolicy Bypass -File "'+$fixture+'" -Directory "'+$directory+'"'
  $start.UseShellExecute=$false;$start.CreateNoWindow=$true
  $process=[Diagnostics.Process]::Start($start);$null=$process.Handle
  try {
   $deadline=[datetime]::UtcNow.AddSeconds(10);$readyPath=Join-Path $directory 'ready.json'
   while(-not (Test-Path -LiteralPath $readyPath) -and -not $process.HasExited -and [datetime]::UtcNow -lt $deadline){Start-Sleep -Milliseconds 50}
   if(-not (Test-Path -LiteralPath $readyPath)){throw 'Native display marker did not become ready.'}
   $ready=Get-Content -LiteralPath $readyPath -Raw|ConvertFrom-Json
   if($ready.pid -ne $process.Id -or $ready.createdFiletime -cne $process.StartTime.ToUniversalTime().ToFileTimeUtc().ToString()){throw 'Fixture creation identity differs.'}
   $refused=$false;$reason=$null;$before=$null;$after=$null
   try {
    if($spec[0] -ceq 'Resize1280x800') {
     $reviewPid=if($spec[1] -ceq 'wrong-pid'){$PID}else{$process.Id}
     $resize=[OpenNavX.ReviewWindowNative]::Resize1280x800([IntPtr]$ready.handle,$reviewPid)
     $before=$resize.Before;$after=$resize.After
     [OpenNavX.ReviewWindowNative]::AssertCapture([IntPtr]$ready.handle,$process.Id,$after)
    } else {
    [OpenNavX.ReviewWindowNative]::Foreground([IntPtr]$ready.handle,$process.Id)
    $before=[OpenNavX.ReviewWindowNative]::AssertFrame([IntPtr]$ready.handle,$process.Id)
    if($spec[0] -ceq 'Capture') {
     if($spec[1] -ceq 'prototype-moved') {
      [IO.File]::WriteAllText((Join-Path $directory 'mutate'),'move fixed owned surface')
      $deadline=[datetime]::UtcNow.AddSeconds(3)
      while(-not (Test-Path -LiteralPath (Join-Path $directory 'mutated')) -and [datetime]::UtcNow -lt $deadline){Start-Sleep -Milliseconds 50}
      if(-not (Test-Path -LiteralPath (Join-Path $directory 'mutated'))){throw 'Capture mutation was not performed.'}
     }
     [OpenNavX.ReviewWindowNative]::AssertCapture([IntPtr]$ready.handle,$process.Id,$before)
    } elseif($spec[0] -ceq 'PanRight') {
     $script:displayProfile=$directory
     $commit='a'*40
     $data=@{build_commit=$commit;build_purpose='INSTALLED PRODUCT';data_mode='OPENCPN selected navigation';ui_page='Navigation';
       runtime=@{display=@{route_creation_active=$false;chart_region=@{x=[int]$ready.chart.left;y=[int]$ready.chart.top;
         width=[int]($ready.chart.right-$ready.chart.left);height=[int]($ready.chart.bottom-$ready.chart.top)}}}}
     # A fresh decoy at the former wrong path must never authorize input.
     Write-Record (Join-Path $directory 'opennav-diagnostics.json') $data
     $logs=Join-Path $directory 'opennav-logs';$null=New-Item -ItemType Directory -Path $logs
     $diagnostic=Join-Path $logs 'opennav-diagnostics.json'
     if($spec[1] -ceq 'wrong-geometry'){$data.runtime.display.chart_region.x+=1}
     if($spec[1] -ceq 'wrong-commit'){$data.build_commit='b'*40}
     if($spec[1] -cne 'missing-diagnostics') {
      Write-Record $diagnostic $data
      if($spec[1] -ceq 'stale-diagnostics'){[IO.File]::SetLastWriteTimeUtc($diagnostic,[datetime]::UtcNow.AddSeconds(-10))}
     }
     Invoke-WindowReviewPan ([IntPtr]$ready.handle) $process.Id $directory $commit
    } else {[OpenNavX.ReviewWindowNative]::Click([IntPtr]$ready.handle,$process.Id,$spec[0])}
    $after=[OpenNavX.ReviewWindowNative]::AssertFrame([IntPtr]$ready.handle,$process.Id)
    }
   } catch {$refused=$true;$reason=$_.Exception.Message}
   $clickPath=Join-Path $directory 'clicks.txt';[string[]]$clicks=@()
   if(Test-Path -LiteralPath $clickPath){$clicks=@(Get-Content -LiteralPath $clickPath)}
   if($spec[2] -ceq 'CAPTURE') {
    if($refused -or @($clicks).Count -or $before.Shell -cne 'prototype' -or $before.Surfaces.Count -lt 3){throw ('Native prototype capture failed: '+$spec[1]+': '+$reason)}
   } elseif($spec[2] -ceq 'RESIZED'){
    if($refused -or @($clicks).Count -or $after.Maximized -or $after.Bounds.Width -ne 1280 -or $after.Bounds.Height -ne 800){throw ('Native fixed resize failed: '+$spec[1]+': '+$reason)}
    if($spec[1] -ceq 'maximized-offscreen' -and (-not $resize.RestoreRequested -or -not $before.Maximized)){throw 'Oversized restore fixture did not actually maximize first.'}
   } elseif($spec[2]){
    if($refused -or (@($clicks) -join ',') -cne $spec[2]){throw ('Native display action failed: '+$spec[0]+'/'+$spec[1]+': '+$reason)}
    if($spec[0] -ceq 'Display' -and [OpenNavX.ReviewWindowNative]::VisiblePageLabels([IntPtr]$ready.handle) -cnotcontains 'OpenNav product page: Display'){throw 'Display action did not enter its page.'}
    if($spec[0] -ceq 'ToggleFullscreen' -and ($before.Maximized -eq $after.Maximized -or $after.Maximized -ne ($spec[1] -ceq 'normal'))){throw 'Fullscreen/window fixture did not change actual frame state.'}
   } elseif(-not $refused -or @($clicks).Count){throw 'Unsafe/ambiguous native display fixture received a callback.'}
   $results.Add(@{action=$spec[0];case=$spec[1];refused=$refused;refusal=$reason;clicks=@($clicks);before=$before;after=$after;pid=$process.Id;createdFiletime=$ready.createdFiletime})
  } finally {
   try {
    [IO.File]::WriteAllText((Join-Path $directory 'release'),'release fixed display marker')
    if(-not $process.WaitForExit(30000)){throw 'Native display marker did not exit within its bound; no force termination.'}
    $process.Dispose();Remove-Item -LiteralPath $directory -Recurse -Force
   } catch {$cleanupErrors.Add($_.Exception.Message)}
  }
 }
 if($cleanupErrors.Count){throw 'Native display cleanup failed; inspect retained evidence.'}
} catch {$errorText=$_.Exception.Message;throw}
finally {
 $null=[OpenNavX.ReviewWindowNative]::SetThreadDpiAwarenessContext($oldDpi)
 [IO.File]::WriteAllText((Join-Path $evidencePath 'native-display-results.json'),(@{status=$(if($errorText -or $cleanupErrors.Count){'failed'}else{'passed'});error=$errorText;cleanupErrors=$cleanupErrors.ToArray();cases=$results.ToArray();
  nativeHelperSha256=(Get-Digest (Join-Path $PSScriptRoot 'boat\ReviewWindowNative.cs'));fixtureSha256=(Get-Digest $fixture);productLaunched=$false;physicalOutput=$false}|ConvertTo-Json -Depth 10))
}

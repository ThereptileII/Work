# Actual pinned official OpenCPN first-start warning, isolated native CI only.
# No production launch authority is replaced: this exercises the same narrowly
# scoped capture/Agree primitives against an empty portable stock application.
[CmdletBinding()]
param([Parameter(Mandatory=$true)][string]$Evidence,[ValidateSet('en_US','sv')][string]$Locale='en_US')
Set-StrictMode -Version Latest;$ErrorActionPreference='Stop';$ProgressPreference='SilentlyContinue'
if([Environment]::OSVersion.Platform -ne 'Win32NT' -or $env:GITHUB_ACTIONS -cne 'true'){throw 'This actual-application fixture is native disposable CI only.'}
if(@(Get-Process -Name opencpn -ErrorAction SilentlyContinue).Count){throw 'Close every navigation application before the isolated fixture.'}
. (Join-Path $PSScriptRoot 'boat\Common.ps1')
. (Join-Path $PSScriptRoot 'boat\StockWelcome.ps1')
. (Join-Path $PSScriptRoot 'boat\StartupLog.ps1')
Initialize-StockWelcomeNative
Add-Type -Path (Join-Path $PSScriptRoot 'boat\StockReviewNative.cs')
Add-Type -TypeDefinition @'
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;
public static class StockWarningFixtureInventory {
 [StructLayout(LayoutKind.Sequential)] private struct Point {public int X,Y;}
 public sealed class Item {public long Handle,Owner,Parent;public uint ProcessId;public int Id;public string Title,Class;public bool Visible,Enabled;public List<Item> Children;}
 private delegate bool Callback(IntPtr h,IntPtr p);
 [DllImport("user32.dll")] private static extern bool EnumWindows(Callback cb,IntPtr p);
 [DllImport("user32.dll")] private static extern bool EnumChildWindows(IntPtr h,Callback cb,IntPtr p);
 [DllImport("user32.dll")] private static extern uint GetWindowThreadProcessId(IntPtr h,out uint p);
 [DllImport("user32.dll",CharSet=CharSet.Unicode)] private static extern int GetWindowTextW(IntPtr h,StringBuilder s,int n);
 [DllImport("user32.dll",CharSet=CharSet.Unicode)] private static extern int GetClassNameW(IntPtr h,StringBuilder s,int n);
 [DllImport("user32.dll")] private static extern IntPtr GetWindow(IntPtr h,uint what);
 [DllImport("user32.dll")] private static extern IntPtr GetParent(IntPtr h);
 [DllImport("user32.dll")] private static extern int GetDlgCtrlID(IntPtr h);
 [DllImport("user32.dll")] private static extern bool IsWindowVisible(IntPtr h);
 [DllImport("user32.dll")] private static extern bool IsWindowEnabled(IntPtr h);
 [DllImport("user32.dll")] private static extern bool GetCursorPos(out Point point);
 [DllImport("user32.dll")] private static extern IntPtr GetForegroundWindow();
 public static int[] Cursor(){Point point;if(!GetCursorPos(out point))throw new InvalidOperationException("Fixture cursor observation failed.");return new[]{point.X,point.Y};}
 public static bool ForegroundIs(IntPtr window){return GetForegroundWindow()==window;}
 private static Item Read(IntPtr h){uint p;GetWindowThreadProcessId(h,out p);var t=new StringBuilder(2048);var c=new StringBuilder(256);GetWindowTextW(h,t,t.Capacity);GetClassNameW(h,c,c.Capacity);return new Item{Handle=h.ToInt64(),Owner=GetWindow(h,4).ToInt64(),Parent=GetParent(h).ToInt64(),ProcessId=p,Id=GetDlgCtrlID(h),Title=t.ToString(),Class=c.ToString(),Visible=IsWindowVisible(h),Enabled=IsWindowEnabled(h),Children=new List<Item>()};}
 public static Item[] ReadOwned(int pid){if(pid<=0)throw new InvalidOperationException("Exact fixture PID required.");var found=new List<Item>();Exception error=null;int count=0;
  EnumWindows(delegate(IntPtr h,IntPtr p){try{uint owner;GetWindowThreadProcessId(h,out owner);if(owner!=(uint)pid)return true;var item=Read(h);found.Add(item);if(++count>2048)throw new InvalidOperationException("Bounded fixture inventory exceeded.");
   EnumChildWindows(h,delegate(IntPtr child,IntPtr ignored){try{var c=Read(child);if(c.ProcessId!=(uint)pid)throw new InvalidOperationException("Foreign fixture child.");item.Children.Add(c);if(++count>2048)throw new InvalidOperationException("Bounded child inventory exceeded.");return true;}catch(Exception e){error=e;return false;}},IntPtr.Zero);return error==null;
  }catch(Exception e){error=e;return false;}},IntPtr.Zero);if(error!=null)throw error;return found.ToArray();}
}
'@
$evidence=Assert-LocalPath ([IO.Path]::GetFullPath($Evidence));$null=New-Item -ItemType Directory -Path $evidence -Force
$runner=Assert-LocalPath $env:RUNNER_TEMP
$temporary=Join-Path $runner ('OpenNav stock warning '+[guid]::NewGuid().ToString('N'))
$null=New-Item -ItemType Directory -Path $temporary
Write-Record (Join-Path $temporary 'fixture.json') @{owner='OpenNavX.OfficialStockWarningFixture.1';temporary=$temporary}
$setup=Join-Path $temporary 'official-setup.exe';$app=Join-Path $temporary 'portable OpenCPN';$exe=Join-Path $app 'opencpn.exe'
$setupHash='e949f55de57611afe2fc0dad5a8ac33795c46ba488cb40ca07b65f639a07b8aa'
$stockHash='7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c'
$normalProfile=Join-Path ([Environment]::GetFolderPath('CommonApplicationData')) 'opencpn'
if(Test-Path -LiteralPath $normalProfile){throw 'Fresh runner must not have a normal OpenCPN profile; it is never moved or modified by this fixture.'}
$expectedTitle=if($Locale -ceq 'sv'){'V'+[char]0x00e4+'lkommen till OpenCPN'}else{'Welcome to OpenCPN'}
$checks=New-Object 'Collections.Generic.List[string]';$failure=$null;$process=$null;$notice=$null;$agreed=$false;$log=$null;$outputTask=$null;$errorTask=$null;$oldDpi=[IntPtr]::Zero
$report=@{status='running';locale=$Locale;checks=@();officialSetupSha256=$setupHash;officialExecutableSha256=$stockHash;physicalOutput=$false;boatAccess=$false;
 acceptanceScope='Actual official stock portable warning primitives only; production commissioning launch/audit and real boat acceptance are separate.'}
function Check([bool]$Okay,[string]$Name){if(-not $Okay){throw ('FAILED: '+$Name)};$checks.Add($Name);Write-Host ('PASS: '+$Name)}
function Refuse([scriptblock]$Body,[string]$Name){$bad=$false;try{$null=& $Body}catch{$bad=$true};Check $bad $Name}
function CopyNotice($Info){$copy=New-Object OpenNavX.StockWelcomeNative+NoticeInfo;foreach($field in $Info.GetType().GetFields()){$field.SetValue($copy,$field.GetValue($Info))};return $copy}
try {
 Invoke-WebRequest -UseBasicParsing -Uri 'https://github.com/OpenCPN/OpenCPN/releases/download/Release_5.12.4/opencpn_5.12.4-0%2B3720.37fd0cd_setup.exe' -OutFile $setup -TimeoutSec 120
 Check ((Get-Digest $setup) -ceq $setupHash) 'Exact official setup verified before extraction'
 $seven=Join-Path ${env:ProgramFiles} '7-Zip\7z.exe';if(-not [IO.File]::Exists($seven)){throw 'Reviewed runner 7-Zip is required.'}
 & $seven x -y ('-o'+$app) $setup | Out-File -LiteralPath (Join-Path $evidence 'extraction.log') -Encoding utf8
 if($LASTEXITCODE -ne 0){throw 'Official payload extraction failed.'}
 Check ((Get-Digest $exe) -ceq $stockHash) 'Extracted executable is byte-identical to the qualified official release'
 foreach($resource in @('uidata\styles.xml','s57data','basemap_shp','plugins','wxbase32u_vc14x.dll')){Check (Test-Path -LiteralPath (Join-Path $app $resource)) ('Official dependency/resource present: '+$resource)}
 $pluginNames=@(Get-ChildItem -LiteralPath (Join-Path $app 'plugins') -Filter '*_pi.dll' -File -Recurse|ForEach-Object {$_.Name}|Sort-Object)
 Check (($pluginNames -join '|') -ceq 'chartdldr_pi.dll|dashboard_pi.dll|grib_pi.dll|wmm_pi.dll') 'Only the four reviewed bundled plugins exist in portable loader tree'
 if($Locale -ceq 'sv'){
  foreach($catalogue in @(
   @('share\locale\sv\LC_MESSAGES\opencpn.mo','c57052ffd88fde07d379f2f27256eb77d77511fe34ff4d519cea898bc639dec4'),
   @('share\locale\sv\LC_MESSAGES\wxstd.mo','f3f490b0ac48373fb4ef3a54babd8957263024f9ddcd3df5756affa50851e09e'))){
   Check ((Get-Digest (Join-Path $app $catalogue[0])) -ceq $catalogue[1]) ('Exact official Swedish catalogue: '+$catalogue[0])
  }
 }
 $ini=Join-Path $app 'opencpn.ini';$log=Join-Path $app 'opencpn.log'
 if((Test-Path -LiteralPath $ini) -or (Test-Path -LiteralPath $log)){throw 'Official archive unexpectedly contains a profile/log.'}
 # Model the actual version-upgrade path. NavMessageShown=1 avoids first-install
 # portable chart hints; the old version STILL requires the genuine warning.
 $text="[Settings]`r`nConfigVersionString=Version 5.12.2-0+b69f44c`r`nNavMessageShown=1`r`nLocale=$Locale`r`nShowStatusBar=1`r`nShowMenuBar=1`r`nOpenGL=0`r`n[Settings/GlobalState]`r`nFrameWinX=900`r`nFrameWinY=640`r`nFrameWinPosX=20`r`nFrameWinPosY=20`r`nFrameMax=0`r`n[Settings/NMEADataSource]`r`nDataConnections=`r`n"
 foreach($name in $pluginNames){$text+="[PlugIns/$name]`r`nbEnabled=0`r`n"}
 [IO.File]::WriteAllText($ini,$text,(New-Object Text.UTF8Encoding($false)))
 Copy-Item -LiteralPath $ini -Destination (Join-Path $evidence 'portable-before.ini')
 $report.beforeIniSha256=Get-Digest $ini
 $private=Join-Path $temporary 'private environment';$null=New-Item -ItemType Directory -Path $private
 foreach($folder in @('Local','Roaming')){$null=New-Item -ItemType Directory -Path (Join-Path $private $folder)}
 $start=New-Object Diagnostics.ProcessStartInfo;$start.FileName=$exe;$start.Arguments='--portable --no_opengl';$start.WorkingDirectory=$app;$start.UseShellExecute=$false
 $start.RedirectStandardOutput=$true;$start.RedirectStandardError=$true
 $start.EnvironmentVariables['LOCALAPPDATA']=Join-Path $private 'Local';$start.EnvironmentVariables['APPDATA']=Join-Path $private 'Roaming'
 $start.EnvironmentVariables['PATH']=$app+';'+[Environment]::GetFolderPath('System')+';'+$env:WINDIR
 $process=[Diagnostics.Process]::Start($start);$null=$process.Handle
 $outputTask=$process.StandardOutput.ReadToEndAsync();$errorTask=$process.StandardError.ReadToEndAsync()
 Check ($process.Path -ieq $exe -and (Get-Digest $process.Path) -ceq $stockHash) 'Launched only the exact official executable in its new portable directory'
 $report.processId=$process.Id;$report.processCreatedFiletime=$process.StartTime.ToUniversalTime().ToFileTimeUtc().ToString();$report.arguments=$start.Arguments
 $oldDpi=[OpenNavX.StockReviewNative]::SetThreadDpiAwarenessContext([IntPtr](-4));if($oldDpi -eq [IntPtr]::Zero){throw 'Physical-pixel DPI context unavailable.'}
 $deadline=[datetime]::UtcNow.AddSeconds(60)
 do {
  Start-Sleep -Milliseconds 250;$process.Refresh();if($process.HasExited){throw 'Official portable app exited before its warning.'}
  $windows=@([StockWarningFixtureInventory]::ReadOwned($process.Id))
 } while(@($windows|Where-Object {$_.Visible -and $_.Title -ceq $expectedTitle}).Count -ne 1 -and [datetime]::UtcNow -lt $deadline)
 Write-Record (Join-Path $evidence 'actual-window-metadata.json') @{windows=$windows;processId=$process.Id;processCreatedFiletime=$report.processCreatedFiletime}
 Check (@($windows|Where-Object {$_.Visible -and $_.Title -ceq $expectedTitle}).Count -eq 1) 'Actual pinned wx first-start warning appeared'
 Start-Sleep -Milliseconds 500
 $notice=[OpenNavX.StockWelcomeNative]::Inspect($process.Id)
 # Exercise the separate fixed-caption action against the real wx modal. No
 # target coordinates are passed to production code, and no agreement occurs.
 $startedTicks=$process.StartTime.ToUniversalTime().Ticks
 Refuse {[OpenNavX.StockWelcomeNative]::FocusCaption(0,$startedTicks)} 'Caption focus refuses a different process before input'
 Refuse {[OpenNavX.StockWelcomeNative]::FocusCaption($process.Id,($startedTicks+1))} 'Caption focus refuses replaced process creation identity before input'
 Add-Type -AssemblyName System.Windows.Forms
 $cover=New-Object Windows.Forms.Form;$cover.FormBorderStyle=[Windows.Forms.FormBorderStyle]::None
 $cover.ShowInTaskbar=$false;$cover.TopMost=$true;$cover.StartPosition=[Windows.Forms.FormStartPosition]::Manual
 $cover.Text='Disposable unrelated fixture window';$cover.Bounds=New-Object Drawing.Rectangle($notice.Bounds.Left,$notice.Bounds.Top,$notice.Bounds.Width,80)
 $script:fixtureCoverClicks=0;$cover.add_MouseDown({$script:fixtureCoverClicks++})
 try {
  $cover.Show();$cover.Activate();[Windows.Forms.Application]::DoEvents()
  Check ([StockWarningFixtureInventory]::ForegroundIs($cover.Handle)) 'Disposable covering window owns foreground for refusal case'
  $cursorBefore=[StockWarningFixtureInventory]::Cursor();$refusal=$null
  try{$null=[OpenNavX.StockWelcomeNative]::FocusCaption($process.Id,$startedTicks)}catch{$refusal=$_.Exception.ToString()}
  [Windows.Forms.Application]::DoEvents()
  Check ($refusal -and ($refusal.Contains('covered') -or $refusal.Contains('obscured'))) 'Foreign window over the exact caption refuses before input'
  Check (([StockWarningFixtureInventory]::Cursor() -join ',') -ceq ($cursorBefore -join ',') -and $script:fixtureCoverClicks -eq 0) 'Covered-caption refusal leaves cursor and foreign window untouched'
  # Keep a genuine different foreground window, entirely clear of the warning.
  $screen=[Windows.Forms.Screen]::PrimaryScreen.WorkingArea
  $away=New-Object Drawing.Rectangle(($screen.Right-160),($screen.Bottom-100),160,100)
  $warning=New-Object Drawing.Rectangle($notice.Bounds.Left,$notice.Bounds.Top,$notice.Bounds.Width,$notice.Bounds.Height)
  if($away.IntersectsWith($warning)){throw 'Disposable desktop has no clear location for unrelated fixture window.'}
  $cover.Bounds=$away;$cover.Activate();[Windows.Forms.Application]::DoEvents()
  Check ([StockWarningFixtureInventory]::ForegroundIs($cover.Handle)) 'Unrelated fixture remains foreground without obscuring the caption'
  $focusIntent=Join-Path $evidence 'focus-intent.json'
  $focused=Invoke-StockWelcomeFocus $process.Id $startedTicks $focusIntent
  [Windows.Forms.Application]::DoEvents()
  Check ((Test-Path -LiteralPath $focusIntent) -and $focused.Modal -eq $notice.Modal -and $focused.Agree -eq $notice.Agree -and $focused.Cancel -eq $notice.Cancel) 'One durable caption-focus intent preserves actual warning and both buttons'
  $report.focusImageSha256=Save-StockWelcomeCapture $process.Id $focused (Join-Path $evidence 'actual-stock-focused-warning.png')
  Check ([StockWarningFixtureInventory]::ForegroundIs([IntPtr]$notice.Modal) -and $script:fixtureCoverClicks -eq 0) 'Fixed caption click focuses the warning without clicking the unrelated window or Agree'
  $report.captionFocusVerified=$true
 } finally {$cover.Close();$cover.Dispose();[Windows.Forms.Application]::DoEvents()}
 $notice=[OpenNavX.StockWelcomeNative]::Inspect($process.Id)
 $report.notice=$notice
 Assert-StockWelcomeWindow $notice $process.Id
 Check ($notice.Title -ceq $expectedTitle) 'Actual warning uses the unchanged requested profile locale'
 $image=Join-Path $evidence 'actual-stock-welcome.png';$imageHash=Save-StockWelcomeCapture $process.Id $notice $image
 $report.imageSha256=$imageHash
 Check ([IO.File]::Exists($image)) 'Same production capture primitive saved the actual HTML warning pixels'
 Refuse {[OpenNavX.StockWelcomeNative]::AssertUnchanged(0,$notice)} 'Wrong process identity refuses before any acknowledgement'
 $bad=CopyNotice $notice;$bad.Agree=$notice.Cancel
 Refuse {[OpenNavX.StockWelcomeNative]::AssertUnchanged($process.Id,$bad)} 'Captured Cancel handle cannot substitute for Agree'
 foreach($field in @('Title','ModalClass','AgreeText','CancelText','HtmlClass','HtmlName')){
  $bad=CopyNotice $notice;$bad.$field='changed'
  Refuse {[OpenNavX.StockWelcomeNative]::AssertUnchanged($process.Id,$bad)} ('Changed captured text or class refuses: '+$field)
 }
 $bad=CopyNotice $notice;$bounds=$bad.Bounds;$bounds.Left++;$bad.Bounds=$bounds
 Refuse {[OpenNavX.StockWelcomeNative]::AssertUnchanged($process.Id,$bad)} 'Changed inspected geometry refuses acknowledgement'
 Refuse {Invoke-StockWelcomeAgreement $process.Id $notice ('0'*64) (Join-Path $evidence 'wrong-hash-before.png') (Join-Path $evidence 'wrong-hash-intent.json')} 'Wrong reviewed pixels refuse before intent or Agree'
 Check (-not (Test-Path -LiteralPath (Join-Path $evidence 'wrong-hash-intent.json'))) 'Rejected image proof created no acknowledgement intent'
 [OpenNavX.StockWelcomeNative]::AssertUnchanged($process.Id,$notice)
 $intent=Join-Path $evidence 'agree-intent.json'
 Invoke-StockWelcomeAgreement $process.Id $notice $imageHash (Join-Path $evidence 'before-agree.png') $intent
 $agreed=$true;Check (Test-Path -LiteralPath $intent) 'Same production primitive durably records one intent before Agree'
 $deadline=[datetime]::UtcNow.AddSeconds(45);$frame=[IntPtr]::Zero
 do {
  Start-Sleep -Milliseconds 250;$process.Refresh();if($process.HasExited){throw 'Official stock exited after Agree instead of exposing its frame.'}
  try{[OpenNavX.StockReviewNative]::Foreground($process.MainWindowHandle,$process.Id);$null=[OpenNavX.StockReviewNative]::AssertFrame($process.MainWindowHandle,$process.Id);$frame=$process.MainWindowHandle}catch{}
 } while($frame -eq [IntPtr]::Zero -and [datetime]::UtcNow -lt $deadline)
 Check ($frame -ne [IntPtr]::Zero) 'Agree actually dismissed the warning and exposed the enabled stock main frame'
 $deadline=[datetime]::UtcNow.AddSeconds(45)
 do{Start-Sleep -Milliseconds 250;$initialized=Test-StartupInitializedSince ([byte[]]@()) (Read-StartupLogBytes $log)}while(-not $initialized -and -not $process.HasExited -and [datetime]::UtcNow -lt $deadline)
 Check $initialized 'Portable application completed a fresh startup after the accepted caution'
 $window=[OpenNavX.StockReviewNative]::AssertFrame($frame,$process.Id)
 $bitmap=New-Object Drawing.Bitmap($window.Bounds.Width,$window.Bounds.Height);$graphics=[Drawing.Graphics]::FromImage($bitmap)
 try {
  [OpenNavX.StockReviewNative]::AssertCapture($frame,$process.Id,$window)
  $graphics.CopyFromScreen($window.Bounds.Left,$window.Bounds.Top,0,0,$bitmap.Size)
  [OpenNavX.StockReviewNative]::AssertCapture($frame,$process.Id,$window)
  $afterImage=Join-Path $evidence 'actual-stock-after-agree.png';$bitmap.Save($afterImage,[Drawing.Imaging.ImageFormat]::Png)
 } finally {$graphics.Dispose();$bitmap.Dispose()}
 $report.afterImageSha256=Get-Digest $afterImage
 $report.afterNativeWindow=$window
 Refuse {[OpenNavX.StockWelcomeNative]::AssertUnchanged($process.Id,$notice)} 'Old dismissed warning cannot be acknowledged a second time'
 Check ($process.CloseMainWindow() -and $process.WaitForExit(30000)) 'Actual official portable application closed normally without force termination'
 Check ($process.ExitCode -eq 0) 'Official stock normal close succeeded'
 Copy-Item -LiteralPath $ini -Destination (Join-Path $evidence 'portable-after.ini')
 $profile=Read-ProfileForAudit $ini
 Check ($profile['Settings/NMEADataSource/DataConnections'] -ceq '') 'Portable session retained empty marine connection list'
 foreach($name in $pluginNames){Check ($profile['PlugIns/'+$name+'/bEnabled'] -ceq '0') ('Bundled plugin remained disabled: '+$name)}
 Check ($profile['Settings/Locale'] -ceq $Locale) 'User locale preserved exactly after normal startup and close'
 Check ($profile['Settings/NavMessageShown'] -ceq '1') 'Actual first-start acceptance was persisted by OpenCPN itself'
 Check (-not (Test-Path -LiteralPath $normalProfile)) 'Normal shared profile was never created or modified'
 Check ((Get-Digest $exe) -ceq $stockHash) 'Official executable stayed unchanged throughout the fixture'
 $report.status='passed'
} catch {$failure=$_.Exception.Message;$report.status='failed';$report.error=$failure;$report.stack=$_.ScriptStackTrace;throw}
finally {
 if($process){
  if(-not $process.HasExited){
   # A failure may leave only an ordinary frame. Close just that exact owned
   # frame if the normal production guard accepts; unknown modals stay untouched.
   try{
    if($process.Path -ine $exe -or (Get-Digest $process.Path) -cne $stockHash){throw 'Fixture process image changed.'}
    $null=[OpenNavX.StockReviewNative]::AssertFrame($process.MainWindowHandle,$process.Id)
    $report.cleanupNormalCloseRequested=$process.CloseMainWindow()
    if($report.cleanupNormalCloseRequested){$report.cleanupNormalExit=$process.WaitForExit(5000)}
   }catch{$report.cleanupRefusal=$_.Exception.Message}
  }
  if(-not $process.HasExited){
   try{$report.failureWindows=@([StockWarningFixtureInventory]::ReadOwned($process.Id))}catch{$report.failureInventoryError=$_.Exception.Message}
   $report.processLeftForDisposableRunnerTeardown=$true
  }
  foreach($stream in @(@('stdout',$outputTask),@('stderr',$errorTask))){
   if($stream[1] -and $stream[1].IsCompleted){try{[IO.File]::WriteAllText((Join-Path $evidence ('stock-'+$stream[0]+'.log')),$stream[1].GetAwaiter().GetResult())}catch{}}
  }
  $process.Dispose()
 }
 if($oldDpi -ne [IntPtr]::Zero){$null=[OpenNavX.StockReviewNative]::SetThreadDpiAwarenessContext($oldDpi)}
 if($log -and [IO.File]::Exists($log)){try{[IO.File]::WriteAllBytes((Join-Path $evidence 'portable-opencpn.log'),(Read-StartupLogBytes $log))}catch{$report.logCaptureError=$_.Exception.Message}}
 $report.agreeSent=$agreed;$report.checkCount=$checks.Count;$report.checks=$checks.ToArray()
 $report.welcomeHelperSha256=Get-Digest (Join-Path $PSScriptRoot 'boat\StockWelcome.ps1');$report.welcomeNativeSha256=Get-Digest (Join-Path $PSScriptRoot 'boat\StockWelcomeNative.cs')
 Write-Record (Join-Path $evidence 'stock-warning-results.json') $report
}

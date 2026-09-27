# Fresh native PS5.1 JSON -> production acknowledgement primitive regression.
# Only a marked disposable official-stock portable fixture may reach the helper.
[CmdletBinding()]
param([Parameter(Mandatory=$true)][string]$Packet,[Parameter(Mandatory=$true)][string]$ExpectedHash)
Set-StrictMode -Version Latest;$ErrorActionPreference='Stop'
if($env:GITHUB_ACTIONS -cne 'true' -or [Environment]::OSVersion.Platform -ne 'Win32NT' -or $PSVersionTable.PSEdition -cne 'Desktop'){throw 'Native disposable Windows PowerShell 5.1 fixture only.'}
$tools=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../../tools/boat'))
. (Join-Path $tools 'Common.ps1')
. (Join-Path $tools 'StockWelcome.ps1')
Add-Type -Path (Join-Path $tools 'StockReviewNative.cs')
$Packet=Assert-LocalPath $Packet
if($ExpectedHash -cnotmatch '^[a-f0-9]{64}$' -or (Get-Digest $Packet) -cne $ExpectedHash){throw 'Exact saved inspection packet required.'}
$record=Read-Record $Packet;$root=Assert-LocalPath $record.fixtureRoot
if([IO.Path]::GetDirectoryName($root) -ine (Assert-LocalPath $env:RUNNER_TEMP) -or [IO.Path]::GetFileName($root) -cnotmatch '^OpenNav stock warning [a-f0-9]{32}$'){throw 'Not the disposable fixture root.'}
$marker=Read-Record (Join-Path $root 'fixture.json')
if($marker.owner -cne 'OpenNavX.OfficialStockWarningFixture.1' -or $marker.temporary -cne $root){throw 'Fixture owner differs.'}
$exe=Assert-LocalPath (Join-Path $root 'portable OpenCPN/opencpn.exe');$ini=Assert-LocalPath (Join-Path $root 'portable OpenCPN/opencpn.ini')
if((Get-Digest $exe) -cne '7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c' -or (Get-Digest $ini) -cne $record.profileSha256){throw 'Exact isolated stock/profile bytes changed.'}
$profile=Read-ProfileForAudit $ini
if($profile['Settings/NMEADataSource/DataConnections'] -cne ''){throw 'The fixture must have no marine connections.'}
foreach($name in @('chartdldr_pi.dll','dashboard_pi.dll','grib_pi.dll','wmm_pi.dll')){if($profile['PlugIns/'+$name+'/bEnabled'] -cne '0'){throw 'Fixture plugin is not disabled.'}}
$process=Get-Process -Id $record.processId;$null=$process.Handle
$oldDpi=[OpenNavX.StockReviewNative]::SetThreadDpiAwarenessContext([IntPtr](-4));$cover=$null
try {
 if($oldDpi -eq [IntPtr]::Zero -or $process.HasExited -or $process.Path -ine $exe -or
    $process.SessionId -ne [Diagnostics.Process]::GetCurrentProcess().SessionId -or
    $process.StartTime.ToUniversalTime().Ticks -ne $record.processStartedUtcTicks){throw 'Exact isolated fixture process differs.'}
 $native=Get-CimInstance Win32_Process -Filter ('ProcessId='+$process.Id)
 if($native.CommandLine -cne ('"'+$exe.Replace('/','\')+'" --portable --no_opengl')){throw 'Fixture is not the exact portable invocation.'}
 $directory=[IO.Path]::GetDirectoryName($Packet)
 $image=Join-Path $directory 'actual-stock-welcome.png'
 if((Get-Digest $image) -cne $record.imageSha256){throw 'Saved reviewed pixels changed.'}
 # Keep the deserialized Width/Height present. Production reconstructs the
 # writable C# fields and validates both derived dimensions before capture.
 if($null -eq $record.nativeWindow.Bounds.Width -or $null -eq $record.nativeWindow.Bounds.Height){throw 'Actual serialized read-only dimensions missing.'}
 # Reproduce a separate review task taking foreground: only an owned inert,
 # non-overlapping fixture form. The production primitive must ordinarily
 # activate the unchanged warning itself before comparing the saved pixels.
 Add-Type -AssemblyName System.Windows.Forms
 Add-Type -TypeDefinition @'
using System;
using System.Runtime.InteropServices;
public static class AckFixtureForeground {
 [DllImport("user32.dll")] private static extern IntPtr GetForegroundWindow();
 public static bool Is(IntPtr window){return GetForegroundWindow()==window;}
}
'@
 $screen=[Windows.Forms.Screen]::PrimaryScreen.WorkingArea
 $away=New-Object Drawing.Rectangle(($screen.Right-160),($screen.Bottom-100),160,100)
 $b=$record.nativeWindow.Bounds;$warning=New-Object Drawing.Rectangle($b.Left,$b.Top,$b.Width,$b.Height)
 if($away.IntersectsWith($warning)){throw 'No clear fixture foreground-window position.'}
 $cover=New-Object Windows.Forms.Form;$cover.FormBorderStyle=[Windows.Forms.FormBorderStyle]::None
 $cover.ShowInTaskbar=$false;$cover.StartPosition=[Windows.Forms.FormStartPosition]::Manual
 $cover.Text='Disposable acknowledgement task foreground';$cover.Bounds=$away
 $cover.Show();$cover.Activate();[Windows.Forms.Application]::DoEvents()
 if(-not [AckFixtureForeground]::Is($cover.Handle)){throw 'Fresh acknowledgement fixture did not own foreground before the production call.'}
 Invoke-StockWelcomeAgreement $process.Id $record.nativeWindow $record.imageSha256 (Join-Path $directory 'before-agree.png') (Join-Path $directory 'agree-intent.json')
 $attempts=@(Get-ChildItem -LiteralPath $directory -Filter 'before-agree.png.settle-*.png' -File | Sort-Object Name)
 if($attempts.Count -lt 2 -or $attempts.Count -gt 20 -or (Get-Digest $attempts[-1].FullName) -cne $record.imageSha256 -or (Get-Digest $attempts[-2].FullName) -cne $record.imageSha256){throw 'Actual agreement did not retain two consecutive exact reviewed images.'}
 [pscustomobject]@{status='passed';freshProcessId=$PID;powerShellEdition=$PSVersionTable.PSEdition;powerShellMajor=$PSVersionTable.PSVersion.Major;serializedWidth=$record.nativeWindow.Bounds.Width;serializedHeight=$record.nativeWindow.Bounds.Height;sharedProductionAgreement=$true;ownForegroundBeforeAgreement=$true;ordinaryActivationRestoredWarning=$true;fullImageSettleCaptures=$attempts.Count;twoConsecutiveReviewedImages=$true} | ConvertTo-Json
} finally {if($null -ne $cover){$cover.Close();$cover.Dispose()};if($oldDpi -ne [IntPtr]::Zero){$null=[OpenNavX.StockReviewNative]::SetThreadDpiAwarenessContext($oldDpi)};$process.Dispose()}

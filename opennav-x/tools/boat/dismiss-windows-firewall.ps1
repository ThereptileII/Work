# A single normal close request to a separately inspected Microsoft firewall
# prompt. No Allow button, network policy, service, app or device is controlled.
[CmdletBinding()]
param([Parameter(Mandatory=$true)][string]$Inspection,
      [Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{64}$')][string]$InspectionSha256,
      [string]$Workspace='C:\XNav',[string]$OutputDirectory='', [switch]$Interactive)
. (Join-Path $PSScriptRoot 'Preparation.ps1')
if (-not $Interactive) {
  $sid=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value
  $explorers=@(Get-CimInstance Win32_Process -Filter "Name='explorer.exe'" | Where-Object {(Invoke-CimMethod -InputObject $_ -MethodName GetOwnerSid).Sid -eq $sid})
  if ($explorers.Count -ne 1) {throw 'One interactive desktop required.'}
  $directory=New-PreparationDirectory ([pscustomobject]@{workspace=$Workspace;sid=$sid}) 'firewall-cancel'
  $name='OpenNavX-Firewall-Cancel-'+[guid]::NewGuid().ToString('N')
  $action=New-ScheduledTaskAction -Execute (Join-Path $env:WINDIR 'System32\WindowsPowerShell\v1.0\powershell.exe') -Argument ('-NoProfile -NonInteractive -ExecutionPolicy Bypass -File "'+$PSCommandPath+'" -Interactive -OutputDirectory "'+$directory+'" -Inspection "'+(Assert-LocalPath $Inspection)+'" -InspectionSha256 '+$InspectionSha256)
  $principal=New-ScheduledTaskPrincipal -UserId $sid -LogonType Interactive -RunLevel Limited
  $settings=New-ScheduledTaskSettingsSet -ExecutionTimeLimit (New-TimeSpan -Minutes 2) -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries
  $task=$null
  try {
    $task=Register-ScheduledTask -TaskName $name -Action $action -Principal $principal -Settings $settings
    Start-ScheduledTask -TaskName $name
    $result=Join-Path $directory 'result.json';$deadline=[datetime]::UtcNow.AddSeconds(55)
    while (-not [IO.File]::Exists($result) -and [datetime]::UtcNow -lt $deadline) {Start-Sleep -Milliseconds 250}
    if (-not [IO.File]::Exists($result)) {throw ('Inspect '+$directory+' before any further action; no retry.')}
    Write-Output $result
  } finally {if($task){Unregister-ScheduledTask -TaskName $name -Confirm:$false}}
  exit
}
$directory=Assert-LocalPath $OutputDirectory
if ([IO.Path]::GetDirectoryName($directory) -ine (Join-Path $Workspace 'runs') -or [IO.Path]::GetFileName($directory) -cnotmatch '^\d{8}-\d{6}-firewall-cancel-[a-f0-9]{8}$' -or @(Get-ChildItem -LiteralPath $directory -Force).Count) {throw 'New private evidence directory required.'}
$result=@{status='failed';utc=[datetime]::UtcNow.ToString('o');closeRequestSent=$false;allowInvoked=$false;inspectionSha256=$InspectionSha256;sourceSha256=(Get-Digest $PSCommandPath)}
try {
  if ((Get-Digest $Inspection) -cne $InspectionSha256) {throw 'Inspection changed.'}
  $review=Read-Record $Inspection
  if ($review.readOnly -ne $true -or $review.actionsSent -ne 0 -or $review.window.title -cne 'Windows Security' -or $review.window.className -cne 'Shell_SystemDialogProxy') {throw 'Expected inspected Windows Security proxy.'}
  $at=[datetime]::Parse($review.utc).ToUniversalTime()
  if ($at -gt [datetime]::UtcNow -or ([datetime]::UtcNow-$at).TotalMinutes -gt 30) {throw 'Inspection expired.'}
  $expected=Join-Path $env:WINDIR 'System32\PickerHost.exe'
  $p=Get-Process -Id $review.process.id -ErrorAction Stop
  if ($p.Path -ine $expected -or $review.process.path -ine $expected -or $p.SessionId -ne [Diagnostics.Process]::GetCurrentProcess().SessionId -or $p.StartTime.ToUniversalTime().Ticks -ne ([datetime]::Parse($review.process.startedUtc)).ToUniversalTime().Ticks -or (Get-Digest $expected) -cne $review.process.sha256) {throw 'Exact Microsoft prompt process identity required.'}
  $signature=Get-AuthenticodeSignature -LiteralPath $expected
  if ($signature.Status -ne 'Valid' -or $signature.SignerCertificate.Subject -cne 'CN=Microsoft Windows, O=Microsoft Corporation, L=Redmond, S=Washington, C=US') {throw 'Microsoft Windows signature required.'}
  Add-Type -TypeDefinition @'
using System;using System.Text;using System.Runtime.InteropServices;
public static class OpenNavFirewallCancel {
 [DllImport("user32.dll")] static extern uint GetWindowThreadProcessId(IntPtr h,out uint p);
 [DllImport("user32.dll",CharSet=CharSet.Unicode)] static extern int GetWindowText(IntPtr h,StringBuilder s,int n);
 [DllImport("user32.dll",CharSet=CharSet.Unicode)] static extern int GetClassName(IntPtr h,StringBuilder s,int n);
 [DllImport("user32.dll")] public static extern bool IsWindow(IntPtr h);
 [DllImport("user32.dll",SetLastError=true)] static extern bool PostMessage(IntPtr h,uint m,IntPtr w,IntPtr l);
 public static void Check(IntPtr h,int pid){uint owner;GetWindowThreadProcessId(h,out owner);var title=new StringBuilder(256);var cl=new StringBuilder(256);GetWindowText(h,title,256);GetClassName(h,cl,256);
  if(owner!=(uint)pid||title.ToString()!="Windows Security"||cl.ToString()!="Shell_SystemDialogProxy")throw new InvalidOperationException("Prompt identity changed");}
 public static bool RequestClose(IntPtr h,int pid){Check(h,pid);return PostMessage(h,0x0010,IntPtr.Zero,IntPtr.Zero);}
}
'@
  $handle=[IntPtr][long]$review.window.handle
  [OpenNavFirewallCancel]::Check($handle,$p.Id)
  # A durable intent prevents an ambiguous UI response from authorizing retry.
  Write-Record (Join-Path $directory 'intent.json') @{action='WM_CLOSE';window=$review.window;process=$review.process;inspectionSha256=$InspectionSha256}
  $result.closeRequestSent=[OpenNavFirewallCancel]::RequestClose($handle,$p.Id)
  Start-Sleep -Seconds 2
  $result.proxyDestroyed=-not [OpenNavFirewallCancel]::IsWindow($handle)
  $result.status='attention' # Proxy destruction does not prove sheet dismissal.
  $result.review='Normal close only. Capture the application separately; no process termination and no network permission granted by this tool.'
} catch {$result.error=$_.Exception.Message}
Write-Record (Join-Path $directory 'result.json') $result

# Disposable CI-only native controls. This does not load OpenCPN or any plugin.
[CmdletBinding()]
param([Parameter(Mandatory=$true)][string]$Directory)
Set-StrictMode -Version Latest;$ErrorActionPreference='Stop'
$root=[IO.Path]::GetFullPath($Directory)
if(-not $root.StartsWith([IO.Path]::GetTempPath(),[StringComparison]::OrdinalIgnoreCase) -or
   [IO.Path]::GetFileName($root) -cnotmatch '^opennav-mode-window-[a-f0-9]{32}$'){throw 'Only a unique temporary native fixture is allowed.'}
$record=Get-Content -LiteralPath (Join-Path $root 'fixture.json') -Raw|ConvertFrom-Json
if($record.owner -cne 'OpenNavX.NativeModeWindow.Fixture.1' -or $record.mode -cnotin @('--xnav','--legacy','--safe-mode') -or
   $record.case -cnotin @('normal','ambiguous','replace-on-down','hidden-menu','modal')){throw 'Unknown fixed fixture.'}
Add-Type -AssemblyName System.Windows.Forms
Add-Type -TypeDefinition @'
using System;
using System.Runtime.InteropServices;
public static class OpenNavModeFixtureLabel {
 [DllImport("user32.dll",CharSet=CharSet.Unicode)] public static extern bool SetWindowTextW(IntPtr h,string text);
}
'@
$form=New-Object Windows.Forms.Form
$form.Text=switch($record.mode){'--xnav'{'OpenNav X / OpenCPN'};'--legacy'{'OpenCPN / Legacy'};'--safe-mode'{'OpenNav Safe Mode / OpenCPN'}}
$form.StartPosition='Manual';$form.Location=New-Object Drawing.Point(20,20);$form.Size=New-Object Drawing.Size(900,640)
$form.BackColor=[Drawing.Color]::FromArgb(10,24,32)
$script:controls=New-Object 'Collections.Generic.List[object]'
function SaveClick([string]$Caption){[IO.File]::AppendAllText((Join-Path $root 'clicks.txt'),$Caption+"`n")}
if($record.mode -ceq '--xnav') {
 $panel=New-Object Windows.Forms.Panel;$panel.Location=New-Object Drawing.Point(20,50);$panel.Size=New-Object Drawing.Size(820,450);$form.Controls.Add($panel)
 $null=$panel.Handle;$null=[OpenNavModeFixtureLabel]::SetWindowTextW($panel.Handle,'OpenNav product page: System')
 $y=20
 foreach($caption in @('Open Legacy OpenCPN','Restart XNav','Safe Mode','STBY')) {
  $button=New-Object Windows.Forms.Button;$button.Text=$caption;$button.Size=New-Object Drawing.Size(300,60);$button.Location=New-Object Drawing.Point(20,$y);$y+=75
  $button.Add_Click({param($sender,$event) SaveClick $sender.Text});$panel.Controls.Add($button);$script:controls.Add($button)
 }
 if($record.case -ceq 'ambiguous') {
  $button=New-Object Windows.Forms.Button;$button.Text='Open Legacy OpenCPN';$button.Size=New-Object Drawing.Size(300,60);$button.Location=New-Object Drawing.Point(400,20)
  $button.Add_Click({param($sender,$event) SaveClick $sender.Text});$panel.Controls.Add($button)
 }
 if($record.case -ceq 'replace-on-down') {
  $script:controls[0].Add_MouseDown({param($sender,$event)
   $parent=$sender.Parent;$location=$sender.Location;$size=$sender.Size;$sender.Dispose()
   $replacement=New-Object Windows.Forms.Button;$replacement.Text='Open Legacy OpenCPN';$replacement.Location=$location;$replacement.Size=$size
   $replacement.Add_Click({SaveClick 'UNSAFE_REPLACEMENT'});$parent.Controls.Add($replacement)
  })
 }
} elseif($record.case -cne 'hidden-menu') {
 $menu=New-Object Windows.Forms.MainMenu;$group=New-Object Windows.Forms.MenuItem('&Tools')
 $entry=New-Object Windows.Forms.MenuItem('Switch to XNav');$entry.Add_Click({SaveClick 'Switch to XNav'});$null=$group.MenuItems.Add($entry)
 $other=New-Object Windows.Forms.MenuItem('Activate route');$other.Add_Click({SaveClick 'UNSAFE_ROUTE'});$null=$group.MenuItems.Add($other)
 $null=$menu.MenuItems.Add($group);$form.Menu=$menu
}
$started=[datetime]::UtcNow;$timer=New-Object Windows.Forms.Timer;$timer.Interval=100
$timer.Add_Tick({if((Test-Path -LiteralPath (Join-Path $root 'release')) -or ([datetime]::UtcNow-$started).TotalSeconds -gt 25){$form.Close()}})
$form.Add_Shown({
 [IO.File]::WriteAllText((Join-Path $root 'ready.json'),(@{pid=$PID;handle=$form.Handle.ToInt64();createdFiletime=[Diagnostics.Process]::GetCurrentProcess().StartTime.ToUniversalTime().ToFileTimeUtc().ToString()}|ConvertTo-Json -Compress))
 $timer.Start()
 if($record.case -ceq 'modal'){$dialog=New-Object Windows.Forms.Form;$dialog.Text='Unexpected modal';$dialog.Size=New-Object Drawing.Size(300,200);$null=$dialog.ShowDialog($form);$dialog.Dispose()}
})
try{[Windows.Forms.Application]::Run($form)}finally{$timer.Dispose();$form.Dispose()}

# Disposable native controls only; no navigation application or marine input.
[CmdletBinding()]
param([Parameter(Mandatory=$true)][string]$Directory)
Set-StrictMode -Version Latest;$ErrorActionPreference='Stop'
$root=[IO.Path]::GetFullPath($Directory)
if(-not $root.StartsWith([IO.Path]::GetTempPath(),[StringComparison]::OrdinalIgnoreCase) -or
   [IO.Path]::GetFileName($root) -cnotmatch '^opennav-display-window-[a-f0-9]{32}$'){throw 'Unique temporary display fixture required.'}
$record=Get-Content -LiteralPath (Join-Path $root 'fixture.json') -Raw|ConvertFrom-Json
if($record.owner -cne 'OpenNavX.NativeDisplayWindow.Fixture.1' -or
   $record.action -cnotin @('Display','ToggleFullscreen','ToggleOrientation','CyclePalette') -or
   $record.case -cnotin @('normal','return','course','wrong-page','ambiguous','replace-on-down','rename-on-down','move-on-down','duplicate-on-down','modal')){throw 'Unknown fixed fixture.'}
Add-Type -AssemblyName System.Windows.Forms
Add-Type -TypeDefinition @'
using System;
using System.Runtime.InteropServices;
public static class OpenNavDisplayFixtureLabel {
 [DllImport("user32.dll",CharSet=CharSet.Unicode)] public static extern bool SetWindowTextW(IntPtr h,string text);
}
'@
$form=New-Object Windows.Forms.Form
$form.Text='OpenNav X / OpenCPN';$form.StartPosition='Manual'
$form.Location=New-Object Drawing.Point(20,20);$form.Size=New-Object Drawing.Size(900,640)
$form.BackColor=[Drawing.Color]::FromArgb(10,24,32)
function Button($Parent,[string]$Label,[int]$X,[int]$Y,[int]$Width=180) {
 $button=New-Object Windows.Forms.Button;$button.Text=$Label;$button.Size=New-Object Drawing.Size($Width,56)
 $button.Location=New-Object Drawing.Point($X,$Y);$Parent.Controls.Add($button);return $button
}
function SaveClick([string]$Caption){[IO.File]::AppendAllText((Join-Path $root 'clicks.txt'),$Caption+"`n")}
$top=New-Object Windows.Forms.Panel;$top.Location=New-Object Drawing.Point(0,0);$top.Size=New-Object Drawing.Size(860,64);$form.Controls.Add($top)
$null=Button $top 'Menu' 10 4 100
$palette=Button $top 'Day' 130 4 100
$palette.Add_Click({param($sender,$event) SaveClick ('Status '+$sender.Text);$sender.Text=switch($sender.Text){'Day'{'Dusk'};'Dusk'{'Night'};default{'Day'}}})
$bottom=New-Object Windows.Forms.Panel;$bottom.Location=New-Object Drawing.Point(0,520);$bottom.Size=New-Object Drawing.Size(860,64);$form.Controls.Add($bottom)
$null=Button $bottom 'Navigation' 10 4
$decoy=Button $bottom 'STBY' 220 4;$decoy.Add_Click({SaveClick 'UNSAFE_STBY'})
$panel=New-Object Windows.Forms.Panel;$panel.Location=New-Object Drawing.Point(10,72);$panel.Size=New-Object Drawing.Size(840,430);$form.Controls.Add($panel)
$null=$panel.Handle
$pageLabel=switch($record.action){'Display'{'OpenNav product page: Settings'};'ToggleFullscreen'{'OpenNav product page: Display'};'CyclePalette'{'OpenNav product page: Display'};default{''}}
if($record.case -ceq 'wrong-page'){$pageLabel='OpenNav product page: Autopilot configuration'}
$null=[OpenNavDisplayFixtureLabel]::SetWindowTextW($panel.Handle,$pageLabel)
$script:full=$false
$script:button=$null
switch($record.action) {
 'Display' {
  $script:button=Button $panel 'DISPLAY' 20 20 260
  $script:button.Add_Click({SaveClick 'DISPLAY';$null=[OpenNavDisplayFixtureLabel]::SetWindowTextW($panel.Handle,'OpenNav product page: Display')})
 }
 'ToggleFullscreen' {
  $script:button=Button $panel 'Fullscreen / window' 20 20 300
  $script:button.Add_Click({
   SaveClick 'Fullscreen / window'
   if($script:full){$form.WindowState='Normal';$form.FormBorderStyle='Sizable';$form.Location=New-Object Drawing.Point(20,20);$form.Size=New-Object Drawing.Size(900,640)}
   else{$form.FormBorderStyle='None';$form.WindowState='Maximized'}
   $script:full=-not $script:full
  })
 }
 'ToggleOrientation' {
  $null=Button $panel '+' 20 20 100;$null=Button $panel ([string][char]0x2212) 20 90 100;$null=Button $panel 'Center' 20 160 100
  $label=if($record.case -ceq 'course'){'Course'}else{'North'}
  $script:button=Button $panel $label 20 230 100
  $script:button.Add_Click({param($sender,$event) SaveClick $sender.Text;$sender.Text=if($sender.Text -ceq 'North'){'Course'}else{'North'}})
 }
 'CyclePalette' {
  foreach($label in @('Day','Dusk','Night')) {
   $x=switch($label){'Day'{20};'Dusk'{230};default{440}}
   $decoy=Button $panel $label $x 20;$decoy.Add_Click({SaveClick 'UNSAFE_PAGE_PALETTE'})
  }
  $script:button=$palette
 }
}
if($record.case -ceq 'ambiguous') {
 $other=Button $script:button.Parent $script:button.Text 420 130 300;$other.Add_Click({SaveClick 'UNSAFE_DUPLICATE'})
}
if($record.case -cin @('replace-on-down','rename-on-down','move-on-down','duplicate-on-down')) {
 $script:button.Add_MouseDown({param($sender,$event)
  switch($record.case) {
   'replace-on-down' {
    $parent=$sender.Parent;$location=$sender.Location;$size=$sender.Size;$label=$sender.Text;$sender.Dispose()
    $replacement=New-Object Windows.Forms.Button;$replacement.Text=$label;$replacement.Location=$location;$replacement.Size=$size
    $replacement.Add_Click({SaveClick 'UNSAFE_REPLACEMENT'});$parent.Controls.Add($replacement)
   }
   'rename-on-down' {$sender.Text='STBY'}
   'move-on-down' {$sender.Left+=10}
   'duplicate-on-down' {$other=Button $sender.Parent $sender.Text 420 130 300;$other.Add_Click({SaveClick 'UNSAFE_DUPLICATE'})}
  }
 })
}
$started=[datetime]::UtcNow;$timer=New-Object Windows.Forms.Timer;$timer.Interval=100
$timer.Add_Tick({if((Test-Path -LiteralPath (Join-Path $root 'release')) -or ([datetime]::UtcNow-$started).TotalSeconds -gt 25){$form.Close()}})
$form.Add_Shown({
 if($record.case -ceq 'return'){$form.FormBorderStyle='None';$form.WindowState='Maximized';$script:full=$true}
 [IO.File]::WriteAllText((Join-Path $root 'ready.json'),(@{pid=$PID;handle=$form.Handle.ToInt64();createdFiletime=[Diagnostics.Process]::GetCurrentProcess().StartTime.ToUniversalTime().ToFileTimeUtc().ToString()}|ConvertTo-Json -Compress))
 $timer.Start()
 if($record.case -ceq 'modal'){$dialog=New-Object Windows.Forms.Form;$dialog.Text='Unexpected modal';$dialog.Size=New-Object Drawing.Size(300,200);$null=$dialog.ShowDialog($form);$dialog.Dispose()}
})
try{[Windows.Forms.Application]::Run($form)}finally{$timer.Dispose();$form.Dispose()}

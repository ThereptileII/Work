# Disposable native controls only; no navigation application or marine input.
[CmdletBinding()]
param([Parameter(Mandatory=$true)][string]$Directory)
Set-StrictMode -Version Latest;$ErrorActionPreference='Stop'
$root=[IO.Path]::GetFullPath($Directory)
if(-not $root.StartsWith([IO.Path]::GetTempPath(),[StringComparison]::OrdinalIgnoreCase) -or
   [IO.Path]::GetFileName($root) -cnotmatch '^opennav-display-window-[a-f0-9]{32}$'){throw 'Unique temporary display fixture required.'}
$record=Get-Content -LiteralPath (Join-Path $root 'fixture.json') -Raw|ConvertFrom-Json
if($record.owner -cne 'OpenNavX.NativeDisplayWindow.Fixture.1' -or
   $record.action -cnotin @('Display','ToggleFullscreen','ToggleOrientation','CyclePalette','PanRight','Resize1280x800','Capture','Navigation','Route','AIS','Instruments','ZoomIn','ZoomOut','System','RevealInterfaceRecovery','InterfaceRecovery') -or
   $record.case -cnotin @('normal','return','course','wrong-page','wrong-geometry','canvas-child','ambiguous','replace-on-down','rename-on-down','move-on-down','duplicate-on-down','modal','maximized-offscreen','partial-offscreen','entirely-offscreen','minimized','demo','wrong-pid','prototype-normal','prototype-anchor','prototype-pilot','prototype-alerts','prototype-preferences','prototype-two-sheets','prototype-passage','prototype-traffic','prototype-back','prototype-unknown','prototype-wrong-owner','prototype-duplicate','prototype-clipped','prototype-signature','prototype-moved','prototype-rail-duplicate','prototype-modal','prototype-zoom-wrong-owner','prototype-zoom-wrong-pid','prototype-zoom-hidden','prototype-zoom-occluded','prototype-zoom-wrong-surface','prototype-zoom-replace-on-down','prototype-system-normal','prototype-system-duplicate','prototype-system-hidden','prototype-system-wrong-owner','prototype-recovery-normal','prototype-recovery-hidden','prototype-recovery-duplicate','prototype-recovery-no-progress','prototype-recovery-changed-body','prototype-recovery-wrong-owner','prototype-recovery-reveal','prototype-recovery-clipped','prototype-recovery-wrong-surface')){throw 'Unknown fixed fixture.'}
Add-Type -AssemblyName System.Windows.Forms
Add-Type -TypeDefinition @'
using System;
using System.Runtime.InteropServices;
public static class OpenNavDisplayFixtureLabel {
 [DllImport("user32.dll",CharSet=CharSet.Unicode)] public static extern bool SetWindowTextW(IntPtr h,string text);
}
'@
Add-Type -ReferencedAssemblies System.Windows.Forms -TypeDefinition @'
using System;
using System.IO;
using System.Windows.Forms;
public sealed class OpenNavPanFixtureCanvas : Panel {
 public string Output;
 protected override void WndProc(ref Message m) {
  if(m.WParam.ToInt64()==0x27 && (m.Msg==0x100 || m.Msg==0x101))
   File.AppendAllText(Output,m.Msg==0x100 ? "PAN_RIGHT_DOWN\n" : "PAN_RIGHT_UP\n");
  base.WndProc(ref m);
 }
}
public sealed class OpenNavRecoveryFixtureBody : Panel {
 public string Output;
 public bool FreezeScroll;
 protected override void WndProc(ref Message m) {
  if(m.Msg==0x115) {
   File.AppendAllText(Output,m.WParam.ToInt64()==3 ? "PAGE_DOWN\n" : "UNEXPECTED_SCROLL\n");
   if(FreezeScroll)return;
  }
  base.WndProc(ref m);
 }
}
'@
$form=New-Object Windows.Forms.Form
$form.Text='SKAGER / OpenCPN';$form.StartPosition='Manual'
$form.Location=New-Object Drawing.Point(20,20);$form.Size=New-Object Drawing.Size(900,640)
$form.BackColor=[Drawing.Color]::FromArgb(10,24,32)
function Button($Parent,[string]$Label,[int]$X,[int]$Y,[int]$Width=180) {
 $button=New-Object Windows.Forms.Button;$button.Text=$Label;$button.Size=New-Object Drawing.Size($Width,56)
 $button.Location=New-Object Drawing.Point($X,$Y);$Parent.Controls.Add($button);return $button
}
function SaveClick([string]$Caption){[IO.File]::AppendAllText((Join-Path $root 'clicks.txt'),$Caption+"`n")}
$top=New-Object Windows.Forms.Panel;$top.Location=New-Object Drawing.Point(0,0);$top.Size=New-Object Drawing.Size(860,64);$form.Controls.Add($top)
$null=Button $top $(if($record.case -ceq 'demo'){'Demo'}else{'Menu'}) 10 4 100
$palette=Button $top 'Day' 130 4 100
$palette.Add_Click({param($sender,$event) SaveClick ('Status '+$sender.Text);$sender.Text=switch($sender.Text){'Day'{'Dusk'};'Dusk'{'Night'};default{'Day'}}})
$bottom=New-Object Windows.Forms.Panel;$bottom.Location=New-Object Drawing.Point(0,520);$bottom.Size=New-Object Drawing.Size(860,64);$form.Controls.Add($bottom)
$null=Button $bottom 'Navigation' 10 4
$decoy=Button $bottom 'STBY' 220 4;$decoy.Add_Click({SaveClick 'UNSAFE_STBY'})
$panel=if($record.action -ceq 'PanRight'){New-Object OpenNavPanFixtureCanvas}else{New-Object Windows.Forms.Panel}
if($record.action -ceq 'PanRight'){$panel.Output=Join-Path $root 'clicks.txt'}
$panel.Location=New-Object Drawing.Point(10,72);$panel.Size=New-Object Drawing.Size(840,430);$form.Controls.Add($panel)
$null=$panel.Handle
$pageLabel=switch($record.action){'Display'{'SKAGER product page: Settings'};'ToggleFullscreen'{'SKAGER product page: Display'};'CyclePalette'{'SKAGER product page: Display'};default{''}}
if($record.case -ceq 'wrong-page'){$pageLabel='SKAGER product page: Autopilot configuration'}
$null=[OpenNavDisplayFixtureLabel]::SetWindowTextW($panel.Handle,$pageLabel)
$script:full=$false
$script:button=$null
switch($record.action) {
 'Display' {
  $script:button=Button $panel 'DISPLAY' 20 20 260
  $script:button.Add_Click({SaveClick 'DISPLAY';$null=[OpenNavDisplayFixtureLabel]::SetWindowTextW($panel.Handle,'SKAGER product page: Display')})
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
 'PanRight' {
  if($record.case -ceq 'canvas-child') {
   $child=New-Object Windows.Forms.Panel;$child.Dock='Fill';$panel.Controls.Add($child)
  }
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
$script:surfaces=New-Object 'Collections.Generic.List[Windows.Forms.Form]'
$prototype=$record.case.StartsWith('prototype-',[StringComparison]::Ordinal)
if($prototype) {
 $top.Controls[0].Dispose();$bottom.Dispose()
 $null=Button $top 'Alerts' 250 4 100
 $rail=New-Object Windows.Forms.Panel;$rail.Location=New-Object Drawing.Point(0,64);$rail.Size=New-Object Drawing.Size(90,520);$form.Controls.Add($rail);$rail.BringToFront()
 $index=0
 foreach($label in @('Chart','Passage','Traffic','Energy','Instruments','Anchor','Radar','Settings')) {
  $item=Button $rail $label 0 ($index*60) 88;$item.Add_Click({param($sender,$event) SaveClick $sender.Text});$index++
 }
 if($record.case -ceq 'prototype-rail-duplicate'){$null=Button $rail 'Chart' 0 480 88}
 if($record.case -ceq 'prototype-preferences'){$null=Button $rail 'Vessel profile' 0 480 88}
}
function Surface([string]$Title,[int]$X,[int]$Y,[int]$Width,[int]$Height,[string[]]$Labels,[switch]$Heading,[switch]$Unowned) {
 $surface=New-Object Windows.Forms.Form;$surface.Text=$Title;$surface.FormBorderStyle='None';$surface.ShowInTaskbar=$false
 $surface.StartPosition='Manual';$surface.Location=New-Object Drawing.Point($X,$Y);$surface.Size=New-Object Drawing.Size($Width,$Height)
 $surface.BackColor=[Drawing.Color]::FromArgb(20,35,45)
 $parent=$surface
 if($Heading){$parent=New-Object Windows.Forms.Panel;$parent.Location=New-Object Drawing.Point(0,0);$parent.Size=New-Object Drawing.Size($Width,89);$surface.Controls.Add($parent)}
 $offset=0;foreach($label in $Labels){$null=Button $parent $label $offset 0 44;$offset+=44}
 if($Unowned){$surface.TopMost=$true;$surface.Show()}else{$surface.Show($form)}
 $script:surfaces.Add($surface)
}
function SystemFixtureLinks {
 $intro=New-Object Windows.Forms.Panel;$intro.Location=New-Object Drawing.Point(22,122);$intro.Size=New-Object Drawing.Size(386,156);$script:settingsBody.Controls.Add($intro)
 $links=@('Installation & recovery','Updates','Backups','Diagnostics','Plugins','Help & guides','About & licenses','Run vessel setup','Interface & recovery','Advanced / Legacy Settings')
 for($i=0;$i -lt $links.Count;$i++) {
  $link=Button $script:settingsBody $links[$i] 22 (278+72*$i) 386;$link.Height=72
  if($links[$i] -ceq 'Interface & recovery') {
   $script:recoveryTarget=$link
   $link.Add_Click({SaveClick 'Interface & recovery';$script:preferences.Hide();$null=[OpenNavDisplayFixtureLabel]::SetWindowTextW($panel.Handle,'SKAGER product page: System')})
  } else {$link.Add_Click({SaveClick 'UNSAFE_SYSTEM_LINK'})}
  if($links[$i] -cin @('Installation & recovery','Updates','Backups','About & licenses','Run vessel setup')){$link.Enabled=$false}
 }
 if($record.case -ceq 'prototype-recovery-hidden'){$script:recoveryTarget.Hide()}
 if($record.case -ceq 'prototype-recovery-duplicate'){$duplicate=Button $script:settingsBody 'Interface & recovery' 22 1040 386;$duplicate.Height=72;$duplicate.Add_Click({SaveClick 'UNSAFE_DUPLICATE'})}
 if($record.case -ceq 'prototype-recovery-changed-body'){$script:settingsBody.Add_Scroll({param($sender,$event) $sender.Width-=1})}
}
function SettingsSurface {
 # Frozen SettingsDrawer.cpp/Drawer.cpp: direct heading and scroll body;
 # all eight 37-DIP tabs in one body child; 72-DIP links directly in the body.
 $script:preferences=New-Object Windows.Forms.Form;$script:preferences.Text='SKAGER preferences'
 $script:preferences.FormBorderStyle='None';$script:preferences.ShowInTaskbar=$false;$script:preferences.StartPosition='Manual'
 $script:preferences.Location=New-Object Drawing.Point(360,100);$script:preferences.Size=New-Object Drawing.Size(432,460)
 if($record.case -ceq 'prototype-recovery-wrong-surface'){$script:preferences.Text='Chart presentation'}
 $heading=New-Object Windows.Forms.Panel;$heading.Location=New-Object Drawing.Point(1,1);$heading.Size=New-Object Drawing.Size(430,89);$script:preferences.Controls.Add($heading)
 $close=Button $heading 'Close' 336 18 86;$close.Height=44
 $script:settingsBody=New-Object OpenNavRecoveryFixtureBody;$script:settingsBody.Output=Join-Path $root 'scrolls.txt'
 $script:settingsBody.FreezeScroll=$record.case -ceq 'prototype-recovery-no-progress'
 $script:settingsBody.Location=New-Object Drawing.Point(1,90);$script:settingsBody.Size=New-Object Drawing.Size(430,369)
 $script:settingsBody.AutoScroll=$true;$script:preferences.Controls.Add($script:settingsBody)
 $tabs=New-Object Windows.Forms.Panel;$tabs.Location=New-Object Drawing.Point(22,20);$tabs.Size=New-Object Drawing.Size(386,79);$script:settingsBody.Controls.Add($tabs)
 $labels=@('Vessel','Navigation','Sensors','Autopilot','Radar','Display','System','Help');$widths=@(57,86,70,76,58,68,68,48);$x=0;$y=0
 for($i=0;$i -lt $labels.Count;$i++) {
  if($x -and $x+$widths[$i] -gt $tabs.Width){$x=0;$y+=42}
  $tab=Button $tabs $labels[$i] $x $y $widths[$i];$tab.Height=37;$x+=$widths[$i]+5
  if($labels[$i] -ceq 'System') {
   $tab.Add_Click({SaveClick 'System';SystemFixtureLinks})
   if($record.case -ceq 'prototype-system-hidden'){$tab.Hide()}
  }
 }
 if($record.case -ceq 'prototype-system-duplicate'){$duplicate=Button $tabs 'System' 280 42 68;$duplicate.Height=37;$duplicate.Add_Click({SaveClick 'UNSAFE_DUPLICATE'})}
 # The System action must create its content through the actual click callback;
 # reveal/link cases instead start on the already-selected System section.
 if($record.action -cne 'System'){SystemFixtureLinks}
 if($record.case -cin @('prototype-system-wrong-owner','prototype-recovery-wrong-owner')){$script:preferences.TopMost=$true;$script:preferences.Show()}else{$script:preferences.Show($form)}
 $script:surfaces.Add($script:preferences)
}
$started=[datetime]::UtcNow;$timer=New-Object Windows.Forms.Timer;$timer.Interval=100
$timer.Add_Tick({
 if($record.case -ceq 'prototype-moved' -and (Test-Path -LiteralPath (Join-Path $root 'mutate')) -and -not (Test-Path -LiteralPath (Join-Path $root 'mutated'))) {
  $script:surfaces[0].Left+=10;[IO.File]::WriteAllText((Join-Path $root 'mutated'),'surface moved')
 }
 if((Test-Path -LiteralPath (Join-Path $root 'release')) -or ([datetime]::UtcNow-$started).TotalSeconds -gt 25){$form.Close()}})
$form.Add_Shown({
 if($record.case -cin @('maximized-offscreen','partial-offscreen')) {
  $form.Location=New-Object Drawing.Point(-200,-100);$form.Size=New-Object Drawing.Size(1600,1000)
  if($record.case -ceq 'maximized-offscreen'){$form.WindowState='Maximized'}
 }
 if($record.case -ceq 'entirely-offscreen'){$form.Location=New-Object Drawing.Point(-4000,-3000)}
 if($record.case -ceq 'minimized'){$form.WindowState='Minimized'}
 if($record.case -ceq 'return'){$form.FormBorderStyle='None';$form.WindowState='Maximized';$script:full=$true}
 if($prototype) {
  $last=if($record.case -ceq 'prototype-signature'){'AUTO'}else{[string][char]0x2212}
  Surface 'SKAGER chart tools' 150 100 190 64 @('Measure','Waypoint','+',$last)
  if($record.action -cin @('ZoomIn','ZoomOut')) {
   # Shell.cpp: 4+44+44+5+44+44+4 wide; 44-high buttons with 4-DIP top/bottom.
   $tools=$script:surfaces[0];$tools.Size=New-Object Drawing.Size(189,52);$positions=@(4,48,97,141)
   for($i=0;$i -lt $tools.Controls.Count;$i++){$b=$tools.Controls[$i];$b.Location=New-Object Drawing.Point($positions[$i],4);$b.Size=New-Object Drawing.Size(44,44)}
   $target=@($tools.Controls|Where-Object {$_.Text -ceq $(if($record.action -ceq 'ZoomIn'){'+'}else{[string][char]0x2212})})[0]
   $target.Add_Click({param($sender,$event) SaveClick $sender.Text})
   if($record.case -ceq 'prototype-zoom-wrong-owner'){$tools.Owner=$null;$tools.TopMost=$true}
   if($record.case -ceq 'prototype-zoom-wrong-surface'){$tools.Text='Unknown chart tools'}
   if($record.case -ceq 'prototype-zoom-hidden'){$target.Hide()}
   if($record.case -ceq 'prototype-zoom-occluded'){$cover=New-Object Windows.Forms.Panel;$cover.Location=$target.Location;$cover.Size=$target.Size;$tools.Controls.Add($cover);$cover.BringToFront()}
   if($record.case -ceq 'prototype-zoom-replace-on-down'){$target.Add_MouseDown({param($sender,$event) $parent=$sender.Parent;$location=$sender.Location;$size=$sender.Size;$sender.Dispose();$replacement=Button $parent '+' $location.X $location.Y $size.Width;$replacement.Height=$size.Height;$replacement.Add_Click({SaveClick 'UNSAFE_REPLACEMENT'})})}
  }
  Surface 'SKAGER chart orientation' 150 180 68 90 @('North')
  Surface 'SKAGER follow boat' 150 290 142 56 @('Follow boat')
  if($record.case -ceq 'prototype-two-sheets'){
   Surface 'SKAGER preferences' 360 100 432 460 @('Close') -Heading
   Surface 'SKAGER passage' 390 100 398 460 @('Close') -Heading
  }
  if($record.case -ceq 'prototype-anchor'){Surface 'SKAGER anchor watch' 390 100 398 460 @('Close') -Heading}
  if($record.case -ceq 'prototype-pilot'){Surface 'SKAGER autopilot' 390 100 398 460 @('Close') -Heading}
  if($record.case -ceq 'prototype-alerts'){Surface 'SKAGER alerts' 390 100 398 460 @('Close') -Heading}
  if($record.case -ceq 'prototype-preferences'){Surface 'SKAGER preferences' 360 100 432 460 @('Close') -Heading}
  if($record.case -ceq 'prototype-passage'){Surface 'SKAGER passage' 390 100 398 460 @('Close') -Heading}
  if($record.case -cin @('prototype-traffic','prototype-back')){Surface 'SKAGER vessel traffic' 390 100 398 460 @($(if($record.case -ceq 'prototype-back'){'Back'}else{'Close'})) -Heading}
  if($record.case -ceq 'prototype-unknown'){Surface 'Unknown plugin popup' 390 100 240 200 @('Close')}
  if($record.case -ceq 'prototype-wrong-owner'){Surface 'SKAGER passage' 390 100 398 460 @('Close') -Heading -Unowned}
  if($record.case -ceq 'prototype-duplicate'){Surface 'SKAGER chart tools' 390 100 190 64 @('Measure','Waypoint','+',[string][char]0x2212)}
  if($record.case -ceq 'prototype-clipped'){$script:surfaces[0].Left=$form.Right-20}
  if($record.action -cin @('System','RevealInterfaceRecovery','InterfaceRecovery')){SettingsSurface}
 }
 $chart=$panel.RectangleToScreen($panel.ClientRectangle)
 [IO.File]::WriteAllText((Join-Path $root 'ready.json'),(@{pid=$PID;handle=$form.Handle.ToInt64();createdFiletime=[Diagnostics.Process]::GetCurrentProcess().StartTime.ToUniversalTime().ToFileTimeUtc().ToString();chart=@{left=$chart.Left;top=$chart.Top;right=$chart.Right;bottom=$chart.Bottom}}|ConvertTo-Json -Depth 4 -Compress))
 $timer.Start()
 if($record.case -cin @('modal','prototype-modal')){$dialog=New-Object Windows.Forms.Form;$dialog.Text='Unexpected modal';$dialog.Size=New-Object Drawing.Size(300,200);$null=$dialog.ShowDialog($form);$dialog.Dispose()}
})
try{[Windows.Forms.Application]::Run($form)}finally{$timer.Dispose();foreach($surface in $script:surfaces){$surface.Dispose()};$form.Dispose()}

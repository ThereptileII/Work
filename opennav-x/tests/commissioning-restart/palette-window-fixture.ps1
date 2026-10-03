# Disposable harmless desktop only; never loads product, profile or plugins.
[CmdletBinding()]
param([Parameter(Mandatory=$true)][string]$Directory)
Set-StrictMode -Version Latest;$ErrorActionPreference='Stop'
$root=[IO.Path]::GetFullPath($Directory)
if(-not $root.StartsWith([IO.Path]::GetTempPath(),[StringComparison]::OrdinalIgnoreCase) -or [IO.Path]::GetFileName($root) -cnotmatch '^opennav-palette-window-[a-f0-9]{32}$'){throw 'Unique temporary palette fixture required.'}
$record=Get-Content -LiteralPath (Join-Path $root 'fixture.json') -Raw|ConvertFrom-Json
if($record.owner -cne 'OpenNavX.PaletteWindow.Fixture.1' -or $record.palette -cnotin @('XNav','Standard') -or $record.case -cnotin @('normal','duplicate-choice','hidden-choice','replace-choice','wrong-sheet','wrong-detail','unowned-sheet','duplicate-confirm','replace-confirm','obscured-sheet','reveal','hidden-target','duplicate-target','changed-body','no-progress')){throw 'Unknown fixed palette fixture.'}
Add-Type -AssemblyName System.Windows.Forms
Add-Type -TypeDefinition @'
using System;
using System.Runtime.InteropServices;
using System.Windows.Forms;
public static class PaletteLabel {
 [DllImport("user32.dll",CharSet=CharSet.Unicode)] public static extern bool SetWindowTextW(IntPtr h,string s);
}
public class NoScrollPalettePanel:Panel {
 protected override void WndProc(ref Message m){if(m.Msg==0x115)return;base.WndProc(ref m);}
}
'@ -ReferencedAssemblies @('System.Windows.Forms','System.Drawing')
function LabelWindow($Control,[string]$Text){$null=$Control.Handle;$null=[PaletteLabel]::SetWindowTextW($Control.Handle,$Text)}
function Log([string]$Text){[IO.File]::AppendAllText((Join-Path $root 'clicks.txt'),$Text+"`n")}
function Button($Parent,[string]$Text,[int]$X,[int]$Y,[int]$Width=250) {
 $b=New-Object Windows.Forms.Button;$b.Text=$Text;$b.Size=New-Object Drawing.Size($Width,52);$b.Location=New-Object Drawing.Point($X,$Y);$Parent.Controls.Add($b);return $b
}
$form=New-Object Windows.Forms.Form;$form.Text='SKAGER / OpenCPN';$form.StartPosition='Manual';$form.Location=New-Object Drawing.Point(15,15);$form.Size=New-Object Drawing.Size(1100,740)
$script:sheet=$null;$script:cover=$null;$script:drawer=$null;$script:layers=$null
$script:navCase=$record.case -cin @('reveal','hidden-target','duplicate-target','changed-body','no-progress')
$rail=New-Object Windows.Forms.Panel;$rail.Location=New-Object Drawing.Point(5,5);$rail.Size=New-Object Drawing.Size(1060,60);$form.Controls.Add($rail)
$x=0;foreach($label in @('Chart','Passage','Traffic','Energy','Instruments','Anchor','Radar','Settings')) {$b=Button $rail $label $x 0 120;$x+=125}
$panel=New-Object Windows.Forms.Panel;$panel.Location=New-Object Drawing.Point(20,90);$panel.Size=New-Object Drawing.Size(1030,590);$form.Controls.Add($panel);LabelWindow $panel 'SKAGER product page: Display'
function OpenSheet {
 $script:sheet=New-Object Windows.Forms.Form;$script:sheet.Text=$(if($record.case -ceq 'wrong-sheet'){'Another confirmation'}else{'Change chart style'})
 $script:sheet.StartPosition='Manual';$script:sheet.Location=New-Object Drawing.Point(($form.Left+200),($form.Top+210));$script:sheet.Size=New-Object Drawing.Size(620,350)
 $content=New-Object Windows.Forms.Panel;$content.Location=New-Object Drawing.Point(0,0);$content.Size=New-Object Drawing.Size(600,140);$script:sheet.Controls.Add($content)
 $title=New-Object Windows.Forms.Label;$title.Text='Change chart style';$title.Location=New-Object Drawing.Point(15,10);$title.Size=New-Object Drawing.Size(580,35);$content.Controls.Add($title)
 $detail=New-Object Windows.Forms.Label;$detail.Text="Restart SKAGER to apply the chart presentation.`nRoutes, charts and navigation settings are preserved.";$detail.Location=New-Object Drawing.Point(15,55);$detail.Size=New-Object Drawing.Size(580,80);$content.Controls.Add($detail)
 if($record.case -ceq 'wrong-detail'){$detail.Text='Unreviewed palette instructions'}
 $cancel=Button $script:sheet 'Cancel' 15 230 220;$cancel.Add_Click({Log 'UNEXPECTED_CANCEL';$script:sheet.Close()})
 $accept=Button $script:sheet 'Save and restart' 260 230 260;$accept.Add_Click({Log 'confirmed';$script:sheet.Close()})
 if($record.case -ceq 'duplicate-confirm'){$duplicate=Button $script:sheet 'Save and restart' 260 150 260;$duplicate.Add_Click({Log 'UNSAFE_DUPLICATE'})}
 if($record.case -ceq 'replace-confirm'){$accept.Add_MouseDown({param($sender,$e) $parent=$sender.Parent;$sender.Dispose();$b=Button $parent 'Save and restart' 260 230 260;$b.Add_Click({Log 'UNSAFE_REPLACEMENT'})})}
 $script:sheet.Add_Shown({if($record.case -ceq 'obscured-sheet'){$script:cover=New-Object Windows.Forms.Form;$script:cover.StartPosition='Manual';$script:cover.Location=$script:sheet.Location;$script:cover.Size=New-Object Drawing.Size(200,120);$script:cover.TopMost=$true;$script:cover.Show();$script:sheet.Activate()}})
 if($record.case -ceq 'unowned-sheet'){$form.Enabled=$false;$script:sheet.Show();$script:sheet.Activate()}else{$null=$script:sheet.ShowDialog($form)}
}
foreach($palette in @('XNav','Standard')) {
 $caption=$(if($palette -ceq 'XNav'){'SKAGER'}else{'Standard'});$b=Button $panel $caption $(if($palette -ceq 'XNav'){20}else{320}) 40
 $b.Add_Click({param($sender,$e) Log ('selected:'+ $sender.Text);OpenSheet})
 if($palette -ceq $record.palette) {
  if($record.case -ceq 'hidden-choice'){$b.Hide()}
  if($record.case -ceq 'replace-choice'){$b.Add_MouseDown({param($sender,$e) $parent=$sender.Parent;$text=$sender.Text;$sender.Dispose();$b=Button $parent $text 20 40;$b.Add_Click({Log 'UNSAFE_REPLACEMENT'})})}
  if($record.case -ceq 'duplicate-choice'){$null=Button $panel $caption 20 130}
 }
}
function OpenDrawer {
 $script:drawer=New-Object Windows.Forms.Form;$script:drawer.Text='Chart presentation';$script:drawer.FormBorderStyle='None';$script:drawer.ShowInTaskbar=$false;$script:drawer.StartPosition='Manual';$script:drawer.Location=New-Object Drawing.Point(($form.Left+600),($form.Top+120));$script:drawer.Size=New-Object Drawing.Size(440,540)
 $heading=New-Object Windows.Forms.Panel;$heading.Location=New-Object Drawing.Point(0,0);$heading.Size=New-Object Drawing.Size(440,65);$script:drawer.Controls.Add($heading);$close=Button $heading 'Close' 310 5 120;$close.Add_Click({$script:drawer.Close()})
 $body=if($record.case -ceq 'no-progress'){New-Object NoScrollPalettePanel}else{New-Object Windows.Forms.Panel}
 $body.Location=New-Object Drawing.Point(0,70);$body.Size=New-Object Drawing.Size(440,465);$body.AutoScroll=$true;$script:drawer.Controls.Add($body)
 $target=Button $body 'Chart palette preferences' 15 950 390;$target.Add_Click({Log 'preferences';$script:drawer.Close()})
 if($record.case -ceq 'hidden-target'){$target.Hide();$body.AutoScrollMinSize=New-Object Drawing.Size(0,1100)}
 if($record.case -ceq 'duplicate-target'){$null=Button $body 'Chart palette preferences' 15 850 390}
 if($record.case -ceq 'changed-body'){$body.Add_Scroll({param($sender,$e) $sender.Width-=1})}
 $script:drawer.Show($form);$form.Activate()
}
$started=[datetime]::UtcNow;$timer=New-Object Windows.Forms.Timer;$timer.Interval=100
$timer.Add_Tick({if((Test-Path -LiteralPath (Join-Path $root 'release')) -or ([datetime]::UtcNow-$started).TotalSeconds -gt 30){if($script:cover){$script:cover.Close()};if($script:sheet){$script:sheet.Close()};if($script:drawer){$script:drawer.Close()};if($script:layers){$script:layers.Close()};$form.Close()}})
$form.Add_Shown({
 if($script:navCase) {
  $panel.Hide();$script:layers=New-Object Windows.Forms.Form;$script:layers.Text='SKAGER chart layers';$script:layers.FormBorderStyle='None';$script:layers.ShowInTaskbar=$false;$script:layers.StartPosition='Manual';$script:layers.Location=New-Object Drawing.Point(($form.Left+30),($form.Top+150));$script:layers.Size=New-Object Drawing.Size(90,60)
  $layer=Button $script:layers 'Layers' 2 2 85;$layer.Add_Click({Log 'layers';OpenDrawer});$script:layers.Show($form);$form.Activate()
 }
 [IO.File]::WriteAllText((Join-Path $root 'ready.json'),(@{pid=$PID;handle=$form.Handle.ToInt64();createdFiletime=[Diagnostics.Process]::GetCurrentProcess().StartTime.ToUniversalTime().ToFileTimeUtc().ToString()}|ConvertTo-Json -Compress));$timer.Start()
})
try{[Windows.Forms.Application]::Run($form)}finally{foreach($w in @($script:sheet,$script:cover,$script:drawer,$script:layers)){if($w){$w.Dispose()}};$timer.Dispose();$form.Dispose()}

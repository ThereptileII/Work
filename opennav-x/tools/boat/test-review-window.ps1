# Pure policy and compilation checks. No GUI, process, registry, hardware or boat access.
[CmdletBinding()]
param()
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'ReviewWindow.ps1')
$checks=New-Object 'Collections.Generic.List[string]'
function Pass([string]$Name,[scriptblock]$Body){& $Body;$checks.Add($Name)}
function Refuse([string]$Name,[scriptblock]$Body){$rejected=$false;try{$null=& $Body}catch{$rejected=$true};if(-not $rejected){throw ('Unsafe review accepted: '+$Name)};$checks.Add($Name)}
function CopyValue($Value){return ($Value | ConvertTo-Json -Depth 8 | ConvertFrom-Json)}
$now=[datetime]::UtcNow
$exe='C:\XNav\installed\generations\'+('b'*32)+'\app\opencpn.exe'
$job=[pscustomobject]@{action='ReviewWindow';reviewAction='Capture';workspace='C:\XNav';executable=$exe;executableSha256=('c'*64);processId=42;buildCommit=('a'*40);generation=('b'*32)}
$installed=[pscustomobject]@{executable=$exe;state=[pscustomobject]@{current=('b'*32)};ownership=[pscustomobject]@{version='0.4.0-beta2';commit=('a'*40)}}
$build=[pscustomobject]@{test_fixtures=$false;build_purpose='INSTALLED PRODUCT';version='0.4.0-beta2';commit=('a'*40);executable_sha256=('c'*64)}
$launch=[pscustomobject]@{status='passed';action='Launch';mode='--xnav';pid=42;utc=$now.AddMinutes(-1).ToString('o')}
$request=[pscustomobject]@{action='Launch';mode='--xnav';executable=$exe;executableSha256=('c'*64);workspace='C:\XNav'}
foreach($fileName in @('ReviewWindow.ps1','review-window.ps1')) {
  Pass "Parses $fileName without executing environment APIs" {
    $tokens=$null;$errors=$null;$null=[Management.Automation.Language.Parser]::ParseFile((Join-Path $PSScriptRoot $fileName),[ref]$tokens,[ref]$errors)
    if($errors.Count){throw ($errors | Out-String)}
  }
}
Pass 'Native helper compiles without executing Win32 APIs' {Initialize-WindowReviewNative}
foreach($action in Get-WindowReviewActions) {
  Pass "Allows one fixed display action: $action" {$value=CopyValue $job;$value.reviewAction=$action;Assert-WindowReviewPolicy $value $installed $build $launch $request $now}
}
foreach($action in @('AUTO','STBY','STANDBY','TRACK','WIND','AlterCourse','EnableControl','ActivateRoute','CreateRoute','DeleteWaypoint','Import','Export','Plugin','Save','Restart','Legacy','Demo','Click','Key','CtrlShiftV','capture','Capture;AUTO','')) {
  Refuse "Refuses non-reviewed action: $action" {$value=CopyValue $job;$value.reviewAction=$action;Assert-WindowReviewPolicy $value $installed $build $launch $request $now}
}
foreach($field in @('generation','buildCommit','executableSha256','executable','processId')) {
  Refuse "Refuses changed job $field" {$value=CopyValue $job;$value.$field=$(if($field -eq 'processId'){43}else{'unexpected'});Assert-WindowReviewPolicy $value $installed $build $launch $request $now}
}
foreach($fixtures in @($true,'false',$null,0)) {
  Refuse 'Refuses synthetic/ambiguous fixture flag' {$value=CopyValue $build;$value.test_fixtures=$fixtures;Assert-WindowReviewPolicy $job $installed $value $launch $request $now}
}
foreach($field in @('build_purpose','version','commit','executable_sha256')) {
  Refuse "Refuses mismatched build $field" {$value=CopyValue $build;$value.$field='unexpected';Assert-WindowReviewPolicy $job $installed $value $launch $request $now}
}
foreach($field in @('status','action','mode','pid')) {
  Refuse "Refuses mismatched launch $field" {$value=CopyValue $launch;$value.$field=$(if($field -eq 'pid'){43}else{'unexpected'});Assert-WindowReviewPolicy $job $installed $build $value $request $now}
}
foreach($field in @('action','mode','executable','executableSha256','workspace')) {
  Refuse "Refuses mismatched launch request $field" {$value=CopyValue $request;$value.$field='unexpected';Assert-WindowReviewPolicy $job $installed $build $launch $value $now}
}
foreach($at in @($now.AddHours(-5),$now.AddSeconds(1))) {
  Refuse 'Refuses expired/future launch evidence' {$value=CopyValue $launch;$value.utc=$at.ToString('o');Assert-WindowReviewPolicy $job $installed $build $value $request $now}
}
$process=[pscustomobject]@{Id=42;Path=$exe;SessionId=1;MainWindowHandle=123;HasExited=$false;StartTime=$now.AddSeconds(-59)}
Pass 'Exact PID/path/session and audited start time pass without touching a process' {Assert-WindowReviewProcess $process $job $launch 1}
foreach($field in @('Id','Path','SessionId','MainWindowHandle','HasExited','StartTime')) {
  Refuse "Refuses mismatched process $field" {
    $value=CopyValue $process
    $value.$field=switch($field){'Id'{43};'Path'{'C:\Other\opencpn.exe'};'SessionId'{2};'MainWindowHandle'{0};'HasExited'{$true};'StartTime'{$now.AddHours(-1)}}
    Assert-WindowReviewProcess $value $job $launch 1
  }
}
Pass 'All pointer labels resolve to actual current source controls' {
  $root=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..').Replace('\',[IO.Path]::DirectorySeparatorChar))
  $source='';foreach($name in @('Shell.cpp','ProductPanel.cpp','ProductSettings.cpp','Theme.h')){$source+=[IO.File]::ReadAllText((Join-Path $root ('src/ui/'+$name)))}
  foreach($action in Get-WindowReviewActions | Where-Object {$_ -cnotin @('Capture','Resize1280x800','Escape')}) {
    foreach($label in [OpenNavX.ReviewWindowNative]::ActionLabels($action)) {
      if(-not $source.Contains('"'+$label+'"')){throw ('Reviewed label absent from source: '+$label)}
    }
  }
}
foreach($action in @('STBY','AUTO','TRACK','WIND','Save','Click','Key','EnableControl','Escape')) {
  Refuse "Native pointer API cannot resolve unsafe/unrelated action: $action" {[OpenNavX.ReviewWindowNative]::ActionLabels($action)}
}
Pass 'Native helper has no global keyboard/mouse injector or arbitrary command API' {
  $source=[IO.File]::ReadAllText((Join-Path $PSScriptRoot 'ReviewWindowNative.cs'))
  if($source -match 'extern[^;]*(SendInput|keybd_event|mouse_event|SetCursorPos)' -or $source -match 'public static.*(SendMessage|Navigate|ActionKey)'){throw 'Unrestricted/global input API present.'}
}
[pscustomobject]@{status='passed';count=$checks.Count;checks=@($checks);nativeInteropCompiled=$true;nativeActionsExecuted=$false;boatAccess=$false;hardwareCommands=$false} | ConvertTo-Json -Depth 5

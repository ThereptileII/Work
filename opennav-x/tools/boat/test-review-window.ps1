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
foreach($fileName in @('ReviewWindow.ps1','review-window.ps1','../test-display-window-native.ps1','../prototype/capture-reviewed-native.ps1','../../tests/display-review/window-fixture.ps1')) {
  Pass "Parses $fileName without executing environment APIs" {
    $tokens=$null;$errors=$null;$null=[Management.Automation.Language.Parser]::ParseFile((Join-Path $PSScriptRoot $fileName),[ref]$tokens,[ref]$errors)
    if($errors.Count){throw ($errors | Out-String)}
  }
}
Pass 'Native helper compiles without executing Win32 APIs' {Initialize-WindowReviewNative}
Pass 'New native Preferences cases are admitted by the disposable fixture catalog' {
 $tokens=$null;$errors=$null
 $ast=[Management.Automation.Language.Parser]::ParseFile((Join-Path $PSScriptRoot '../../tests/display-review/window-fixture.ps1'),[ref]$tokens,[ref]$errors)
 $catalog=@($ast.FindAll({param($node)
  $node -is [Management.Automation.Language.BinaryExpressionAst] -and
  $node.Operator -eq [Management.Automation.Language.TokenKind]::Cnotin -and
  $node.Left.Extent.Text -ceq '$record.case'
 },$true))
 if($catalog.Count -ne 1){throw 'Fixture case allowlist must be explicit and unique.'}
 $names=@($catalog[0].Right.SafeGetValue())
 foreach($name in @('prototype-preferences','prototype-two-sheets','prototype-anchor','prototype-pilot','prototype-alerts')) {
  if($names -cnotcontains $name){throw ('Native fixture refuses its new test case: '+$name)}
 }
}
$rail=@('Chart','Passage','Traffic','Energy','Instruments','Anchor','Radar','Settings')
Pass 'Prototype shell requires all eight exact sibling controls' {
 if(-not [OpenNavX.ReviewWindowNative]::IsPrototypeNavigation($rail)){throw 'Prototype rail rejected.'}
}
Pass 'Prototype profile button is the only supported ninth sibling' {
 if(-not [OpenNavX.ReviewWindowNative]::IsPrototypeNavigation($rail+@('Vessel profile'))){throw 'Exact profile rail rejected.'}
 foreach($labels in @(($rail+@('Vessel Profile')),($rail+@('Vessel profile','AUTO')),($rail+@('Vessel profile','Vessel profile')))) {
  if([OpenNavX.ReviewWindowNative]::IsPrototypeNavigation($labels)){throw 'Ambiguous extended rail accepted.'}
 }
}
foreach($labels in @(@('Chart','Passage'),($rail+@('AUTO')),(@('Chart')+$rail[0..6]),@('chart','Passage','Traffic','Energy','Instruments','Anchor','Radar','Settings'))) {
 Pass 'Incomplete duplicate extended or case-changed rails do not identify a shell' {
  if([OpenNavX.ReviewWindowNative]::IsPrototypeNavigation($labels)){throw 'Ambiguous shell accepted.'}
 }
}
foreach($spec in @(
 @('OpenNav chart tools',@('Measure','Waypoint','+',[string][char]0x2212),@(),$true),
 @('OpenNav chart orientation',@('North'),@(),$true),
 @('OpenNav chart orientation',@('Course'),@(),$true),
 @('OpenNav follow boat',@('Follow boat'),@(),$true),
 @('OpenNav passage',@(),@('Close'),$true),
 @('OpenNav preferences',@(),@('Close'),$true),
 @('OpenNav anchor watch',@(),@('Close'),$true),
 @('OpenNav autopilot',@(),@('Close'),$true),
 @('OpenNav alerts',@(),@('Close'),$true),
 @('OpenNav source health',@(),@('Close'),$true),
 @('OpenNav source health',@('AUTO'),@('Close'),$false),
 @('OpenNav source health',@(),@('Back'),$false),
 @('OpenNav alerts',@('Acknowledge'),@('Close'),$false),
 @('OpenNav alerts',@(),@('Close','Back'),$false),
 @('OpenNav anchor watch',@('Set anchor'),@('Close'),$false),
 @('OpenNav autopilot',@('Auto'),@('Close'),$false),
 @('OpenNav anchor watch',@(),@('Back'),$false),
 @('OpenNav autopilot',@(),@('Close','Back'),$false),
 @('OpenNav preferences',@('AUTO'),@('Close'),$false),
 @('OpenNav preferences',@(),@('Back'),$false),
 @('OpenNav preferences',@(),@('Close','Back'),$false),
 @('OpenNav vessel traffic',@(),@('Back'),$true),
 @('OpenNav vessel traffic',@(),@('Close'),$true),
 @('Unexpected modal',@(),@('Close'),$false),
 @('OpenNav passage',@('AUTO'),@('Close'),$false),
 @('OpenNav passage',@(),@('Close','Close'),$false),
 @('OpenNav passage',@(),@('Back'),$false),
 @('OpenNav vessel traffic',@(),@('Close','Back'),$false),
 @('OpenNav chart tools',@('Measure','Waypoint','+','+'),@(),$false),
 @('OpenNav chart orientation',@('North','Course'),@(),$false),
 @('OpenNav follow boat',@('Follow boat','STBY'),@(),$false))) {
 Pass ('Fixed owned capture signature '+$spec[0]+' / '+$spec[3]) {
  if([OpenNavX.ReviewWindowNative]::IsPrototypeSurface($spec[0],$spec[1],$spec[2]) -ne $spec[3]){throw 'Capture signature mismatch.'}
 }
}
foreach($spec in @(@('Navigation','Chart'),@('Route','Passage'),@('AIS','Traffic'),@('Instruments','Instruments'),@('Menu','Settings'))) {
 Pass ('Fixed prototype rail mapping '+$spec[0]) {
  if([OpenNavX.ReviewWindowNative]::PrototypeRailLabel($spec[0]) -cne $spec[1]){throw 'Prototype action mapping changed.'}
 }
}
foreach($action in @('AUTO','STBY','TRACK','WIND','Set AISStream key','Enabled','End navigation','Plot new passage')) {
 Refuse ('No prototype actuator or mutation action '+$action) {[OpenNavX.ReviewWindowNative]::PrototypeRailLabel($action)}
}
foreach($action in Get-WindowReviewActions) {
  Pass "Allows one fixed display action: $action" {$value=CopyValue $job;$value.reviewAction=$action;Assert-WindowReviewPolicy $value $installed $build $launch $request $now}
}
foreach($action in @('AUTO','STBY','STANDBY','TRACK','WIND','AlterCourse','EnableControl','ActivateRoute','CreateRoute','DeleteWaypoint','Import','Export','Plugin','Save','Restart','RestartLegacy','RestartSafe','RestartXNav','Legacy','Demo','Click','Key','CtrlShiftV','capture','Capture;AUTO','')) {
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
  $source+=[IO.File]::ReadAllText((Join-Path $root 'src/integration/OpenCPNIntegration.cpp'))
  foreach($action in Get-WindowReviewActions | Where-Object {$_ -cnotin @('Capture','Resize1280x800','Escape','PanRight','SelectFirstVisibleWaypoint','SelectFirstVisibleAis')}) {
    foreach($label in [OpenNavX.ReviewWindowNative]::ActionLabels($action)) {
      if(-not $source.Contains('"'+$label+'"')){throw ('Reviewed label absent from source: '+$label)}
    }
  }
}
$chartData=[pscustomobject]@{build_commit=('a'*40);build_purpose='INSTALLED PRODUCT';data_mode='OPENCPN selected navigation';ui_page='Navigation';runtime=[pscustomobject]@{display=[pscustomobject]@{route_creation_active=$false;chart_region=[pscustomobject]@{x=80;y=100;width=900;height=600}}}}
Pass 'Fresh source-identified installed chart maps exact pixel rectangle' {
  $r=Convert-WindowReviewChart $chartData ('a'*40) $now.AddSeconds(-1) $now
  if($r.Left -ne 80 -or $r.Top -ne 100 -or $r.Right -ne 980 -or $r.Bottom -ne 700){throw 'Chart rectangle changed.'}
}
foreach($field in @('build_commit','build_purpose','data_mode','ui_page')) {
  Refuse "Pan refuses changed diagnostic $field" {$v=CopyValue $chartData;$v.$field='unexpected';Convert-WindowReviewChart $v ('a'*40) $now $now}
}
foreach($at in @($now.AddSeconds(-6),$now.AddSeconds(1))) {
  Refuse 'Pan refuses old/future diagnostic geometry' {Convert-WindowReviewChart $chartData ('a'*40) $at $now}
}
foreach($value in @($true,'false',$null,0)) {
  Refuse 'Pan refuses active/ambiguous route editing' {$v=CopyValue $chartData;$v.runtime.display.route_creation_active=$value;Convert-WindowReviewChart $v ('a'*40) $now $now}
}
foreach($value in @(-1,0,99,32769,'900',900.5,$null)) {
  Refuse 'Pan refuses malformed/hidden chart geometry' {$v=CopyValue $chartData;$v.runtime.display.chart_region.width=$value;Convert-WindowReviewChart $v ('a'*40) $now $now}
}
Pass 'Display controls retain exact source page scopes; orientation requires navigation chart tools' {
  if([OpenNavX.ReviewWindowNative]::ActionContext('Display') -cne 'OpenNav product page: Settings' -or
     [OpenNavX.ReviewWindowNative]::ActionContext('ToggleFullscreen') -cne 'OpenNav product page: Display' -or
     [OpenNavX.ReviewWindowNative]::ActionContext('ToggleOrientation') -cne 'Navigation chart tools'){throw 'Display action scope changed.'}
}
foreach($action in @('Fullscreen','North','Course','SetOrientation','SetResolution','SetDpi','Brightness','EnableControl')) {
  Refuse "No unreviewed display/system action: $action" {[OpenNavX.ReviewWindowNative]::ActionContext($action)}
}
foreach($action in @('STBY','AUTO','TRACK','WIND','Save','Click','Key','EnableControl','Escape')) {
  Refuse "Native pointer API cannot resolve unsafe/unrelated action: $action" {[OpenNavX.ReviewWindowNative]::ActionLabels($action)}
}
Pass 'Native helper has no global keyboard/mouse injector or arbitrary command API' {
  $source=[IO.File]::ReadAllText((Join-Path $PSScriptRoot 'ReviewWindowNative.cs'))
  if($source -match 'extern[^;]*(SendInput|keybd_event|mouse_event|SetCursorPos)' -or $source -match 'public static.*(SendMessage|Navigate|ActionKey)'){throw 'Unrestricted/global input API present.'}
}

function Row([long]$Handle,[string]$Label,[int]$Top=100,[int]$Left=100,[bool]$Enabled=$true,[bool]$Visible=$true,[bool]$DirectChild=$true) {
  $row=New-Object OpenNavX.ReviewWindowNative+SelectionRow
  $row.Handle=$Handle;$row.Label=$Label;$row.Top=$Top;$row.Left=$Left
  $row.Enabled=$Enabled;$row.Visible=$Visible;$row.DirectChild=$DirectChild
  return $row
}
$waypoint='SelectFirstVisibleWaypoint';$ais='SelectFirstVisibleAis'
$wpPage=[OpenNavX.ReviewWindowNative]::SelectionPage($waypoint)
$aisPage=[OpenNavX.ReviewWindowNative]::SelectionPage($ais)
Pass 'Waypoint selection uses the first fully visible row by screen order, not enumeration or user name' {
  $rows=@((Row 1 'Late / mark' 300),(Row 2 'Early / in route' 100),(Row 3 'Other / mark' 100 200))
  if([OpenNavX.ReviewWindowNative]::ChooseSelectionRow($waypoint,$wpPage,$rows).Handle -ne 2){throw 'Unexpected first row'}
}
Pass 'Disabled, clipped and indirect controls cannot become list selections' {
  $rows=@((Row 1 'Disabled / mark' 0 0 $false),(Row 2 'Clipped / mark' 0 0 $true $false),
    (Row 3 'Nested / mark' 0 0 $true $true $false),(Row 4 'Usable / mark'))
  if([OpenNavX.ReviewWindowNative]::ChooseSelectionRow($waypoint,$wpPage,$rows).Handle -ne 4){throw 'Unsafe selection'}
}
foreach($label in @('Vessel / Active','Vessel / Active / Under way using engine','Vessel / Inactive','Vessel / Lost',
  'Vessel / Position doubtful','Beacon / Active distress beacon','Beacon / Distress beacon testing',
  'Vessel / Active / Navigation status unavailable / ALARM','Name / with / separator / Lost','Åland / Active / Förtöjd')) {
  Pass "Read-only AIS detail admits current upstream health/status label: $label" {
    if([OpenNavX.ReviewWindowNative]::ChooseSelectionRow($ais,$aisPage,@((Row 1 $label))).Handle -ne 1){throw 'Reviewed AIS row was not selected'}
  }
}
foreach($label in @('Refresh target list','Show / hide AIS on chart','AUTO','TRACK','Create waypoint at chart center','GO TO',
  'Edit waypoint','Delete waypoint','Open Legacy OpenCPN','Safe Mode','Refresh catalog','',"vessel / Active`nAUTO",('X'*2047))) {
  Pass "Fixed command or malformed caption cannot be a read-only list row: $($label.Substring(0,[Math]::Min(40,$label.Length)))" {
    if([OpenNavX.ReviewWindowNative]::IsSelectionLabel($waypoint,$label) -or [OpenNavX.ReviewWindowNative]::IsSelectionLabel($ais,$label)){throw 'Command/malformed caption accepted'}
  }
}
foreach($page in @('OpenNav product page: Autopilot','OpenNav product page: Routes','OpenNav product page: Waypoint detail',$aisPage,'')) {
  Refuse 'Waypoint selection requires the exact list page even if another page has a matching caption' {
    [OpenNavX.ReviewWindowNative]::ChooseSelectionRow($waypoint,$page,@((Row 1 'Example / mark')))
  }
}
Refuse 'AIS selection refuses a waypoint page' {[OpenNavX.ReviewWindowNative]::ChooseSelectionRow($ais,$wpPage,@((Row 1 'Vessel / Active')))}
Refuse 'No observed rows means unavailable, never an invented waypoint or AIS target' {[OpenNavX.ReviewWindowNative]::ChooseSelectionRow($waypoint,$wpPage,@())}
Refuse 'Duplicate native control identities are ambiguous' {[OpenNavX.ReviewWindowNative]::ChooseSelectionRow($waypoint,$wpPage,@((Row 1 'A / mark'),(Row 1 'B / mark' 200)))}
Refuse 'Overlapping first native rows are ambiguous' {[OpenNavX.ReviewWindowNative]::ChooseSelectionRow($waypoint,$wpPage,@((Row 1 'A / mark'),(Row 2 'B / mark')))}
Refuse 'A null row is invalid' {[OpenNavX.ReviewWindowNative]::ChooseSelectionRow($waypoint,$wpPage,@($null))}
Refuse 'An invalid native HWND is refused' {[OpenNavX.ReviewWindowNative]::ChooseSelectionRow($waypoint,$wpPage,@((Row 0 'A / mark')))}
Refuse 'Bounded row inventory refuses unbounded input' {[OpenNavX.ReviewWindowNative]::ChooseSelectionRow($waypoint,$wpPage,(New-Object 'OpenNavX.ReviewWindowNative+SelectionRow[]' 4097))}
foreach($action in @('Select','SelectAisByName','SelectWaypointById','RestartLegacy','RestartSafe','RestartXNav','GO TO','SelectFirstVisibleAIS')) {
  Refuse "No arbitrary row or mode-restart native action: $action" {[OpenNavX.ReviewWindowNative]::SelectionPage($action)}
}
Pass 'Dynamic selection grammar matches the reviewed page/source boundaries' {
  $root=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..').Replace('\',[IO.Path]::DirectorySeparatorChar))
  $panel=[IO.File]::ReadAllText((Join-Path $root 'src/ui/ProductPanel.cpp'))
  $bridge=[IO.File]::ReadAllText((Join-Path $root 'src/integration/NavigationObjects.cpp'))
  foreach($literal in @('" / in route"','" / mark"','ShowPage(ProductPage::WaypointDetail','ShowAis(id, mode_)')) {
    if(-not $panel.Contains($literal)){throw 'Actual read-only row callback/label changed; review selection boundaries'}
  }
  foreach($literal in @('"Lost"','"Position doubtful"','"Active"','"Inactive"','"Active distress beacon"','"Distress beacon testing"')) {
    if(-not $bridge.Contains($literal)){throw 'Actual AIS status grammar changed; review selection boundaries'}
  }
}
[pscustomobject]@{status='passed';count=$checks.Count;checks=@($checks);nativeInteropCompiled=$true;nativeActionsExecuted=$false;boatAccess=$false;hardwareCommands=$false} | ConvertTo-Json -Depth 5

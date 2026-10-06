# Inert policy/selector fixtures. Compiles the actual P/Invoke implementation;
# never calls user32, schedules a task, starts a product or opens a COM port.
[CmdletBinding()]
param([switch]$PortableContracts,[switch]$IsolatedLocal)
Set-StrictMode -Version Latest;$ErrorActionPreference='Stop'
if([Environment]::OSVersion.Platform -ne 'Win32NT' -and -not $PortableContracts){throw 'Use explicit portable contract mode.'}
if([Environment]::OSVersion.Platform -eq 'Win32NT' -and $env:GITHUB_ACTIONS -cne 'true' -and -not $IsolatedLocal){throw 'Use disposable native CI or explicit isolated fixture mode.'}
. (Join-Path $PSScriptRoot 'ManualPilotUi.ps1')
Initialize-ManualPilotUiNative
$checks=0
function Check([scriptblock]$Call){$null=& $Call;$script:checks++}
function Refuse([scriptblock]$Call){$failed=$false;try{$null=& $Call}catch{$failed=$true};if(-not $failed){throw 'Unsafe manual UI operation accepted'};$script:checks++}
function Same($A,$B){if($A -cne $B){throw "Fixture mismatch: $A / $B"}}
function Clone($V){return $V|ConvertTo-Json -Depth 32|ConvertFrom-Json}
foreach($file in @('ManualPilotUi.ps1','manual-pilot-ui.ps1','Common.ps1','InteractiveJob.ps1')){
 $tokens=$null;$errors=$null;$null=[Management.Automation.Language.Parser]::ParseFile((Join-Path $PSScriptRoot $file),[ref]$tokens,[ref]$errors)
 Check {if($errors.Count){throw ($errors|Out-String)}}
}
$actions=@('ScrollUp','ScrollDown','OpenSettings','OpenPilot','PilotTab','PilotConnection','BackToPilot','Advanced','OpenIdentity','RefreshIdentity','Permit','DisplayOnly','AcceptPermission','Enable','Disable','AcceptEnable','Auto','AcceptAuto','Standby','Minus1','Plus1','Minus10','Plus10','SetInterface','SetName','SaveIdentity','CancelIdentity','CancelPermission','CancelEnable','CancelAuto')
foreach($action in $actions){
 $target=[OpenNavX.ManualPilotUiNative]::Target($action)
 $modal=if($target[0] -cin @('ST4000 translator identity','Permit manual pilot commands?','Enable physical pilot control?','Request AUTO')){$target[0]}else{''}
 $row=New-Object OpenNavX.ManualPilotUiNative+Control
 $row.Handle=42;$row.Parent=24;$row.Context=$target[0];$row.Label=$target[1];$row.Class=if($action -cin @('SetName','SetInterface')){'Edit'}else{'wxWindowNR'}
 $row.Enabled=$true;$row.Visible=$true;$row.Contained=$true
 Check {Same ([OpenNavX.ManualPilotUiNative]::Choose($action,$modal,@($row))).Handle 42}
 Refuse {[OpenNavX.ManualPilotUiNative]::Choose($action,'Unrelated permission dialog',@($row))}
 Refuse {[OpenNavX.ManualPilotUiNative]::Choose($action,$modal,@($row,$row))}
 foreach($property in @('Enabled','Visible','Contained')){$row.$property=$false;Refuse {[OpenNavX.ManualPilotUiNative]::Choose($action,$modal,@($row))};$row.$property=$true}
 $row.Context='wrong owner';Refuse {[OpenNavX.ManualPilotUiNative]::Choose($action,$modal,@($row))}
 # Original read-only native allowlist must not acquire physical/manual actions.
 if($action -cnotin @('OpenSettings','OpenPilot')){Refuse {[OpenNavX.ReviewWindowNative]::ActionLabels($action)}}
}
foreach($action in @('Track','Wind','SendNmea','WM_COMMAND','Click','SetText','auto')){Refuse {[OpenNavX.ManualPilotUiNative]::Target($action)}}
foreach($title in @('SKAGER chart tools','SKAGER chart orientation','SKAGER chart layers','SKAGER follow boat')){
 Check {Same ([OpenNavX.ManualPilotUiNative]::IsPassiveChartSurface($title)) $true}
 Refuse {[OpenNavX.ManualPilotUiNative]::Target($title)}
}
foreach($title in @('SKAGER preferences','SKAGER chart tools extra','Unreviewed overlay')){Check {Same ([OpenNavX.ManualPilotUiNative]::IsPassiveChartSurface($title)) $false}}
$name='c0508700e76004d2';$nonce='a'*32
Check {Assert-ManualPilotUiAction 'Observe' '' $nonce}
Check {Assert-ManualPilotUiAction 'SetName' $name $nonce}
foreach($case in @(@('Observe','extra',$nonce),@('SetName','0000000000000000',$nonce),@('SetInterface','COM9',$nonce),@('Standby','',('../'+'a'*32)))){Refuse {Assert-ManualPilotUiAction $case[0] $case[1] $case[2]}}
$identities=[OpenNavX.ManualPilotUiNative]::ParseIdentities("COM8 / NAME $name / address 204`n")
Check {Same $identities.Length 1;Same $identities[0].Name $name;Same $identities[0].Address 204}
foreach($text in @("COM9 / NAME $name / address 204","COM8 / NAME $name / address 254",'COM8 / NAME 0000000000000000 / address 204',"Fake COM8 / NAME $name / address 204")){Check {Same ([OpenNavX.ManualPilotUiNative]::ParseIdentities($text)).Length 0}}
$now=[datetime]::UtcNow
$p=[pscustomobject]@{enabled=$false;serial_session_enabled=$false;configured_permission=$false;simulated=$false;track_capability=$false;wind_capability=$false;output_unavailable=$false;fresh=$true;source="ST4000 / NMEA2000 / COM8/NAME-$name/source-204";mode='STANDBY';feedback_sequence='3';connection_epoch='7';command_id='0';command_state='None';control_capability=$true;
 discovery=[pscustomobject]@{verified_identities=1;identity_conflicts=0;traffic_limit_exceeded=$false};receive_diagnostics=[pscustomobject]@{sources=@([pscustomobject]@{interface='COM8';pgn=60928;address=204;age_ms='20'})}}
$diag=[pscustomobject]@{build_commit=('b'*40);build_purpose='INSTALLED PRODUCT';data_mode='OPENCPN selected navigation';xnav_hardware_output_policy='manual-commissioning';xnav_manual_control_contract=1;test_fixtures=$false;runtime=[pscustomobject]@{test_fixtures=$false;pilot=$p;display=[pscustomobject]@{route_creation_active=$false};replay=[pscustomobject]@{active=$false}}}
Check {Assert-ManualPilotUiDiagnostics $diag ('b'*40) $now $now.AddSeconds(-10) $now}
Refuse {Assert-ManualPilotUiDiagnostics $diag ('b'*40) $now.AddSeconds(-6) $now.AddSeconds(-10) $now}
Refuse {Assert-ManualPilotUiDiagnostics $diag ('b'*40) $now.AddSeconds(-11) $now.AddSeconds(-10) $now}
Refuse {Assert-ManualPilotUiDiagnostics $diag ('c'*40) $now $now.AddSeconds(-10) $now}
foreach($field in @('simulated','track_capability','wind_capability','output_unavailable')){$bad=Clone $diag;$bad.runtime.pilot.$field=$true;Refuse {Assert-ManualPilotUiDiagnostics $bad ('b'*40) $now $now.AddSeconds(-10) $now}}
foreach($field in @('test_fixtures','xnav_manual_control_contract','data_mode')){$bad=Clone $diag;$bad.$field='unexpected';Refuse {Assert-ManualPilotUiDiagnostics $bad ('b'*40) $now $now.AddSeconds(-10) $now}}
$bad=Clone $diag;$bad.runtime.test_fixtures=$true;Refuse {Assert-ManualPilotUiDiagnostics $bad ('b'*40) $now $now.AddSeconds(-10) $now}
$binding=[pscustomobject]@{interface='COM8';name=$name;permission='display-only'}
$snapshot=[pscustomobject]@{Identities=$identities;InterfaceValue='COM8';NameValue=$name}
Check {Assert-ManualPilotUiState SetName $name $p $binding $snapshot '' ''}
Check {Assert-ManualPilotUiState SaveIdentity '' $p $binding $snapshot '' ''}
foreach($field in @('interface','pgn','address','age_ms')){
 $bad=Clone $p;$bad.receive_diagnostics.sources[0].$field=$(switch($field){interface{'COM9'} pgn{127250} address{205} age_ms{'30001'}})
 Refuse {Assert-ManualPilotUiState SetName $name $bad $binding $snapshot '' ''}
}
$badSnapshot=Clone $snapshot;$badSnapshot.Identities=@();Refuse {Assert-ManualPilotUiState SetName $name $p $binding $badSnapshot '' ''}
$badSnapshot=Clone $snapshot;$badSnapshot.NameValue='0000000000000000';Refuse {Assert-ManualPilotUiState SaveIdentity '' $p $binding $badSnapshot '' ''}
Check {Assert-ManualPilotUiState Permit '' $p $binding $snapshot '' ''}
$bad=Clone $p;$bad.fresh=$false;Refuse {Assert-ManualPilotUiState Permit '' $bad $binding $snapshot '' ''}
$binding.permission='manual';$p.configured_permission=$true
Check {Assert-ManualPilotUiState Enable '' $p $binding $snapshot '7' ''}
Refuse {Assert-ManualPilotUiState Enable '' $p $binding $snapshot '6' ''}
$p.enabled=$true;$p.serial_session_enabled=$true
Refuse {Assert-ManualPilotUiState Enable '' $p $binding $snapshot '7' ''}
Check {Assert-ManualPilotUiState Disable '' $p $binding $snapshot '' ''}
foreach($action in @('Standby','Auto','AcceptAuto')){Check {Assert-ManualPilotUiState $action '' $p $binding $snapshot '7' '0'}}
Refuse {Assert-ManualPilotUiState Plus1 '' $p $binding $snapshot '7' '0'}
$p.mode='AUTO'
foreach($action in @('Minus1','Plus1','Minus10','Plus10')){Check {Assert-ManualPilotUiState $action '' $p $binding $snapshot '7' '0'}}
foreach($field in @('fresh','enabled','serial_session_enabled','configured_permission','control_capability')){$bad=Clone $p;$bad.$field=$false;Refuse {Assert-ManualPilotUiState Plus1 '' $bad $binding $snapshot '7' '0'}}
Refuse {Assert-ManualPilotUiState Plus1 '' $p $binding $snapshot '7' '1'}
$p.command_state='Pending';Refuse {Assert-ManualPilotUiState Plus1 '' $p $binding $snapshot '7' '0'}
Check {Assert-ManualPilotUiState Standby '' $p $binding $snapshot '7' '0'}
foreach($source in @('ST4000 / NMEA2000 / COM8/NAME-c0508700e76004d3/source-204',"ST4000 / NMEA2000 / COM8/NAME-$name/source-254",'Heading / source 204')){$bad=Clone $p;$bad.source=$source;Refuse {Assert-ManualPilotUiState Standby '' $bad $binding $snapshot '7' '0'}}
# Exercise the real live profile wrapper and existing preservation policy on
# inert files: permission normalization must never mask opaque/unknown changes.
$fixtureRoot=Join-Path ([IO.Path]::GetTempPath()) ('manual-ui-profile-'+[guid]::NewGuid().ToString('N'))
$null=New-Item -ItemType Directory $fixtureRoot
$originalLocalPath=${function:Assert-LocalPath}
function Assert-LocalPath([string]$Path){$p=[IO.Path]::GetFullPath($Path);if(-not $p.StartsWith($fixtureRoot+[IO.Path]::DirectorySeparatorChar,[StringComparison]::Ordinal)){throw 'Escaped inert profile fixture'};return $p}
try {
 $alpha='OpenNavXSettings 1\n"battery" "existing source"\n"curve" "opaque calibration"\n'
 $manual=$alpha+'"pilot.interface" "COM8"\n"pilot.name" "c0508700e76004d2"\n"pilot.permission" "manual"\n'
 $connection='0;0;;0;1;COM8;115200;0;1;0;;0;;0;0;1;0;1;Gateway;0;;0'
 $text="[Settings]`r`nPersistActiveRoute=0`r`nActiveRoute=`r`n[Settings/NMEADataSource]`r`nDataConnections=$connection`r`n[Settings/GlobalState]`r`nFrameWinX=1024`r`n[OpenNav]`r`nAlphaSettings=$alpha`r`n"
 $output=Join-Path $fixtureRoot 'output.ini';$current=Join-Path $fixtureRoot 'opencpn.ini'
 $encoding=New-Object Text.UTF8Encoding($false)
 [IO.File]::WriteAllText($output,$text,$encoding)
 $live=$text.Replace($alpha,$manual).Replace('FrameWinX=1024','FrameWinX=1280')
 [IO.File]::WriteAllText($current,$live,$encoding)
 $v=[pscustomobject]@{paths=[pscustomobject]@{directory=$fixtureRoot};context=[pscustomobject]@{profile=$fixtureRoot}}
 $beforeHash=Get-Digest $current
 Check {Same (Assert-ManualPilotUiProfile $v).permission 'manual';Same (Get-Digest $current) $beforeHash}
 foreach($bad in @($live.Replace('opaque calibration','changed'),$live.Replace('ActiveRoute=','ActiveRoute=unexpected-route'),($live+"UnknownLiveField=1`r`n"))){
  [IO.File]::WriteAllText($current,$bad,$encoding);$badHash=Get-Digest $current
  Refuse {Assert-ManualPilotUiProfile $v};Check {Same (Get-Digest $current) $badHash}
 }
 # Same durable writer as the action journal: reused nonce cannot replace an
 # uncertain first intent, even if no result receipt exists.
 $intent=Join-Path $fixtureRoot ('ui-'+$nonce+'-intent.json')
 Write-Record $intent @{action='Standby';retryAllowed=$false}
 Refuse {Write-Record $intent @{action='Auto'}}
 Check {Same (Read-Record $intent).action 'Standby'}
}finally{Set-Item Function:Assert-LocalPath $originalLocalPath;Remove-Item -LiteralPath $fixtureRoot -Recurse -Force}
$nativeInput=$false
if([Environment]::OSVersion.Platform -eq 'Win32NT') {
 Add-Type -Path (Join-Path $PSScriptRoot 'ManualPilotUiFixture.cs')
 $oldDpi=[OpenNavX.ReviewWindowNative]::SetThreadDpiAwarenessContext([IntPtr](-4));$fixture=$null
 try {
  if($oldDpi -eq [IntPtr]::Zero){throw 'Native fixture requires per-monitor DPI context.'}
  $fixture=New-Object OpenNavX.ManualPilotUiFixture
  $processId=[Diagnostics.Process]::GetCurrentProcess().Id
  Check {$snapshot=[OpenNavX.ManualPilotUiNative]::Observe($fixture.Frame,$processId);Same ([OpenNavX.ManualPilotUiNative]::Choose('Advanced',$snapshot.Modal,$snapshot.Controls)).Label 'Advanced connection setup'}
  foreach($title in @('SKAGER chart tools','SKAGER chart orientation','SKAGER chart layers','SKAGER follow boat')){
   $fixture.ShowOverlay($title,$false)
   Check {$snapshot=[OpenNavX.ManualPilotUiNative]::Observe($fixture.Frame,$processId);if(@($snapshot.Controls|Where-Object {$_.Context -ceq $title}).Count){throw 'Passive overlay became an action root'}}
  }
  $fixture.ShowOverlay('Unreviewed overlay',$false)
  Refuse {[OpenNavX.ManualPilotUiNative]::Observe($fixture.Frame,$processId)}
  $fixture.ShowOverlay('SKAGER chart tools',$true)
  Refuse {[OpenNavX.ManualPilotUiNative]::Observe($fixture.Frame,$processId)}
  $fixture.ShowOverlay('SKAGER chart tools',$false)
  Check {[OpenNavX.ManualPilotUiNative]::Act($fixture.Frame,$processId,'Advanced','');$fixture.Pump();Same $fixture.Clicks 1}
  $fixture.Disable()
  Refuse {[OpenNavX.ManualPilotUiNative]::Act($fixture.Frame,$processId,'Advanced','')}
  Check {Same $fixture.Clicks 1}
  $nativeInput=$true
 }finally{if($fixture){$fixture.Dispose()};if($oldDpi -ne [IntPtr]::Zero){$null=[OpenNavX.ReviewWindowNative]::SetThreadDpiAwarenessContext($oldDpi)}}
}
[pscustomobject]@{status='passed';checks=$checks;nativeInertClick=$nativeInput;scope='Actual C# selector/action policy and PS runtime guards; P/Invoke compiled; native Windows additionally creates and clicks its own inert fixture; no physical/product input';nativeWindows=([Environment]::OSVersion.Platform -eq 'Win32NT')}|ConvertTo-Json

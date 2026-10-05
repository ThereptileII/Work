# Pure policy, actual wire/journal and native compilation tests. No live window,
# process launch, task, marine connection or installed profile is touched.
[CmdletBinding()]
param()
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'RestartWindowReview.ps1')
Initialize-RestartNative
Initialize-RestartWindowNative
$script:checks=0
function Check([bool]$Okay,[string]$Name){if(-not $Okay){throw ('FAILED: '+$Name)};$script:checks++}
function Refuse([scriptblock]$Body,[string]$Name){$failed=$false;try{$null=& $Body}catch{$failed=$true};Check $failed $Name}
function CopyValue($Value){return ($Value|ConvertTo-Json -Depth 12|ConvertFrom-Json)}
$now=[datetime]::UtcNow
$session=[pscustomobject]@{session=('a'*64);recordSha256=('b'*64);windowsSessionId='1';executable='C:\XNav\app\opencpn.exe';executableSha256=('c'*64);
 helper='C:\XNav\app\opennav-restart.exe';helperSha256=('d'*64);workingDirectory='C:\XNav\app';path='C:\XNav\app;C:\Windows\System32;C:\Windows'}
$parent=[pscustomobject]@{pid='42';createdFiletime='133000000000000000'}
$arm=[pscustomobject]@{owner=$script:RestartOwner;session=$session.session;recordSha256=$session.recordSha256;
 parentPid=$parent.pid;parentCreatedFiletime=$parent.createdFiletime;mode='--legacy';createdUtc=$now.AddSeconds(-10).ToString('o')}
$ready=[pscustomobject]@{owner=$script:RestartOwner;session=$session.session;recordSha256=$session.recordSha256;parent=$parent;
 broker=[pscustomobject]@{pid='43';createdFiletime='133000000001000000'};mode='--legacy';createdUtc=$now.AddSeconds(-5).ToString('o')}
Assert-RestartReady $ready $arm $session $now;Check $true 'exact listening identity accepted'
foreach($field in @('owner','session','recordSha256','mode')) {
 $bad=CopyValue $ready;$bad.$field='wrong';Refuse {Assert-RestartReady $bad $arm $session $now} ('changed ready '+$field)
 $bad=CopyValue $arm;$bad.$field='wrong';Refuse {Assert-RestartReady $ready $bad $session $now} ('changed Arm '+$field)
}
foreach($field in @('pid','createdFiletime')) {
 $bad=CopyValue $ready;$bad.parent.$field='99';Refuse {Assert-RestartReady $bad $arm $session $now} ('changed parent '+$field)
 $bad=CopyValue $ready;$bad.broker.$field='0';Refuse {Assert-RestartReady $bad $arm $session $now} ('invalid broker '+$field)
}
foreach($time in @($now.AddSeconds(1),$now.AddSeconds(-90),$now.AddHours(-1))) {
 $bad=CopyValue $ready;$bad.createdUtc=$time.ToString('o');Refuse {Assert-RestartReady $bad $arm $session $now} 'expired/future ready refused'
}
$bad=CopyValue $arm;$bad.createdUtc=$now.AddSeconds(-60).ToString('o');Refuse {Assert-RestartReady $ready $bad $session $now} 'late broker readiness refused'
foreach($mode in @('--xnav','--legacy','--safe-mode')) {
 Assert-RestartWindowAction 'ReviewRestartChild' $mode 'Capture';Check $true ('child capture '+$mode)
 Assert-RestartWindowAction 'ReviewRestartChild' $mode 'Resize1280x800';Check $true ('child resize '+$mode)
 foreach($unsafe in @('AUTO','STBY','EnableControl','ActivateRoute','RestartXNav','RequestMode','Click','Key','Launch','Arm','Collect','Capture;AUTO','')) {
  Refuse {Assert-RestartWindowAction 'ReviewRestartChild' $mode $unsafe} ('unsafe display action '+$mode+' '+$unsafe)
 }
}
foreach($action in Get-WindowReviewActions) {Assert-RestartWindowAction 'ReviewRestartChild' '--xnav' $action;Check $true ('same display-only XNav action '+$action)}
foreach($mode in @('--legacy','--safe-mode')) {Refuse {Assert-RestartWindowAction 'ReviewRestartChild' $mode 'PilotView'} 'Legacy does not use XNav actions'}
foreach($bad in @('--demo','xnav','','--xnav --legacy')) {Refuse {Assert-RestartWindowAction 'RequestGuardedMode' $bad 'RequestMode'} 'unknown mode refused'}
foreach($from in @('--xnav','--legacy','--safe-mode')) {
 foreach($to in @('--xnav','--legacy','--safe-mode')) {
  if($from -ceq '--xnav' -or $to -ceq '--xnav') {$caption=[OpenNavX.RestartWindowNative]::Caption($from,$to);Check ($caption.Length -gt 0) 'source-reviewed mode path'}
  else {Refuse {[OpenNavX.RestartWindowNative]::Caption($from,$to)} 'no invented Legacy/Safe-to-Legacy/Safe path'}
 }
}
# Palette choice is explicit, case-sensitive, XNav-only, and never an ordinary
# display action. Check actual policy methods, not only source spelling.
foreach($palette in @('XNav','Standard')) {
 Assert-RestartWindowAction 'RequestGuardedMode' '--xnav' 'RequestChartPalette';Check $true 'guarded palette request'
 Check ([OpenNavX.RestartWindowNative]::PaletteCaption($palette) -ceq $(if($palette -ceq 'XNav'){'SKAGER'}else{'Standard'})) 'actual fixed source palette caption'
 $a=CopyValue $arm;$r=CopyValue $ready;$a.mode='--xnav';$r.mode='--xnav';$a.createdUtc=$now.AddSeconds(-10).ToString('o');$r.createdUtc=$now.AddSeconds(-5).ToString('o')
 $a|Add-Member chartPalette $palette;$r|Add-Member chartPalette $palette
 Assert-RestartReady $r $a $session $now;Check $true 'same immutable palette arm/readiness'
 $r.chartPalette=$(if($palette -ceq 'XNav'){'Standard'}else{'XNav'})
 Refuse {Assert-RestartReady $r $a $session $now} 'opposite readiness choice refused'
 $before=@{'Settings/Foo'='1';'OpenNav/InterfaceMode'='xnav';'OpenNav/ChartPresentationV1'='XNav'}
 $after=$before.Clone();$after['OpenNav/ChartPresentationV1']=$palette
 $null=Assert-RestartIniDelta $before $after '--xnav' $palette;Check $true 'exact chosen palette delta/no-delta accepted'
 $after['OpenNav/ChartPresentationV1']=$(if($palette -ceq 'XNav'){'Standard'}else{'XNav'})
 Refuse {Assert-RestartIniDelta $before $after '--xnav' $palette} 'opposite resulting palette refused including unchanged result'
 $after['OpenNav/ChartPresentationV1']='Standard';Refuse {Assert-RestartIniDelta $before $after '--xnav'} 'old unarmed calls still refuse palette delta'
 $after.Remove('OpenNav/ChartPresentationV1');Refuse {Assert-RestartIniDelta $before $after '--xnav' $palette} 'missing resulting palette refused'
 $after=$before.Clone();$after['OpenNav/ChartPresentationV1']=$palette;$after['Settings/UploadConnection']='enabled'
 Refuse {Assert-RestartIniDelta $before $after '--xnav' $palette} 'palette does not authorize connection delta'
 foreach($mode in @('--legacy','--safe-mode')) {Refuse {Assert-RestartChartPalette $mode $palette} 'palette cannot arm Legacy/Safe';Refuse {Assert-RestartWindowAction 'RequestGuardedMode' $mode 'RequestChartPalette'} 'palette needs actual SKAGER parent'}
}
foreach($bad in @('xnav','standard','XNav Standard','XNav;Start-Process','Paper','76','XNav ')) {
 Refuse {Assert-RestartChartPalette '--xnav' $bad} 'only exact fixed palette intent'
 Refuse {[OpenNavX.RestartWindowNative]::PaletteCaption($bad)} 'no arbitrary palette caption'
}
foreach($action in @('RequestChartPalette','ConfirmPalette','Save and restart','XNav','Standard')) {
 Refuse {Assert-RestartWindowAction 'ReviewRestartChild' '--xnav' $action} 'ordinary child review cannot select or confirm palette'
}
$root=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$ui=[IO.File]::ReadAllText((Join-Path $root 'src/ui/ProductPanel.cpp'))
$bridge=[IO.File]::ReadAllText((Join-Path $root 'src/integration/OpenCPNIntegration.cpp'))
$brand=[IO.File]::ReadAllText((Join-Path $root 'src/application/Brand.h'))
$titleConstants=@{'--xnav'='WindowTitle';'--legacy'='LegacyTitle';'--safe-mode'='SafeModeTitle'}
foreach($mode in @('--xnav','--legacy','--safe-mode')) {
 Check ($brand.Contains('"'+[OpenNavX.RestartWindowNative]::Title($mode)+'"') -and $bridge.Contains('application::brand::'+$titleConstants[$mode])) 'exact mode title derives from actual installed source'
 Check ($ui.Contains('"'+[OpenNavX.RestartWindowNative]::Caption('--xnav',$mode)+'"')) 'exact System button derives from actual source'
}
Check ($brand.Contains('"Switch to SKAGER"') -and $bridge.Contains('application::brand::SwitchToModern')) 'actual Legacy return menu exists'
foreach($name in @('RestartWindowReview.ps1','review-restart-window.ps1')) {
 $errors=$null;$tokens=$null;$null=[Management.Automation.Language.Parser]::ParseFile((Join-Path $PSScriptRoot $name),[ref]$tokens,[ref]$errors)
 Check ($errors.Count -eq 0) ('script parse '+$name)
}
$native=[IO.File]::ReadAllText((Join-Path $PSScriptRoot 'RestartWindowNative.cs'))
Check (-not ($native -match 'extern[^;]*(SendInput|keybd_event|mouse_event|SetCursorPos)' -or $native -match 'public static.*(SendMessage|Key|ClickAt)')) 'no global or arbitrary native input API'

$temporary=Join-Path ([IO.Path]::GetTempPath()) ('opennav-restart-review-'+[guid]::NewGuid().ToString('N'))
$null=New-Item -ItemType Directory -Path $temporary
# Permit only this newly created disposable journal tree on either OS. No
# copied production reader can escape to a real profile/installed generation.
function Assert-LocalPath([string]$Path) {
 $full=[IO.Path]::GetFullPath($Path)
 if($full -cne $temporary -and -not $full.StartsWith($temporary+[IO.Path]::DirectorySeparatorChar,[StringComparison]::Ordinal)){throw 'Test escaped disposable journal tree.'}
 return $full
}
try {
 $before=Join-Path $temporary 'before.ini';[IO.File]::WriteAllText($before,"[Settings]`nFoo=1`n[OpenNav]`nInterfaceMode=xnav`n")
 $session|Add-Member beforeIni $before;$session|Add-Member beforeIniSha256 (Get-Digest $before)
 $record=Join-Path $temporary 'session.json'
 Write-Record (Join-Path $temporary 'cold-launch-consumed.json') @{owner=$script:RestartOwner;session=$session.session;recordSha256=$session.recordSha256;status='consumed-before-start';mode='--xnav'}
 Write-Record (Join-Path $temporary 'cold-child.json') @{owner=$script:RestartOwner;session=$session.session;recordSha256=$session.recordSha256;pid=$parent.pid;createdFiletime=$parent.createdFiletime;launchSha256=(Get-Digest (Join-Path $temporary 'cold-launch-consumed.json'))}
 $chain=Get-RestartBaseline $session $record $parent -ReviewOnly
 Check ($chain.mode -ceq '--xnav' -and $null -eq $chain.completionPath -and $chain.completedTransitions -eq 0) 'cold proof stays cold, never invented completion'
 $pending=$chain.nextDirectory;$null=New-Item -ItemType Directory -Path $pending
 Refuse {Get-RestartBaseline $session $record $parent -ReviewOnly} 'unmarked incomplete transition refused'
 Refuse {Get-RestartBaseline $session $record $parent -PendingDirectory $pending} 'normal arming cannot skip pending transition'
 $ready|Add-Member beforeSha256 $session.beforeIniSha256
 Write-Record (Join-Path $pending 'ready.json') $ready
 $chain=Get-RestartBaseline $session $record $parent -ReviewOnly -PendingDirectory $pending
 Check ($chain.mode -ceq '--xnav' -and $chain.completedTransitions -eq 0) 'exact current armed ready directory can be inspected without renewal'
 Write-Record (Join-Path $pending 'ui-intent-consumed.json') @{status='consumed-before-ui-action';nonce=('f'*64)}
 $intentHash=Get-Digest (Join-Path $pending 'ui-intent-consumed.json')
 Refuse {Write-Record (Join-Path $pending 'ui-intent-consumed.json') @{status='retry'}} 'UI intent exclusive create refuses repeat'
 Check ((Get-Digest (Join-Path $pending 'ui-intent-consumed.json')) -ceq $intentHash) 'failed retry preserves consumed intent'
 Write-Record (Join-Path $pending 'failure.json') @{status='denied'}
 Refuse {Get-RestartBaseline $session $record $parent -ReviewOnly -PendingDirectory $pending} 'broker refusal cannot be treated as listening'
 Remove-Item -LiteralPath (Join-Path $pending 'failure.json')
 $post=Join-Path $pending 'post-close.ini';[IO.File]::WriteAllText($post,"[Settings]`nFoo=1`n[OpenNav]`nInterfaceMode=legacy`n")
 $request=@{protocol=1;kind='request';session=$session.session;recordSha256=$session.recordSha256;nonce=('e'*64);parentPid=$parent.pid;parentCreatedFiletime=$parent.createdFiletime;parentExitCode='0';helperPid='43';helperCreatedFiletime='133000000001000000';windowsSessionId='1';executable=$session.executable;executableSha256=$session.executableSha256;helper=$session.helper;helperSha256=$session.helperSha256;workingDirectory=$session.workingDirectory;path=$session.path;arguments=@('--legacy')}
 $wire=[Text.Encoding]::UTF8.GetBytes(($request|ConvertTo-Json -Compress));Save-RestartWire (Join-Path $pending 'request.json') $wire;$requestHash=Get-RestartBytesHash $wire
 $child=[pscustomobject]@{pid='44';createdFiletime='133000000002000000'}
 $receipt=@{protocol=1;kind='receipt';session=$session.session;recordSha256=$session.recordSha256;nonce=('e'*64);requestSha256=$requestHash;permitId=('1'*64);status='started';childPid=$child.pid;childCreatedFiletime=$child.createdFiletime;win32Error='0'}
 Save-RestartWire (Join-Path $pending 'receipt.json') ([Text.Encoding]::UTF8.GetBytes(($receipt|ConvertTo-Json -Compress)))
 $permit=Publish-RestartPermit $pending @{owner=$script:RestartOwner;session=$session.session;recordSha256=$session.recordSha256;status='consumed-before-allow';beforeSha256=$session.beforeIniSha256;profileSha256=(Get-Digest $post);requestSha256=$requestHash;mode='--legacy';permitId=('1'*64)}
 $completion=Join-Path $pending 'completion.json'
 $outcome=@{owner=$script:RestartOwner;status='child-identity-verified';session=$session.session;recordSha256=$session.recordSha256;permitSha256=(Get-Digest $permit);receiptSha256=(Get-Digest (Join-Path $pending 'receipt.json'));requestSha256=$requestHash;child=$child}
 Write-Record $completion $outcome
 $chain=Get-RestartBaseline $session $record $child -ReviewOnly
 Check ($chain.mode -ceq '--legacy' -and $chain.completionPath -ceq $completion -and $chain.completionSha256 -ceq (Get-Digest $completion) -and $chain.child.pid -ceq '44') 'child review uses actual complete request/permit/receipt proof'
 Refuse {Get-RestartBaseline $session $record $parent -ReviewOnly} 'old parent cannot use child review'
 Refuse {Get-RestartBaseline $session $record $child -ReviewOnly -PendingDirectory $pending} 'completed transition cannot become another request'
 $wrongChild=[pscustomobject]@{pid='44';createdFiletime='133000000002000001'}
 Refuse {Get-RestartBaseline $session $record $wrongChild -ReviewOnly} 'reused child PID refused'
 # Keep the receipt immutable; corrupt only the independently claimed child.
 Remove-Item -LiteralPath $completion;$outcome.child=$wrongChild;Write-Record $completion $outcome
 Refuse {Get-RestartBaseline $session $record $child -ReviewOnly} 'completion cannot claim a different receipt child'
 Remove-Item -LiteralPath $completion;$outcome.child=$child;Write-Record $completion $outcome
 $nextPending=Join-Path $temporary 'transition-0002';$null=New-Item -ItemType Directory -Path $nextPending
 $ready.parent=$child;$ready.beforeSha256=Get-Digest $post;Write-Record (Join-Path $nextPending 'ready.json') $ready
 $extra=Join-Path $temporary 'transition-0003';$null=New-Item -ItemType Directory -Path $extra
 Refuse {Get-RestartBaseline $session $record $child -ReviewOnly -PendingDirectory $nextPending} 'a pending transition cannot hide a later transition'
 Remove-Item -LiteralPath $nextPending,$extra -Recurse
 for($index=2;$index -le 16;$index++) {
  $priorHash=Get-Digest $post;$priorChild=$child
  $mode=if($index%2 -eq 0){'--xnav'}else{'--legacy'}
  $request.parentPid=$priorChild.pid;$request.parentCreatedFiletime=$priorChild.createdFiletime;$request.arguments=@($mode)
  $request.helperPid=(100+$index).ToString();$request.helperCreatedFiletime=(133000000003000000L+($index*10000L)).ToString()
  $dir=Join-Path $temporary ('transition-{0:d4}' -f $index);$null=New-Item -ItemType Directory -Path $dir
  $post=Join-Path $dir 'post-close.ini';[IO.File]::WriteAllText($post,("[Settings]`nFoo=1`n[OpenNav]`nInterfaceMode="+$mode.Substring(2)+"`n"))
  $wire=[Text.Encoding]::UTF8.GetBytes(($request|ConvertTo-Json -Compress));Save-RestartWire (Join-Path $dir 'request.json') $wire;$requestHash=Get-RestartBytesHash $wire
  $child=[pscustomobject]@{pid=(200+$index).ToString();createdFiletime=(133000000003001000L+($index*10000L)).ToString()}
  $receipt.requestSha256=$requestHash;$receipt.childPid=$child.pid;$receipt.childCreatedFiletime=$child.createdFiletime
  Save-RestartWire (Join-Path $dir 'receipt.json') ([Text.Encoding]::UTF8.GetBytes(($receipt|ConvertTo-Json -Compress)))
  $permit=Publish-RestartPermit $dir @{owner=$script:RestartOwner;session=$session.session;recordSha256=$session.recordSha256;status='consumed-before-allow';beforeSha256=$priorHash;profileSha256=(Get-Digest $post);requestSha256=$requestHash;mode=$mode;permitId=('1'*64)}
  Write-Record (Join-Path $dir 'completion.json') @{owner=$script:RestartOwner;status='child-identity-verified';session=$session.session;recordSha256=$session.recordSha256;permitSha256=(Get-Digest $permit);receiptSha256=(Get-Digest (Join-Path $dir 'receipt.json'));requestSha256=$requestHash;child=$child}
 }
 Refuse {Get-RestartBaseline $session $record $child} 'normal Arm still refuses a seventeenth transition'
 $chain=Get-RestartBaseline $session $record $child -ReviewOnly
 Check ($chain.completedTransitions -eq 16 -and $chain.child.pid -ceq $child.pid) 'the final sixteenth child can be reviewed without authorizing another restart'
 # A separate disposable palette intent tests real file hashes and replay.
 $paletteDir=Join-Path $temporary 'palette-proof';$null=New-Item -ItemType Directory -Path $paletteDir
 $pa=[pscustomobject]@{owner=$script:RestartOwner;session=$session.session;recordSha256=$session.recordSha256;mode='--xnav';chartPalette='Standard';parentPid=$parent.pid;parentCreatedFiletime=$parent.createdFiletime;transition=$paletteDir}
 $paPath=Join-Path $temporary ('arm-'+$parent.pid+'-'+$parent.createdFiletime+'.json');Write-Record $paPath $pa
 $pr=[pscustomobject]@{owner=$script:RestartOwner;session=$session.session;recordSha256=$session.recordSha256;mode='--xnav';chartPalette='Standard';parent=$parent;beforeSha256=$session.beforeIniSha256}
 Write-Record (Join-Path $paletteDir 'ready.json') $pr
 $pi=[pscustomobject]@{owner='OpenNavX.GuardedModeIntent.1';session=$session.session;recordSha256=$session.recordSha256;mode='--xnav';fromMode='--xnav';chartPalette='Standard';parent=$parent;command=@{Palette='Standard'};status='consumed-before-ui-action';armSha256=(Get-Digest $paPath);readySha256=(Get-Digest (Join-Path $paletteDir 'ready.json'))}
 Write-Record (Join-Path $paletteDir 'ui-intent-consumed.json') $pi
 $pp=Read-RestartPaletteProof $paletteDir $session $parent '--xnav' 'Standard' $session.beforeIniSha256
 $null=Read-RestartPaletteProof $paletteDir $session $parent '--xnav' 'Standard' $session.beforeIniSha256 $pp;Check $true 'actual immutable palette proof rereads exact arm/ready/intent'
 Refuse {Read-RestartPaletteProof $paletteDir $session $parent '--xnav' 'XNav' $session.beforeIniSha256 $pp} 'opposite palette replay refused'
 Refuse {Read-RestartPaletteProof $paletteDir $session $parent '--xnav' 'Standard' ('f'*64) $pp} 'different baseline replay refused'
 $changed=CopyValue $pp;$changed.intentSha256='e'*64
 Refuse {Read-RestartPaletteProof $paletteDir $session $parent '--xnav' 'Standard' $session.beforeIniSha256 $changed} 'changed consumed intent hash refused'
 foreach($name in @('ready.json','ui-intent-consumed.json')) {
  $path=Join-Path $paletteDir $name;$bytes=[IO.File]::ReadAllBytes($path);[IO.File]::AppendAllText($path,' ')
  Refuse {Read-RestartPaletteProof $paletteDir $session $parent '--xnav' 'Standard' $session.beforeIniSha256 $pp} ('mutated immutable '+$name)
  [IO.File]::WriteAllBytes($path,$bytes)
 }
} finally {Remove-Item -LiteralPath $temporary -Recurse -Force}
[pscustomobject]@{status='passed';checks=$script:checks;nativeWindowExecuted=$false;applicationLaunched=$false;boatTouched=$false;physicalOutput=$false}|ConvertTo-Json

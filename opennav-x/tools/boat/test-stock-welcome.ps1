# Pure evidence-policy checks and C# compile; no Windows API, app or boat access.
[CmdletBinding()]
param()
. (Join-Path $PSScriptRoot 'StockReview.ps1')
$checks=New-Object 'Collections.Generic.List[string]'
function Pass([string]$Name,[scriptblock]$Body) { & $Body;$checks.Add($Name) }
function Refuse([string]$Name,[scriptblock]$Body) { $failed=$false;try {$null=& $Body} catch {$failed=$true};if(-not $failed){throw ('Unsafe warning evidence accepted: '+$Name)};$checks.Add($Name) }
function CopyValue($Value) {
  $copy=$Value | ConvertTo-Json -Depth 12 | ConvertFrom-Json
  # PowerShell 7 may parse ISO strings into DateTime; native 5.1 does not. Keep
  # the fixture's protocol timestamp exact, independent of host JSON defaults.
  if($Value.PSObject.Properties.Name -contains 'utc'){$copy.utc=$Value.utc}
  return $copy
}
$now=[datetime]::UtcNow
$job=[pscustomobject]@{processId=42;executableSha256='7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c';launchResultSha256=('a'*64);launchRequestSha256=('b'*64);welcomeHelperSha256=('c'*64);welcomeNativeSha256=('d'*64)}
$inspection=[pscustomobject]@{status='passed';action='ReviewStock';reviewAction='InspectWelcome';mode='StockLegacy';processId=42;utc=$now.AddMinutes(-1).ToString('o');executableSha256=$job.executableSha256;launchResultSha256=$job.launchResultSha256;launchRequestSha256=$job.launchRequestSha256;welcomeHelperSha256=$job.welcomeHelperSha256;welcomeNativeSha256=$job.welcomeNativeSha256;imageSha256=('e'*64);
  nativeWindow=[pscustomobject]@{ProcessId=42;Title='Welcome to OpenCPN';ModalClass='#32770';AgreeText='Agree';CancelText='Cancel';AgreeId=5100;CancelId=5101;HtmlClass='wxWindowNR';HtmlName='htmlWindow';Frame=123;Modal=456;Agree=789;Cancel=790;Html=791;Dpi=96;Bounds=[pscustomobject]@{Left=100;Top=100;Right=700;Bottom=500;Width=600;Height=400}}}
Pass 'Exact stock inspection links same launch, helpers and pinned English caution' {Assert-StockWelcomeInspection $inspection $job $now}
foreach($field in @('status','action','reviewAction','mode','executableSha256','launchResultSha256','launchRequestSha256','welcomeHelperSha256','welcomeNativeSha256','imageSha256')) {
  Refuse "Different or missing inspection $field" {$v=CopyValue $inspection;$v.$field='changed';Assert-StockWelcomeInspection $v $job $now}
}
Refuse 'Different launched PID' {$v=CopyValue $inspection;$v.processId=43;Assert-StockWelcomeInspection $v $job $now}
foreach($time in @($now.AddSeconds(1),$now.AddMinutes(-31))) {Refuse 'Expired or future inspection' {$v=CopyValue $inspection;$v.utc=$time.ToString('o');Assert-StockWelcomeInspection $v $job $now}}
foreach($field in @('Title','ModalClass','AgreeText','CancelText','HtmlClass','HtmlName')) {Refuse "Different warning $field" {$v=CopyValue $inspection;$v.nativeWindow.$field='changed';Assert-StockWelcomeInspection $v $job $now}}
foreach($field in @('ProcessId','AgreeId','CancelId')) {Refuse "Different native warning $field" {$v=CopyValue $inspection;$v.nativeWindow.$field=99;Assert-StockWelcomeInspection $v $job $now}}
foreach($title in @('Welcome to OpenCPN ','welcome to OpenCPN','License','Purchase charts')) {Refuse 'No unreviewed, approximate or unrelated dialog acceptance' {$v=CopyValue $inspection;$v.nativeWindow.Title=$title;Assert-StockWelcomeInspection $v $job $now}}
Pass 'Native source compiles without calling Windows' {Initialize-StockWelcomeNative}
$swedish=CopyValue $inspection
$swedish.nativeWindow.Title='V'+[char]0x00e4+'lkommen till OpenCPN';$swedish.nativeWindow.AgreeText='Acceptera';$swedish.nativeWindow.CancelText='Avbryt'
Pass 'Exact Swedish tuple retains U+00E4 under Windows PowerShell script decoding' {
  if([int]$swedish.nativeWindow.Title[1] -ne 0x00e4){throw 'Swedish title encoding differs'}
  Assert-StockWelcomeInspection $swedish $job $now
}
$tupleMethod=[OpenNavX.StockWelcomeNative].GetMethod('SupportedTuple',[Reflection.BindingFlags]'NonPublic,Static')
foreach($title in @($inspection.nativeWindow.Title,$swedish.nativeWindow.Title)){
 foreach($agree in @('Agree','Acceptera')){foreach($cancel in @('Cancel','Avbryt')){
  $v=CopyValue $inspection;$v.nativeWindow.Title=$title;$v.nativeWindow.AgreeText=$agree;$v.nativeWindow.CancelText=$cancel
  $accepted=($title -ceq 'Welcome to OpenCPN' -and $agree -ceq 'Agree' -and $cancel -ceq 'Cancel') -or
    ($title -ceq $swedish.nativeWindow.Title -and $agree -ceq 'Acceptera' -and $cancel -ceq 'Avbryt')
  if($accepted){Pass 'Complete pinned language tuple accepted by shared inspection' {Assert-StockWelcomeInspection $v $job $now}}
  else{Refuse 'Mixed English and Swedish tuple refused' {Assert-StockWelcomeInspection $v $job $now}}
  Pass 'Native tuple classifier matches exact policy without Windows API calls' {
   if($tupleMethod.Invoke($null,[object[]]@($title,$agree,$cancel)) -ne $accepted){throw 'Native and inspection tuple decisions differ'}
  }
 }}
}
$fieldsMethod=[OpenNavX.StockWelcomeNative].GetMethod('ChangedFields',[Reflection.BindingFlags]'NonPublic,Static')
$sameMethod=[OpenNavX.StockWelcomeNative].GetMethod('SameNotice',[Reflection.BindingFlags]'NonPublic,Static')
$native=(Convert-StockWelcomeWindow $inspection.nativeWindow).PSObject.BaseObject
Pass 'Exact copied native observation remains identical' {if(-not $sameMethod.Invoke($null,[object[]]@($native,(Convert-StockWelcomeWindow (CopyValue $inspection).nativeWindow).PSObject.BaseObject))){throw 'Identical native evidence differs'}}
foreach($field in $native.GetType().GetFields()){
 Pass ('Every captured native identity field participates in recheck: '+$field.Name) {
  $changed=(Convert-StockWelcomeWindow (CopyValue $inspection).nativeWindow).PSObject.BaseObject
  if($field.Name -ceq 'Bounds'){$r=$changed.Bounds;$r.Left++;$changed.Bounds=$r}
  elseif($field.FieldType -eq [string]){$field.SetValue($changed,'different')}
  else{$field.SetValue($changed,[Convert]::ChangeType(99,$field.FieldType))}
  if($sameMethod.Invoke($null,[object[]]@($native,$changed))){throw ('Omitted identity field '+$field.Name)}
  if($fieldsMethod.Invoke($null,[object[]]@($native,$changed)) -cne $field.Name){throw ('Incorrect bounded mismatch diagnostic '+$field.Name)}
 }
}
Pass 'Identical observations produce no mismatch detail' {if($fieldsMethod.Invoke($null,[object[]]@($native,$native)) -cne ''){throw 'Unchanged observation misreported'}}
Pass 'Changing the entire language tuple invalidates captured observation' {
 $changed=(Convert-StockWelcomeWindow $swedish.nativeWindow).PSObject.BaseObject
 if($sameMethod.Invoke($null,[object[]]@($native,$changed))){throw 'Changed language reused old observation'}
}
Pass 'Recorded HWND and nested rectangle round trip into exact native type' {
  $copy=CopyValue $inspection
  $info=(Convert-StockWelcomeWindow $copy.nativeWindow)
  if($info.Frame -ne 123 -or $info.Modal -ne 456 -or $info.Bounds.Width -ne 600 -or $info.Bounds.Height -ne 400){throw 'Typed inspection conversion lost identity/geometry'}
}
foreach($captured in @($inspection.nativeWindow,$swedish.nativeWindow)) {
 Pass 'Actual C# NoticeInfo JSON includes read-only dimensions and reconstructs all fields exactly' {
  $typed=(Convert-StockWelcomeWindow $captured).PSObject.BaseObject
  $serialized=$typed | ConvertTo-Json -Depth 8 | ConvertFrom-Json
  if($serialized.Bounds.Width -ne 600 -or $serialized.Bounds.Height -ne 400){throw 'Actual read-only properties missing from JSON fixture'}
  $restored=(Convert-StockWelcomeWindow $serialized).PSObject.BaseObject
  if(-not $sameMethod.Invoke($null,[object[]]@($typed,$restored))){throw 'Actual C# JSON roundtrip changed captured identity'}
 }
}
foreach($name in @('Width','Height','Left','Right','Top','Bottom')) {
 Refuse ('Inconsistent serialized rectangle '+$name) {$v=CopyValue $inspection;$v.nativeWindow.Bounds.$name++;Convert-StockWelcomeWindow $v.nativeWindow}
}
foreach($value in @($null,'600',600.5,[long]2147483648)) {
 Refuse 'Invalid serialized dimension cannot coerce into a native rectangle' {$v=CopyValue $inspection;$v.nativeWindow.Bounds.Width=$value;Convert-StockWelcomeWindow $v.nativeWindow}
}
Refuse 'Missing derived dimension refuses instead of inferring omitted evidence' {$v=CopyValue $inspection;$v.nativeWindow.Bounds.PSObject.Properties.Remove('Width');Convert-StockWelcomeWindow $v.nativeWindow}
Refuse 'Unknown native window field refuses instead of dropping evidence' {$v=CopyValue $inspection;$v.nativeWindow|Add-Member extra 'unexpected';Convert-StockWelcomeWindow $v.nativeWindow}
Pass 'Native surface offers no generic selectors, messages, keys or coordinates' {
  $allowed=@('Inspect','AssertUnchanged','Agree','FocusCaption')
  foreach($method in [OpenNavX.StockWelcomeNative].GetMethods([Reflection.BindingFlags]'Public,Static,DeclaredOnly')) {if($method.Name -cnotin $allowed){throw 'Unexpected native operation'}}
  $source=[IO.File]::ReadAllText((Join-Path $PSScriptRoot 'StockWelcomeNative.cs'))
  if($source -match 'extern[^;]*(keybd_event|mouse_event|SetCursorPos|AttachThreadInput)'){throw 'Unbounded input API exposed'}
  $focus=[OpenNavX.StockWelcomeNative].GetMethod('FocusCaption')
  if(($focus.GetParameters().ParameterType.FullName -join ',') -cne 'System.Int32,System.Int64'){throw 'Focus accepts arbitrary target input'}
}
foreach($fileName in @('StockWelcome.ps1','StockReview.ps1','review-stock.ps1')) {Pass "Parses $fileName" {$tokens=$null;$errors=$null;$null=[Management.Automation.Language.Parser]::ParseFile((Join-Path $PSScriptRoot $fileName),[ref]$tokens,[ref]$errors);if($errors.Count){throw ($errors|Out-String)}}}
# Private diagnostics have no extra callable UI action. Exercise interpretation
# and JSON encoding without Windows, then the real read-only APIs on Windows.
$desktopType=[OpenNavX.StockWelcomeNative]
$relationMethod=$desktopType.GetMethod('DesktopRelation',[Reflection.BindingFlags]'NonPublic,Static')
foreach($case in @(
  @{args=@('Default',$true,0,'Default',$true,0);expected='HELPER_DESKTOP_RECEIVES_INPUT'},
  @{args=@('Default',$false,0,'Winlogon',$true,0);expected='HELPER_DESKTOP_NOT_INPUT_DESKTOP'},
  @{args=@('Default',$false,0,$null,$null,5);expected='INPUT_DESKTOP_UNAVAILABLE'},
  @{args=@('Default',$false,0,'Default',$false,0);expected='INPUT_DESKTOP_CHANGED_OR_DISCONNECTED'},
  @{args=@($null,$null,5,'Default',$true,0);expected='DESKTOP_METADATA_INCOMPLETE'}
)) {
  Pass ('Desktop metadata classification preserves uncertainty: '+$case.expected) {
    if($relationMethod.Invoke($null,[object[]]$case.args) -cne $case.expected){throw 'Desktop observations misclassified'}
  }
}
$jsonMethod=$desktopType.GetMethod('JsonString',[Reflection.BindingFlags]'NonPublic,Static')
Pass 'Desktop name cannot inject JSON fields or collect additional text' {
  $text='desktop"\'+[char]10+[char]0x00e4
  $encoded=$jsonMethod.Invoke($null,[object[]]@($text))
  $decoded=('{"name":'+$encoded+'}'|ConvertFrom-Json)
  if($decoded.name -cne $text -or @($decoded.PSObject.Properties).Count -ne 1){throw 'Unescaped desktop metadata'}
  if($jsonMethod.Invoke($null,[object[]]@($null)) -cne 'null'){throw 'Missing name became text'}
}
Pass 'Desktop PInvoke ABI uses pointer-sized handles and 32-bit lengths' {
  $flags=[Reflection.BindingFlags]'NonPublic,Static'
  $open=$desktopType.GetMethod('OpenInputDesktop',$flags)
  if($open.ReturnType -ne [IntPtr] -or (($open.GetParameters().ParameterType.FullName)-join ',') -cne 'System.UInt32,System.Boolean,System.UInt32'){throw 'OpenInputDesktop ABI differs'}
  $query=$desktopType.GetMethod('GetUserObjectInformationW',$flags)
  if((($query.GetParameters().ParameterType.FullName)-join ',') -cne 'System.IntPtr,System.Int32,System.IntPtr,System.UInt32,System.UInt32&'){throw 'User-object query ABI differs'}
  $attribute=$query.GetCustomAttributes([Runtime.InteropServices.DllImportAttribute],$false)[0]
  if(-not $attribute.SetLastError -or -not $attribute.ExactSpelling -or $attribute.CharSet -ne [Runtime.InteropServices.CharSet]::Unicode){throw 'Native Unicode/error contract differs'}
  $source=[IO.File]::ReadAllText((Join-Path $PSScriptRoot 'StockWelcomeNative.cs'))
  if($source -match 'extern[^;]*(SwitchDesktop|SetThreadDesktop|SetProcessWindowStation|AttachThreadInput|SetUserObject|LockWorkStation)'){throw 'Desktop mutation API added'}
}
Pass 'GUI thread structure and read-only query preserve pointer width' {
  $flags=[Reflection.BindingFlags]'NonPublic'
  $guiType=$desktopType.GetNestedType('GuiThreadInfo',$flags)
  $instance=[Activator]::CreateInstance($guiType)
  if([Runtime.InteropServices.Marshal]::SizeOf($instance) -ne (24+6*[IntPtr]::Size)){throw 'GUITHREADINFO native size differs'}
  $method=$desktopType.GetMethod('GetGUIThreadInfo',[Reflection.BindingFlags]'NonPublic,Static')
  if($method.GetParameters()[0].ParameterType -ne [uint32] -or -not $method.GetParameters()[1].ParameterType.IsByRef){throw 'GUI query signature differs'}
}
$guiFlags=$desktopType.GetMethod('GuiFlagsJson',[Reflection.BindingFlags]'NonPublic,Static')
foreach($case in @(
  @{bits=[uint32]0;menu=$false;moveSize=$false;systemMenu=$false;popupMenu=$false},
  @{bits=[uint32]2;menu=$false;moveSize=$true;systemMenu=$false;popupMenu=$false},
  @{bits=[uint32]4;menu=$true;moveSize=$false;systemMenu=$false;popupMenu=$false},
  @{bits=[uint32]8;menu=$false;moveSize=$false;systemMenu=$true;popupMenu=$false},
  @{bits=[uint32]16;menu=$false;moveSize=$false;systemMenu=$false;popupMenu=$true},
  @{bits=[uint32]31;menu=$true;moveSize=$true;systemMenu=$true;popupMenu=$true},
  @{bits=[uint32]64;menu=$false;moveSize=$false;systemMenu=$false;popupMenu=$false}
)) {
  Pass ('Source-defined GUI flags remain distinct: '+$case.bits) {
    $value=$guiFlags.Invoke($null,[object[]]@($case.bits))|ConvertFrom-Json
    if($value.raw -ne $case.bits){throw 'Raw state flags were lost'}
    foreach($key in @('menu','moveSize','systemMenu','popupMenu')){if($value.$key -ne $case.$key){throw ('Incorrect GUI bit '+$key)}}
  }
}
Pass 'Unavailable GUI query remains unknown, never false inactive state' {
  $value=$guiFlags.Invoke($null,[object[]]@($null))|ConvertFrom-Json
  foreach($property in $value.PSObject.Properties){if($null -ne $property.Value){throw 'Missing GUI state presented as known'}}
}
Pass 'Diagnostic JSON numbers remain invariant under localized negative signs' {
  $before=[Threading.Thread]::CurrentThread.CurrentCulture
  try {
    $culture=[Globalization.CultureInfo]::InvariantCulture.Clone();$culture.NumberFormat.NegativeSign='different'
    [Threading.Thread]::CurrentThread.CurrentCulture=$culture
    $stateType=$desktopType.GetNestedType('DesktopState',[Reflection.BindingFlags]'NonPublic')
    $state=[Activator]::CreateInstance($stateType)
    foreach($field in @('HandleError','NameError','InputError')){$stateType.GetField($field).SetValue($state,-1)}
    $json=$stateType.GetMethod('Json').Invoke($state,@())|ConvertFrom-Json
    if($json.handleError -ne -1 -or $json.nameError -ne -1 -or $json.inputError -ne -1){throw 'Locale changed numeric diagnostic syntax'}
  } finally {[Threading.Thread]::CurrentThread.CurrentCulture=$before}
}
$desktopProbe=$null
if([Environment]::OSVersion.Platform -eq 'Win32NT') {
  Pass 'Native desktop query reads metadata and closes only its opened input handle' {
    $probeMethod=$desktopType.GetMethod('DesktopDiagnostic',[Reflection.BindingFlags]'NonPublic,Static')
    $script:desktopProbe=$probeMethod.Invoke($null,@())|ConvertFrom-Json
    if($desktopProbe.helperDesktop.handleError -ne 0 -or $desktopProbe.helperDesktop.nameError -ne 0 -or $desktopProbe.helperDesktop.inputError -ne 0 -or
       [string]::IsNullOrEmpty($desktopProbe.helperDesktop.name)){throw 'Borrowed caller desktop metadata could not be decoded'}
    if($desktopProbe.inputDesktop.handleError -eq 0) {
      if($desktopProbe.inputHandleClosed -ne $true -or $desktopProbe.closeError -ne 0){throw 'Opened input handle was not closed'}
    } elseif($null -ne $desktopProbe.inputHandleClosed){throw 'Unavailable input handle falsely claimed closed'}
    if($desktopProbe.foreground.pid -lt 0 -or $desktopProbe.foreground.threadId -lt 0){throw 'Malformed numeric foreground observation'}
    if($desktopProbe.relation -cnotin @('HELPER_DESKTOP_RECEIVES_INPUT','HELPER_DESKTOP_NOT_INPUT_DESKTOP','INPUT_DESKTOP_UNAVAILABLE','INPUT_DESKTOP_CHANGED_OR_DISCONNECTED','DESKTOP_METADATA_INCOMPLETE')){throw 'Unknown diagnostic relation'}
  }
}
Add-Type -TypeDefinition @'
using System;
namespace OpenNavX {
  public static class StockActivationFixture {
    public static int Checks;
    public static Action Failure(){return delegate{Checks++;throw new InvalidOperationException("Exact modal still not foreground");};}
    public static Action Success(){return delegate{Checks++;};}
  }
}
'@
$activation=$desktopType.GetMethod('VerifyActivation',[Reflection.BindingFlags]'NonPublic,Static')
Pass 'Accepted foreground request never substitutes for actual guarded verification' {
  [OpenNavX.StockActivationFixture]::Checks=0
  $errorText=$null
  try{$activation.Invoke($null,[object[]]@($true,$true,[OpenNavX.StockActivationFixture]::Failure()))}catch{$errorText=$_.Exception.ToString()}
  if([OpenNavX.StockActivationFixture]::Checks -ne 1 -or $errorText -notlike '*focusRequestReturned=true*' -or $errorText -notlike '*Exact modal still not foreground*'){throw 'Transmitted request was mistaken for verified foreground'}
}
Pass 'Rendezvous timeout stops before any capture verification' {
  [OpenNavX.StockActivationFixture]::Checks=0
  $errorText=$null
  try{$activation.Invoke($null,[object[]]@($false,$false,[OpenNavX.StockActivationFixture]::Success()))}catch{$errorText=$_.Exception.ToString()}
  if([OpenNavX.StockActivationFixture]::Checks -ne 0 -or $errorText -notlike '*focusRequestReturned=false*' -or $errorText -notlike '*WM_NULL activation rendezvous*'){throw 'Failed rendezvous reached verification'}
}
Pass 'Request return value is diagnostic; successful exact-modal checks remain mandatory' {
  [OpenNavX.StockActivationFixture]::Checks=0
  $activation.Invoke($null,[object[]]@($false,$true,[OpenNavX.StockActivationFixture]::Success()))
  if([OpenNavX.StockActivationFixture]::Checks -ne 1){throw 'Actual verification skipped'}
}
# Deterministic capture sequencing tests exercise the production settling body
# with retained disposable file bytes; native actual-window tests remain separate.
& {
 $temporary=Join-Path ([IO.Path]::GetTempPath()) ('OpenNav-warning-settle-'+[guid]::NewGuid().ToString('N'))
 $null=New-Item -ItemType Directory -Path $temporary
 function Get-Digest([string]$Path) { $sha=[Security.Cryptography.SHA256]::Create();try{return ([BitConverter]::ToString($sha.ComputeHash([IO.File]::ReadAllBytes($Path)))).Replace('-','').ToLowerInvariant()}finally{$sha.Dispose()} }
 function Start-Sleep { param([int]$Milliseconds) if($Milliseconds -ne 150){throw 'Unexpected settling interval'} }
 function Save-StockWelcomeCapture([int]$ProcessId,$Info,[string]$Path) {
  if($ProcessId -ne 42 -or $Info.Modal -ne 456){throw 'Original identity lost'}
  $script:settleCalls++
  if($script:settleDelay){[Threading.Thread]::Sleep(5100)}
  if($script:settleThrowAt -eq $script:settleCalls){throw 'TEST exact native identity changed'}
  if(Test-Path -LiteralPath $Path){throw 'Observation overwrite'}
  $value=$script:settleSequence[[Math]::Min($script:settleCalls-1,$script:settleSequence.Count-1)]
  [IO.File]::WriteAllText($Path,$value)
  return Get-Digest $Path
 }
 function ResetSettle($Sequence) {$script:settleSequence=$Sequence;$script:settleCalls=0;$script:settleThrowAt=0;$script:settleDelay=$false}
 try {
  $seed=Join-Path $temporary 'expected';[IO.File]::WriteAllText($seed,'reviewed pixels');$expected=Get-Digest $seed
  Pass 'Delayed complete frames must become two consecutive exact reviewed frames' {
   ResetSettle @('painting','painting','reviewed pixels','reviewed pixels');$out=Join-Path $temporary 'delayed.png'
   $hash=Save-StockWelcomeSettledCapture 42 $inspection.nativeWindow $expected $out
   if($hash -cne $expected -or $settleCalls -ne 4 -or (Get-Digest $out) -cne $expected -or @(Get-ChildItem -LiteralPath $temporary -Filter 'delayed.png.settle-*.png').Count -ne 4){throw 'Delayed capture proof/retention differs'}
  }
  Pass 'A single match never hides a later mismatching complete frame' {
   ResetSettle @('reviewed pixels','painting','reviewed pixels','reviewed pixels');$out=Join-Path $temporary 'transient.png'
   $null=Save-StockWelcomeSettledCapture 42 $inspection.nativeWindow $expected $out
   if($settleCalls -ne 4){throw 'Nonconsecutive reviewed frame was accepted'}
  }
  Pass 'Persistent mismatch is bounded and publishes no acknowledged image' {
   ResetSettle @('changed full image');$out=Join-Path $temporary 'mismatch.png';$failed=$false
   try{$null=Save-StockWelcomeSettledCapture 42 $inspection.nativeWindow $expected $out}catch{$failed=$true}
   if(-not $failed -or $settleCalls -ne 20 -or (Test-Path -LiteralPath $out)){throw 'Mismatch did not fail within capture bound'}
  }
  Pass 'Changed native identity aborts immediately without capture retries or publication' {
   ResetSettle @('painting','reviewed pixels');$script:settleThrowAt=2;$out=Join-Path $temporary 'identity.png';$failed=$false
   try{$null=Save-StockWelcomeSettledCapture 42 $inspection.nativeWindow $expected $out}catch{$failed=$true}
   if(-not $failed -or $settleCalls -ne 2 -or (Test-Path -LiteralPath $out)){throw 'Identity refusal retried or published'}
  }
  Pass 'An expired absolute settling deadline refuses even a matching capture' {
   ResetSettle @('reviewed pixels');$script:settleDelay=$true;$out=Join-Path $temporary 'deadline.png';$failed=$false
   try{$null=Save-StockWelcomeSettledCapture 42 $inspection.nativeWindow $expected $out}catch{$failed=$true}
   if(-not $failed -or $settleCalls -ne 1 -or (Test-Path -LiteralPath $out)){throw 'Expired matching capture became authority'}
  }
  Pass 'Existing final image is never overwritten while settling' {
   ResetSettle @('reviewed pixels');$failed=$false
   try{$null=Save-StockWelcomeSettledCapture 42 $inspection.nativeWindow $expected $seed}catch{$failed=$true}
   if(-not $failed -or $settleCalls -ne 0 -or (Get-Digest $seed) -cne $expected){throw 'Existing evidence changed'}
  }
 } finally {Remove-Item -LiteralPath $temporary -Recurse -Force}
}
function Restore-StockWelcomeAgreementForeground([int]$ProcessId,$Info) { if($script:rejectAgreementForeground){throw 'TEST ordinary foreground activation refused'} }
$script:rejectAgreementForeground=$false
$script:captured=0;$script:written=0
function Save-StockWelcomeSettledCapture([int]$ProcessId,$Info,[string]$ExpectedImageHash,[string]$BeforeImage){$script:captured++;return ('f'*64)}
function Write-Record([string]$Path,$Record){$script:written++;throw 'TEST stop after durable-intent boundary; no native APIs invoked'}
Refuse 'Malformed capture hash stops before capture or native APIs' {Invoke-StockWelcomeAgreement 42 $null 'bad' 'before.png' 'intent.json'}
if($script:captured -ne 0 -or $script:written -ne 0){throw 'Malformed evidence reached capture/journal'}
$script:rejectAgreementForeground=$true
Refuse 'Blocked ordinary activation refuses before capture and intent' {Invoke-StockWelcomeAgreement 42 $inspection.nativeWindow ('f'*64) 'before.png' 'intent.json'}
if($script:captured -ne 0 -or $script:written -ne 0){throw 'Blocked ordinary foreground activation reached capture/journal'}
$script:rejectAgreementForeground=$false
Refuse 'Changed live pixels refuse before journal and before native Agree' {Invoke-StockWelcomeAgreement 42 $inspection.nativeWindow ('e'*64) 'before.png' 'intent.json'}
if($script:captured -ne 1 -or $script:written -ne 0){throw 'Pixel mismatch reached acknowledgement intent'}
Refuse 'Durable intent failure prevents native Agree' {Invoke-StockWelcomeAgreement 42 $inspection.nativeWindow ('f'*64) 'before.png' 'intent.json'}
if($script:captured -ne 2 -or $script:written -ne 1){throw 'Intent boundary was not exercised'}
$native=[OpenNavX.StockWelcomeNative]
$select=$native.GetMethod('ChooseCaptionPoint',[Reflection.BindingFlags]'NonPublic,Static')
function Rectangle($Left,$Top,$Right,$Bottom){return [OpenNavX.StockWelcomeNative+Rect]@{Left=$Left;Top=$Top;Right=$Right;Bottom=$Bottom}}
$window=Rectangle 100 100 700 500;$title=Rectangle 104 104 696 128
Pass 'Point derives only from bounded native caption geometry' {
 $point=$select.Invoke($null,[object[]]@($window,$title,[uint32]0))
 if($point.X -ne 400 -or $point.Y -ne 116){throw 'Unexpected candidate'}
}
Pass 'Negative virtual-screen coordinates retain signed hit-test semantics' {
 $point=$select.Invoke($null,[object[]]@((Rectangle -700 -500 -100 -100),(Rectangle -696 -496 -104 -472),[uint32]0))
 if($point.X -ne -400 -or $point.Y -ne -484){throw 'Signed coordinates lost'}
}
foreach($state in @([uint32]1,[uint32]8,[uint32]32768,[uint32]65536)){
 Refuse ('Unavailable/pressed/invisible/offscreen caption state '+$state) {$select.Invoke($null,[object[]]@($window,$title,$state))}
}
foreach($rect in @((Rectangle 0 104 696 128),(Rectangle 104 104 800 128),(Rectangle 104 90 696 128),(Rectangle 104 104 696 600),(Rectangle 104 104 106 128),(Rectangle 104 104 696 105))){
 Refuse 'Clipped or empty caption cannot provide a point' {$select.Invoke($null,[object[]]@($window,$rect,[uint32]0))}
}
Refuse 'Coordinates outside native signed shorts cannot wrap into a different target' {$select.Invoke($null,[object[]]@((Rectangle 50000 100 50600 500),(Rectangle 50004 104 50596 128),[uint32]0))}
Pass 'TITLEBARINFO native structure has six fixed DWORD states' {
 $type=$native.GetNestedType('TitleBarInfo',[Reflection.BindingFlags]'NonPublic')
 if([Runtime.InteropServices.Marshal]::SizeOf([Activator]::CreateInstance($type)) -ne 44){throw 'TITLEBARINFO ABI differs'}
}
# Pure injected delegates exercise the production delivery decision without
# invoking any input API. Actual official en/sv modal acceptance is a native gate.
Add-Type -TypeDefinition @'
using System;
namespace OpenNavX {
 public static class CaptionDeliveryFixture {
  public static int Deliveries,Releases,Verifications;public static uint Count,ReleaseCount;public static bool Changed;
  public static Func<uint> Delivery(){return delegate{Deliveries++;return Count;};}
  public static Func<uint> Release(){return delegate{Releases++;return ReleaseCount;};}
  public static Action Verify(){return delegate{Verifications++;if(Changed)throw new InvalidOperationException("Changed/foreign warning after input");};}
  public static void Reset(uint count,uint release,bool changed){Deliveries=Releases=Verifications=0;Count=count;ReleaseCount=release;Changed=changed;}
 }
}
'@
$deliver=$native.GetMethod('DeliverCaptionInput',[Reflection.BindingFlags]'NonPublic,Static')
foreach($count in @([uint32]0,[uint32]1,[uint32]2,[uint32]4)){
 foreach($released in @([uint32]0,[uint32]1)){
  Pass ('Uncertain input count '+$count+' only permits one release cleanup, result '+$released) {
   [OpenNavX.CaptionDeliveryFixture]::Reset($count,$released,$false);$errorText=$null
   try{$deliver.Invoke($null,[object[]]@([OpenNavX.CaptionDeliveryFixture]::Delivery(),[OpenNavX.CaptionDeliveryFixture]::Release(),[OpenNavX.CaptionDeliveryFixture]::Verify()))}catch{$errorText=$_.Exception.ToString()}
   $expectedRelease=if($count -gt 0){1}else{0}
   if($errorText -notlike '*delivery uncertain*' -or [OpenNavX.CaptionDeliveryFixture]::Deliveries -ne 1 -or
      [OpenNavX.CaptionDeliveryFixture]::Releases -ne $expectedRelease -or [OpenNavX.CaptionDeliveryFixture]::Verifications -ne 0){throw 'Partial input repeated a press, skipped cleanup or claimed focus'}
  }
 }
}
foreach($changed in @($false,$true)){
 Pass ('Full delivery still verifies exact foreground/identity; changed='+$changed) {
  [OpenNavX.CaptionDeliveryFixture]::Reset(3,0,$changed);$errorText=$null
  try{$deliver.Invoke($null,[object[]]@([OpenNavX.CaptionDeliveryFixture]::Delivery(),[OpenNavX.CaptionDeliveryFixture]::Release(),[OpenNavX.CaptionDeliveryFixture]::Verify()))}catch{$errorText=$_.Exception.ToString()}
  if([OpenNavX.CaptionDeliveryFixture]::Deliveries -ne 1 -or [OpenNavX.CaptionDeliveryFixture]::Releases -ne 0 -or [OpenNavX.CaptionDeliveryFixture]::Verifications -ne 1){throw 'Delivery accepted as verification'}
  if($changed -and $errorText -notlike '*Changed/foreign warning*'){throw 'Changed target passed'}
  if(-not $changed -and $errorText){throw $errorText}
 }
}
Pass 'INPUT/MOUSEINPUT native layout uses pointer alignment on Win32 and Win64' {
 $inputType=$native.GetNestedType('NativeInput',[Reflection.BindingFlags]'NonPublic')
 $mouseType=$native.GetNestedType('MouseInput',[Reflection.BindingFlags]'NonPublic')
 if([Runtime.InteropServices.Marshal]::SizeOf([Activator]::CreateInstance($inputType)) -ne $(if([IntPtr]::Size -eq 8){40}else{28}) -or
    [Runtime.InteropServices.Marshal]::SizeOf([Activator]::CreateInstance($mouseType)) -ne $(if([IntPtr]::Size -eq 8){32}else{24})){throw 'INPUT ABI differs'}
}
$idle=$native.GetMethod('GuiInputIdle',[Reflection.BindingFlags]'NonPublic,Static')
Pass 'Capture, menu and move loops refuse independent of foreground identity' {
 foreach($bits in @([uint32]2,[uint32]4,[uint32]8,[uint32]16)){if($idle.Invoke($null,[object[]]@($bits,[IntPtr]::Zero))){throw 'Busy GUI accepted'}}
 if($idle.Invoke($null,[object[]]@([uint32]0,[IntPtr]123))){throw 'Mouse capture accepted'}
 if(-not $idle.Invoke($null,[object[]]@([uint32]1,[IntPtr]::Zero))){throw 'Caret blinking was confused with busy input'}
}
$absolute=$native.GetMethod('AbsoluteCoordinate',[Reflection.BindingFlags]'NonPublic,Static')
Pass 'Absolute input respects virtual desktop bounds and negative origins' {
 if($absolute.Invoke($null,[object[]]@(-1920,-1920,3840)) -ne 0 -or $absolute.Invoke($null,[object[]]@(1919,-1920,3840)) -ne 65535){throw 'Virtual desktop endpoints differ'}
}
foreach($argsValue in @(@(1920,-1920,3840),@(-1921,-1920,3840),@(0,0,1),@(0,0,65537))){Refuse 'Bad desktop size/point cannot route input elsewhere' {$absolute.Invoke($null,[object[]]$argsValue)}}
$suitable=$native.GetMethod('InputDesktopSuitable',[Reflection.BindingFlags]'NonPublic,Static')
$stateType=$native.GetNestedType('DesktopState',[Reflection.BindingFlags]'NonPublic')
function DesktopState($Name,$Receives){$v=[Activator]::CreateInstance($stateType);$stateType.GetField('Name').SetValue($v,$Name);$stateType.GetField('ReceivesInput').SetValue($v,$Receives);return $v}
Pass 'Focus admits only two known active Default desktop observations' {
 if(-not $suitable.Invoke($null,[object[]]@((DesktopState 'Default' $true),(DesktopState 'Default' $true)))){throw 'Exact input desktop refused'}
 foreach($value in @((DesktopState 'Winlogon' $true),(DesktopState 'Default' $false),(DesktopState 'Default' $null))){if($suitable.Invoke($null,[object[]]@((DesktopState 'Default' $true),$value))){throw 'Unknown/noninteractive desktop accepted'}}
 $bad=DesktopState 'Default' $true;$stateType.GetField('NameError').SetValue($bad,5)
 if($suitable.Invoke($null,[object[]]@($bad,(DesktopState 'Default' $true)))){throw 'Failed metadata became permission'}
}
Refuse 'Focus result cannot become an acknowledgement inspection' {$v=CopyValue $inspection;$v.reviewAction='FocusWelcome';Assert-StockWelcomeInspection $v $job $now}
$beforeWrites=$script:written
Refuse 'Caption journal failure prevents native input' {Invoke-StockWelcomeFocus 42 123 'intent.json'}
if($script:written -ne $beforeWrites+1){throw 'Caption focus skipped exclusive durable intent'}
Refuse 'Unknown PID refuses before caption journal or input' {Invoke-StockWelcomeFocus 0 123 'intent.json'}
if($script:written -ne $beforeWrites+1){throw 'Malformed focus identity reached journal'}
[pscustomobject]@{status='passed';count=$checks.Count;checks=$checks.ToArray();nativeInteropCompiled=$true;windowsApisInvoked=($null -ne $desktopProbe);desktopReadOnlyProbe=$desktopProbe;applicationLaunched=$false;boatAccess=$false;hardwareCommands=$false;actualStockModalAcceptance=$false} | ConvertTo-Json -Depth 6

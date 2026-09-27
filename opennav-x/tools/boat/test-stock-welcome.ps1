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
  nativeWindow=[pscustomobject]@{ProcessId=42;Title='Welcome to OpenCPN';ModalClass='#32770';AgreeText='Agree';CancelText='Cancel';AgreeId=5100;CancelId=5101;HtmlClass='wxWindowNR';HtmlName='htmlWindow';Frame=123;Modal=456;Agree=789;Cancel=790;Html=791;Dpi=96;Bounds=[pscustomobject]@{Left=100;Top=100;Right=700;Bottom=500}}}
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
$native=[OpenNavX.StockWelcomeNative+NoticeInfo]$inspection.nativeWindow
Pass 'Exact copied native observation remains identical' {if(-not $sameMethod.Invoke($null,[object[]]@($native,[OpenNavX.StockWelcomeNative+NoticeInfo](CopyValue $inspection).nativeWindow))){throw 'Identical native evidence differs'}}
foreach($field in $native.GetType().GetFields()){
 Pass ('Every captured native identity field participates in recheck: '+$field.Name) {
  $changed=[OpenNavX.StockWelcomeNative+NoticeInfo](CopyValue $inspection).nativeWindow
  if($field.Name -ceq 'Bounds'){$r=$changed.Bounds;$r.Left++;$changed.Bounds=$r}
  elseif($field.FieldType -eq [string]){$field.SetValue($changed,'different')}
  else{$field.SetValue($changed,[Convert]::ChangeType(99,$field.FieldType))}
  if($sameMethod.Invoke($null,[object[]]@($native,$changed))){throw ('Omitted identity field '+$field.Name)}
  if($fieldsMethod.Invoke($null,[object[]]@($native,$changed)) -cne $field.Name){throw ('Incorrect bounded mismatch diagnostic '+$field.Name)}
 }
}
Pass 'Identical observations produce no mismatch detail' {if($fieldsMethod.Invoke($null,[object[]]@($native,$native)) -cne ''){throw 'Unchanged observation misreported'}}
Pass 'Changing the entire language tuple invalidates captured observation' {
 $changed=[OpenNavX.StockWelcomeNative+NoticeInfo]$swedish.nativeWindow
 if($sameMethod.Invoke($null,[object[]]@($native,$changed))){throw 'Changed language reused old observation'}
}
Pass 'Recorded HWND and nested rectangle round trip into exact native type' {
  $copy=CopyValue $inspection
  $info=[OpenNavX.StockWelcomeNative+NoticeInfo]$copy.nativeWindow
  if($info.Frame -ne 123 -or $info.Modal -ne 456 -or $info.Bounds.Width -ne 600 -or $info.Bounds.Height -ne 400){throw 'Typed inspection conversion lost identity/geometry'}
}
Pass 'Native surface offers no generic selectors, messages, keys or coordinates' {
  $allowed=@('Inspect','AssertUnchanged','Agree')
  foreach($method in [OpenNavX.StockWelcomeNative].GetMethods([Reflection.BindingFlags]'Public,Static,DeclaredOnly')) {if($method.Name -cnotin $allowed){throw 'Unexpected native operation'}}
  $source=[IO.File]::ReadAllText((Join-Path $PSScriptRoot 'StockWelcomeNative.cs'))
  if($source -match 'extern[^;]*(SendInput|keybd_event|mouse_event|SetCursorPos)'){throw 'Global input injection exposed'}
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
$script:captured=0;$script:written=0
function Save-StockWelcomeCapture([int]$ProcessId,$Info,[string]$Path){$script:captured++;return ('f'*64)}
function Write-Record([string]$Path,$Record){$script:written++;throw 'TEST stop after durable-intent boundary; no native APIs invoked'}
Refuse 'Malformed capture hash stops before capture or native APIs' {Invoke-StockWelcomeAgreement 42 $null 'bad' 'before.png' 'intent.json'}
if($script:captured -ne 0 -or $script:written -ne 0){throw 'Malformed evidence reached capture/journal'}
Refuse 'Changed live pixels refuse before journal and before native Agree' {Invoke-StockWelcomeAgreement 42 $null ('e'*64) 'before.png' 'intent.json'}
if($script:captured -ne 1 -or $script:written -ne 0){throw 'Pixel mismatch reached acknowledgement intent'}
Refuse 'Durable intent failure prevents native Agree' {Invoke-StockWelcomeAgreement 42 $null ('f'*64) 'before.png' 'intent.json'}
if($script:captured -ne 2 -or $script:written -ne 1){throw 'Intent boundary was not exercised'}
[pscustomobject]@{status='passed';count=$checks.Count;checks=$checks.ToArray();nativeInteropCompiled=$true;windowsApisInvoked=($null -ne $desktopProbe);desktopReadOnlyProbe=$desktopProbe;applicationLaunched=$false;boatAccess=$false;hardwareCommands=$false;actualStockModalAcceptance=$false} | ConvertTo-Json -Depth 6

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
$script:captured=0;$script:written=0
function Save-StockWelcomeCapture([int]$ProcessId,$Info,[string]$Path){$script:captured++;return ('f'*64)}
function Write-Record([string]$Path,$Record){$script:written++;throw 'TEST stop after durable-intent boundary; no native APIs invoked'}
Refuse 'Malformed capture hash stops before capture or native APIs' {Invoke-StockWelcomeAgreement 42 $null 'bad' 'before.png' 'intent.json'}
if($script:captured -ne 0 -or $script:written -ne 0){throw 'Malformed evidence reached capture/journal'}
Refuse 'Changed live pixels refuse before journal and before native Agree' {Invoke-StockWelcomeAgreement 42 $null ('e'*64) 'before.png' 'intent.json'}
if($script:captured -ne 1 -or $script:written -ne 0){throw 'Pixel mismatch reached acknowledgement intent'}
Refuse 'Durable intent failure prevents native Agree' {Invoke-StockWelcomeAgreement 42 $null ('f'*64) 'before.png' 'intent.json'}
if($script:captured -ne 2 -or $script:written -ne 1){throw 'Intent boundary was not exercised'}
[pscustomobject]@{status='passed';count=$checks.Count;checks=$checks.ToArray();nativeInteropCompiled=$true;windowsApisInvoked=$false;applicationLaunched=$false;boatAccess=$false;hardwareCommands=$false;actualStockModalAcceptance=$false} | ConvertTo-Json -Depth 6

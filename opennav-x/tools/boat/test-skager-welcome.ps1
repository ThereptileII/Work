# Policy/evidence checks and a disposable Win32 hierarchy. No app or boat access.
[CmdletBinding()]
param()
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'InstalledWelcome.ps1')
$checks=New-Object 'Collections.Generic.List[string]'
function Pass([string]$Name,[scriptblock]$Body){& $Body;$checks.Add($Name)}
function Refuse([string]$Name,[scriptblock]$Body){
  $failed=$false
  try{$null=& $Body}catch{$failed=$true}
  if(-not $failed){throw ('Unsafe SKAGER warning accepted: '+$Name)}
  $checks.Add($Name)
}
function CopyValue($Value){return ($Value | ConvertTo-Json -Depth 12 | ConvertFrom-Json)}

# Independent expected product text, including its final newline. Do not read
# the expectation from the validator under test or substitute a short caption.
$noticeText='OpenCPN is distributed in the hope that it will be useful, '+
  'but WITHOUT ANY WARRANTY; without even the implied '+
  'warranty of MERCHANTABILITY or FITNESS FOR A '+
  'PARTICULAR PURPOSE.'+"`n`n"+
  'See the GNU General Public License for more details.'+"`n`n"+
  'OpenCPN must only be used in conjunction with approved '+
  'paper charts and traditional methods of navigation.'+"`n`n"+
  'DO NOT rely upon OpenCPN for safety of life or property.'+"`n`n"+
  'Please click "Agree" and proceed, or "Cancel" to quit.'+"`n"
Pass 'Separate SKAGER native policy compiles without Windows calls' {Initialize-SkagerWelcomeNative}
$native=[OpenNavX.SkagerWelcomeNative]
$flags=[Reflection.BindingFlags]'NonPublic,Static'
$validate=$native.GetMethod('ValidateControls',$flags)
$same=$native.GetMethod('SameNotice',$flags)
$changed=$native.GetMethod('ChangedFields',$flags)
if($null -eq $validate -or $null -eq $same -or $null -eq $changed){throw 'Required native policy seam missing'}

function NewControl([long]$Handle,[int]$Id,[string]$Class,[string]$Text,[int]$Style=0){
  $value=New-Object OpenNavX.SkagerWelcomeNative+ControlInfo
  $value.Handle=$Handle;$value.Parent=456;$value.ProcessId=42;$value.Id=$Id
  $value.Class=$Class;$value.Text=$Text;$value.Style=$Style
  $value.Visible=$true;$value.Enabled=$true
  return $value
}
function NewControls {
  NewControl 789 5100 'wxWindowNR' 'Agree'
  NewControl 790 5101 'wxWindowNR' 'Cancel'
  NewControl 791 -1 'Edit' $noticeText 0x0804
  NewControl 792 -1 'Static' 'Welcome to SKAGER'
  NewControl 793 -1 'Static' ('Navigation safety notice '+[char]0x00b7+' OpenCPN')
}
function ValidateControls($Controls,[int]$ProcessId=42,[long]$Modal=456){
  $typed=[OpenNavX.SkagerWelcomeNative+ControlInfo[]]@($Controls)
  $null=$validate.Invoke($null,[object[]]@($ProcessId,$Modal,$typed))
}
Pass 'Actual five direct SKAGER children and full notice accepted' {ValidateControls (NewControls)}
Pass 'Native Edit CRLF representation preserves exact full notice' {
  $value=@(NewControls);$value[2].Text=$noticeText.Replace("`n","`r`n");ValidateControls $value
}
foreach($processId in @(0,-1,43)){Refuse "Invalid or different process $processId" {ValidateControls (NewControls) $processId}}
foreach($modal in @(0,457)){Refuse "Invalid or different modal $modal" {ValidateControls (NewControls) 42 $modal}}
foreach($index in 0..4){
  Refuse "Missing required child $index" {$value=@(NewControls);ValidateControls @(for($other=0;$other -lt $value.Count;$other++){if($other -ne $index){$value[$other]}})}
  foreach($field in @('ProcessId','Parent','Handle','Visible')){
    Refuse "Child $index changed $field" {
      $value=@(NewControls)
      switch($field){
        'ProcessId'{$value[$index].ProcessId=43}
        'Parent'{$value[$index].Parent=457}
        'Handle'{$value[$index].Handle=0}
        'Visible'{$value[$index].Visible=$false}
      }
      ValidateControls $value
    }
  }
}
foreach($index in 0..4){Refuse "Disabled notice child $index" {$value=@(NewControls);$value[$index].Enabled=$false;ValidateControls $value}}
Refuse 'Duplicate nonzero native handles' {$value=@(NewControls);$value[1].Handle=$value[0].Handle;ValidateControls $value}
Refuse 'Extra action cannot accompany the notice' {ValidateControls (@(NewControls)+(NewControl 794 5102 'wxWindowNR' 'Other'))}
Refuse 'Extra content cannot accompany the notice' {ValidateControls (@(NewControls)+(NewControl 794 -1 'Static' 'Other'))}
Refuse 'Additional readonly notice is ambiguous' {ValidateControls (@(NewControls)+(NewControl 794 -1 'Edit' $noticeText 0x0804))}
foreach($index in @(0,1)){
  foreach($class in @('Button','Static','Edit')){Refuse "Action $index cannot become $class" {$value=@(NewControls);$value[$index].Class=$class;ValidateControls $value}}
  foreach($label in @('Agree ','agree','Acceptera','Avbryt','Purchase charts')){Refuse "Action $index requires its exact English label: $label" {$value=@(NewControls);$value[$index].Text=$label;ValidateControls $value}}
  Refuse "Action $index wrong native ID" {$value=@(NewControls);$value[$index].Id=5102;ValidateControls $value}
}
Refuse 'Swapped Agree and Cancel IDs' {$value=@(NewControls);$value[0].Id=5101;$value[1].Id=5100;ValidateControls $value}
Pass 'Decorative static text does not replace the exact body and action contract' {
  $value=@(NewControls);$value[3].Text='Decorative heading'
  $value[4].Text='Navigation safety notice '+[char]0x00c2+[char]0x00b7+' OpenCPN';ValidateControls $value
}
foreach($style in @(0,0x0800,0x0004)){Refuse "Notice lacks readonly multiline style: $style" {$value=@(NewControls);$value[2].Style=$style;ValidateControls $value}}
foreach($text in @('',$noticeText.TrimEnd(),$noticeText.Replace('WITHOUT ANY WARRANTY','WITH A WARRANTY'),$noticeText.Replace("`n","`r"),($noticeText+' '))){
  Refuse 'Full notice rejects omission, alteration or extra text' {$value=@(NewControls);$value[2].Text=$text;ValidateControls $value}
}
Refuse 'Stock HTML surface cannot substitute for native readonly notice' {$value=@(NewControls);$value[2].Class='wxWindowNR';$value[2].Text='htmlWindow';ValidateControls $value}

$observation=[pscustomobject]@{
  Frame=123;Modal=456;Agree=789;Cancel=790;Body=791;ProcessId=42;AgreeId=5100;CancelId=5101;Dpi=96
  Bounds=[pscustomobject]@{Left=100;Top=100;Right=800;Bottom=700;Width=700;Height=600}
  AgreeBounds=[pscustomobject]@{Left=600;Top=620;Right=760;Bottom=668;Width=160;Height=48}
  Title='Welcome to SKAGER';FrameTitle='SKAGER / OpenCPN';ModalClass='#32770'
  BodyClass='Edit';BodyText=$noticeText;BodyStyle=0x0804
  AgreeClass='wxWindowNR';CancelClass='wxWindowNR';AgreeText='Agree';CancelText='Cancel'
}
Pass 'Exact typed observation satisfies SKAGER window policy' {Assert-SkagerWelcomeWindow $observation 42}
$typed=(Convert-SkagerWelcomeWindow $observation).PSObject.BaseObject
Pass 'Actual C# JSON reconstructs every captured field and derived dimension' {
  $serialized=$typed | ConvertTo-Json -Depth 8 | ConvertFrom-Json
  $restored=(Convert-SkagerWelcomeWindow $serialized).PSObject.BaseObject
  if($serialized.Bounds.Width -ne 700 -or $serialized.AgreeBounds.Height -ne 48 -or -not $same.Invoke($null,[object[]]@($typed,$restored))){throw 'Native evidence roundtrip changed identity or geometry'}
}
Pass 'Identical observations produce no mismatch fields' {
  if(-not $same.Invoke($null,[object[]]@($typed,$typed)) -or $changed.Invoke($null,[object[]]@($typed,$typed)) -cne ''){throw 'Identical observation differs'}
}
foreach($field in $typed.GetType().GetFields()){
  Pass ('Every native field invalidates previous observation: '+$field.Name) {
    $other=(Convert-SkagerWelcomeWindow (CopyValue $observation)).PSObject.BaseObject
    if($field.Name -in @('Bounds','AgreeBounds')){$rectangle=$field.GetValue($other);$rectangle.Left++;$field.SetValue($other,$rectangle)}
    elseif($field.FieldType -eq [string]){$field.SetValue($other,'changed')}
    else{$field.SetValue($other,[Convert]::ChangeType(99,$field.FieldType))}
    if($same.Invoke($null,[object[]]@($typed,$other)) -or $changed.Invoke($null,[object[]]@($typed,$other)) -cne $field.Name){throw ('Unchecked or misreported captured field '+$field.Name)}
  }
}
foreach($rectangle in @('Bounds','AgreeBounds')){
  foreach($field in @('Left','Top','Right','Bottom','Width','Height')){
    Refuse "Inconsistent serialized $rectangle.$field" {$value=CopyValue $observation;$value.$rectangle.$field++;Convert-SkagerWelcomeWindow $value}
  }
  foreach($invalid in @($null,'48',48.5,[long]2147483648)){
    Refuse "Invalid serialized $rectangle.Height cannot be coerced" {$value=CopyValue $observation;$value.$rectangle.Height=$invalid;Convert-SkagerWelcomeWindow $value}
  }
  Refuse "Missing derived $rectangle.Width cannot be inferred" {$value=CopyValue $observation;$value.$rectangle.PSObject.Properties.Remove('Width');Convert-SkagerWelcomeWindow $value}
}
foreach($field in @('Title','FrameTitle','ModalClass','BodyClass','BodyText','AgreeClass','CancelClass','AgreeText','CancelText')){
  Refuse "Different native warning $field" {$value=CopyValue $observation;$value.$field='changed';Convert-SkagerWelcomeWindow $value}
}
Refuse 'Stock title is not a SKAGER title alias' {$value=CopyValue $observation;$value.Title='Welcome to OpenCPN';Convert-SkagerWelcomeWindow $value}
Refuse 'Changed warning PID' {Assert-SkagerWelcomeWindow $observation 43}
Refuse 'Unknown serialized field is not discarded' {$value=CopyValue $observation;$value | Add-Member extra 'unreviewed';Convert-SkagerWelcomeWindow $value}
Refuse 'Missing notice body is not reconstructed from source' {$value=CopyValue $observation;$value.PSObject.Properties.Remove('BodyText');Convert-SkagerWelcomeWindow $value}

$commit='bf4896833fda8389cfb03924cde7d6c89de5e5e6'
$executable='cb1627969e5564af9ff598e0327b0a6028ea7c774f5ec95d90fa9282fa824ced'
$launch=[pscustomobject]@{mode='--xnav';buildCommit=$commit;executableSha256=$executable}
$job=[pscustomobject]@{buildCommit=$commit;executableSha256=$executable}
Pass 'Exact reviewed installed bf xnav identity selects SKAGER' {if((Get-InstalledWelcomePresentation $launch $job) -cne 'skager'){throw 'Exact modern product selected wrong notice'}}
foreach($mode in @('--legacy','--safe-mode')){
  Pass "Reviewed bf $mode keeps stock notice presentation" {$value=CopyValue $launch;$value.mode=$mode;if((Get-InstalledWelcomePresentation $value $job) -cne 'opencpn'){throw 'Stock mode semantics changed'}}
}
Pass 'Historical installed xnav keeps source-specific stock path' {
  $oldLaunch=CopyValue $launch;$oldJob=CopyValue $job
  $oldLaunch.buildCommit='b'*40;$oldJob.buildCommit=$oldLaunch.buildCommit
  $oldLaunch.executableSha256='a'*64;$oldJob.executableSha256=$oldLaunch.executableSha256
  if((Get-InstalledWelcomePresentation $oldLaunch $oldJob) -cne 'opencpn'){throw 'Historical source silently upgraded to SKAGER'}
}
foreach($hash in @(('a'*64),$executable.ToUpperInvariant(),'', $null)){
  Refuse 'Reviewed bf requires its exact authenticated executable' {
    $badLaunch=CopyValue $launch;$badJob=CopyValue $job
    $badLaunch.executableSha256=$hash;$badJob.executableSha256=$hash
    Get-InstalledWelcomePresentation $badLaunch $badJob
  }
}
foreach($mode in @('','--portable','--xnav --legacy','xnav')){
  Refuse "Unreviewed mode cannot select a notice: $mode" {$value=CopyValue $launch;$value.mode=$mode;Get-InstalledWelcomePresentation $value $job}
}
# Exercise the actual wrapper's ordering with one explicit native-call
# substitution. All other boundaries use local fake operations in this scope;
# neither the real C# UI methods nor files are reached, even by a negative case.
& {
  $tokens=$null;$errors=$null
  $ast=[Management.Automation.Language.Parser]::ParseFile((Join-Path $PSScriptRoot 'SkagerWelcome.ps1'),[ref]$tokens,[ref]$errors)
  if($errors.Count){throw 'SKAGER helper does not parse'}
  $function=$ast.Find({param($node) $node -is [Management.Automation.Language.FunctionDefinitionAst] -and $node.Name -ceq 'Invoke-SkagerWelcomeAgreement'},$true)
  $productionCall='[OpenNavX.SkagerWelcomeNative]::Agree($ProcessId,$Info)'
  $source=$function.Extent.Text
  if(([regex]::Matches($source,[regex]::Escape($productionCall))).Count -ne 1){throw 'Expected exactly one fixed native dispatch'}
  $source=$source.Replace('function Invoke-SkagerWelcomeAgreement(', 'function Invoke-FixtureSkagerAgreement(').Replace($productionCall,'Invoke-FixtureNativeAgree $ProcessId $Info')
  . ([scriptblock]::Create($source))
  $state=@{events=(New-Object 'Collections.Generic.List[string]');foregroundFailure=$false;pixels=('e'*64);writeFailure=$false;used=$false;nativeCalls=0}
  function ResetFixture {
    $state.events.Clear();$state.foregroundFailure=$false;$state.pixels='e'*64
    $state.writeFailure=$false;$state.used=$false;$state.nativeCalls=0
  }
  function Restore-SkagerWelcomeAgreementForeground([int]$ProcessId,$Info){
    $state.events.Add('foreground');if($state.foregroundFailure){throw 'Fixture activation refused'}
  }
  function Save-SkagerWelcomeSettledCapture([int]$ProcessId,$Info,[string]$ExpectedImageHash,[string]$BeforeImage){$state.events.Add('capture');return $state.pixels}
  function Write-Record([string]$Path,$Value){
    $state.events.Add('intent')
    if($state.writeFailure -or $state.used){throw 'Fixture exclusive intent refused'}
    if($Path -cne 'fixture-intent.json' -or $Value.owner -cne 'OpenNavX.SkagerWelcome.1' -or $Value.status -cne 'agree-intent' -or $Value.processId -ne 42 -or $Value.imageSha256 -cne ('e'*64)){throw 'Incorrect durable intent identity'}
    $state.used=$true
  }
  function Invoke-FixtureNativeAgree([int]$ProcessId,$Info){
    if(-not $state.used -or $ProcessId -ne 42 -or $Info.Agree -ne 789){throw 'Dispatch preceded exact one-use authority'}
    $state.events.Add('native');$state.nativeCalls++
  }
  Pass 'Actual wrapper orders foreground, matching capture, intent and one dispatch' {
    ResetFixture;Invoke-FixtureSkagerAgreement 42 $observation ('e'*64) 'fixture-before.png' 'fixture-intent.json'
    if(($state.events -join ',') -cne 'foreground,capture,intent,native' -or $state.nativeCalls -ne 1){throw 'Agreement order differs'}
  }
  Refuse 'Consumed one-use intent cannot dispatch again' {Invoke-FixtureSkagerAgreement 42 $observation ('e'*64) 'fixture-before.png' 'fixture-intent.json'}
  if($state.nativeCalls -ne 1){throw 'Consumed intent allowed another dispatch'}
  foreach($failure in @('foreground','pixels','intent')){
    ResetFixture
    switch($failure){'foreground'{$state.foregroundFailure=$true};'pixels'{$state.pixels='f'*64};'intent'{$state.writeFailure=$true}}
    Refuse "Actual wrapper stops before native dispatch on $failure failure" {Invoke-FixtureSkagerAgreement 42 $observation ('e'*64) 'fixture-before.png' 'fixture-intent.json'}
    if($state.nativeCalls -ne 0){throw ('Failed '+$failure+' still dispatched')}
    if($failure -in @('foreground','pixels') -and $state.events.Contains('intent')){throw 'Rejected foreground or pixels consumed authority'}
  }
  ResetFixture
  Refuse 'Malformed image hash stops before foreground, capture or intent' {Invoke-FixtureSkagerAgreement 42 $observation 'wrong' 'fixture-before.png' 'fixture-intent.json'}
  if($state.events.Count){throw 'Malformed hash reached observation or authority'}
}
Pass 'Native public surface contains only fixed welcome actions' {
  $allowed=@('Inspect','AssertUnchanged','Agree','FocusCaption')
  foreach($method in $native.GetMethods([Reflection.BindingFlags]'Public,Static,DeclaredOnly')){
    if($method.Name -cnotin $allowed){throw ('Unexpected native operation '+$method.Name)}
  }
  foreach($name in @('Inspect','AssertUnchanged','Agree')){if($null -eq $native.GetMethod($name)){throw ('Required fixed operation missing: '+$name)}}
}
Pass 'Disposable native fixture compiles without invoking Windows APIs' {
  Add-Type -Path (Join-Path $PSScriptRoot 'SkagerWelcomeFixture.cs')
}
$delivery=$native.GetMethod('DeliverCaptionInput',$flags)
foreach($count in @([uint32]0,[uint32]1,[uint32]2,[uint32]4)){
  foreach($released in @([uint32]0,[uint32]1)){
    Pass "Partial pointer count $count permits only release cleanup $released and reports uncertain acknowledgement" {
      [SkagerPointerDeliveryFixture]::Reset($count,$released,$false);$errorText=$null
      try{$null=$delivery.Invoke($null,[object[]]@([SkagerPointerDeliveryFixture]::Delivery(),[SkagerPointerDeliveryFixture]::Release(),[SkagerPointerDeliveryFixture]::Verify()))}catch{$errorText=$_.Exception.ToString()}
      $expectedReleases=if($count -gt 0){1}else{0}
      if($errorText -notlike '*delivery uncertain*' -or $errorText -notlike '*acknowledgement may have completed*' -or $errorText -notlike '*No retry*' -or
         [SkagerPointerDeliveryFixture]::Deliveries -ne 1 -or [SkagerPointerDeliveryFixture]::Releases -ne $expectedReleases -or [SkagerPointerDeliveryFixture]::Verifications -ne 0){throw 'Partial delivery repeated a press, claimed completion or omitted uncertainty'}
    }
  }
}
foreach($reject in @($false,$true)){
  Pass "Complete pointer submission still invokes separate verification: refusal=$reject" {
    [SkagerPointerDeliveryFixture]::Reset(3,0,$reject);$errorText=$null
    try{$null=$delivery.Invoke($null,[object[]]@([SkagerPointerDeliveryFixture]::Delivery(),[SkagerPointerDeliveryFixture]::Release(),[SkagerPointerDeliveryFixture]::Verify()))}catch{$errorText=$_.Exception.ToString()}
    if([SkagerPointerDeliveryFixture]::Deliveries -ne 1 -or [SkagerPointerDeliveryFixture]::Releases -ne 0 -or [SkagerPointerDeliveryFixture]::Verifications -ne 1){throw 'Input delivery substituted for verification'}
    if(($reject -and $errorText -notlike '*Fixture verification refused*') -or (-not $reject -and $errorText)){throw 'Verification failure was lost or invented'}
  }
}
$nativeReadback=$null
if([Environment]::OSVersion.Platform -eq 'Win32NT'){
  $fixture=$null;$previousDpi=[SkagerWelcomeFixture]::EnterDpiAwareness()
  try{
    $fixture=New-Object SkagerWelcomeFixture($noticeText)
    $controlText=$native.GetMethod('ControlText',$flags)
    Pass 'Native Edit WM_GETTEXT reads complete safety text across UI threads' {
      $actual=[string]$controlText.Invoke($null,[object[]]@($fixture.Body))
      if($actual.Replace("`r`n","`n") -cne $noticeText){throw 'Native Edit returned only a caption or incomplete text'}
      $controls=@(NewControls);$controls[2].Text=$actual;ValidateControls $controls
    }
    $info=[OpenNavX.SkagerWelcomeNative]::Inspect($PID)
    Pass 'Actual native hierarchy matches the exact disabled-owner modal contract' {
      if($info.Frame -ne $fixture.Frame.ToInt64() -or $info.Modal -ne $fixture.Modal.ToInt64() -or $fixture.OwnerEnabled -or -not $fixture.ModalPresent){throw 'Native fixture identity differs'}
    }
    $fixture.SetBody('Changed navigation instruction')
    Refuse 'Changed real Edit content prevents actual native Agree' {[OpenNavX.SkagerWelcomeNative]::Agree($PID,$info)}
    if($fixture.AgreeCount -or $fixture.CancelCount -or -not $fixture.ModalPresent){throw 'Changed text sent or completed input'}
    $fixture.SetBody(('x'*5000))
    Refuse 'Oversized real Edit refuses instead of accepting truncated text' {$controlText.Invoke($null,[object[]]@($fixture.Body))}
    $fixture.SetBody($noticeText)
    $fixture.MoveAgree(1)
    Refuse 'Changed live Agree geometry prevents actual native dispatch' {[OpenNavX.SkagerWelcomeNative]::Agree($PID,$info)}
    if($fixture.AgreeCount -or $fixture.CancelCount -or -not $fixture.ModalPresent){throw 'Changed geometry sent or completed input'}
    $fixture.MoveAgree(0)
    Pass 'One actual native Agree pointer action completes the exact modal' {
      [OpenNavX.SkagerWelcomeNative]::AssertUnchanged($PID,$info)
      [OpenNavX.SkagerWelcomeNative]::Agree($PID,$info)
      $watch=[Diagnostics.Stopwatch]::StartNew()
      while(($fixture.ModalPresent -or -not $fixture.OwnerEnabled) -and $watch.ElapsedMilliseconds -lt 5000){Start-Sleep -Milliseconds 20}
      if($fixture.ModalPresent -or -not $fixture.OwnerEnabled -or $fixture.AgreeCount -ne 1 -or $fixture.CancelCount -ne 0){throw 'Native submission did not complete exactly one Agree and release its owner'}
    }
    $nativeReadback=[pscustomobject]@{status='passed';syntheticNativeHierarchy=$true;realNativeEdit=$true;actualAgreeInput=$true;agreeCount=$fixture.AgreeCount;cancelCount=$fixture.CancelCount;modalDismissed=$true;ownerEnabled=$true}
  }finally{
    try{if($null -ne $fixture){$fixture.Dispose()}}finally{[SkagerWelcomeFixture]::RestoreDpiAwareness($previousDpi)}
  }
}
[pscustomobject]@{
  status='passed';count=$checks.Count;checks=$checks.ToArray();nativeInteropCompiled=$true
  windowsApisInvoked=($null -ne $nativeReadback);nativeReadbackProbe=$nativeReadback
  applicationLaunched=$false;boatAccess=$false;hardwareCommands=$false;actualSkagerModalAcceptance=$false
} | ConvertTo-Json -Depth 6

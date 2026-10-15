# Import exact engine functions/catch AST; never run its installation entrypoint.
param([switch]$PortableContracts,[switch]$RegressionOnly,[string]$Engine='')
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'
$native=[Environment]::OSVersion.Platform -eq [PlatformID]::Win32NT
if(-not $native -and -not $PortableContracts){throw 'Native Windows required; portable adapters need explicit opt-in.'}
if(-not $Engine){$Engine=Join-Path $PSScriptRoot '../installer/windows/Lifecycle.ps1'}
$errors=@();$ast=[Management.Automation.Language.Parser]::ParseFile($Engine,[ref]$null,[ref]$errors)
if($errors.Count){throw ($errors|Out-String)}
$definitions=@($ast.EndBlock.Statements|Where-Object {$_ -is [Management.Automation.Language.FunctionDefinitionAst]})
foreach($name in @('Log','PlainPath','ReadJson','ShellGroups','AssertShellOwnership','Get-ProtectedBrokerObservation','Assert-NoProtectedBrokerEnrollment','Assert-LifecycleMutation','AtomicJson')){
  $found=@($definitions|Where-Object {$_.Name -ceq $name});if($found.Count -ne 1){throw "Missing unique engine function: $name"}
  $text=$found[0].Extent.Text
  if($PortableContracts -and $name -ceq 'AssertShellOwnership'){
    # Only COM construction is inert; exercise the original foreign-owner branch.
    $text=$text.Replace('New-Object -ComObject WScript.Shell','[pscustomobject]@{}')
  }
  . ([scriptblock]::Create($text))
}
$transactions=@($ast.EndBlock.Statements|Where-Object {$_ -is [Management.Automation.Language.TryStatementAst]})
if($transactions.Count -ne 1 -or $transactions[0].CatchClauses.Count -ne 1){throw 'Unexpected engine catch boundary.'}
$statements=@($transactions[0].CatchClauses[0].Body.Statements|Where-Object {$_ -isnot [Management.Automation.Language.ExitStatementAst]})
$failurePublisher=[scriptblock]::Create(($statements|ForEach-Object {$_.Extent.Text}) -join "`n")
if($PortableContracts){
  function PlainPath([string]$Path){
    $full=[IO.Path]::GetFullPath($Path);$walk=$full
    while($walk){if((Test-Path -LiteralPath $walk) -and ((Get-Item -LiteralPath $walk -Force).Attributes -band [IO.FileAttributes]::ReparsePoint)){throw 'Redirected path refused.'};$walk=[IO.Path]::GetDirectoryName($walk)}
    return $full
  }
}else{
  . (Join-Path $PSScriptRoot '../installer/windows/UpdateTransaction.ps1')
  $nativeSigned=${function:Assert-NoSignedUpdateArmed}
}
$script:Protected='absent';$script:ProtectedCalls=0;$script:SignedCalls=0;$script:BeforeSigned=$null
function Get-ProtectedBrokerObservation {
  $script:ProtectedCalls++
  if($script:Protected -ceq 'late-present'){return $script:ProtectedCalls -ge 2}
  switch($script:Protected){'absent'{return $false};'present'{return $true};'unknown'{return $null};default{throw 'Unknown protected observation.'}}
}
function Assert-NoSignedUpdateArmed([string]$InstallationRoot,[bool]$AllowMissingRoot=$false){
  $script:SignedCalls++
  if($script:BeforeSigned){& $script:BeforeSigned}
  if(-not $PortableContracts){& $nativeSigned $InstallationRoot $AllowMissingRoot;return}
  $null=PlainPath $InstallationRoot
  if(-not (Test-Path -LiteralPath $InstallationRoot)){
    if($AllowMissingRoot){return};throw 'Missing installation root.'
  }
  if(Test-Path -LiteralPath (Join-Path $InstallationRoot 'signed-update-armed.jsonl')){throw 'Signed session armed.'}
}
$checks=0
function Check([bool]$Value,[string]$Name){if(-not $Value){throw "FAILED: $Name"};$script:checks++;Write-Host "PASS: $Name"}
function Refuse([scriptblock]$Operation,[string]$Name){$failed=$false;try{& $Operation|Out-Null}catch{$failed=$true};Check $failed $Name}
$fixture=Join-Path ([IO.Path]::GetTempPath()) ('SkagerOperationReport-'+[guid]::NewGuid().ToString('N'))
$null=[IO.Directory]::CreateDirectory($fixture)
$Root=Join-Path $fixture 'absent-installation';$Programs=Join-Path $fixture 'programs'
$group=Join-Path $Programs 'SKAGER';$null=[IO.Directory]::CreateDirectory($group)
$Owner='OpenNavX.Alpha1.SideBySide.1';$Registry='HKCU:\Software\SkagerOperationReportUnused'
$Utf8=New-Object Text.UTF8Encoding($false);$SessionLog=New-Object 'System.Collections.Generic.List[string]'
$foreign=Join-Path $group 'Skager.lnk';[IO.File]::WriteAllText($foreign,'Foreign shortcut bytes must remain exact.',$Utf8)
$profile=Join-Path $fixture 'profile.ini';$stock=Join-Path $fixture 'stock.exe'
[IO.File]::WriteAllText($profile,'Original fixture profile bytes.',$Utf8);[IO.File]::WriteAllText($stock,'Original inert stock bytes.',$Utf8)
$before=@([IO.File]::ReadAllBytes($foreign),[IO.File]::ReadAllBytes($profile),[IO.File]::ReadAllBytes($stock))|ForEach-Object {[Convert]::ToBase64String($_)}
$Action='Install';$Report=Join-Path $fixture 'failure.json'
$value=@{status='failed';action=$Action;error='inert refusal';log=@()}
function CheckUnchanged {
  $now=@([IO.File]::ReadAllBytes($foreign),[IO.File]::ReadAllBytes($profile),[IO.File]::ReadAllBytes($stock))|ForEach-Object {[Convert]::ToBase64String($_)}
  Check (($before -join '|') -ceq ($now -join '|')) 'Original foreign shortcut, stock and profile bytes remain exact'
  Check (-not (Test-Path -LiteralPath $Root)) 'Failure publication never creates the installation root'
  Check (@(Get-ChildItem -LiteralPath $fixture -Filter '*.tmp').Count -eq 0) 'No unpublished report temporary file remains'
}
try{
  Refuse {AtomicJson $Report $value} 'Default durable writer still refuses an absent installation root'
  Check (-not (Test-Path -LiteralPath $Report)) 'Default refusal publishes no record'
  $script:SignedCalls=0
  try{AssertShellOwnership}catch{& $failurePublisher}
  Check ([IO.File]::Exists($Report)) 'Actual foreign-shell catch publishes its requested failure report before root creation'
  $record=ReadJson $Report
  Check ($record.status -ceq 'failed' -and $record.action -ceq 'Install' -and $record.error -like '*no verified SKAGER owner*') 'Original foreign-shell refusal reason and failed action remain exact'
  Check ($script:SignedCalls -eq 2) 'Failure report rechecks signed-session absence before write and publication'
  CheckUnchanged
  if($RegressionOnly){return}
  foreach($actionName in @('Install','Update')){
    $Action=$actionName;$value.action=$Action;[IO.File]::Delete($Report)
    AtomicJson $Report $value -ExternalFailureReport
    Check ((ReadJson $Report).action -ceq $Action) ('Exact external failed report supports '+$Action+' without root creation')
  }
  $Action='Install';$value.action=$Action
  foreach($protected in @('present','unknown','unavailable')){
    $script:Protected=$protected;$script:SignedCalls=0;[IO.File]::Delete($Report)
    Refuse {AtomicJson $Report $value -ExternalFailureReport} ('Protected '+$protected+' refuses external failure publication')
    Check ($script:SignedCalls -eq 0 -and -not [IO.File]::Exists($Report)) 'Protected refusal precedes signed observation and every report write'
  }
  $script:Protected='absent'
  $script:BeforeSigned={throw [UnauthorizedAccessException]::new('Inert inaccessible signed-session observation.')}
  Refuse {AtomicJson $Report $value -ExternalFailureReport} 'Inaccessible signed-session observation refuses before report writing'
  Check (-not [IO.File]::Exists($Report)) 'Unknown signed-session state publishes no report'
  $script:BeforeSigned=$null
  $script:Protected='late-present';$script:ProtectedCalls=0;$script:SignedCalls=0
  Refuse {AtomicJson $Report $value -ExternalFailureReport} 'Protected namespace appearing after temporary write refuses publication'
  Check ($script:SignedCalls -eq 1 -and -not [IO.File]::Exists($Report)) 'Late protected refusal prevents the second signed check and report publication'
  CheckUnchanged
  $script:Protected='absent'
  foreach($bad in @('different-report','root-report','root-child-report','success-status','wrong-action','unsupported-action')){
    $path=$Report;$candidate=$value.Clone();$Action='Install'
    switch($bad){
      'different-report'{$path=Join-Path $fixture 'different.json'}
      'root-report'{$path=$Root;$Report=$path}
      'root-child-report'{$path=Join-Path $Root 'state.json';$Report=$path}
      'success-status'{$candidate.status='passed'}
      'wrong-action'{$candidate.action='Update'}
      'unsupported-action'{$Action='Repair';$candidate.action='Repair'}
    }
    Refuse {AtomicJson $path $candidate -ExternalFailureReport} ('Failure-report scope refuses '+$bad)
    $Report=Join-Path $fixture 'failure.json'
  }
  $Action='Install';$value.action=$Action
  $null=[IO.Directory]::CreateDirectory($Root)
  $armed=Join-Path $Root 'signed-update-armed.jsonl';[IO.File]::WriteAllText($armed,'Inert armed presence; content does not authorize anything.',$Utf8)
  Refuse {AtomicJson $Report $value -ExternalFailureReport} 'An existing armed root still refuses failure-report publication'
  Check (-not [IO.File]::Exists($Report)) 'Armed refusal publishes no report'
  [IO.File]::Delete($armed)
  $script:SignedCalls=0
  $script:BeforeSigned={if($script:SignedCalls -eq 2){[IO.File]::WriteAllText($armed,'Late inert armed presence.',$Utf8)}}
  Refuse {AtomicJson $Report $value -ExternalFailureReport} 'A session armed after temporary write refuses publication'
  Check (-not [IO.File]::Exists($Report) -and [IO.File]::ReadAllText($armed) -ceq 'Late inert armed presence.') 'Late armed marker remains exact and no report is published'
  $script:BeforeSigned=$null;[IO.File]::Delete($armed);[IO.Directory]::Delete($Root)
  # The report path must remain plain; never redirect publication through a link.
  $target=Join-Path $fixture 'target';$link=Join-Path $fixture 'redirect'
  $null=[IO.Directory]::CreateDirectory($target)
  if($PortableContracts){$null=New-Item -ItemType SymbolicLink -Path $link -Target $target}
  else{& cmd.exe /d /c "mklink /J `"$link`" `"$target`""|Out-Null;if($LASTEXITCODE -ne 0){throw 'Disposable junction creation failed.'}}
  try{
    $Report=Join-Path $link 'failure.json'
    Refuse {AtomicJson $Report $value -ExternalFailureReport} 'Redirected external report ancestry refuses before writing'
    Check (-not [IO.File]::Exists((Join-Path $target 'failure.json'))) 'Redirect target remains untouched'
    $originalRoot=$Root;$Root=$link;$Report=Join-Path $fixture 'failure.json'
    try{Refuse {AtomicJson $Report $value -ExternalFailureReport} 'Redirected installation-root ancestry cannot grant failure publication'}finally{$Root=$originalRoot}
  }finally{if($PortableContracts){Remove-Item -LiteralPath $link}else{[IO.Directory]::Delete($link)}}
  $Report=Join-Path $fixture 'failure.json'
  CheckUnchanged
  [pscustomobject]@{status='passed';count=$checks;nativeSignedBarrier=(-not $PortableContracts);foreignShellCom=(-not $PortableContracts);protectedObservation='inert refusal adapters';applicationLaunched=$false;physicalOutput=$false}|ConvertTo-Json -Depth 3
}finally{if([IO.Directory]::Exists($fixture)){Remove-Item -LiteralPath $fixture -Recurse -Force}}

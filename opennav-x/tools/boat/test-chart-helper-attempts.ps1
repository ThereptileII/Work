# Disposable workspace tests for one-shot chart-helper attempt reservation.
# Never starts OpenCPN, the vendor helper, or the native shutdown transport.
[CmdletBinding()]
param([switch]$PortableContracts,[switch]$IsolatedLocal)

$native=[Environment]::OSVersion.Platform -eq 'Win32NT'
if(-not $native -and -not $PortableContracts){throw 'Explicit portable contracts required off Windows.'}
if($native -and -not $IsolatedLocal -and $env:GITHUB_ACTIONS -cne 'true'){throw 'Use disposable CI or explicit isolated local tests.'}

. (Join-Path $PSScriptRoot 'Common.ps1')
$script:actualAssertLocalPath=(Get-Command Assert-LocalPath).ScriptBlock
$script:actualWriteRecord=(Get-Command Write-Record).ScriptBlock
$script:actualGetDigest=(Get-Command Get-Digest).ScriptBlock
$testRoot=Join-Path ([IO.Path]::GetTempPath()) ('OpenNav chart-helper attempts '+[guid]::NewGuid().ToString('N'))
$null=New-Item -ItemType Directory -Path $testRoot
$script:portableAcl=@{}
$script:events=New-Object 'Collections.Generic.List[object]'
$script:wrapperContext=$null
$script:wrapperCold=$null
$script:wrapperRuns=0

function Assert-LocalPath([string]$Path) {
  if(-not $Path){throw 'Empty fixture path.'}
  $full=[IO.Path]::GetFullPath($Path)
  $prefix=$testRoot.TrimEnd([IO.Path]::DirectorySeparatorChar,[IO.Path]::AltDirectorySeparatorChar)+[IO.Path]::DirectorySeparatorChar
  if(-not $full.StartsWith($prefix,[StringComparison]::OrdinalIgnoreCase)){throw 'Chart-helper fixture escaped its disposable root.'}
  $walk=$full
  while($walk -and $walk.Length -ge $testRoot.Length){
    if((Test-Path -LiteralPath $walk) -and ((Get-Item -LiteralPath $walk -Force).Attributes -band [IO.FileAttributes]::ReparsePoint)){throw 'Redirected chart-helper fixture path refused.'}
    if($walk.Equals($testRoot,[StringComparison]::OrdinalIgnoreCase)){break}
    $walk=[IO.Path]::GetDirectoryName($walk)
  }
  if($native){return & $script:actualAssertLocalPath $full}
  return $full
}
function Get-Digest([string]$Path) {
  $algorithm=[Security.Cryptography.SHA256]::Create();$stream=[IO.File]::OpenRead($Path)
  try{return ([BitConverter]::ToString($algorithm.ComputeHash($stream))).Replace('-','').ToLowerInvariant()}
  finally{$stream.Dispose();$algorithm.Dispose()}
}
function Write-Record([string]$Path,$Record) {
  $script:events.Add([pscustomobject]@{kind='record';path=[IO.Path]::GetFullPath($Path)})
  & $script:actualWriteRecord $Path $Record
}
function Check([bool]$Value,[string]$Name) {if(-not $Value){throw $Name};$script:checks.Add($Name)}
function Refuse([string]$Name,[scriptblock]$Action) {$failed=$false;try{$null=& $Action}catch{$failed=$true};Check $failed $Name}
function New-FixtureWorkspace([string]$Name) {
  $workspace=Join-Path $testRoot $Name
  $runs=Join-Path $workspace 'runs'
  [void](New-Item -ItemType Directory -Path $runs -Force)
  return $workspace
}
function New-FixtureIntent([string]$Workspace,[string]$Name) {
  $run=Join-Path (Join-Path $Workspace 'runs') $Name
  [void](New-Item -ItemType Directory -Path $run -Force)
  $intent=Join-Path $run 'intent.json'
  & $script:actualWriteRecord $intent @{owner='fixture';kind='attempt-test'}
  return $intent
}
function Assert-NativePrivateLedger([string]$Ledger,[string]$Sid) {
  $allowed=@($Sid,'S-1-5-18','S-1-5-32-544')
  foreach($path in @($Ledger,(Join-Path $Ledger 'chart-helper-shutdown-51001-639000000000000000.json'))){
    if(-not (Test-Path -LiteralPath $path)){continue}
    $acl=Microsoft.PowerShell.Security\Get-Acl -LiteralPath $path
    if($path -ieq $Ledger -and -not $acl.AreAccessRulesProtected){throw 'Native ledger DACL is inheriting.'}
    if($acl.GetOwner([Security.Principal.SecurityIdentifier]).Value -cnotin $allowed){throw 'Native ledger has an unapproved owner.'}
    foreach($rule in $acl.GetAccessRules($true,$true,[Security.Principal.SecurityIdentifier])){
      if($rule.IdentityReference.Value -cnotin $allowed -or $rule.AccessControlType -ne [Security.AccessControl.AccessControlType]::Allow){throw 'Native ledger has an unapproved access rule.'}
    }
  }
}
function Ensure-PortableChartHelperGlobalLedger([string]$Ledger,[string]$Sid) {
  $null=New-Item -ItemType Directory -Path $Ledger -Force
  $key=[IO.Path]::GetFullPath($Ledger)
  if(-not $script:portableAcl.ContainsKey($key)){$script:portableAcl[$key]=@{protected=$true;owner=$Sid;rules=@($Sid,'S-1-5-18','S-1-5-32-544')}}
  $state=$script:portableAcl[$key]
  if(-not $state.protected -or $state.owner -cnotin @($Sid,'S-1-5-18','S-1-5-32-544') -or @($state.rules|Where-Object {$_ -notin @($Sid,'S-1-5-18','S-1-5-32-544')}).Count){throw 'Portable fixture ledger ACL marker is unapproved.'}
}

try {
  $script:actualGetDigest=(Get-Command Get-Digest).ScriptBlock
  $script:checks=New-Object 'Collections.Generic.List[string]'
  $sourcePath=Join-Path $PSScriptRoot 'ChartHelperShutdown.ps1'
  $source=[IO.File]::ReadAllText($sourcePath)
  if(-not $native){
    # Keep production locator, path, scan and publication code intact. Only
    # replace its Windows ACL helper with an explicit synthetic ACL contract.
    $parseErrors=$null;$tokens=$null
    $ast=[Management.Automation.Language.Parser]::ParseInput($source,[ref]$tokens,[ref]$parseErrors)
    if($parseErrors.Count){throw ($parseErrors|Out-String)}
    $aclFunction=$ast.Find({param($node) $node -is [Management.Automation.Language.FunctionDefinitionAst] -and $node.Name -ceq 'Ensure-ChartHelperGlobalLedger'},$true)
    if(-not $aclFunction){throw 'Named production ledger ACL seam is missing.'}
    $source=$source.Remove($aclFunction.Extent.StartOffset,$aclFunction.Extent.EndOffset-$aclFunction.Extent.StartOffset)
    $portableAclFunction=@'
function Ensure-ChartHelperGlobalLedger([string]$Ledger,[string]$Sid) {
  $ledger=Assert-LocalPath $Ledger
  Ensure-PortableChartHelperGlobalLedger $ledger $Sid
}
'@
    $source=$source.Insert($aclFunction.Extent.StartOffset,$portableAclFunction)
  }
  $source=[regex]::Replace($source,'(?m)^\. \(Join-Path \$PSScriptRoot ''StockReview\.ps1''\)\r?\n','')
  if(-not $native){
    $portablePathCheck=@'
$intent.StartsWith((Join-Path $workspace 'runs')+'\',[StringComparison]::OrdinalIgnoreCase)
'@ -replace '\r?\n$',''
    $portablePathReplacement=@'
$intent.StartsWith((Join-Path $workspace 'runs')+[IO.Path]::DirectorySeparatorChar,[StringComparison]::OrdinalIgnoreCase)
'@ -replace '\r?\n$',''
    $source=$source.Replace($portablePathCheck,$portablePathReplacement)
  }
  . ([scriptblock]::Create($source))

  $sid=if($native){[Security.Principal.WindowsIdentity]::GetCurrent().User.Value}else{'S-1-5-21-100-1001'}
  $pidValue=51001;$ticks=[long]639000000000000000
  $workspace=New-FixtureWorkspace 'direct'
  $intent=New-FixtureIntent $workspace 'first-capture'
  $ledger=Join-Path $workspace 'chart-helper-attempts'
  $marker=Join-Path $ledger ('chart-helper-shutdown-'+$pidValue+'-'+$ticks+'.json')
  $first=New-ChartHelperGlobalLocator $workspace $sid $pidValue $ticks $intent 'OpenNavX.ChartHelperShutdown.1'
  Check ($first -ieq $marker -and (Test-Path -LiteralPath $marker)) 'First exact attempt publishes one workspace-global locator'
  $record=Read-Record $marker
  Check ($record.owner -ceq 'OpenNavX.ChartHelperShutdown.1' -and $record.intent -ceq $intent -and $record.intentSha256 -ceq (Get-Digest $intent)) 'Global locator binds exact owner, private intent and digest'
  $originalMarkerHash=Get-Digest $marker
  if($native){Assert-NativePrivateLedger $ledger $sid;Check $true 'Native ledger and marker have protected allowlisted Windows ACLs'}
  else {Check ($script:portableAcl[[IO.Path]::GetFullPath($ledger)].protected) 'Portable ACL marker explicitly substitutes for Windows ACL inspection'}

  Refuse 'Duplicate attempt is refused without overwriting its global marker' {New-ChartHelperGlobalLocator $workspace $sid $pidValue $ticks $intent 'OpenNavX.ChartHelperShutdown.1'}
  $newRunIntent=New-FixtureIntent $workspace 'fresh-capture'
  Refuse 'Fresh capture cannot authorize a second attempt for the same helper identity' {New-ChartHelperGlobalLocator $workspace $sid $pidValue $ticks $newRunIntent 'OpenNavX.ChartHelperShutdown.1'}
  Check ((Get-Digest $marker) -ceq $originalMarkerHash) 'Original global marker remains unchanged after duplicate refusals'

  $legacyWorkspace=New-FixtureWorkspace 'legacy'
  $legacyIntent=New-FixtureIntent $legacyWorkspace 'old-run'
  $legacyName='chart-helper-shutdown-51002-639000000000000001.json'
  & $script:actualWriteRecord (Join-Path (Split-Path -Parent $legacyIntent) $legacyName) @{owner='legacy'}
  Refuse 'Legacy per-run locator refuses migration to a new global attempt' {New-ChartHelperGlobalLocator $legacyWorkspace $sid 51002 ([long]639000000000000001) $legacyIntent 'OpenNavX.OrphanChartHelperShutdown.1'}
  Check (-not (Test-Path -LiteralPath (Join-Path $legacyWorkspace 'chart-helper-attempts'))) 'Legacy refusal occurs before creating a global ledger'

  if($native){
    $aclWorkspace=New-FixtureWorkspace 'foreign-acl';$aclIntent=New-FixtureIntent $aclWorkspace 'first-capture'
    $aclLedger=Join-Path $aclWorkspace 'chart-helper-attempts'
    $null=New-ChartHelperGlobalLocator $aclWorkspace $sid 51003 ([long]639000000000000002) $aclIntent 'OpenNavX.ChartHelperShutdown.1'
    $savedAcl=Microsoft.PowerShell.Security\Get-Acl -LiteralPath $aclLedger
    try {
      $tampered=Microsoft.PowerShell.Security\Get-Acl -LiteralPath $aclLedger
      $everyone=New-Object Security.Principal.SecurityIdentifier('S-1-1-0')
      $tampered.AddAccessRule((New-Object Security.AccessControl.FileSystemAccessRule($everyone,'Read','Allow')))
      Microsoft.PowerShell.Security\Set-Acl -LiteralPath $aclLedger -AclObject $tampered
      Refuse 'Foreign ACL on an existing global ledger refuses a fresh helper attempt' {New-ChartHelperGlobalLocator $aclWorkspace $sid 51004 ([long]639000000000000003) $aclIntent 'OpenNavX.ChartHelperShutdown.1'}
      Check (-not (Test-Path -LiteralPath (Join-Path $aclLedger 'chart-helper-shutdown-51004-639000000000000003.json'))) 'Foreign ACL refusal publishes no marker'
    } finally {Microsoft.PowerShell.Security\Set-Acl -LiteralPath $aclLedger -AclObject $savedAcl}
    Check $true 'Native ACL tamper fixture restored its original DACL'
    $restoredMarker=New-ChartHelperGlobalLocator $aclWorkspace $sid 51004 ([long]639000000000000003) $aclIntent 'OpenNavX.ChartHelperShutdown.1'
    Check (Test-Path -LiteralPath $restoredMarker) 'Restored native ACL permits a fresh distinct attempt'
  } else {
    $foreignWorkspace=New-FixtureWorkspace 'foreign-acl';$foreignIntent=New-FixtureIntent $foreignWorkspace 'first-capture'
    $foreignLedger=Join-Path $foreignWorkspace 'chart-helper-attempts'
    $null=New-ChartHelperGlobalLocator $foreignWorkspace $sid 51003 ([long]639000000000000002) $foreignIntent 'OpenNavX.ChartHelperShutdown.1'
    $script:portableAcl[[IO.Path]::GetFullPath($foreignLedger)].rules+=@('S-1-1-0')
    Refuse 'Portable foreign-ACL marker refuses a fresh helper attempt' {New-ChartHelperGlobalLocator $foreignWorkspace $sid 51004 ([long]639000000000000003) $foreignIntent 'OpenNavX.ChartHelperShutdown.1'}
  }

  $reparseWorkspace=New-FixtureWorkspace 'reparse'
  $reparseIntent=New-FixtureIntent $reparseWorkspace 'first-capture'
  $outside=Join-Path $testRoot 'outside-ledger';[void](New-Item -ItemType Directory -Path $outside)
  $reparseLedger=Join-Path $reparseWorkspace 'chart-helper-attempts'
  if($native){$null=New-Item -ItemType Junction -Path $reparseLedger -Target $outside}
  else {$null=New-Item -ItemType SymbolicLink -Path $reparseLedger -Target $outside}
  Refuse 'Reparse-point global ledger refuses attempt publication' {New-ChartHelperGlobalLocator $reparseWorkspace $sid 51005 ([long]639000000000000004) $reparseIntent 'OpenNavX.ChartHelperShutdown.1'}
  Check (@(Get-ChildItem -LiteralPath $outside -Force).Count -eq 0) 'Reparse refusal writes no marker through redirected path'

  # Execute both old wrappers with only their source imports, native Add-Type,
  # OS guard and static transport call substituted. The real global locator,
  # Write-Record ordering, and legacy per-run marker writes remain active.
  function New-PreparationDirectory($Context,[string]$Purpose) {
    $script:wrapperRuns++
    $path=Join-Path (Join-Path $Context.workspace 'runs') ('wrapper-'+$script:wrapperRuns+'-'+$Purpose)
    [void](New-Item -ItemType Directory -Path $path -Force)
    return $path
  }
  function Read-ChartHelperContext {return $script:wrapperContext}
  function Read-OrphanChartHelper {return $script:wrapperContext}
  function Invoke-RecordedShutdown {param($PidValue,$Start,$Parent,$Session,$Path);$script:events.Add([pscustomobject]@{kind='shutdown'});return [pscustomobject]@{Succeeded=$true;testTransport='substituted'}}
  function Get-EventIndex([string]$Kind,[string]$Path) {
    for($index=0;$index -lt $script:events.Count;$index++){
      $event=$script:events[$index]
      if($event.kind -ceq $Kind -and (-not $Path -or $event.path -ieq $Path)){return $index}
    }
    return -1
  }
  function Invoke-Wrapper([string]$Path,[hashtable]$Arguments) {
    $body=[IO.File]::ReadAllText($Path)
    $body=[regex]::Replace($body,'(?m)^\. \(Join-Path \$PSScriptRoot ''[^'']+''\)\r?\n','')
    $body=[regex]::Replace($body,'(?m)^if\(\[Environment\]::OSVersion\.Platform -ne ''Win32NT''\)\{[^\r\n]*\}\r?\n','')
    $body=[regex]::Replace($body,'(?m)^Add-Type -Path [^\r\n]+\r?\n','')
    $body=$body.Replace("(Get-Digest (Join-Path `$PSScriptRoot 'ChartHelperShutdownNative.cs'))", "'fixture-native-helper-sha256'")
    $body=$body.Replace('[OpenNavX.ChartHelperShutdownNative]::Shutdown($HelperProcessId,$ExpectedHelperStartedUtcTicks,[int]$context.launch.pid,[int]$context.launch.sessionId,$context.helperPath)', '$(Invoke-RecordedShutdown -PidValue $HelperProcessId -Start $ExpectedHelperStartedUtcTicks -Parent $context.launch.pid -Session $context.launch.sessionId -Path $context.helperPath)')
    $body=$body.Replace('[OpenNavX.ChartHelperShutdownNative]::Shutdown($HelperProcessId,$ExpectedHelperStartedUtcTicks,$ParentProcessId,[int]$context.helper.sessionId,$context.helper.path)', '$(Invoke-RecordedShutdown -PidValue $HelperProcessId -Start $ExpectedHelperStartedUtcTicks -Parent $ParentProcessId -Session $context.helper.sessionId -Path $context.helper.path)')
    $errors=$null;$tokens=$null;$null=[Management.Automation.Language.Parser]::ParseInput($body,[ref]$tokens,[ref]$errors)
    if($errors.Count){throw ($errors|Out-String)}
    & ([scriptblock]::Create($body)) @Arguments | Out-Null
  }
  $wrapperWorkspace=New-FixtureWorkspace 'wrappers'
  $wrapperCold=Join-Path (Join-Path $wrapperWorkspace 'runs') 'cold-baseline'
  [void](New-Item -ItemType Directory -Path $wrapperCold -Force)
  $wrapperPid=52001;$wrapperTicks=[long]639000000000000010
  $helper=[pscustomobject]@{pid=$wrapperPid;parentPid=52000;sessionId=1;sid=$sid;startedUtc=[datetime]::UtcNow.ToString('o');path='C:\fixture\oexserverd.exe'}
  $launch=[pscustomobject]@{pid=52000;sessionId=1;sid=$sid}
  $script:wrapperContext=[pscustomobject]@{launch=$launch;helper=$helper;helperPath=$helper.path;cold=$wrapperCold;context=[pscustomobject]@{workspace=$wrapperWorkspace;sid=$sid}}
  $script:events.Clear()
  $normalArgs=@{Workspace=$wrapperWorkspace;LaunchResult=(New-FixtureIntent $wrapperWorkspace 'launch');ExpectedLaunchSha256=('a'*64);ExpectedRequestSha256=('b'*64);HelperProcessId=$wrapperPid;ExpectedHelperStartedUtcTicks=$wrapperTicks}
  Invoke-Wrapper (Join-Path $PSScriptRoot 'stop-chart-helper.ps1') $normalArgs
  $globalPath=Join-Path (Join-Path $wrapperWorkspace 'chart-helper-attempts') ('chart-helper-shutdown-'+$wrapperPid+'-'+$wrapperTicks+'.json')
  $legacyPath=Join-Path $wrapperCold ([IO.Path]::GetFileName($globalPath))
  $globalIndex=Get-EventIndex 'record' $globalPath
  $legacyIndex=Get-EventIndex 'record' $legacyPath
  $shutdownIndex=Get-EventIndex 'shutdown' ''
  Check ($globalIndex -ge 0 -and $legacyIndex -ge 0 -and $shutdownIndex -ge 0 -and $globalIndex -lt $legacyIndex -and $legacyIndex -lt $shutdownIndex) 'Normal stop wrapper reserves globally before legacy locator and transport'
  $normalShutdownCount=@($script:events|Where-Object {$_.kind -ceq 'shutdown'}).Count
  Refuse 'Normal stop wrapper refuses duplicate retry before transport' {Invoke-Wrapper (Join-Path $PSScriptRoot 'stop-chart-helper.ps1') $normalArgs}
  Check (@($script:events|Where-Object {$_.kind -ceq 'shutdown'}).Count -eq $normalShutdownCount) 'Normal duplicate retry does not dispatch transport'

  $script:events.Clear();$orphanPid=52002;$orphanTicks=[long]639000000000000011
  $orphanHelper=[pscustomobject]@{pid=$orphanPid;parentPid=51999;sessionId=1;sid=$sid;startedUtc=[datetime]::UtcNow.ToString('o');path='C:\fixture\oexserverd.exe'}
  $script:wrapperContext=[pscustomobject]@{helper=$orphanHelper;cold=$wrapperCold;context=[pscustomobject]@{workspace=$wrapperWorkspace;sid=$sid};helperPath=$orphanHelper.path}
  $orphanRecord=New-FixtureIntent $wrapperWorkspace 'orphan-source'
  $orphanArgs=@{Workspace=$wrapperWorkspace;Record=$orphanRecord;ExpectedRecordSha256=('c'*64);HelperProcessId=$orphanPid;ParentProcessId=51999;ExpectedHelperStartedUtcTicks=$orphanTicks}
  Invoke-Wrapper (Join-Path $PSScriptRoot 'stop-orphan-chart-helper.ps1') $orphanArgs
  $orphanGlobal=Join-Path (Join-Path $wrapperWorkspace 'chart-helper-attempts') ('chart-helper-shutdown-'+$orphanPid+'-'+$orphanTicks+'.json')
  $orphanLegacy=Join-Path $wrapperCold ([IO.Path]::GetFileName($orphanGlobal))
  $orphanGlobalIndex=Get-EventIndex 'record' $orphanGlobal
  $orphanLegacyIndex=Get-EventIndex 'record' $orphanLegacy
  $orphanShutdownIndex=Get-EventIndex 'shutdown' ''
  Check ($orphanGlobalIndex -ge 0 -and $orphanLegacyIndex -ge 0 -and $orphanShutdownIndex -ge 0 -and $orphanGlobalIndex -lt $orphanLegacyIndex -and $orphanLegacyIndex -lt $orphanShutdownIndex) 'Orphan stop wrapper reserves globally before legacy locator and transport'
  $orphanShutdownCount=@($script:events|Where-Object {$_.kind -ceq 'shutdown'}).Count
  Refuse 'Orphan stop wrapper refuses duplicate retry before transport' {Invoke-Wrapper (Join-Path $PSScriptRoot 'stop-orphan-chart-helper.ps1') $orphanArgs}
  Check (@($script:events|Where-Object {$_.kind -ceq 'shutdown'}).Count -eq $orphanShutdownCount) 'Orphan duplicate retry does not dispatch transport'
  foreach($file in @('ChartHelperShutdown.ps1','stop-chart-helper.ps1','stop-orphan-chart-helper.ps1')){
    $errors=$null;$tokens=$null;$null=[Management.Automation.Language.Parser]::ParseFile((Join-Path $PSScriptRoot $file),[ref]$tokens,[ref]$errors)
    Check ($errors.Count -eq 0) ('Parses '+$file)
  }
  [pscustomobject]@{status='passed';groups=$script:checks.Count;checks=$script:checks.ToArray();platform=if($native){'native-windows-disposable'}else{'portable-with-explicit-acl-substitution'};nativeAclExecuted=$native;nativeShutdownTransportExecuted=$false;vendorHelperExecuted=$false;boatAccess=$false}|ConvertTo-Json -Depth 6
} finally {Remove-Item -LiteralPath $testRoot -Recurse -Force -ErrorAction SilentlyContinue}

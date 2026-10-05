# Disposable contracts only. Does not inspect actual boat/profile/plugin state.
[CmdletBinding()]
param([switch]$PortableContracts,[switch]$IsolatedLocal,[switch]$AdoptionFixture,[switch]$ResourceAdoptionFixture,[switch]$PreservationFixture)
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'
$native=[Environment]::OSVersion.Platform -eq 'Win32NT'
$originalTestSearchPath=$env:PATH
if($PreservationFixture -and ($AdoptionFixture -or $ResourceAdoptionFixture)){throw 'Preservation and migration fixtures run independently.'}
if (-not $native -and -not $PortableContracts) { throw 'Native Windows required unless portable contracts are explicitly requested.' }
if ($native -and -not $IsolatedLocal -and $env:GITHUB_ACTIONS -ne 'true') { throw 'Use disposable CI or explicitly choose isolated temporary-file tests.' }
. (Join-Path $PSScriptRoot 'Commissioning.ps1')
$testRoot=Join-Path ([IO.Path]::GetTempPath()) ('OpenNav commissioning '+[guid]::NewGuid().ToString('N'))
$null=New-Item -ItemType Directory -Path $testRoot
if (-not $native) {
  function Assert-LocalPath([string]$Path) {
    $full=[IO.Path]::GetFullPath($Path)
    if ($full -ne $testRoot -and -not $full.StartsWith($testRoot+'/',[StringComparison]::Ordinal)) { throw 'Portable test path escaped its unique temporary root.' }
    $walk=$full
    while ($walk -ne $testRoot) {
      if ((Test-Path -LiteralPath $walk) -and ((Get-Item -LiteralPath $walk -Force).Attributes -band [IO.FileAttributes]::ReparsePoint)) { throw 'Reparse path refused.' }
      $walk=[IO.Path]::GetDirectoryName($walk)
    }
    return $full
  }
}
$checks=New-Object 'Collections.Generic.List[string]'
function Reject([scriptblock]$Action,[string]$Reason) {
  $rejected=$false;try {$null=& $Action} catch {$rejected=$true}
  if (-not $rejected) { throw ('Unsafe operation accepted: '+$Reason) }
}
function Clone($Value) { return $Value | ConvertTo-Json -Depth 16 | ConvertFrom-Json }
try {
  $entryPath=Join-Path $PSScriptRoot 'commission-read-only.ps1'
  $tokens=$null;$errors=$null
  $null=[Management.Automation.Language.Parser]::ParseFile($entryPath,[ref]$tokens,[ref]$errors)
  if ($errors.Count) { throw ($errors | Out-String) }
  $checks.Add('Entry point parses without touching native environment')
  $connection='0;0;;0;1;COM8;115200;0;1;0;;0;;0;0;1;0;1;Gateway;0;;0'
  $gps='0;0;;0;0;COM6;4800;0;0;0;;0;;0;0;1;0;1;GPS;0;;0'
  $text="[Settings]`r`nPersistActiveRoute=0`r`n[Settings/NMEADataSource]`r`nDataConnections=$gps|$connection`r`n[Directories]`r`nChartDir=original`r`n[Other]`r`nLabel=Skärgård`r`n"
  $encoding=New-Object Text.UTF8Encoding($false,$true)
  $bytes=$encoding.GetBytes($text)
  $changed=Get-CommissioningInputBytes $bytes
  $different=@(0..($bytes.Length-1) | Where-Object {$bytes[$_] -ne $changed[$_]})
  if ($changed.Length -ne $bytes.Length -or $different.Count -ne 1 -or $bytes[$different[0]] -ne 49 -or $changed[$different[0]] -ne 48) { throw 'Expected exactly one 1-to-0 byte mutation.' }
  $checks.Add('Exactly one COM8 direction byte changes; GPS, UTF8, CRLF, filters and every other byte remain exact')
  $bom=[byte[]](@(239,187,191)+$bytes)
  $bomChanged=Get-CommissioningInputBytes $bom
  if ($bomChanged[0] -ne 239 -or $bomChanged[1] -ne 187 -or $bomChanged[2] -ne 191 -or $bomChanged.Length -ne $bom.Length) { throw 'UTF8 BOM not preserved.' }
  $checks.Add('Optional UTF8 BOM is retained without text reserialization')
  $case=0
  foreach ($bad in @(
    $text.Replace('COM8','COM9'),
    $text.Replace('[Settings/NMEADataSource]','[OtherSource]'),
    $text.Replace($connection,$connection+'|'+$connection),
    $text.Replace('1;COM8','0;COM8'),
    $text.Replace('COM8;115200;0;1','COM8;115200;0;0'),
    $text.Replace('COM8;115200;0;1','COM8;115200;0;2'),
    $text.Replace('COM6;4800;0;0','COM6;4800;0;1'),
    $text.Replace(';0;0;1;0;1;Gateway',';0;0;1;0;0;Gateway'),
    $text.Replace(';0;0;1;0;1;Gateway',';0;1;1;0;1;Gateway'),
    ($text+"DataConnections=$connection`r`n"))) {
    $case++;Reject {Get-CommissioningInputBytes ($encoding.GetBytes($bad))} ('missing, duplicate, already changed or ambiguous COM8/other output case '+$case)
  }
  Reject {Get-CommissioningInputBytes ([byte[]]@(255,254,255))} 'malformed external UTF8'
  $checks.Add('Wrong/duplicate port, protocol, direction, enabled state, Garmin flag, other output and malformed input are refused')
  $baseline=Join-Path $testRoot 'baseline.ini';$inputProfile=Join-Path $testRoot 'input.ini';$current=Join-Path $testRoot 'current.ini'
  [IO.File]::WriteAllBytes($baseline,$bytes);[IO.File]::WriteAllBytes($inputProfile,$changed);[IO.File]::WriteAllBytes($current,$changed)
  Assert-InputOnlyProfile (Read-ProfileForAudit $inputProfile)
  $diff=@(Get-CommissioningIniDiff $baseline $inputProfile)
  if ($diff.Count -ne 1 -or $diff[0].key -cne 'Settings/NMEADataSource/DataConnections') { throw 'Unexpected semantic edit.' }
  $checks.Add('Strict profile audit sees only the intended connection direction change')
  [IO.File]::AppendAllText($current,"Screen=Instruments`r`n")
  Assert-CommissioningRestoreIni $inputProfile $current
  if (@(Get-CommissioningIniDiff $inputProfile $current).Count -ne 1) { throw 'Post-session changes missing from private diff.' }
  foreach ($bad in @($text,$encoding.GetString($changed).Replace('ChartDir=original','ChartDir=other'),$encoding.GetString($changed).Replace('COM6','COM7'))) {
    [IO.File]::WriteAllText($current,$bad)
    Reject {Assert-CommissioningRestoreIni $inputProfile $current} 'output/source/chart edit cannot be silently erased'
  }
  $checks.Add('Restoration reviews view persistence but refuses changed connection, output or chart-directory configuration')
  $plugins=Join-Path $testRoot 'plugins';$null=New-Item -ItemType Directory -Path $plugins
  $safe=Join-Path $plugins 'safe_pi.dll';$unsafe=Join-Path $plugins 'unsafe_pi.dll';$helper=Join-Path $plugins 'helper.exe'
  [IO.File]::WriteAllText($safe,'safe fixture');[IO.File]::WriteAllText($unsafe,'unsafe fixture');[IO.File]::WriteAllText($helper,'inert helper fixture')
  $trees=@(Get-CommissioningTrees @($plugins));$candidates=@(Get-CommissioningCandidates $trees)
  if ($candidates.Count -ne 2 -or @($trees[0].entries).Count -ne 3) { throw 'Candidate/dependency inventory incomplete.' }
  $treeContext=[pscustomobject]@{pluginRoots=@($plugins)}
  $treeInventory=[pscustomobject]@{context=$treeContext;trees=$trees;plugins=$candidates}
  Assert-CommissioningInventory $treeInventory $treeContext
  $bad=Clone $treeInventory;$bad.trees=@()
  Reject {Assert-CommissioningInventory $bad $treeContext} 'omitted real loader root'
  $bad=Clone $treeInventory;$bad.trees=@($bad.trees[0],$bad.trees[0])
  Reject {Assert-CommissioningInventory $bad $treeContext} 'duplicate loader root'
  $bad=Clone $treeInventory;$bad.plugins=@($bad.plugins[0])
  Reject {Assert-CommissioningInventory $bad $treeContext} 'omitted candidate from declared inventory'
  $checks.Add('Inventory covers the exact actual loader-root set and complete candidate list without duplicates or omissions')
  $evidence=Join-Path $testRoot 'source-review.txt';[IO.File]::WriteAllText($evidence,'Explicit fixture source review, no real plugin qualification.')
  $decisions=@($candidates | ForEach-Object {
    @{path=$_.path;sha256=$_.sha256;decision=$(if($_.path -eq $safe){'retain'}else{'quarantine'});reason='Fixture review';sourceBoundary='Constructor and callbacks fixture';sourceRevision=$(if($_.path -eq $safe){'1'*40}else{$null});startupAndIdleReadOnly=($_.path -eq $safe);evidencePath=$evidence;evidenceSha256=(Get-Digest $evidence)}
  })
  Assert-CommissioningReview $candidates $decisions
  $checks.Add('Every candidate is reviewed; inert helper/dependency bytes stay in the full inventory without execution')
  Reject {Assert-CommissioningReview $candidates @($decisions[0])} 'missing candidate'
  Reject {Assert-CommissioningReview $candidates @($decisions[0],$decisions[0])} 'duplicate candidate'
  $bad=Clone $decisions;$bad[0].sha256='0'*64
  Reject {Assert-CommissioningReview $candidates $bad} 'different installed DLL'
  $bad=Clone $decisions;($bad | Where-Object {$_.decision -ceq 'retain'}).sourceRevision=$null
  Reject {Assert-CommissioningReview $candidates $bad} 'unqualified retained source'
  $bad=Clone $decisions;($bad | Where-Object {$_.decision -ceq 'retain'}).startupAndIdleReadOnly='true'
  Reject {Assert-CommissioningReview $candidates $bad} 'truthy string cannot manufacture read-only assertion'
  $bad=Clone $decisions;($bad | Where-Object {$_.decision -ceq 'quarantine'}).startupAndIdleReadOnly=$true
  Reject {Assert-CommissioningReview $candidates $bad} 'unsafe DLL mislabeled read-only'
  $checks.Add('Missing/duplicate/mismatched DLLs and fabricated or unknown retained provenance fail closed')
  [IO.File]::AppendAllText($evidence,' changed')
  Reject {Assert-CommissioningReview $candidates $decisions} 'review evidence changed'
  [IO.File]::AppendAllText($helper,' changed')
  Reject {Assert-PreparationTree $trees[0]} 'unreviewed helper change'
  $checks.Add('Evidence and non-plugin dependency changes invalidate the review')
  $quarantine=Join-Path $testRoot 'quarantine';$pathDirectory=Join-Path $testRoot 'search';$null=New-Item -ItemType Directory -Path $pathDirectory
  Assert-CommissioningQuarantine $quarantine @($plugins) $pathDirectory
  Reject {Assert-CommissioningQuarantine (Join-Path $plugins 'disabled') @($plugins) $pathDirectory} 'recursive disabled directory'
  Reject {Assert-CommissioningQuarantine $quarantine @($plugins) $testRoot} 'PATH ancestor'
  Reject {Assert-CommissioningQuarantine $quarantine @($plugins) ($pathDirectory+';')} 'empty PATH entry'
  $checks.Add('Quarantine is outside recursive roots and PATH; disabling a DLL in a subdirectory is refused')
  $osRoot=Join-Path $testRoot 'fixture-windows';$null=New-Item -ItemType Directory -Path $osRoot,(Join-Path $osRoot 'System32')
  $launch=Get-CommissioningLaunchEnvironment (Join-Path $plugins 'opencpn.exe') $osRoot
  if ($launch.workingDirectory -ine $plugins -or $launch.path -cne (@($plugins,(Join-Path $osRoot 'System32'),$osRoot) -join ';')) { throw 'Child search path was not explicit and bounded.' }
  Assert-CommissioningQuarantine $quarantine @($plugins) $launch.path
  Reject {Get-CommissioningLaunchEnvironment (Join-Path $plugins 'opencpn.exe') (Join-Path $testRoot 'missing-windows')} 'missing operating system roots'
  $checks.Add('Child PATH and working directory are explicit app/System32/Windows paths without inheriting ambiguous entries')
  if ($native) {
    # Real transaction code, disposable paths/identity only. No installed-state,
    # registry, process, serial, sensor, task or real profile calls are made.
    $workspace=Join-Path $testRoot 'workspace';$profileDirectory=Join-Path $testRoot 'profile';$app=Join-Path $testRoot 'application';$managed=Join-Path $testRoot 'managed'
    $null=New-Item -ItemType Directory -Path $workspace,$profileDirectory,$app,$managed,(Join-Path $app 'plugins')
    $fixtureIni=Join-Path $profileDirectory 'opencpn.ini'
    $padding=21380-$bytes.Length
    [byte[]]$fixtureBytes=@($bytes)+@($encoding.GetBytes('#'+('x'*($padding-3))+"`r`n"))
    if ($fixtureBytes.Length -ne 21380) { throw 'Fixture must exercise exact-length production replacement.' }
    [IO.File]::WriteAllBytes($fixtureIni,$fixtureBytes)
    $unsafe=Join-Path $managed 'unsafe_pi.dll';$unsafeSecond=Join-Path $managed 'unsafe_second_pi.dll';$safe=Join-Path $managed 'safe_pi.dll'
    [IO.File]::WriteAllText($unsafe,'quarantined fixture');[IO.File]::WriteAllText($unsafeSecond,'second quarantined fixture');[IO.File]::WriteAllText($safe,'retained fixture')
    $fixtureSid=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value
    $env:PATH=$pathDirectory # This test process only; restored in finally.
    $fixtureContext=[pscustomobject]@{sid=$fixtureSid;session=1;localAppData=$testRoot;profile=$profileDirectory;application=$app;managed=$managed;workspace=$workspace;executable=(Join-Path $app 'opencpn.exe');installation=$null;pluginRoots=@($managed,(Join-Path $app 'plugins') | Sort-Object);launchEnvironment=(Get-CommissioningLaunchEnvironment (Join-Path $app 'opencpn.exe') $osRoot)}
    function Get-CommissioningContext([string]$Workspace) {
      if ($Workspace -ine $fixtureContext.workspace) { throw 'Fixture context escaped.' };return $fixtureContext
    }
    function Assert-PreparationClosed([string[]]$Roots) {
      foreach($root in $Roots){if(-not $root.StartsWith($testRoot+'\',[StringComparison]::OrdinalIgnoreCase)){throw 'Fixture process guard escaped.'}}
    }
    # Only dependency import is removed: no production transaction lines change.
    $body=[IO.File]::ReadAllText($entryPath).Replace(". (Join-Path `$PSScriptRoot 'Commissioning.ps1')",'')
    $invoke=[scriptblock]::Create($body)
    Reject {& $invoke -Action Inventory -Workspace $workspace} 'public baseline cannot accept synthetic profile'
    $checks.Add('Public transaction rejects a synthetic baseline before preparation')
    # Explicit test-only binding after verifying public rejection. Actual script
    # offers no parameter to override the real recovered baseline hash.
    $script:CommissioningBaseline=Get-Digest $fixtureIni
    $inventoryResult=(& $invoke -Action Inventory -Workspace $workspace) | ConvertFrom-Json
    $inventory=Read-Record $inventoryResult.record
    $evidence=Join-Path $testRoot 'native-review.txt';[IO.File]::WriteAllText($evidence,'Disposable native source-review fixture.')
    $decisions=@($inventory.plugins | ForEach-Object {@{path=$_.path;sha256=$_.sha256;decision=$(if($_.path -eq $safe){'retain'}else{'quarantine'});reason='Fixture';sourceBoundary='Fixture';sourceRevision=$(if($_.path -eq $safe){'1'*40}else{$null});startupAndIdleReadOnly=($_.path -eq $safe);evidencePath=$evidence;evidenceSha256=(Get-Digest $evidence)}})
    $plan=Join-Path $workspace 'plan.json'
    Write-Record $plan @{schema=1;owner='OpenNavX.ReadOnlyCommissioning.Plan.1';inventoryPath=$inventoryResult.record;inventorySha256=$inventoryResult.recordSha256;reviewedUtc=[DateTime]::UtcNow.ToString('o');plugins=$decisions}
    $preparedResult=(& $invoke -Action Prepare -Workspace $workspace -Plan $plan -ExpectedPlanSha256 (Get-Digest $plan)) | ConvertFrom-Json
    $prepared=Read-Record $preparedResult.record
    if ((Get-Digest $fixtureIni) -cne $script:CommissioningBaseline -or -not [IO.File]::Exists($unsafe)) { throw 'Prepare modified a source.' }
    $checks.Add('Native Prepare preserves original INI/DLLs and copies complete review evidence and recovery bytes')
    $arguments=@{Workspace=$workspace;Record=$preparedResult.record;ExpectedRecordSha256=$preparedResult.recordSha256}
    $savedUnsafe=[IO.File]::ReadAllBytes($unsafe)
    [IO.File]::AppendAllText($unsafe,' external change')
    Reject {& $invoke -Action Apply @arguments} 'plugin changed after Prepare'
    if ((Get-Digest $fixtureIni) -cne $script:CommissioningBaseline -or (Test-Path -LiteralPath (Join-Path $workspace 'commissioning-active.json'))) { throw 'Rejected Apply mutated profile or claimed active ownership.' }
    [IO.File]::WriteAllBytes($unsafe,$savedUnsafe)
    $checks.Add('Native Apply refuses intervening DLL changes before mutation or active ownership')
    $applied=(& $invoke -Action Apply @arguments) | ConvertFrom-Json
    if ((Get-Digest $fixtureIni) -cne $prepared.inputSha256 -or [IO.File]::Exists($unsafe) -or -not [IO.File]::Exists($safe)) { throw 'Native Apply did not isolate only the selected DLL.' }
    Reject {& $invoke -Action Apply @arguments} 'Apply replay'
    $checks.Add('Native Apply changes exactly one INI byte, atomically quarantines the selected DLL, preserves retained DLL and refuses replay')
    [IO.File]::WriteAllText($unsafe,'unexpected new candidate')
    Reject {& $invoke -Action InspectRestore @arguments} 'source/destination collision'
    Remove-Item -LiteralPath $unsafe
    $checks.Add('Native inspection refuses a reintroduced DLL without overwriting either original or quarantine')
    [IO.File]::AppendAllText($fixtureIni,"[Display]`r`nPanel=Instruments`r`n")
    $inspection=(& $invoke -Action InspectRestore @arguments) | ConvertFrom-Json
    $restoreArguments=@{Inspection=$inspection.inspection;ExpectedInspectionSha256=$inspection.inspectionSha256;ReviewedCurrentIniSha256=$inspection.currentIniSha256}
    [IO.File]::AppendAllText($fixtureIni,"Extra=user-change`r`n")
    Reject {& $invoke -Action Restore @arguments @restoreArguments} 'post-inspection user edit'
    if ([IO.File]::Exists($unsafe)) { throw 'Rejected Restore returned an unsafe plugin.' }
    $checks.Add('Native Restore rejects a user edit after inspection without overwriting it or returning control plugins')
    $inspection=(& $invoke -Action InspectRestore @arguments) | ConvertFrom-Json
    $restoreArguments=@{Inspection=$inspection.inspection;ExpectedInspectionSha256=$inspection.inspectionSha256;ReviewedCurrentIniSha256=$inspection.currentIniSha256}
    $restored=(& $invoke -Action Restore @arguments @restoreArguments) | ConvertFrom-Json
    if ((Get-Digest $fixtureIni) -cne $script:CommissioningBaseline -or -not [IO.File]::Exists($unsafe) -or (Test-Path -LiteralPath (Join-Path $workspace 'commissioning-active.json'))) { throw 'Native restoration incomplete.' }
    if ($restored.applicationLaunched -ne $false -or $restored.doNotAutoLaunch -ne $true) { throw 'Restoration must not imply launch permission.' }
    Assert-PreparationTree $inventory.trees[0];Assert-PreparationTree $inventory.trees[1]
    $checks.Add('Native inspected Restore retains post-session evidence, restores exact working baseline and DLL inventory, and never launches')
    function New-FixtureTransaction {
      $inventoryResult=(& $invoke -Action Inventory -Workspace $workspace) | ConvertFrom-Json
      $inventory=Read-Record $inventoryResult.record
      $decisions=@($inventory.plugins | ForEach-Object {@{path=$_.path;sha256=$_.sha256;decision=$(if($_.path -eq $safe){'retain'}else{'quarantine'});reason='Fixture';sourceBoundary='Fixture';sourceRevision=$(if($_.path -eq $safe){'1'*40}else{$null});startupAndIdleReadOnly=($_.path -eq $safe);evidencePath=$evidence;evidenceSha256=(Get-Digest $evidence)}})
      $plan=Join-Path $workspace ('plan-'+[guid]::NewGuid().ToString('N')+'.json')
      Write-Record $plan @{schema=1;owner='OpenNavX.ReadOnlyCommissioning.Plan.1';inventoryPath=$inventoryResult.record;inventorySha256=$inventoryResult.recordSha256;reviewedUtc=[DateTime]::UtcNow.ToString('o');plugins=$decisions}
      $preparedResult=(& $invoke -Action Prepare -Workspace $workspace -Plan $plan -ExpectedPlanSha256 (Get-Digest $plan)) | ConvertFrom-Json
      return @{Workspace=$workspace;Record=$preparedResult.record;ExpectedRecordSha256=$preparedResult.recordSha256}
    }
    function Restore-FixtureTransaction($Arguments) {
      $inspection=(& $invoke -Action InspectRestore @Arguments) | ConvertFrom-Json
      $restoreArguments=@{Inspection=$inspection.inspection;ExpectedInspectionSha256=$inspection.inspectionSha256;ReviewedCurrentIniSha256=$inspection.currentIniSha256}
      $null=& $invoke -Action Restore @Arguments @restoreArguments
      if ((Get-Digest $fixtureIni) -cne $script:CommissioningBaseline -or -not [IO.File]::Exists($unsafe) -or -not [IO.File]::Exists($unsafeSecond) -or (Test-Path -LiteralPath (Join-Path $workspace 'commissioning-active.json'))) { throw 'Interrupted transaction recovery did not restore all originals.' }
      Assert-PreparationTree $inventory.trees[0];Assert-PreparationTree $inventory.trees[1]
    }
    foreach ($boundary in @(
      '[IO.File]::Move($item.path,$item.destination)',
      'Publish-PreparedProfile $ini $inputProfile $baselineInfo.sha256 $prepared.inputSha256 $baselineInfo.bytes $apply')) {
      if (-not $body.Contains($boundary)) { throw 'Failure-injection boundary missing from actual entrypoint.' }
      $fault=[scriptblock]::Create($body.Replace($boundary,$boundary+"`nthrow 'Disposable injected interruption after Apply mutation'"))
      $arguments=New-FixtureTransaction
      Reject {& $fault -Action Apply @arguments} 'injected Apply interruption'
      if (-not (Test-Path -LiteralPath (Join-Path $workspace 'commissioning-active.json'))) { throw 'Interrupted Apply lost its durable ownership journal.' }
      Restore-FixtureTransaction $arguments
    }
    $checks.Add('Native recovery handles Apply interruption after first DLL move and after INI publication with exact restoration')
    foreach ($boundary in @(
      'Publish-PreparedProfile $ini $restoreSource $ReviewedCurrentIniSha256 $restoreHash $restoreBytes $restore',
      '[IO.File]::Move($item.destination,$item.path)',
      '# Only remove our exact short-lived ownership marker after durable completion.')) {
      if (-not $body.Contains($boundary)) { throw 'Failure-injection boundary missing from actual entrypoint.' }
      $fault=[scriptblock]::Create($body.Replace($boundary,$boundary+"`nthrow 'Disposable injected interruption after Restore mutation'"))
      $arguments=New-FixtureTransaction
      $null=& $invoke -Action Apply @arguments
      $inspection=(& $invoke -Action InspectRestore @arguments) | ConvertFrom-Json
      $restoreArguments=@{Inspection=$inspection.inspection;ExpectedInspectionSha256=$inspection.inspectionSha256;ReviewedCurrentIniSha256=$inspection.currentIniSha256}
      Reject {& $fault -Action Restore @arguments @restoreArguments} 'injected Restore interruption'
      if (-not (Test-Path -LiteralPath (Join-Path $workspace 'commissioning-active.json'))) { throw 'Interrupted Restore lost its durable ownership journal.' }
      Restore-FixtureTransaction $arguments
    }
    $checks.Add('Native recovery handles Restore interruption after baseline INI, first DLL return and durable completion before marker removal')
    if($AdoptionFixture) {
      if($ResourceAdoptionFixture) {
        # Actual transaction and locator readers with inert owned files only.
        $fixtureInstalled=[pscustomobject]@{root=(Join-Path $testRoot 'owned');generation=(Join-Path $testRoot 'owned/generation');
          executable=(Join-Path $testRoot 'owned/generation/app/opencpn.exe');ownership=[pscustomobject]@{commit=('b'*40)};state=[pscustomobject]@{stock=[pscustomobject]@{path=$fixtureContext.executable}}}
        $null=New-Item -ItemType Directory -Force (Join-Path $fixtureInstalled.generation 'app')
        foreach($path in @($fixtureInstalled.executable,$fixtureContext.executable,(Join-Path $fixtureInstalled.root 'state.json'),(Join-Path $fixtureInstalled.generation 'ownership.json'))) {[IO.File]::WriteAllText($path,'Inert resource fixture; never executed')}
        $fixtureStockHash=Get-Digest $fixtureContext.executable;$originalDigestFunction=${function:Get-Digest}
        function Get-Digest([string]$Path){$hash=& $originalDigestFunction $Path;if($Path -ceq $fixtureContext.executable -and $hash -ceq $fixtureStockHash){return '7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c'};return $hash}
        $marker=Join-Path $fixtureInstalled.generation 'app/OPENNAV_INSTALLED_STOCK';[IO.File]::WriteAllText($marker,$fixtureContext.executable,$encoding)
        $fixtureInstalled.ownership | Add-Member managedFiles @([pscustomobject]@{path='app/OPENNAV_INSTALLED_STOCK';sha256=(Get-Digest $marker)})
        foreach($relative in @('tcdata/harmonics-dwf-20210110-free.tcd','tcdata/HARMONICS_NO_US.IDX','tcdata/HARMONICS_NO_US','gshhs/poly-c-1.dat','basemap_shp/basemap_low.shp','sounds/2bells.wav')) {
          $path=Join-Path $app $relative;$null=New-Item -ItemType Directory -Force ([IO.Path]::GetDirectoryName($path));[IO.File]::WriteAllText($path,'Inert resource')
        }
        function Get-Installed {return $fixtureInstalled}
        $fixtureContext.installation=[pscustomobject]@{root=$fixtureInstalled.root;generation=$fixtureInstalled.generation;executable=$fixtureInstalled.executable;commit=$fixtureInstalled.ownership.commit;
          executableSha256=(Get-Digest $fixtureInstalled.executable);stateSha256=(Get-Digest (Join-Path $fixtureInstalled.root 'state.json'));ownershipSha256=(Get-Digest (Join-Path $fixtureInstalled.generation 'ownership.json'))}
        $text=[IO.File]::ReadAllText($fixtureIni).Replace('[Directories]',"[Directories]`r`nBaseShapefileDir=")
        [IO.File]::WriteAllText($fixtureIni,$text,$encoding);$script:CommissioningBaseline=Get-Digest $fixtureIni
        # Keep production exact-byte backup size; fixture baseline must remain 21380.
        $fixtureBytes=[IO.File]::ReadAllBytes($fixtureIni);$excess=$fixtureBytes.Length-21380
        $text=$text.Replace(('#'+('x'*($padding-3))),('#'+('x'*($padding-3-$excess))))
        [IO.File]::WriteAllText($fixtureIni,$text,$encoding);$script:CommissioningBaseline=Get-Digest $fixtureIni
      }
      $arguments=New-FixtureTransaction;$null=& $invoke -Action Apply @arguments
      $beforeMigration=[IO.File]::ReadAllBytes($fixtureIni)
      $migrationText=$encoding.GetString($beforeMigration).Replace('PersistActiveRoute=0',"PersistActiveRoute=0`r`nConfigVersionString=Version 5.12.4-0+37fd0cd Build 2025-09-12`r`nNavMessageShown=1`r`nLocale=sv")
      $migrationText+="[Settings/GlobalState]`r`nFrameWinX=1280`r`nFrameWinY=800`r`n"
      if($ResourceAdoptionFixture) {
        $migrationText=$migrationText.Replace("BaseShapefileDir=`r`n",("BaseShapefileDir="+(Get-InstalledCommissioningBasemap $fixtureInstalled)+"`r`n"))
      }
      [IO.File]::WriteAllText($fixtureIni,$migrationText,$encoding)
      $migrationBytes=[IO.File]::ReadAllBytes($fixtureIni)
      $inspection=(& $invoke -Action InspectRestore @arguments)|ConvertFrom-Json
      $inspected=Read-Record $inspection.inspection;$parentDir=[IO.Path]::GetDirectoryName($arguments.Record)
      $reviewPath=Join-Path $workspace ('migration-review-'+[guid]::NewGuid().ToString('N')+'.json')
      $entries=@(Get-CommissioningIniDiff (Join-Path $parentDir 'input-only.ini') $inspected.savedIni|ForEach-Object {@{key=$_.key;before=$_.before;after=$_.after;reason='Disposable explicit startup migration fixture';sourceBoundary='Pinned navutil startup/display writes';sourceRevision='37fd0cddb7334fe489e9f18aa163977a9c5c84f7'}})
      Write-Record $reviewPath @{schema=1;owner='OpenNavX.ProfileMigrationReview.1';parentPreparedSha256=$arguments.ExpectedRecordSha256;inspectionSha256=$inspection.inspectionSha256;
        beforeSha256=(Get-Digest (Join-Path $parentDir 'input-only.ini'));afterSha256=$inspection.currentIniSha256;reviewedUtc=[datetime]::UtcNow.ToString('o');changes=$entries}
      $adoptBody=[IO.File]::ReadAllText((Join-Path $PSScriptRoot 'prepare-baseline-adoption.ps1')).Replace(". (Join-Path `$PSScriptRoot 'Commissioning.ps1')",'')
      $prepareAdoption=[scriptblock]::Create($adoptBody)
      if($ResourceAdoptionFixture) {
        $withoutAdoption=@{Inspection=$inspection.inspection;ExpectedInspectionSha256=$inspection.inspectionSha256;ReviewedCurrentIniSha256=$inspection.currentIniSha256}
        Reject {& $invoke -Action Restore @arguments @withoutAdoption} 'resource default may not be silently erased by baseline restoration'
        $checks.Add('Native cold inspection proves owned resource locator and refuses restoration without per-key adoption')
      }
      $proposal=(& $prepareAdoption @arguments -Inspection $inspection.inspection -ExpectedInspectionSha256 $inspection.inspectionSha256 -MigrationReview $reviewPath -ExpectedReviewSha256 (Get-Digest $reviewPath))|ConvertFrom-Json
      if((Get-Digest $fixtureIni) -cne $inspection.currentIniSha256 -or [IO.File]::Exists($unsafe)){throw 'Adoption Prepare changed profile or returned a quarantined plugin'}
      $checks.Add('Native adoption preparation pins exact post-close inspection and independent per-key review without changing any live bytes')
      $adoptArguments=@{AdoptionProposal=$proposal.proposal;ExpectedAdoptionSha256=$proposal.proposalSha256}
      Reject {& $invoke -Action Inventory -Workspace $workspace -BaselineRecord $proposal.proposal -ExpectedBaselineSha256 $proposal.proposalSha256} 'unfinished adoption cannot authorize another commissioning session'
      $restoreArguments=@{Inspection=$inspection.inspection;ExpectedInspectionSha256=$inspection.inspectionSha256;ReviewedCurrentIniSha256=$inspection.currentIniSha256}
      $boundary='Publish-PreparedProfile $ini $restoreSource $ReviewedCurrentIniSha256 $restoreHash $restoreBytes $restore'
      $fault=[scriptblock]::Create($body.Replace($boundary,$boundary+"`nthrow 'Disposable failure after adopted profile atomic publication'"))
      Reject {& $fault -Action Restore @arguments @restoreArguments @adoptArguments} 'adoption interruption after exact publication'
      $expectedAdopted=Get-CommissioningOutputBytes $migrationBytes
      if((Get-Digest $fixtureIni) -cne (Get-CommissioningHash $expectedAdopted) -or -not(Test-Path -LiteralPath (Join-Path $workspace 'commissioning-active.json')) -or [IO.File]::Exists($unsafe)){throw 'Interrupted adoption lost ownership or published incorrect bytes'}
      $checks.Add('Native interrupted adoption preserves every migrated byte except COM8 direction and retains original recovery ownership before plugin restoration')
      Reject {& $invoke -Action InspectRestore @arguments} 'interrupted adoption cannot change restore target by omitting the proposal'
      $inspection=(& $invoke -Action InspectRestore @arguments @adoptArguments)|ConvertFrom-Json
      $restoreArguments=@{Inspection=$inspection.inspection;ExpectedInspectionSha256=$inspection.inspectionSha256;ReviewedCurrentIniSha256=$inspection.currentIniSha256}
      $restored=(& $invoke -Action Restore @arguments @restoreArguments @adoptArguments)|ConvertFrom-Json
      if(-not $restored.baselineRecord -or (Get-Digest $fixtureIni) -cne (Get-CommissioningHash $expectedAdopted) -or -not[IO.File]::Exists($unsafe) -or -not[IO.File]::Exists($unsafeSecond) -or (Test-Path -LiteralPath (Join-Path $workspace 'commissioning-active.json'))){throw 'Adoption recovery failed to preserve migration and restore complete original plugin inventory'}
      $resolved=Read-CommissioningBaseline $workspace $restored.baselineRecord $restored.baselineRecordSha256
      if($resolved.sha256 -cne (Get-Digest $fixtureIni) -or (Read-ProfileForAudit $fixtureIni)['Settings/Locale'] -cne 'sv'){throw 'Adopted lineage lost actual migrated Swedish profile'}
      $checks.Add('Native explicit recovery completes canonical lineage only after original DLLs and preserved migrated profile are verified')
      Reject {& $invoke -Action Inventory -Workspace $workspace} 'adopted baseline is never silently substituted for recovered root'
      $baselineArguments=@{BaselineRecord=$restored.baselineRecord;ExpectedBaselineSha256=$restored.baselineRecordSha256}
      $nextInventory=(& $invoke -Action Inventory -Workspace $workspace @baselineArguments)|ConvertFrom-Json
      Reject {& $invoke -Action Prepare -Workspace $workspace -Plan $plan -ExpectedPlanSha256 (Get-Digest $plan) @baselineArguments} 'old source inventory does not cover new baseline'
      $nextData=Read-Record $nextInventory.record
      $freshDecisions=@($nextData.plugins|ForEach-Object{@{path=$_.path;sha256=$_.sha256;decision=$(if($_.path -eq $safe){'retain'}else{'quarantine'});reason='Fresh isolated source approval for adopted baseline';sourceBoundary='Fixture';sourceRevision=$(if($_.path -eq $safe){'1'*40}else{$null});startupAndIdleReadOnly=($_.path -eq $safe);evidencePath=$evidence;evidenceSha256=(Get-Digest $evidence)}})
      $nextPlan=Join-Path $workspace ('fresh-plan-'+[guid]::NewGuid().ToString('N')+'.json')
      Write-Record $nextPlan @{schema=1;owner='OpenNavX.ReadOnlyCommissioning.Plan.1';inventoryPath=$nextInventory.record;inventorySha256=$nextInventory.recordSha256;reviewedUtc=[datetime]::UtcNow.ToString('o');plugins=$freshDecisions}
      $nextPrepared=(& $invoke -Action Prepare -Workspace $workspace -Plan $nextPlan -ExpectedPlanSha256 (Get-Digest $nextPlan) @baselineArguments)|ConvertFrom-Json
      $nextArguments=@{Workspace=$workspace;Record=$nextPrepared.record;ExpectedRecordSha256=$nextPrepared.recordSha256}
      $null=& $invoke -Action Apply @nextArguments
      if((Get-Digest $fixtureIni) -cne (Get-CommissioningHash $migrationBytes)){throw 'New read-only session did not reuse preserved migrated input-only bytes'}
      $nextInspection=(& $invoke -Action InspectRestore @nextArguments)|ConvertFrom-Json
      $null=& $invoke -Action Restore @nextArguments -Inspection $nextInspection.inspection -ExpectedInspectionSha256 $nextInspection.inspectionSha256 -ReviewedCurrentIniSha256 $nextInspection.currentIniSha256
      if((Get-Digest $fixtureIni) -cne $resolved.sha256){throw 'New transaction reset migrated baseline'}
      $checks.Add('Native subsequent Inventory/Prepare requires explicit lineage and fresh source plan; Apply/Restore preserves the adopted configuration')
    }
    if($PreservationFixture) {
      # Exercise the actual WMM proof/entrypoints with explicitly inert resource
      # pins; the portable suite separately asserts the immutable production pins.
      $wmmGeneration=Join-Path $testRoot 'wmm-owned/generation';$wmmInstall=Split-Path $wmmGeneration -Parent
      $fixtureWmmPins=[ordered]@{};$wmmOwned=@()
      foreach($root in @($app,(Join-Path $wmmGeneration 'app'))){$null=New-Item -ItemType Directory (Join-Path $root 'plugins/wmm_pi/data') -Force}
      foreach($name in @('WMM.COF','wmm_live.svg','wmm_pi.svg')) {
        foreach($root in @($app,(Join-Path $wmmGeneration 'app'))){[IO.File]::WriteAllText((Join-Path $root ('plugins/wmm_pi/data/'+$name)),('Inert WMM resource '+$name),$encoding)}
        $fixtureWmmPins[$name]=Get-Digest (Join-Path $app ('plugins/wmm_pi/data/'+$name))
        $wmmOwned+=@{path=('app/plugins/wmm_pi/data/'+$name);sha256=$fixtureWmmPins[$name]}
      }
      function Get-CommissioningWmmResourcePins {return $fixtureWmmPins}
      $wmmExe=Join-Path $wmmGeneration 'app/opencpn.exe';[IO.File]::WriteAllText($wmmExe,'INERT; never executed',$encoding)
      $wmmOwn=Join-Path $wmmGeneration 'ownership.json';Write-Record $wmmOwn @{owner='OpenNavX.Alpha1.SideBySide.1';commit=('a'*40);managedFiles=$wmmOwned}
      $wmmState=Join-Path $wmmInstall 'state.json';Write-Record $wmmState @{fixture='original installed state'}
      $fixtureWmmInstalled=[pscustomobject]@{root=$wmmInstall;generation=$wmmGeneration;executable=$wmmExe;ownership=(Read-Record $wmmOwn)}
      function Get-Installed {return $fixtureWmmInstalled}
      $fixtureContext.installation=[pscustomobject]@{root=$wmmInstall;generation=$wmmGeneration;executable=$wmmExe;commit=('a'*40);
        executableSha256=(Get-Digest $wmmExe);stateSha256=(Get-Digest $wmmState);ownershipSha256=(Get-Digest $wmmOwn)}
      $wmmBefore=Get-CommissioningWmmLocation $app;$wmmAfter=Get-CommissioningWmmLocation (Join-Path $wmmGeneration 'app')
      $wmmBaseline=[IO.File]::ReadAllText($fixtureIni).Replace('[Directories]',("[Directories]`r`nWMMDataLocation="+$wmmBefore))
      $excess=$encoding.GetByteCount($wmmBaseline)-21380
      $wmmBaseline=$wmmBaseline.Replace(('#'+('x'*($padding-3))),('#'+('x'*($padding-3-$excess))))
      [IO.File]::WriteAllText($fixtureIni,$wmmBaseline,$encoding);$script:CommissioningBaseline=Get-Digest $fixtureIni
      if((Get-Item $fixtureIni).Length -ne 21380){throw 'WMM baseline must retain exact fixture root length'}
      $preservationBase=[IO.File]::ReadAllBytes($fixtureIni)
      $navFixture=Join-Path $profileDirectory 'navobj.xml';[IO.File]::WriteAllText($navFixture,'<gpx>inert unchanged route fixture</gpx>',$encoding)
      $navHash=Get-Digest $navFixture
      $preserveBody=[IO.File]::ReadAllText((Join-Path $PSScriptRoot 'prepare-session-preservation.ps1')).Replace(". (Join-Path `$PSScriptRoot 'Commissioning.ps1')",'')
      $preparePreservation=[scriptblock]::Create($preserveBody)
      function New-PreservationFixture {
        [IO.File]::WriteAllBytes($fixtureIni,$preservationBase)
        $transactionArgs=New-FixtureTransaction;$null=& $invoke -Action Apply @transactionArgs
        $alpha='OpenNavXSettings 1\n"battery" ""\n"capacity" "20"\n"consumption" "measured"\n"corridor" "50"\n"current" "unconfigured"\n"display.instruments" "sog,depth"\n"display.rail" "sog,heading"\n"draft" "1"\n"efficiency" ""\n"hotel" ""\n"margin" "1"\n"minimum_speed" "1"\n"model_source" ""\n"reserve" "20"\n'
        $currentText=[IO.File]::ReadAllText($fixtureIni).Replace($wmmBefore,$wmmAfter).Replace('PersistActiveRoute=0',"PersistActiveRoute=0`r`nActiveRoute=11111111-2222-3333-4444-555555555555")
        $currentText+="[OpenNav]`r`nAlphaSettings=$alpha`r`n[OpenNav/OnlineAIS/v1]`r`nEnabled=1`r`n[PlugIns/wmm_pi.dll]`r`nbEnabled=1`r`n[Settings/GlobalState]`r`nFrameWinX=1280`r`n"
        [IO.File]::WriteAllText($fixtureIni,$currentText,$encoding)
        $seen=(& $invoke -Action InspectRestore @transactionArgs)|ConvertFrom-Json
        $inspected=Read-Record $seen.inspection;$parent=[IO.Path]::GetDirectoryName($transactionArgs.Record)
        $review=Join-Path $workspace ('preservation-review-'+[guid]::NewGuid().ToString('N')+'.json')
        $entries=@(Get-CommissioningIniDiff (Join-Path $parent 'input-only.ini') $inspected.savedIni|ForEach-Object{@{key=$_.key;before=$_.before;after=$_.after;origin='unverified';decision='preserve-current';reason='Synthetic current user settings; no source or launch authority'}})
        Write-Record $review @{schema=1;owner='OpenNavX.SessionPreservationReview.1';parentPreparedSha256=$transactionArgs.ExpectedRecordSha256;inspectionSha256=$seen.inspectionSha256;
          beforeSha256=(Get-Digest (Join-Path $parent 'input-only.ini'));afterSha256=$seen.currentIniSha256;reviewedUtc=[datetime]::UtcNow.ToString('o');
          provenance='current-user-state;origin-unverified';preservationOnly=$true;launchPermission=$false;changes=$entries}
        $proposed=(& $preparePreservation @transactionArgs -Inspection $seen.inspection -ExpectedInspectionSha256 $seen.inspectionSha256 -PreservationReview $review -ExpectedReviewSha256 (Get-Digest $review))|ConvertFrom-Json
        return @{arguments=$transactionArgs;inspection=$seen;proposal=$proposed;choice=@{PreservationProposal=$proposed.proposal;ExpectedPreservationSha256=$proposed.proposalSha256};
          target=(Get-CommissioningHash (Get-CommissioningOutputBytes ([IO.File]::ReadAllBytes($fixtureIni))))}
      }
      function Restore-PreservationFixture($Fixture) {
        $transactionArgs=$Fixture.arguments;$choice=$Fixture.choice
        $seen=(& $invoke -Action InspectRestore @transactionArgs @choice)|ConvertFrom-Json
        $result=(& $invoke -Action Restore @transactionArgs @choice -Inspection $seen.inspection -ExpectedInspectionSha256 $seen.inspectionSha256 -ReviewedCurrentIniSha256 $seen.currentIniSha256)|ConvertFrom-Json
        if((Get-Digest $fixtureIni) -cne $Fixture.target -or (Get-Digest $navFixture) -cne $navHash -or
            -not [IO.File]::Exists($unsafe) -or -not [IO.File]::Exists($unsafeSecond) -or (Test-Path (Join-Path $workspace 'commissioning-active.json'))){throw 'Preservation did not restore exact settings/plugins/navigation without active ownership.'}
        $resolved=Read-CommissioningBaseline $workspace $result.baselineRecord $result.baselineRecordSha256
        if($resolved.sha256 -cne $Fixture.target -or -not $result.doNotAutoLaunch -or $result.applicationLaunched){throw 'Preservation lineage or no-launch result differs.'}
        return $result
      }
      $fixture=New-PreservationFixture;$arguments=$fixture.arguments;$choice=$fixture.choice
      $parentDir=[IO.Path]::GetDirectoryName($arguments.Record)
      foreach($path in @($wmmOwn,(Join-Path $wmmGeneration 'app/plugins/wmm_pi/data/WMM.COF'))) {
        $saved=[IO.File]::ReadAllBytes($path);[IO.File]::AppendAllText($path,' late WMM mutation')
        Reject {& $invoke -Action InspectRestore @arguments @choice} 'late owned WMM manifest/resource drift'
        [IO.File]::WriteAllBytes($path,$saved)
      }
      Reject {& $invoke -Action Restore @arguments -Inspection $fixture.inspection.inspection -ExpectedInspectionSha256 $fixture.inspection.inspectionSha256 -ReviewedCurrentIniSha256 $fixture.inspection.currentIniSha256} 'WMM delta cannot be erased by legacy baseline restore'
      $checks.Add('Native WMM stock-to-parent path proof requires explicit preservation and refuses live owned resource/manifest drift')
      $before=Get-Digest $fixtureIni;$quarantineState=Read-Record $arguments.Record
      Reject {Read-CommissioningBaseline $workspace $fixture.proposal.proposal $fixture.proposal.proposalSha256} 'proposed preservation is not a usable baseline'
      $lock=Open-CommissioningRestoreLock $parentDir
      try {Reject {& $invoke -Action InspectRestore @arguments @choice} 'concurrent restore lock';Reject {& $invoke -Action Restore @arguments -Inspection $fixture.inspection.inspection -ExpectedInspectionSha256 $fixture.inspection.inspectionSha256 -ReviewedCurrentIniSha256 $fixture.inspection.currentIniSha256} 'legacy and preservation restore share the same lock'}finally{$lock.Dispose()}
      if((Get-Digest $fixtureIni) -cne $before -or [IO.File]::Exists($unsafe)){throw 'Blocked concurrent entry changed originals.'}
      $checks.Add('Native preservation proposal makes no original changes and shared exclusive lock refuses competing restore variants')
      foreach($path in @($fixtureIni,$navFixture,$quarantineState.quarantine[0].destination)) {
        $saved=[IO.File]::ReadAllBytes($path);[IO.File]::AppendAllText($path,' late mutation')
        Reject {& $invoke -Action InspectRestore @arguments @choice} 'late profile/navigation/plugin mutation cannot be re-inspected into approval'
        [IO.File]::WriteAllBytes($path,$saved)
      }
      $originalAcl=(Get-Acl -LiteralPath $navFixture).Sddl
      # Keep the original filesystem object: Set-Acl may normalize inheritance
      # control flags, so reapplying its SDDL is not an exact cleanup operation.
      $aclOriginal=Join-Path $workspace ('acl-original-'+[guid]::NewGuid().ToString('N'))
      [IO.File]::Move($navFixture,$aclOriginal)
      [IO.File]::Copy($aclOriginal,$navFixture)
      $changedAcl=New-Object Security.AccessControl.FileSecurity
      $changedAcl.SetSecurityDescriptorSddlForm($originalAcl)
      $everyone=New-Object Security.Principal.SecurityIdentifier('S-1-1-0')
      $changedAcl.AddAccessRule((New-Object Security.AccessControl.FileSystemAccessRule($everyone,'Read','Allow')))
      Set-Acl -LiteralPath $navFixture -AclObject $changedAcl
      try {Reject {& $invoke -Action InspectRestore @arguments @choice} 'late other-profile ACL mutation'}finally{
        [IO.File]::Delete($navFixture);[IO.File]::Move($aclOriginal,$navFixture)
      }
      $fixtureContext.session=2;Reject {& $invoke -Action InspectRestore @arguments @choice} 'changed actual session';$fixtureContext.session=1
      $checks.Add('Native preservation refuses late INI/navigation/plugin bytes, unrelated profile ACL and actual session changes')
      $null=Restore-PreservationFixture $fixture
      $checks.Add('Native explicit preservation keeps substantive settings, route data and every other byte, reverses only COM8 and restores both original DLLs')
      $intentLine=@($body -split "`n"|Where-Object{$_ -like 'Write-Record $restore @*'})
      if($intentLine.Count -ne 1){throw 'Actual restoration intent boundary changed.'}
      foreach($boundary in @($intentLine[0],
        'Publish-PreparedProfile $ini $restoreSource $ReviewedCurrentIniSha256 $restoreHash $restoreBytes $restore',
        '[IO.File]::Move($item.destination,$item.path)',
        '# Only remove our exact short-lived ownership marker after durable completion.')) {
        if(-not $body.Contains($boundary)){throw 'Preservation fault boundary missing.'}
        $fixture=New-PreservationFixture;$arguments=$fixture.arguments;$choice=$fixture.choice;$seen=$fixture.inspection
        $fault=[scriptblock]::Create($body.Replace($boundary,$boundary+"`nthrow 'Disposable preservation interruption'"))
        Reject {& $fault -Action Restore @arguments @choice -Inspection $seen.inspection -ExpectedInspectionSha256 $seen.inspectionSha256 -ReviewedCurrentIniSha256 $seen.currentIniSha256} 'injected preservation interruption'
        if(-not(Test-Path (Join-Path $workspace 'commissioning-active.json'))){throw 'Interrupted preservation lost active ownership.'}
        Reject {& $invoke -Action InspectRestore @arguments} 'omission cannot select old baseline after preservation intent'
        Reject {& $invoke -Action InspectRestore @arguments -AdoptionProposal $fixture.proposal.proposal -ExpectedAdoptionSha256 $fixture.proposal.proposalSha256} 'preservation cannot be relabelled migration'
        $restored=Restore-PreservationFixture $fixture
      }
      $checks.Add('Native preservation resumes exact target after intent, INI publication, first DLL return and durable completion; legacy fallback refused')
      $fixtureContext.installation=[pscustomobject]@{generation='inert-new-generation';commit=('b'*40)}
      Remove-Item -LiteralPath $wmmGeneration -Recurse -Force # Historical proof must not reopen retired resource files.
      $baselineArgs=@{BaselineRecord=$restored.baselineRecord;ExpectedBaselineSha256=$restored.baselineRecordSha256}
      $nextInventory=(& $invoke -Action Inventory -Workspace $workspace @baselineArgs)|ConvertFrom-Json
      $nextData=Read-Record $nextInventory.record
      $decisions=@($nextData.plugins|ForEach-Object{@{path=$_.path;sha256=$_.sha256;decision=$(if($_.path -eq $safe){'retain'}else{'quarantine'});reason='Fresh isolated preservation source review';sourceBoundary='Fixture only';sourceRevision=$(if($_.path -eq $safe){'1'*40}else{$null});startupAndIdleReadOnly=($_.path -eq $safe);evidencePath=$evidence;evidenceSha256=(Get-Digest $evidence)}})
      $freshPlan=Join-Path $workspace ('preserved-plan-'+[guid]::NewGuid().ToString('N')+'.json')
      Write-Record $freshPlan @{schema=1;owner='OpenNavX.ReadOnlyCommissioning.Plan.1';inventoryPath=$nextInventory.record;inventorySha256=$nextInventory.recordSha256;reviewedUtc=[datetime]::UtcNow.ToString('o');plugins=$decisions}
      $next=(& $invoke -Action Prepare -Workspace $workspace -Plan $freshPlan -ExpectedPlanSha256 (Get-Digest $freshPlan) @baselineArgs)|ConvertFrom-Json
      if((Get-Digest $fixtureIni) -cne $fixture.target -or (Test-Path (Join-Path $workspace 'commissioning-active.json'))){throw 'Fresh Prepare changed preserved settings or active ownership.'}
      $checks.Add('Native preserved historical lineage supports fresh Inventory/Prepare after generation change without launching or altering settings')
    }
  }
  [pscustomobject]@{status='passed';environment=$(if($native){'native-windows-disposable-filesystem'}else{'linux-powershell-portable-contracts'});count=$checks.Count;checks=@($checks);boatAccess=$false;applicationLaunched=$false;productOrBoatAcceptance=$false} | ConvertTo-Json -Depth 6
} catch { Write-Output $_.ScriptStackTrace; throw
} finally { $env:PATH=$originalTestSearchPath;Remove-Item -LiteralPath $testRoot -Recurse -Force }

# Isolated verification fixtures; never reads installed software or vessel data.
[CmdletBinding()]
param([switch]$PortableContracts,[switch]$IsolatedLocal,[switch]$StockFixture,[switch]$InstalledWelcomeFixture)
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'
$native=[Environment]::OSVersion.Platform -eq 'Win32NT'
if (-not $native -and -not $PortableContracts) { throw 'Explicit portable contracts required off Windows.' }
if ($native -and -not $IsolatedLocal -and $env:GITHUB_ACTIONS -ne 'true') { throw 'Disposable CI or explicit isolated local tests required.' }
. (Join-Path $PSScriptRoot 'Commissioning.ps1')
if ($InstalledWelcomeFixture) {
  if ($StockFixture) { throw 'Installed warning fixture cannot impersonate stock.' }
  . (Join-Path $PSScriptRoot 'InstalledWelcome.ps1')
}
$testRoot=Join-Path ([IO.Path]::GetTempPath()) ('OpenNav commissioning launch '+[guid]::NewGuid().ToString('N'))
$null=New-Item -ItemType Directory -Path $testRoot
$savedPath=$env:PATH
$savedBaseline=$script:CommissioningBaseline
if (-not $native) {
  function Assert-LocalPath([string]$Path) {
    $full=[IO.Path]::GetFullPath($Path)
    if ($full -ne $testRoot -and -not $full.StartsWith($testRoot+'/',[StringComparison]::Ordinal)) { throw 'Fixture path escaped temporary root.' }
    $walk=$full
    while ($walk -ne $testRoot) {
      if ((Test-Path -LiteralPath $walk) -and ((Get-Item -LiteralPath $walk -Force).Attributes -band [IO.FileAttributes]::ReparsePoint)) { throw 'Redirected fixture path.' }
      $walk=[IO.Path]::GetDirectoryName($walk)
    }
    return $full
  }
}
$checks=New-Object 'Collections.Generic.List[string]'
function Reject([scriptblock]$Operation,[string]$Label) {
  $failed=$false;try {$null=& $Operation} catch {$failed=$true}
  if (-not $failed) { throw ('Unsafe launch verification accepted: '+$Label) }
}
function Change-And-Reject([string]$Path,[string]$Label) {
  $originalBytes=[IO.File]::ReadAllBytes($Path)
  try { [IO.File]::AppendAllText($Path,'changed');Reject {& $verify @arguments} $Label }
  finally { [IO.File]::WriteAllBytes($Path,$originalBytes) }
}
try {
  $entry=Join-Path $PSScriptRoot 'verify-commissioning-launch.ps1'
  $tokens=$null;$errors=$null
  $null=[Management.Automation.Language.Parser]::ParseFile($entry,[ref]$tokens,[ref]$errors)
  if ($errors.Count) { throw ($errors | Out-String) }
  # Omit only dependency import. The exact verifier executes, with a fixture
  # identity context; no registry/process/real profile or device call occurs.
  $body=[IO.File]::ReadAllText($entry).Replace(". (Join-Path `$PSScriptRoot 'Commissioning.ps1')",'')
  $verify=[scriptblock]::Create($body)
  $workspace=Join-Path $testRoot 'workspace'
  $directory=Join-Path $workspace 'runs/session'
  $profile=Join-Path $testRoot 'profile'
  $managed=Join-Path $testRoot 'managed'
  $stock=Join-Path $testRoot 'stock/plugins'
  $generation=Join-Path $testRoot 'generation/app/plugins'
  $quarantine=Join-Path $directory 'quarantine'
  $search=Join-Path $testRoot 'search'
  foreach ($path in @($directory,$profile,$managed,$stock,$generation,$quarantine,$search)) { $null=New-Item -ItemType Directory -Path $path -Force }
  $env:PATH=$search
  $exe=Join-Path $testRoot 'generation/app/opencpn.exe'
  [IO.File]::WriteAllText($exe,'not an executable; never launched')
  $installed=[pscustomobject]@{executable=$exe;ownership=[pscustomobject]@{commit=('7'*40)}}
  $fixtureContext=[pscustomobject]@{workspace=$workspace;profile=$profile;pluginRoots=@($managed,$stock,$generation);installation=[pscustomobject]@{executable=$exe;commit=$installed.ownership.commit};launchEnvironment=[pscustomobject]@{workingDirectory=[IO.Path]::GetDirectoryName($exe);path=$search}}
  if ($InstalledWelcomeFixture) {
    $installed | Add-Member root (Join-Path $testRoot 'owned-installation')
    $installed | Add-Member generation (Join-Path $testRoot 'generation')
    $installed | Add-Member state ([pscustomobject]@{stock=[pscustomobject]@{path=(Join-Path $testRoot 'stock/opencpn.exe')}})
    $null=New-Item -ItemType Directory -Path $installed.root
    [IO.File]::WriteAllText((Join-Path $installed.root 'state.json'),'inert owned state')
    [IO.File]::WriteAllText((Join-Path $installed.generation 'ownership.json'),'inert ownership')
    $resourceStock=$installed.state.stock.path
    [IO.File]::WriteAllText($resourceStock,'inert resource-stock fixture; never launched')
    $resourceStockDigest=Get-Digest $resourceStock
    $resourceDigestFunction=${function:Get-Digest}
    function Get-Digest([string]$Path) {
      $hash=& $resourceDigestFunction $Path
      if ($Path -ceq $resourceStock -and $hash -ceq $resourceStockDigest) { return '7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c' }
      return $hash
    }
    $resourceMarker=Join-Path $installed.generation 'app/OPENNAV_INSTALLED_STOCK'
    [IO.File]::WriteAllText($resourceMarker,$resourceStock,(New-Object Text.UTF8Encoding($false)))
    $installed.ownership | Add-Member managedFiles @([pscustomobject]@{path='app/OPENNAV_INSTALLED_STOCK';sha256=(Get-Digest $resourceMarker)})
    $resourceFiles=@('tcdata/harmonics-dwf-20210110-free.tcd','tcdata/HARMONICS_NO_US.IDX','tcdata/HARMONICS_NO_US','gshhs/poly-c-1.dat','basemap_shp/basemap_low.shp','sounds/2bells.wav')
    foreach ($name in $resourceFiles) {
      $path=Join-Path ([IO.Path]::GetDirectoryName($resourceStock)) $name
      $null=New-Item -ItemType Directory -Force ([IO.Path]::GetDirectoryName($path))
      [IO.File]::WriteAllText($path,'inert resource; never loaded')
    }
    $fixtureContext.installation=[pscustomobject]@{root=$installed.root;generation=$installed.generation;executable=$exe;commit=$installed.ownership.commit;
      executableSha256=(Get-Digest $exe);stateSha256=(Get-Digest (Join-Path $installed.root 'state.json'));ownershipSha256=(Get-Digest (Join-Path $installed.generation 'ownership.json'))}
    $fixtureContext | Add-Member localAppData (Join-Path $testRoot 'local')
    $fixtureContext | Add-Member sid 'S-1-5-21-1'
    $fixtureContext | Add-Member session 1
    function Get-InstalledWelcomeEnvironment($Config,$Installed) {
      return [pscustomobject]@{local=(Join-Path $testRoot 'local');profile=$profile;roots=@($fixtureContext.pluginRoots)}
    }
    $verify={param($Workspace,$Audit,$Installed)
      $config=[pscustomobject]@{readOnlyAudit=$Audit;profileDirectory=$profile;stockExecutable=$Installed.state.stock.path}
      $receipt=[pscustomobject]@{commissioning=$Audit.commissioning;sid='S-1-5-21-1';sessionId=1}
      Assert-InstalledWelcomeRuntime $config $Installed $receipt $Workspace
      return $fixtureContext.launchEnvironment
    }
  }
  if ($StockFixture) {
    $exe=Join-Path $testRoot 'stock/opencpn.exe'
    [IO.File]::WriteAllText($exe,'inert stock executable identity fixture; never launched')
    $actualFixtureHash=Get-Digest $exe
    $digestFunction=${function:Get-Digest}
    # Only this inert fixture's known bytes stand in for the official PE oracle.
    # Production retains its hard-coded allowlist. Mutation falls back to real SHA.
    function Get-Digest([string]$Path) {
      $hash=& $digestFunction $Path
      if ($Path -ceq $exe -and $hash -ceq $actualFixtureHash) { return '7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c' }
      return $hash
    }
    $fixtureContext.installation=$null;$fixtureContext.pluginRoots=@($managed,$stock)
    $fixtureContext | Add-Member -NotePropertyName executable -NotePropertyValue $exe
    $fixtureContext.launchEnvironment.workingDirectory=[IO.Path]::GetDirectoryName($exe)
  }
  function Get-CommissioningContext([string]$Workspace) {
    if ($Workspace -cne $fixtureContext.workspace) { throw 'Fixture identity context escaped.' }
    return $fixtureContext
  }
  $baseline=Join-Path $directory 'baseline.ini'
  $input=Join-Path $directory 'input-only.ini'
  $ini=Join-Path $profile 'opencpn.ini'
  $encoding=New-Object Text.UTF8Encoding($false,$true)
  $text="[Settings]`r`nPersistActiveRoute=0`r`n[Settings/NMEADataSource]`r`nDataConnections=0;0;;0;1;COM8;115200;0;1;0;;0;;0;0;1;0;1;Gateway;0;;0`r`n[Directories]`r`nChartDir=original`r`nBaseShapefileDir=`r`n"
  # The real recovered-root contract includes its exact byte length.
  $text+='#'+(' '*(21380-$encoding.GetByteCount($text)-3))+"`r`n"
  $bytes=$encoding.GetBytes($text)
  [IO.File]::WriteAllBytes($baseline,$bytes)
  $script:CommissioningBaseline=Get-Digest $baseline
  [IO.File]::WriteAllBytes($input,(Get-CommissioningInputBytes $bytes))
  [IO.File]::WriteAllBytes($ini,[IO.File]::ReadAllBytes($input))
  $safe=Join-Path $managed 'chart_pi.dll'
  $unsafe=Join-Path $managed 'control_pi.dll'
  $helper=Join-Path $managed 'chart-helper.exe'
  $stockHelper=Join-Path $stock 'stock-runtime.dll'
  $generationHelper=Join-Path $generation 'generation-runtime.dll'
  foreach ($path in @($safe,$unsafe,$helper,$stockHelper,$generationHelper)) { [IO.File]::WriteAllText($path,'inert fixture '+[IO.Path]::GetFileName($path)) }
  $trees=@(Get-CommissioningTrees $fixtureContext.pluginRoots)
  $candidates=@(Get-CommissioningCandidates $trees)
  if ($candidates.Count -ne 2) { throw 'Fixture must exercise multiple independent plugin decisions.' }
  $inventoryPath=Join-Path $directory 'inventory.json'
  Write-Record $inventoryPath @{schema=1;owner='OpenNavX.ReadOnlyCommissioning.Inventory.1';context=$fixtureContext;profileSha256=$script:CommissioningBaseline;trees=$trees;plugins=$candidates}
  $decisions=@();$evidence=@();$moves=@();$index=0
  foreach ($candidate in $candidates) {
    $index++;$review=Join-Path $directory ('review-'+$index+'.txt')
    [IO.File]::WriteAllText($review,'Explicit synthetic source review for isolated filesystem tests, not boat acceptance.')
    $retained=$candidate.path -ceq $safe
    $decisions+=@{path=$candidate.path;sha256=$candidate.sha256;decision=$(if($retained){'retain'}else{'quarantine'});reason='Test only';sourceBoundary='Fixture';sourceRevision=$(if($retained){'1'*40}else{$null});startupAndIdleReadOnly=$retained;evidencePath=$review;evidenceSha256=(Get-Digest $review)}
    $evidence+=@{path=$review;sha256=(Get-Digest $review)}
    if (-not $retained) {
      $backup=Join-Path $directory ('plugin-backup-'+$index+'.bin')
      [IO.File]::Copy($candidate.path,$backup)
      $moves+=@{path=$candidate.path;sha256=$candidate.sha256;backup=$backup;destination=(Join-Path $quarantine ('plugin-'+$index+'.bin'))}
    }
  }
  $planPath=Join-Path $directory 'review-plan.json'
  Write-Record $planPath @{schema=1;owner='OpenNavX.ReadOnlyCommissioning.Plan.1';reviewedUtc=[DateTime]::UtcNow.ToString('o');inventorySha256=(Get-Digest $inventoryPath);plugins=$decisions}
  $parsedPlan=Read-Record $planPath
  if (@($parsedPlan.plugins).Count -ne 2 -or @($evidence).Count -ne 2) { throw 'The parsed multi-plugin plan must preserve each separate review/evidence item.' }
  $record=Join-Path $directory 'prepared.json'
  Write-Record $record @{schema=1;owner=$script:CommissioningOwner;status='prepared';context=$fixtureContext;baselineSha256=$script:CommissioningBaseline;inputSha256=(Get-Digest $input);planSha256=(Get-Digest $planPath);inventorySha256=(Get-Digest $inventoryPath);evidence=$evidence;quarantine=$moves}
  $recordSha=Get-Digest $record
  $active=Join-Path $workspace 'commissioning-active.json'
  Write-Record $active @{schema=1;owner=$script:CommissioningOwner;record=$record;recordSha256=$recordSha}
  foreach ($move in $moves) { [IO.File]::Move($move.path,$move.destination) }
  $applied=Join-Path $directory 'applied.json'
  Write-Record $applied @{schema=1;owner=$script:CommissioningOwner;status='input-only-prepared';recordSha256=$recordSha;profileSha256=(Get-Digest $input);remainingPluginCount=1}
  $audit=[pscustomobject]@{profileIniSha256=(Get-Digest $ini);buildCommit=$installed.ownership.commit;reviewedUtc=[DateTime]::UtcNow.ToString('o');commissioning=[pscustomobject]@{record=$record;recordSha256=$recordSha;appliedSha256=(Get-Digest $applied)}}
  if ($InstalledWelcomeFixture) {
    foreach($field in @('connectionsOutputDisabled','pluginOutputsReviewed','noActiveRouteOutput')) { $audit | Add-Member $field $true }
  }
  $arguments=@{Workspace=$workspace;Audit=$audit;Installed=$installed}
  if ($StockFixture) {
    $audit | Add-Member -NotePropertyName launchKind -NotePropertyValue 'StockLegacy'
    $audit | Add-Member -NotePropertyName executableSha256 -NotePropertyValue '7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c'
    $audit | Add-Member -NotePropertyName upstreamCommit -NotePropertyValue '37fd0cddb7334fe489e9f18aa163977a9c5c84f7'
    $arguments.Remove('Installed');$arguments.Stock=$true
    Reject { & $verify -Workspace $workspace -Audit $audit -Installed $installed } 'installed parameter set cannot accept stock-only context'
    $fixtureContext.installation=[pscustomobject]@{executable=$installed.executable;commit=$installed.ownership.commit}
    Reject { & $verify @arguments } 'stock parameter set cannot reuse installed context'
    $fixtureContext.installation=$null
    Change-And-Reject $exe 'official stock executable identity changed'
    $checks.Add('Stock and installed identities are separate; cross-kind or changed executable refused')
  }
  $verifiedEnvironment=& $verify @arguments
  if ($verifiedEnvironment.workingDirectory -cne $fixtureContext.launchEnvironment.workingDirectory -or $verifiedEnvironment.path -cne $search) { throw 'Returned environment is not the pinned child launch environment.' }
  $checks.Add('Exact applied transaction with complete current loader-root trees and evidence verifies without launch')
  if ($InstalledWelcomeFixture) {
    foreach($path in @($exe,(Join-Path $installed.root 'state.json'),(Join-Path $installed.generation 'ownership.json'))) { Change-And-Reject $path 'installed generation identity changed after cold launch' }
    $checks.Add('Running installed warning proof still pins executable, installation state and ownership bytes')
    foreach($field in @('connectionsOutputDisabled','pluginOutputsReviewed','noActiveRouteOutput')) {
      try { $audit.$field=$false;Reject {& $verify @arguments} 'missing independent read-only approval' } finally {$audit.$field=$true}
    }
    $checks.Add('No read-only commissioning boolean is manufactured or inferred')
    $originalBytes=[IO.File]::ReadAllBytes($ini)
    try {
      $default=Get-InstalledCommissioningBasemap $installed
      $filled=$encoding.GetString($originalBytes).Replace("BaseShapefileDir=`r`n",("BaseShapefileDir="+$default+"`r`n"))
      [IO.File]::WriteAllText($ini,$filled,$encoding)
      $null=& $verify @arguments
      $checks.Add('Running installed review accepts only the owned stock resource fallback for an existing empty preference')
      Reject { Assert-CommissioningRestoreIni $input $ini } 'runtime fallback cannot silently authorize baseline restoration'
      $checks.Add('Cold restoration retains its strict independent review boundary')
      foreach ($path in @($resourceStock,$resourceMarker)) {
        Change-And-Reject $path 'changed stock executable or owned resource marker'
        $checks.Add('Changed exact resource binding refused: '+[IO.Path]::GetFileName($path))
      }
      foreach ($name in $resourceFiles) {
        $path=Join-Path ([IO.Path]::GetDirectoryName($resourceStock)) $name
        $savedBytes=[IO.File]::ReadAllBytes($path)
        try { [IO.File]::WriteAllBytes($path,[byte[]]@());Reject {& $verify @arguments} 'empty stock resource prerequisite' }
        finally {[IO.File]::WriteAllBytes($path,$savedBytes)}
        $checks.Add('Missing selector prerequisite refused: '+$name)
      }
      foreach ($replacement in @('C:\other\basemap_shp',($default+'/'),($default+'/../basemap_shp'),('"'+$default+'"'))) {
        [IO.File]::WriteAllText($ini,$filled.Replace($default,$replacement),$encoding)
        Reject {& $verify @arguments} ('another path spelling or installation cannot be a default: '+$replacement)
        $checks.Add('Changed resource path rejected without normalization')
      }
      $oldValues=Read-ProfileForAudit $input;$newValues=$oldValues.Clone()
      $newValues['Directories/BaseShapefileDir']=$default
      foreach ($prior in @('custom selection',$null)) {
        $old=$oldValues.Clone()
        if ($null -eq $prior) {$old.Remove('Directories/BaseShapefileDir')} else {$old['Directories/BaseShapefileDir']=$prior}
        Reject {Assert-CommissioningProtectedValues $old $newValues $default} 'custom or missing baseline cannot be filled by this observed rule'
        $checks.Add('Existing custom/missing resource preference remains protected')
      }
    } finally {[IO.File]::WriteAllBytes($ini,$originalBytes)}
  }
  foreach ($path in $(if($StockFixture){@($helper,$stockHelper)}else{@($helper,$stockHelper,$generationHelper)})) { Change-And-Reject $path 'changed helper/runtime in an actual loader root' }
  $checks.Add('Changed helpers and runtime dependencies in all applicable roots refuse launch despite unchanged plugin DLLs')
  Change-And-Reject $safe 'changed retained plugin'
  $new=Join-Path $managed 'new-helper.exe'
  try { [IO.File]::WriteAllText($new,'new helper');Reject {& $verify @arguments} 'added non-plugin executable' } finally { Remove-Item -LiteralPath $new }
  $checks.Add('Changed retained DLL or added non-plugin file invalidates the complete tree')
  try { [IO.File]::Copy($moves[0].destination,$unsafe);Reject {& $verify @arguments} 'control DLL restored early' } finally { Remove-Item -LiteralPath $unsafe }
  Change-And-Reject $moves[0].destination 'quarantine bytes changed'
  Change-And-Reject $moves[0].backup 'recovery bytes changed'
  $checks.Add('Returned control plugin and altered quarantine/recovery bytes refuse launch')
  foreach ($path in @($evidence[0].path,$planPath,$inventoryPath,$record,$active,$applied,$baseline,$input)) { Change-And-Reject $path 'changed evidence/ownership/profile proof' }
  $checks.Add('Every copied review, plan, inventory, prepared/active/applied record and profile proof is hash-bound')
  $saved=Join-Path $directory 'applied.saved'
  try { [IO.File]::Move($applied,$saved);Reject {& $verify @arguments} 'Apply not durably complete' } finally { [IO.File]::Move($saved,$applied) }
  $checks.Add('Incomplete Apply cannot authorize launch even when all selected DLLs are absent')
  foreach ($name in @('restore-intent-fixture.json','restored-fixture.json','restored.json')) {
    $path=Join-Path $directory $name
    try { [IO.File]::WriteAllText($path,'{}');Reject {& $verify @arguments} 'restore begun or complete' } finally { Remove-Item -LiteralPath $path }
  }
  $checks.Add('Started or completed restoration blocks every normal launch')
  $originalIni=[IO.File]::ReadAllBytes($ini)
  try {
    [IO.File]::AppendAllText($ini,"[Display]`r`nPage=Instruments`r`n")
    if ($InstalledWelcomeFixture) {
      # An already running application's harmless UI persistence is checked
      # against protected input/chart keys, never used to renew a cold audit.
      $null=& $verify @arguments
    } else { Reject {& $verify @arguments} 'unreviewed normal profile write' }
    $audit.profileIniSha256=Get-Digest $ini
    $null=& $verify @arguments
    [IO.File]::WriteAllText($ini,$encoding.GetString($originalIni).Replace('ChartDir=original','ChartDir=changed'))
    $audit.profileIniSha256=Get-Digest $ini
    Reject {& $verify @arguments} 'chart directory change despite fresh independent hash'
    [IO.File]::WriteAllBytes($ini,$bytes);$audit.profileIniSha256=Get-Digest $ini
    Reject {& $verify @arguments} 'restored output direction despite independent hash'
  } finally { [IO.File]::WriteAllBytes($ini,$originalIni);$audit.profileIniSha256=Get-Digest $ini }
  $checks.Add('Reviewed normal UI persistence can relaunch; changed chart paths or enabled output cannot')
  $audit.reviewedUtc=[DateTime]::UtcNow.AddHours(-25).ToString('o')
  Reject {& $verify @arguments} 'expired audit'
  $audit.reviewedUtc=[DateTime]::UtcNow.AddHours(1).ToString('o')
  Reject {& $verify @arguments} 'future audit'
  $audit.reviewedUtc=[DateTime]::UtcNow.ToString('o')
  if ($StockFixture) {
    $audit.upstreamCommit='0'*40;Reject {& $verify @arguments} 'different stock upstream provenance'
    $audit.upstreamCommit='37fd0cddb7334fe489e9f18aa163977a9c5c84f7'
  } else {
    $audit.buildCommit='0'*40;Reject {& $verify @arguments} 'different build'
    $audit.buildCommit=$installed.ownership.commit
  }
  $checks.Add('Expired/future review and different executable provenance refuse launch')
  $savedContext=$fixtureContext
  try { $fixtureContext=$fixtureContext | ConvertTo-Json -Depth 8 | ConvertFrom-Json;$fixtureContext.pluginRoots=@($managed);Reject {& $verify @arguments} 'actual installed root omitted/changed' }
  finally { $fixtureContext=$savedContext }
  $checks.Add('Changing the actual root/context set invalidates the prepared source review')
  $null=& $verify @arguments
  $checks.Add('All rejected changes leave the original fixture transaction verifiable')
  [pscustomobject]@{status='passed';identity=$(if($StockFixture){'stock-only'}elseif($InstalledWelcomeFixture){'installed-warning-runtime'}else{'installed'});environment=$(if($native){'native-windows-disposable-filesystem'}else{'linux-powershell-portable-contracts'});count=$checks.Count;checks=@($checks);boatAccess=$false;applicationLaunched=$false;hardwareCommands=$false;productOrBoatAcceptance=$false} | ConvertTo-Json -Depth 6
} finally {
  $env:PATH=$savedPath;$script:CommissioningBaseline=$savedBaseline
  Remove-Item -LiteralPath $testRoot -Recurse -Force
}

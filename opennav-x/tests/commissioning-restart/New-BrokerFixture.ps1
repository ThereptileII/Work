# Keep the inert root the same byte length as the fixed recovered-root contract.
# No production baseline/lineage validation is substituted or relaxed.
function New-BrokerFixtureProfileBytes([string]$ChartPalette='') {
 $text="[Settings]`r`nPersistActiveRoute=0`r`n[Settings/NMEADataSource]`r`nDataConnections=0;0;;0;1;COM8;115200;0;1;0;;0;;0;0;1;0;1;Gateway;0;;0`r`n[Directories]`r`nChartDir=original`r`n[OpenNav]`r`nInterfaceMode=xnav`r`n"
 if($ChartPalette){if($ChartPalette -cnotin @('XNav','Standard')){throw 'Fixed synthetic palette required.'};$text+='ChartPresentationV1='+$ChartPalette+"`r`n"}
 $encoding=New-Object Text.UTF8Encoding($false,$true)
 while($encoding.GetByteCount($text)+80 -le 21380){$text+='# '+('-'*76)+"`r`n"}
 $remaining=21380-$encoding.GetByteCount($text)
 if($remaining -lt 0){throw 'Synthetic broker profile exceeded fixed recovered-root size.'}
 $text+=(' '*$remaining)
 $bytes=$encoding.GetBytes($text)
 if($bytes.Length -ne 21380){throw 'Synthetic root byte-size contract differs.'}
 return ,$bytes
}
# Construction only; every file is synthetic and inside a caller-created TEMP
# directory. The application/companion are the verified marker-only test build.
function New-BrokerFixture([string]$Directory,[string]$Binaries,[string]$SourceTools,[string]$FixtureSources,[string]$Case,[switch]$WithoutRestartSession) {
 $null=New-Item -ItemType Directory -Path $Directory
 $scripts=Join-Path $Directory 'scripts';$null=New-Item -ItemType Directory -Path $scripts
 # Copy the same dependency inventory that the unchanged session pins.
 $names=@($script:RestartDependencies)
 foreach($name in @($names|Sort-Object -Unique)){Copy-Item -LiteralPath (Join-Path $SourceTools $name) -Destination (Join-Path $scripts $name)}
 Copy-Item -LiteralPath (Join-Path $FixtureSources 'broker-fixture-identity.ps1') -Destination (Join-Path $scripts 'BrokerFixtureIdentity.ps1')
 # These are the ONLY source substitutions: OS known folders/installation
 # identity in copied dependencies. Broker.ps1 itself stays byte-for-byte exact.
 # Its policy, process, pipe, diff, audit, permit and receipt logic is unchanged.
 foreach($name in @('Common.ps1','RestartCommissioning.ps1')) {
  $path=Join-Path $scripts $name;$body=[IO.File]::ReadAllText($path)
  foreach($pair in @(@("([Environment]::GetFolderPath('CommonApplicationData'))",'($script:BrokerFixture.commonData)'),@("[Environment]::GetFolderPath('LocalApplicationData')",'$script:BrokerFixture.localData'))) {
   if(-not $body.Contains($pair[0])){if($name -ceq 'Common.ps1'){throw 'Known-folder substitution no longer matches inspected source.'};continue}
   $body=$body.Replace($pair[0],$pair[1])
  }
  [IO.File]::WriteAllText($path,$body,(New-Object Text.UTF8Encoding($false)))
 }
 foreach($name in @('Common.ps1','Commissioning.ps1')) {[IO.File]::AppendAllText((Join-Path $scripts $name),"`r`n. (Join-Path `$PSScriptRoot 'BrokerFixtureIdentity.ps1')`r`n")}
 [IO.File]::AppendAllText((Join-Path $scripts 'RestartCommissioning.ps1'),"`r`n`$script:RestartDependencies+='BrokerFixtureIdentity.ps1'`r`n")
 if((Get-Digest (Join-Path $scripts 'RestartCommissioningBroker.ps1')) -cne (Get-Digest (Join-Path $SourceTools 'RestartCommissioningBroker.ps1'))){throw 'Actual production broker was modified.'}
 $workspace=Join-Path $Directory 'workspace';$profile=Join-Path $Directory 'common/opencpn';$local=Join-Path $Directory 'local'
 $installedRoot=Join-Path $local 'OpenNavXAlpha1';$generationId=[guid]::NewGuid().ToString('N');$generation=Join-Path $installedRoot ('generations/'+$generationId)
 $app=Join-Path $generation 'app';$stockApp=Join-Path $Directory 'stock';$managed=Join-Path $local 'opencpn/plugins'
 $preparedDir=Join-Path $workspace 'runs/fixture-commissioning';$quarantine=Join-Path $preparedDir 'quarantine';$search=Join-Path $Directory 'search'
 foreach($path in @($workspace,$profile,$managed,$stockApp,$app,(Join-Path $app 'plugins'),(Join-Path $stockApp 'plugins'),(Join-Path $generation 'docs'),$preparedDir,$quarantine,$search)){$null=New-Item -ItemType Directory -Path $path -Force}
 foreach($name in @('opencpn.exe','opennav-restart.exe')){Copy-Item -LiteralPath (Join-Path $Binaries $name) -Destination (Join-Path $app $name)}
 $exe=Join-Path $app 'opencpn.exe';$helper=Join-Path $app 'opennav-restart.exe';$stockExe=Join-Path $stockApp 'opencpn.exe'
 [IO.File]::WriteAllText($stockExe,'Inert synthetic stock identity; never launched.')
 $commit='1'*40;$sid=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value;$windowsSession=[Diagnostics.Process]::GetCurrentProcess().SessionId
 if($windowsSession -le 0){throw 'Native broker fixture needs interactive CI session; never impersonates another session.'}
 $build=Join-Path $generation 'docs/PRODUCT_BUILD.json'
 Write-Record $build @{commit=$commit;test_fixtures=$false;build_purpose='INSTALLED PRODUCT';commissioning_restart_protocol=1;executable_sha256=(Get-Digest $exe);restart_helper_sha256=(Get-Digest $helper);testOnly='Synthetic marker identity, not a qualified application'}
 $ownership=@{owner='OpenNavX.Alpha1.SideBySide.1';commit=$commit;managedFiles=@(@{path='app/opencpn.exe';sha256=(Get-Digest $exe)},@{path='app/opennav-restart.exe';sha256=(Get-Digest $helper)},@{path='docs/PRODUCT_BUILD.json';sha256=(Get-Digest $build)})}
 Write-Record (Join-Path $generation 'ownership.json') $ownership
 Write-Record (Join-Path $installedRoot 'state.json') @{schema=1;owner=$ownership.owner;current=$generationId;stock=@{path=$stockExe}}
 $roots=@($managed,(Join-Path $stockApp 'plugins'),(Join-Path $app 'plugins')|Sort-Object)
 $environment=Get-CommissioningLaunchEnvironment $exe ([Environment]::GetFolderPath('Windows'))
 $context=[pscustomobject]@{workspace=$workspace;profile=$profile;managed=$managed;application=$stockApp;pluginRoots=$roots;installation=@{executable=$exe;commit=$commit};launchEnvironment=$environment;sid=$sid;session=$windowsSession}
 $baseline=Join-Path $preparedDir 'baseline.ini';$input=Join-Path $preparedDir 'input-only.ini';$ini=Join-Path $profile 'opencpn.ini'
 $bytes=New-BrokerFixtureProfileBytes $(if($Case -clike 'palette-*'){'XNav'}else{''})
 [IO.File]::WriteAllBytes($baseline,$bytes);$baselineHash=Get-Digest $baseline
 [IO.File]::WriteAllBytes($input,(Get-CommissioningInputBytes $bytes));[IO.File]::Copy($input,$ini)
 $safe=Join-Path $managed 'dashboard_pi.dll';$unsafe=Join-Path $managed 'control_pi.dll';$library=Join-Path $managed 'inert-helper.dll'
 foreach($file in @($safe,$unsafe,$library)){[IO.File]::WriteAllText($file,'inert synthetic plugin file; never loaded')}
 $stockSafe=Join-Path $stockApp 'plugins/dashboard_pi.dll';$bundledSafe=Join-Path $app 'plugins/dashboard_pi.dll'
 [IO.File]::WriteAllText($stockSafe,'Distinct inert stock plugin bytes; never loaded')
 [IO.File]::WriteAllText($bundledSafe,'Distinct inert bundled plugin bytes; never loaded')
 $trees=@(Get-CommissioningTrees $roots);$candidates=@(Get-CommissioningCandidates $trees)
 $inventory=Join-Path $preparedDir 'inventory.json'
 Write-Record $inventory @{schema=1;owner='OpenNavX.ReadOnlyCommissioning.Inventory.1';context=$context;profileSha256=$baselineHash;trees=$trees;plugins=$candidates}
 $decisions=@();$evidence=@();$moves=@();$index=0
 foreach($candidate in $candidates) {
  $index++;$review=Join-Path $preparedDir ('review-'+$index+'.txt');[IO.File]::WriteAllText($review,'TEST ONLY inert source review, not a boat attestation.')
  $retain=$candidate.path -cin @($safe,$stockSafe,$bundledSafe)
  $decisions+=@{path=$candidate.path;sha256=$candidate.sha256;decision=$(if($retain){'retain'}else{'quarantine'});sourceRevision=$(if($retain){'1'*40}else{$null});startupAndIdleReadOnly=$retain;reason='Inert test only';sourceBoundary='No loaded DLL';evidencePath=$review;evidenceSha256=(Get-Digest $review)}
  $evidence+=@{path=$review;sha256=(Get-Digest $review)}
  if(-not $retain){$backup=Join-Path $preparedDir ('plugin-backup-'+$index+'.bin');[IO.File]::Copy($candidate.path,$backup);$moves+=@{path=$candidate.path;sha256=$candidate.sha256;backup=$backup;destination=(Join-Path $quarantine ('plugin-'+$index+'.bin'))}}
 }
 $plan=Join-Path $preparedDir 'review-plan.json'
 Write-Record $plan @{schema=1;owner='OpenNavX.ReadOnlyCommissioning.Plan.1';reviewedUtc=[DateTime]::UtcNow.ToString('o');inventorySha256=(Get-Digest $inventory);plugins=$decisions}
 $prepared=Join-Path $preparedDir 'prepared.json'
 Write-Record $prepared @{schema=1;owner=$script:CommissioningOwner;status='prepared';context=$context;baselineSha256=$baselineHash;inputSha256=(Get-Digest $input);planSha256=(Get-Digest $plan);inventorySha256=(Get-Digest $inventory);evidence=$evidence;quarantine=$moves}
 Write-Record (Join-Path $workspace 'commissioning-active.json') @{schema=1;owner=$script:CommissioningOwner;record=$prepared;recordSha256=(Get-Digest $prepared)}
 foreach($move in $moves){[IO.File]::Move($move.path,$move.destination)}
 $applied=Join-Path $preparedDir 'applied.json';Write-Record $applied @{schema=1;owner=$script:CommissioningOwner;status='input-only-prepared';recordSha256=(Get-Digest $prepared);profileSha256=(Get-Digest $input);remainingPluginCount=3}
 $audit=@{profileIniSha256=(Get-Digest $ini);buildCommit=$commit;reviewedUtc=[DateTime]::UtcNow.ToString('o');connectionsOutputDisabled=$true;pluginOutputsReviewed=$true;noActiveRouteOutput=$true;pluginFiles=@($decisions|Where-Object {$_.decision -ceq 'retain' -and $_.path -cne $stockSafe});commissioning=@{record=$prepared;recordSha256=(Get-Digest $prepared);appliedSha256=(Get-Digest $applied)}}
 $target=Join-Path $workspace 'boat-target.json';Write-Record $target @{schema=1;owner='OpenNavX.BoatTarget.1';profileDirectory=$profile;stockExecutable=$stockExe;readOnlyAudit=$audit}
 $fixture=@{owner='OpenNavX.TestOnly.BrokerFixture.1';noMarineCode=$true;root=$Directory;workspace=$workspace;commonData=(Split-Path $profile -Parent);localData=$local;installedRoot=$installedRoot;stockExecutable=$stockExe;stockSha256=(Get-Digest $stockExe);context=$context;baselineSha256=$baselineHash}
 $fixtureFile=Join-Path $scripts 'fixture.identity.json';Write-Record $fixtureFile $fixture
 $shutdownRecord=@{schema=2;physicalCommands=0;reviewedUtc=[DateTime]::UtcNow.ToString('o');plugins=@($decisions|Where-Object {$_.decision -ceq 'retain'}|ForEach-Object {@{path=$_.path;sha256=$_.sha256;plugin='dashboard_pi';revision=$_.sourceRevision;sourceSha256=('2'*64);shutdownBoundary='INERT TEST ONLY'}})}
 if($WithoutRestartSession) {
  $shutdown=Join-Path $Directory 'reviewed-shutdown.json'
  Write-Record $shutdown $shutdownRecord
  return [pscustomobject]@{root=$Directory;scripts=$scripts;workspace=$workspace;app=$app;executable=$exe;helper=$helper;profile=$ini;plugin=$safe;identityHash=(Get-Digest $fixtureFile);shutdown=$shutdown;shutdownSha256=(Get-Digest $shutdown);productBuild=$build;generation=$generation}
 }
 $sessionDir=New-PreparationDirectory ([pscustomobject]@{workspace=$workspace;sid=$sid}) 'restart-session'
 $before=Join-Path $sessionDir 'before.ini';[IO.File]::Copy($ini,$before)
 $shutdown=Join-Path $sessionDir 'shutdown-review.json';Write-Record $shutdown $shutdownRecord
 $deps=@($script:RestartDependencies)+@('BrokerFixtureIdentity.ps1');$pins=@($deps|ForEach-Object {@{name=$_;sha256=(Get-Digest (Join-Path $scripts $_))}})
 $record=Join-Path $sessionDir 'session.json';$now=[DateTime]::UtcNow
 $session=@{schema=1;owner=$script:RestartOwner;session=(New-RestartToken);createdUtc=$now.ToString('o');expiresUtc=$now.AddHours(1).ToString('o');sid=$sid;windowsSessionId=$windowsSession.ToString();workspace=$workspace;generation=$generationId;buildCommit=$commit;executable=$exe;executableSha256=(Get-Digest $exe);helper=$helper;helperSha256=(Get-Digest $helper);productBuild=$build;productBuildSha256=(Get-Digest $build);profile=$ini;beforeIni=$before;beforeIniSha256=(Get-Digest $before);audit=$audit;targetSha256=(Get-Digest $target);workingDirectory=$app;path=$environment.path;toolDirectory=$scripts;scripts=$pins;shutdownReview=$shutdown;shutdownReviewSha256=(Get-Digest $shutdown)}
 if($Case -ceq 'expired'){$session.createdUtc=$now.AddHours(-2).ToString('o');$session.expiresUtc=$now.AddHours(-1).ToString('o')}
 Write-Record $record $session
 return [pscustomobject]@{root=$Directory;scripts=$scripts;app=$app;executable=$exe;helper=$helper;profile=$ini;plugin=$safe;record=$record;recordSha256=(Get-Digest $record);session=$session;sessionDirectory=$sessionDir;identityHash=(Get-Digest $fixtureFile);transition=(Join-Path $sessionDir 'transition-0001')}
}

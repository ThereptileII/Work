# Bounded byte/lineage policies on private temporary files. Native entrypoint
# interruption/restore coverage runs through test-commissioning -AdoptionFixture.
[CmdletBinding()]
param([switch]$PortableContracts,[switch]$IsolatedLocal)
. (Join-Path $PSScriptRoot 'Commissioning.ps1')
$native=[Environment]::OSVersion.Platform -eq 'Win32NT'
if(-not $native -and -not $PortableContracts){throw 'Explicit portable contracts required.'}
if($native -and -not $IsolatedLocal -and $env:GITHUB_ACTIONS -cne 'true'){throw 'Disposable CI or explicit isolated temporary tests required.'}
$testRoot=Join-Path ([IO.Path]::GetTempPath()) ('OpenNav baseline '+[guid]::NewGuid().ToString('N'));$null=New-Item -ItemType Directory -Path $testRoot
$savedBaseline=$script:CommissioningBaseline
if(-not $native){function Assert-LocalPath([string]$Path){$p=[IO.Path]::GetFullPath($Path);if(-not $p.StartsWith($testRoot+'/',[StringComparison]::Ordinal)){throw 'Escaped fixture root'};return $p}}
$checks=New-Object 'Collections.Generic.List[string]'
function Pass([string]$Name,[scriptblock]$Body){& $Body;$checks.Add($Name)}
function Refuse([string]$Name,[scriptblock]$Body){$bad=$false;try{$null=& $Body}catch{$bad=$true};if(-not $bad){throw ('Accepted unsafe adoption: '+$Name)};$checks.Add($Name)}
function Clone($Value){return ($Value|ConvertTo-Json -Depth 16|ConvertFrom-Json)}
function Review([string]$Before,[string]$After){return [pscustomobject]@{schema=1;owner='OpenNavX.ProfileMigrationReview.1';beforeSha256=(Get-Digest $Before);afterSha256=(Get-Digest $After);reviewedUtc=[datetime]::UtcNow.ToString('o');changes=@(Get-CommissioningIniDiff $Before $After|ForEach-Object{[pscustomobject]@{key=$_.key;before=$_.before;after=$_.after;sourceRevision='37fd0cddb7334fe489e9f18aa163977a9c5c84f7';sourceBoundary='Disposable fixture for pinned navutil writes';reason='Synthetic test approval only; never a boat review'}})}}
try {
  $workspace=Join-Path $testRoot 'workspace';$parent=Join-Path $workspace 'runs/20260927-000000-read-only-commissioning-aaaaaaaa';$adoption=Join-Path $workspace 'runs/20260927-000001-baseline-adoption-bbbbbbbb'
  $null=New-Item -ItemType Directory -Path $parent,$adoption -Force
  $encoding=New-Object Text.UTF8Encoding($false,$true)
  $text="[Settings]`r`nConfigVersionString=Version 5.12.2 Build 2025-08-01`r`nNavMessageShown=0`r`nLocale=sv`r`nPersistActiveRoute=0`r`n[Settings/NMEADataSource]`r`nDataConnections=0;0;;0;1;COM8;115200;0;1;0;;0;;0;0;1;0;1;Gateway;0;;0`r`n[Directories]`r`nChartDir=original`r`n"
  $text+='#'+(' '*(21380-$encoding.GetByteCount($text)-3))+"`r`n"
  $baseline=Join-Path $parent 'baseline.ini';$inputFile=Join-Path $parent 'input-only.ini';$current=Join-Path $adoption 'post-session.ini';$output=Join-Path $adoption 'baseline.ini'
  [IO.File]::WriteAllText($baseline,$text,$encoding);[IO.File]::WriteAllBytes($inputFile,(Get-CommissioningInputBytes ($encoding.GetBytes($text))))
  $migrated=$encoding.GetString([IO.File]::ReadAllBytes($inputFile)).Replace('Version 5.12.2 Build 2025-08-01','Version 5.12.4-0+37fd0cd Build 2025-09-12').Replace('NavMessageShown=0','NavMessageShown=1')
  $migrated+="[Settings/GlobalState]`r`nFrameWinX=1280`r`nFrameWinY=800`r`n"
  [IO.File]::WriteAllText($current,$migrated,$encoding);$review=Review $inputFile $current
  Pass 'Each exact startup write needs explicit matching source review; Swedish locale preserved' {$null=Assert-CommissioningMigrationReview $inputFile $current $review;if((Read-ProfileForAudit $current)['Settings/Locale'] -cne 'sv'){throw 'Locale changed'}}
  $outputBytes=Get-CommissioningOutputBytes ([IO.File]::ReadAllBytes($current));[IO.File]::WriteAllBytes($output,$outputBytes)
  Pass 'Reverse changes only COM8 direction, preserves UTF8/CRLF and all migrated bytes' {
    $before=[IO.File]::ReadAllBytes($current);$changed=0
    for($i=0;$i -lt $before.Length;$i++){if($before[$i] -ne $outputBytes[$i]){if($before[$i] -ne 48 -or $outputBytes[$i] -ne 49){throw 'Wrong reversed byte'};$changed++}}
    if($changed -ne 1 -or (Get-CommissioningHash (Get-CommissioningInputBytes $outputBytes)) -cne (Get-Digest $current)){throw 'Not an exact inverse'}
  }
  foreach($bad in @($migrated.Replace('COM8','COM9'),$migrated.Replace('COM8;115200;0;0','COM8;115200;0;1'),$migrated.Replace('1;COM8','0;COM8'),$migrated+"DataConnections=x`r`n")){Refuse 'Malformed or non-input-only reverse refused' {Get-CommissioningOutputBytes ($encoding.GetBytes($bad))}}
  foreach($field in @('owner','beforeSha256','afterSha256')){Refuse "Different migration binding $field" {$v=Clone $review;$v.$field='changed';Assert-CommissioningMigrationReview $inputFile $current $v}}
  Refuse 'Missing approved key' {$v=Clone $review;$v.changes=@($v.changes|Select-Object -Skip 1);Assert-CommissioningMigrationReview $inputFile $current $v}
  foreach($field in @('key','before','after','sourceRevision','sourceBoundary','reason')){Refuse "Changed or missing per-key approval $field" {$v=Clone $review;$v.changes[0].$field='';Assert-CommissioningMigrationReview $inputFile $current $v}}
  foreach($time in @([datetime]::UtcNow.AddHours(-25),[datetime]::UtcNow.AddMinutes(1))){Refuse 'Expired/future migration review' {$v=Clone $review;$v.reviewedUtc=$time.ToString('o');Assert-CommissioningMigrationReview $inputFile $current $v}}
  $badIni=Join-Path $testRoot 'changed.ini'
  $case=0
  foreach($bad in @($migrated.Replace('ChartDir=original','ChartDir=other'),$migrated.Replace('COM8;115200;0;0','COM8;115200;0;1'),($migrated+"[OpenNav/Autopilot]`r`nControlEnabled=1`r`n"),$migrated.Replace('FrameWinX=1280','FrameWinX=99999999'),$migrated.Replace('Version 5.12.4','Version 9.99.9'))){
    $case++
    [IO.File]::WriteAllText($badIni,$bad,$encoding);$badReview=Review $inputFile $badIni
    Refuse "Matching text approval cannot override source/connection/chart/value policy $case" {Assert-CommissioningMigrationReview $inputFile $badIni $badReview}
  }
  $textureBefore=Join-Path $testRoot 'texture-before.ini';$textureAfter=Join-Path $testRoot 'texture-after.ini'
  $textureText="[Settings]`r`nConfigVersionString=Version 5.12.4+37fd0cd Build 2026-09-27`r`nOpenGL=1`r`nGPUTextureMemSize=64`r`n[Settings/NMEADataSource]`r`nDataConnections=0;0;;0;1;COM8;115200;0;0;0;;0;;0;0;1;0;1;Gateway;0;;0`r`n"
  [IO.File]::WriteAllText($textureBefore,$textureText,$encoding)
  [IO.File]::WriteAllText($textureAfter,$textureText.Replace('GPUTextureMemSize=64','GPUTextureMemSize=128'),$encoding)
  $textureReview=Review $textureBefore $textureAfter
  Pass 'Pinned normal startup texture minimum preserves all other settings with explicit exact review' {
    $accepted=@(Assert-CommissioningMigrationReview $textureBefore $textureAfter $textureReview)
    if($accepted.Count -ne 1 -or $accepted[0].key -cne 'Settings/GPUTextureMemSize'){throw 'Unexpected texture delta'}
    $bytes=Get-CommissioningOutputBytes ([IO.File]::ReadAllBytes($textureAfter))
    if((Get-CommissioningHash (Get-CommissioningInputBytes $bytes)) -cne (Get-Digest $textureAfter)){throw 'Restoration altered normalized bytes'}
  }
  Refuse 'Texture normalization cannot approve itself' {$v=Clone $textureReview;$v.changes=@();Assert-CommissioningMigrationReview $textureBefore $textureAfter $v}
  $textureOld=Read-ProfileForAudit $textureBefore;$textureNew=Read-ProfileForAudit $textureAfter
  Pass 'Explicit unchanged expert-off value also follows the pinned minimum' {
    $was=$textureOld.Clone();$is=$textureNew.Clone();$was['Settings/OpenGLExpert']='0';$is['Settings/OpenGLExpert']='0'
    Assert-CommissioningTextureMinimum $was $is
  }
  foreach($kind in @('before-other','after-other','missing-budget','gl-disabled','gl-changed','expert-enabled','expert-changed','expert-malformed','version-unknown','version-changed')) {
    Refuse "Texture normalization refuses $kind" {
      $was=$textureOld.Clone();$is=$textureNew.Clone()
      switch($kind) {
        'before-other' {$was['Settings/GPUTextureMemSize']='32'}
        'after-other' {$is['Settings/GPUTextureMemSize']='256'}
        'missing-budget' {$was.Remove('Settings/GPUTextureMemSize')}
        'gl-disabled' {$was['Settings/OpenGL']='0';$is['Settings/OpenGL']='0'}
        'gl-changed' {$is['Settings/OpenGL']='0'}
        'expert-enabled' {$was['Settings/OpenGLExpert']='1';$is['Settings/OpenGLExpert']='1'}
        'expert-changed' {$is['Settings/OpenGLExpert']='0'}
        'expert-malformed' {$was['Settings/OpenGLExpert']='false';$is['Settings/OpenGLExpert']='false'}
        'version-unknown' {$was['Settings/ConfigVersionString']='Version other';$is['Settings/ConfigVersionString']='Version other'}
        'version-changed' {$is['Settings/ConfigVersionString']='Version 5.12.4+37fd0cd Build 2026-09-28'}
      }
      Assert-CommissioningTextureMinimum $was $is
    }
  }
  $stockBefore=Join-Path $testRoot 'stock-before.ini';$stockAfter=Join-Path $testRoot 'stock-after.ini'
  $menu='Menu:1;10;-20;0;0;0;400;0;0;0;1;0;0;2;32;Segoe UI:rgb(0, 0, 0)'
  $swedish='Meny:1;9;-18;0;0;0;400;0;0;0;1;0;0;2;32;Segoe UI:rgb(0, 0, 0)'
  $stockText="[Settings]`r`nConfigVersionString=Version 5.12.2-0+b69f44c Build 2025-08-01`r`nLocale=sv`r`nLocaleOverride=sv_SE`r`nOpenGL=1`r`nGPUTextureMemSize=128`r`n[Settings/NMEADataSource]`r`nDataConnections=0;0;;0;1;COM8;115200;0;0;0;;0;;0;0;1;0;1;Gateway;0;;0`r`n[Settings/MSWFonts]`r`nsv-00c6075a=$menu`r`nsv-f4c5f476=$swedish`r`nsv_SE-f4c5f476=$swedish`r`n[Settings/GlobalState]`r`nOwnShipLatLon=`"   57.1000,   16.2000`"`r`n"
  $stockMigrated=$stockText.Replace('Version 5.12.2-0+b69f44c Build 2025-08-01','Version 5.12.4-0+37fd0cd Build 2025-09-12').Replace('GPUTextureMemSize=128','GPUTextureMemSize=64').Replace("sv-00c6075a=$menu`r`n",'').Replace('57.1000','57.1001')
  [IO.File]::WriteAllText($stockBefore,$stockText,$encoding);[IO.File]::WriteAllText($stockAfter,$stockMigrated,$encoding)
  $stockReview=Review $stockBefore $stockAfter
  Pass 'Exact reviewed stock GPU upgrade, obsolete English menu font removal and quoted coordinates preserve Swedish replacements' {$null=Assert-CommissioningMigrationReview $stockBefore $stockAfter $stockReview}
  Refuse 'Source-proven removal still needs its explicit per-key review' {$v=Clone $stockReview;$v.changes=@($v.changes|Where-Object {$_.key -cne 'Settings/MSWFonts/sv-00c6075a'});Assert-CommissioningMigrationReview $stockBefore $stockAfter $v}
  Refuse 'Removal review requires an explicit null after property, not an absent property' {
    $v=Clone $stockReview;$entry=@($v.changes|Where-Object {$_.key -ceq 'Settings/MSWFonts/sv-00c6075a'})[0]
    $entry.PSObject.Properties.Remove('after');Assert-CommissioningMigrationReview $stockBefore $stockAfter $v
  }
  [IO.File]::WriteAllText($badIni,$stockMigrated.Replace('[Settings/MSWFonts]',"[Settings/MSWFonts]`r`nsv-00c6075a="),$encoding)
  Refuse 'Retaining the obsolete font with an empty value is not its permitted removal' {Assert-CommissioningMigrationReview $stockBefore $badIni (Review $stockBefore $badIni)}
  $oldValues=Read-ProfileForAudit $stockBefore;$newValues=Read-ProfileForAudit $stockAfter
  foreach($kind in @('budget-other','budget-missing','already-current','gl-disabled','old-version-unknown','menu-changed','locale-changed','translated-changed','translated-removed','wrong-font-key')) {
    Refuse "Observed startup exception cannot authorize other changes: $kind" {
      $was=$oldValues.Clone();$is=$newValues.Clone();$key='Settings/GPUTextureMemSize'
      switch($kind) {
        'budget-other' {$is[$key]='128'}
        'budget-missing' {$was.Remove($key)}
        'already-current' {$was['Settings/ConfigVersionString']=$is['Settings/ConfigVersionString']}
        'gl-disabled' {$is['Settings/OpenGL']='0'}
        'old-version-unknown' {$was['Settings/ConfigVersionString']='Version other'}
        'menu-changed' {$key='Settings/MSWFonts/sv-00c6075a';$was[$key]=$menu.Replace(';10;',';11;')}
        'locale-changed' {$key='Settings/MSWFonts/sv-00c6075a';$is['Settings/Locale']='en_US'}
        'translated-changed' {$key='Settings/MSWFonts/sv-00c6075a';$is['Settings/MSWFonts/sv-f4c5f476']=$swedish.Replace(';9;',';10;')}
        'translated-removed' {$key='Settings/MSWFonts/sv-00c6075a';$is.Remove('Settings/MSWFonts/sv_SE-f4c5f476')}
        'wrong-font-key' {$key='Settings/MSWFonts/sv-f4c5f476'}
      }
      Assert-CommissioningStockUpgradeDelta $key $was $is
    }
  }
  [IO.File]::WriteAllText($badIni,$stockMigrated.Replace("sv-f4c5f476=$swedish`r`n",''),$encoding)
  Refuse 'Other font deletion is not a generic permitted removal' {Assert-CommissioningMigrationReview $stockBefore $badIni (Review $stockBefore $badIni)}
  [IO.File]::WriteAllText($badIni,$stockMigrated.Replace("OpenGL=1`r`n",''),$encoding)
  Refuse 'Unrelated key removal remains forbidden' {Assert-CommissioningMigrationReview $stockBefore $badIni (Review $stockBefore $badIni)}
  $variationKey='Settings/CommPriority/PriorityVariation'
  $variationPrefix='nmea2000 COM8:105;127250|N2k device address: 243 ; PGN: 127250|N2k device address: 35 ; PGN: 127250|N2k device address: 33 ; PGN: 127250|N2k device address: 49 ; PGN: 127250|'
  $variationAppend='N2k device address: 204 ; PGN: 127250|'
  $variationBefore=Join-Path $testRoot 'variation-before.ini';$variationAfter=Join-Path $testRoot 'variation-after.ini'
  $variationText=$stockText.Replace("OpenGL=1`r`n","OpenGL=1`r`nPersistActiveRoute=0`r`n")+"[Settings/CommPriority]`r`nPriorityVariation=$variationPrefix`r`nPriorityHeading=original-heading`r`nPriorityPosition=original-position`r`n[PlugIns/pilot]`r`nbEnabled=0`r`n"
  $variationMigrated=$variationText.Replace("PriorityVariation=$variationPrefix`r`n","PriorityVariation=$variationPrefix$variationAppend`r`n")
  [IO.File]::WriteAllText($variationBefore,$variationText,$encoding);[IO.File]::WriteAllText($variationAfter,$variationMigrated,$encoding)
  $variationReview=Review $variationBefore $variationAfter
  Pass 'Exact observed sixth variation source requires and accepts one hash-bound per-key review' {
    $accepted=@(Assert-CommissioningMigrationReview $variationBefore $variationAfter $variationReview)
    if($accepted.Count -ne 1 -or $accepted[0].key -cne $variationKey){throw 'Unexpected accepted delta'}
  }
  Pass 'Restored baseline preserves the learned source and reverses only the temporary connection byte' {
    $preserved=Get-CommissioningOutputBytes ([IO.File]::ReadAllBytes($variationAfter))
    if((Get-CommissioningHash (Get-CommissioningInputBytes $preserved)) -cne (Get-Digest $variationAfter) -or
       -not $encoding.GetString($preserved).Contains("PriorityVariation=$variationPrefix$variationAppend`r`n")){throw 'Observed source was erased or another byte changed'}
  }
  Refuse 'Observed source discovery cannot approve itself without per-key review' {$v=Clone $variationReview;$v.changes=@();Assert-CommissioningMigrationReview $variationBefore $variationAfter $v}
  foreach($field in @('beforeSha256','afterSha256')) {
    Refuse "Variation review must retain exact $field" {$v=Clone $variationReview;$v.$field='0'*64;Assert-CommissioningMigrationReview $variationBefore $variationAfter $v}
  }
  $badVariation=@{
    reordered=$variationPrefix.Replace('address: 243','address: TEMP').Replace('address: 35','address: 243').Replace('address: TEMP','address: 35')+$variationAppend
    prepended=$variationAppend+$variationPrefix
    inserted=$variationPrefix.Replace('address: 49',('address: 204 ; PGN: 127250|N2k device address: 49'))
    deleted=$variationPrefix.Replace('N2k device address: 35 ; PGN: 127250|','')+$variationAppend
    duplicate=$variationPrefix+'N2k device address: 49 ; PGN: 127250|'
    multiple=$variationPrefix+$variationAppend+$variationAppend
    changedprefix=$variationPrefix.Replace('COM8:105','COM8:106')+$variationAppend
    otheraddress=$variationPrefix+$variationAppend.Replace('204','205')
    otherpgn=$variationPrefix+$variationAppend.Replace('127250','127251')
    truncated=($variationPrefix+$variationAppend).TrimEnd('|')
    leading=' '+$variationPrefix+$variationAppend
    trailing=$variationPrefix+$variationAppend+' '
    quoted='"'+$variationPrefix+$variationAppend+'"'
    missing=''
  }
  foreach($kind in $badVariation.Keys) {
    [IO.File]::WriteAllText($badIni,$variationMigrated.Replace($variationPrefix+$variationAppend,$badVariation[$kind]),$encoding)
    Refuse "Other variation priority change refused despite matching review: $kind" {Assert-CommissioningMigrationReview $variationBefore $badIni (Review $variationBefore $badIni)}
  }
  [IO.File]::WriteAllText($badIni,$variationText.Replace('PriorityVariation=','PriorityVariation= '),$encoding)
  Refuse 'Original five-source prefix must be exact, not whitespace-normalized' {Assert-CommissioningMigrationReview $badIni $variationAfter (Review $badIni $variationAfter)}
  $unrelatedVariation=@{
    indented=$variationMigrated.Replace('PriorityVariation=',' PriorityVariation=')
    keycase=$variationMigrated.Replace('PriorityVariation=','priorityvariation=')
    ambiguous=$variationMigrated+"[Other]`r`nPriorityVariation=ambiguous`r`n"
    heading=$variationMigrated.Replace('PriorityHeading=original-heading','PriorityHeading=changed')
    position=$variationMigrated.Replace('PriorityPosition=original-position','PriorityPosition=changed')
    output=$variationMigrated.Replace('COM8;115200;0;0','COM8;115200;0;1')
    route=$variationMigrated.Replace('PersistActiveRoute=0','PersistActiveRoute=1')
    plugin=$variationMigrated.Replace('bEnabled=0','bEnabled=1')
    core=$variationMigrated.Replace('OpenGL=1','OpenGL=0')
  }
  foreach($kind in $unrelatedVariation.Keys) {
    [IO.File]::WriteAllText($badIni,$unrelatedVariation[$kind],$encoding)
    Refuse "Observed append cannot authorize unrelated change: $kind" {Assert-CommissioningMigrationReview $variationBefore $badIni (Review $variationBefore $badIni)}
  }
  # A source-reviewed default is a separate installed proof, never a chart-path wildcard.
  $resourceContext=[pscustomobject]@{application=(Join-Path $testRoot 'stock');executable=(Join-Path $testRoot 'stock/opencpn.exe');installation=[pscustomobject]@{generation='owned';commit=('b'*40);ownershipSha256=('c'*64);stateSha256=('d'*64)}}
  $resourceParent=[pscustomobject]@{context=$resourceContext}
  $resourceProof=[pscustomobject]@{owner='OpenNavX.InstalledResourceReview.1';generation='owned';commit=('b'*40);ownershipSha256=('c'*64);stateSha256=('d'*64);
    stockPath=$resourceContext.executable;stockExecutableSha256='7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c';
    markerSha256=(Get-CommissioningHash ($encoding.GetBytes($resourceContext.executable)));basemapDefault=(Join-Path $resourceContext.application 'basemap_shp').Replace('\','\\')}
  $resourceBefore=Join-Path $testRoot 'resource-before.ini';$resourceAfter=Join-Path $testRoot 'resource-after.ini'
  $resourceText=$encoding.GetString([IO.File]::ReadAllBytes($inputFile)).Replace('ChartDir=original',"ChartDir=original`r`nBaseShapefileDir=")
  [IO.File]::WriteAllText($resourceBefore,$resourceText,$encoding)
  [IO.File]::WriteAllText($resourceAfter,$resourceText.Replace("BaseShapefileDir=`r`n",("BaseShapefileDir="+$resourceProof.basemapDefault+"`r`n")),$encoding)
  $resourceReview=Review $resourceBefore $resourceAfter
  Pass 'Empty basemap fills only from hash-bound historical installed stock resource evidence and per-key review' {
    $default=Assert-CommissioningResourceProof $resourceParent $resourceProof
    $null=Assert-CommissioningMigrationReview $resourceBefore $resourceAfter $resourceReview ([datetime]::UtcNow) $default
  }
  Refuse 'Ordinary cold restoration still refuses unreviewed installed path migration' {Assert-CommissioningRestoreIni $resourceBefore $resourceAfter}
  Refuse 'Migration review alone cannot grant a protected directory change' {Assert-CommissioningMigrationReview $resourceBefore $resourceAfter $resourceReview}
  Refuse 'Proven resource default still needs an explicit per-key review' {$v=Clone $resourceReview;$v.changes=@();Assert-CommissioningMigrationReview $resourceBefore $resourceAfter $v ([datetime]::UtcNow) $resourceProof.basemapDefault}
  foreach($field in @('owner','generation','commit','ownershipSha256','stateSha256','stockPath','stockExecutableSha256','markerSha256','basemapDefault')) {
    Refuse "Changed resource evidence $field" {$v=Clone $resourceProof;$v.$field+='changed';Assert-CommissioningResourceProof $resourceParent $v}
  }
  Refuse 'Stock-only parent cannot inherit installed resource identity' {$v=Clone $resourceParent;$v.context.installation=$null;Assert-CommissioningResourceProof $v $resourceProof}
  foreach($kind in @('custom','other-chart','connection','missing-key')) {
    $value=switch($kind){'custom'{$resourceText.Replace("BaseShapefileDir=`r`n","BaseShapefileDir=custom`r`n")};'other-chart'{$resourceText.Replace('ChartDir=original','ChartDir=changed')};'connection'{$resourceText.Replace('COM8','COM9')};'missing-key'{$resourceText.Replace("BaseShapefileDir=`r`n",'')}}
    [IO.File]::WriteAllText($badIni,$value,$encoding)
    Refuse "Default proof cannot override $kind configuration" {Assert-CommissioningMigrationReview $badIni $resourceAfter (Review $badIni $resourceAfter) ([datetime]::UtcNow) $resourceProof.basemapDefault}
  }
  $context=[pscustomobject]@{sid='S-1-5-21-1';profile=(Join-Path $testRoot 'profile')}
  $prepared=[pscustomobject]@{owner=$script:CommissioningOwner;status='prepared';context=$context;baselineSha256=(Get-Digest $baseline);inputSha256=(Get-Digest $inputFile)}
  Refuse 'Synthetic root cannot replace fixed public recovery baseline' {Get-PreparedCommissioningBaseline $prepared $parent $workspace}
  # Isolated test root only, after proving production root refusal. No CLI flag
  # or baseline proposal can change this production constant.
  $script:CommissioningBaseline=Get-Digest $baseline
  $record=Join-Path $parent 'prepared.json';Write-Record $record $prepared;$recordHash=Get-Digest $record
  $savedIni=Join-Path $parent 'post-session-fixture.ini';Copy-PreparationFile $current $savedIni (Get-Digest $current) (Get-Item $current).Length
  $inspection=Join-Path $parent 'restore-inspection-fixture.json';Write-Record $inspection @{owner='OpenNavX.ReadOnlyCommissioning.RestoreInspection.1';recordSha256=$recordHash;currentIniSha256=(Get-Digest $current);savedIni=$savedIni}
  $review|Add-Member parentPreparedSha256 $recordHash;$review|Add-Member inspectionSha256 (Get-Digest $inspection)
  $reviewPath=Join-Path $adoption 'migration-review.json';Write-Record $reviewPath $review
  $proposal=Join-Path $adoption 'proposal.json';Write-Record $proposal @{schema=1;owner=$script:CommissioningBaselineOwner;status='proposed';createdUtc=[datetime]::UtcNow.ToString('o');parentPrepared=$record;parentPreparedSha256=$recordHash;inspection=$inspection;inspectionSha256=(Get-Digest $inspection);currentIniSha256=(Get-Digest $current);reviewSha256=(Get-Digest $reviewPath);baselineSha256=(Get-Digest $output);baselineBytes=$outputBytes.Length}
  Pass 'Exact reviewed proposal validates but is not a completed baseline' {$null=Read-CommissioningAdoptionProposal $workspace $proposal (Get-Digest $proposal) $record $recordHash}
  Refuse 'Proposal alone cannot authorize new Inventory' {Read-CommissioningBaseline $workspace $proposal (Get-Digest $proposal)}
  $complete=Join-Path $parent 'restored-fixture.json';Write-Record $complete @{owner=$script:CommissioningOwner;status='restored';recordSha256=$recordHash;profileSha256=(Get-Digest $output);adoptionProposalSha256=(Get-Digest $proposal);pluginInventoryRestored=$true;originalOutputConfigurationRestored=$true}
  $adopted=Join-Path $adoption 'adopted-baseline.json';Write-Record $adopted @{schema=1;owner=$script:CommissioningBaselineOwner;status='adopted';parentPrepared=$record;parentPreparedSha256=$recordHash;proposalSha256=(Get-Digest $proposal);baselineSha256=(Get-Digest $output);baselineBytes=$outputBytes.Length;restoreCompletion=$complete;restoreCompletionSha256=(Get-Digest $complete)}
  $adoptedHash=Get-Digest $adopted
  Pass 'Completed exact lineage resolves migrated baseline without replacing global pin' {$resolved=Read-CommissioningBaseline $workspace $adopted $adoptedHash;if($resolved.sha256 -cne (Get-Digest $output) -or $script:CommissioningBaseline -cne (Get-Digest $baseline)){throw 'Global root changed'}}
  foreach($path in @($baseline,$inputFile,$record,$savedIni,$inspection,$reviewPath,$proposal,$output,$complete)) {
    $saved=[IO.File]::ReadAllBytes($path)
    try{[IO.File]::AppendAllText($path,'changed');Refuse 'Every original/approved/completed lineage byte remains pinned' {Read-CommissioningBaseline $workspace $adopted $adoptedHash}}
    finally{[IO.File]::WriteAllBytes($path,$saved)}
  }
  Refuse 'Excessive or cyclic lineage is bounded' {Read-CommissioningBaseline $workspace $adopted $adoptedHash 8}
  $next=Join-Path $workspace 'runs/next';$null=New-Item -ItemType Directory -Path $next
  Copy-PreparationFile $output (Join-Path $next 'baseline.ini') (Get-Digest $output) $outputBytes.Length
  [IO.File]::WriteAllBytes((Join-Path $next 'input-only.ini'),(Get-CommissioningInputBytes $outputBytes))
  $nextPrepared=[pscustomobject]@{owner=$script:CommissioningOwner;status='prepared';context=$context;baselineSha256=(Get-Digest $output);inputSha256=(Get-Digest (Join-Path $next 'input-only.ini'));baselineReference=[pscustomobject]@{record=$adopted;recordSha256=$adoptedHash}}
  Pass 'Next generation may pin exact adopted lineage without reusing its source approvals' {$null=Get-PreparedCommissioningBaseline $nextPrepared $next $workspace}
  Refuse 'Another profile cannot inherit approved migration' {$v=Clone $nextPrepared;$v.context.profile+='-other';Get-PreparedCommissioningBaseline $v $next $workspace}
  Refuse 'Missing lineage cannot silently adopt arbitrary current INI' {$v=Clone $nextPrepared;$v.PSObject.Properties.Remove('baselineReference');Get-PreparedCommissioningBaseline $v $next $workspace}
  $intent=Join-Path $parent 'restore-intent-fixture.json'
  Write-Record $intent @{schema=1;owner=$script:CommissioningOwner;recordSha256=$recordHash;afterSha256=(Get-Digest $output);adoptionProposalSha256=(Get-Digest $proposal)}
  Pass 'Interrupted restoration can resume only the same durable adopted target' {Assert-CommissioningRestoreTarget $parent $recordHash (Get-Digest $output) (Get-Digest $proposal)}
  Refuse 'Omitting adoption on retry cannot reset migrated state' {Assert-CommissioningRestoreTarget $parent $recordHash (Get-Digest $baseline) ''}
  Refuse 'Another proposal with same final bytes cannot take over restoration' {Assert-CommissioningRestoreTarget $parent $recordHash (Get-Digest $output) ('f'*64)}
  Refuse 'Another transaction cannot reuse restoration intent' {Assert-CommissioningRestoreTarget $parent ('f'*64) (Get-Digest $output) (Get-Digest $proposal)}
  foreach($fileName in @('CommissioningBaseline.ps1','InstalledResourceReview.ps1','prepare-baseline-adoption.ps1','commission-read-only.ps1')){Pass "Parses $fileName" {$tokens=$null;$errors=$null;$null=[Management.Automation.Language.Parser]::ParseFile((Join-Path $PSScriptRoot $fileName),[ref]$tokens,[ref]$errors);if($errors.Count){throw ($errors|Out-String)}}}
  $nativeResult=$null
  if($native) {
    $nativeArgs=@{AdoptionFixture=$true};if($IsolatedLocal){$nativeArgs.IsolatedLocal=$true}
    $nativeResult=(& (Join-Path $PSScriptRoot 'test-commissioning.ps1') @nativeArgs)|ConvertFrom-Json
    if($nativeResult.status -cne 'passed'){throw 'Native adoption transaction suite failed'}
    $nativeArgs.ResourceAdoptionFixture=$true
    $resourceNative=(& (Join-Path $PSScriptRoot 'test-commissioning.ps1') @nativeArgs)|ConvertFrom-Json
    if($resourceNative.status -cne 'passed'){throw 'Native installed resource adoption transaction failed'}
    $nativeResult=[pscustomobject]@{status='passed';count=($nativeResult.count+$resourceNative.count);stock=$nativeResult;installedResources=$resourceNative}
  }
  [pscustomobject]@{status='passed';count=($checks.Count+$(if($nativeResult){$nativeResult.count}else{0}));checks=$checks.ToArray();nativeTransactions=$nativeResult;environment=$(if($native){'native-disposable-filesystem'}else{'portable-filesystem-policy'});applicationLaunched=$false;boatAccess=$false;physicalOutput=$false;nativeTransactionAcceptance=($null -ne $nativeResult)}|ConvertTo-Json -Depth 8
}finally{$script:CommissioningBaseline=$savedBaseline;Remove-Item -LiteralPath $testRoot -Recurse -Force}

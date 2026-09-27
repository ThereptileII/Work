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
  foreach($fileName in @('CommissioningBaseline.ps1','prepare-baseline-adoption.ps1','commission-read-only.ps1')){Pass "Parses $fileName" {$tokens=$null;$errors=$null;$null=[Management.Automation.Language.Parser]::ParseFile((Join-Path $PSScriptRoot $fileName),[ref]$tokens,[ref]$errors);if($errors.Count){throw ($errors|Out-String)}}}
  $nativeResult=$null
  if($native) {
    $nativeArgs=@{AdoptionFixture=$true};if($IsolatedLocal){$nativeArgs.IsolatedLocal=$true}
    $nativeResult=(& (Join-Path $PSScriptRoot 'test-commissioning.ps1') @nativeArgs)|ConvertFrom-Json
    if($nativeResult.status -cne 'passed'){throw 'Native adoption transaction suite failed'}
  }
  [pscustomobject]@{status='passed';count=($checks.Count+$(if($nativeResult){$nativeResult.count}else{0}));checks=$checks.ToArray();nativeTransactions=$nativeResult;environment=$(if($native){'native-disposable-filesystem'}else{'portable-filesystem-policy'});applicationLaunched=$false;boatAccess=$false;physicalOutput=$false;nativeTransactionAcceptance=($null -ne $nativeResult)}|ConvertTo-Json -Depth 8
}finally{$script:CommissioningBaseline=$savedBaseline;Remove-Item -LiteralPath $testRoot -Recurse -Force}

# Only the specifically inspected zero-filled INI/TMP pair. Never auto-launches.
[CmdletBinding()]
param(
  [ValidateSet('Prepare','Apply','Verify')][string]$Action='Prepare',
  [string]$Workspace='C:\XNav',
  [string]$Candidate,
  [string]$Record,
  [string]$ExpectedRecordSha256
)
. (Join-Path $PSScriptRoot 'Preparation.ps1')
$context=Get-PreparationContext $Workspace
$original=Join-Path $context.profile 'opencpn.ini'
$originalHash='f84114bfa5c456c4b8f5f073141abb95537155f9f75d78dedc12db3ed5ab3867'
$candidateHash='a2e416b7d35d6d2f82dcf6b3a97136a8fba5a133435c2047e07e09022426aef9'
if ($Action -ceq 'Prepare') {
  Assert-AuthorizedRecovery $original $Candidate
  $candidate=Assert-LocalPath $Candidate
  $snapshot=Get-PreparationTree $context.profile
  $directory=New-PreparationDirectory $context 'zero-profile-recovery'
  $intent=Join-Path $directory 'intent.json'
  Write-Record $intent @{owner='OpenNavX.ExactZeroProfileRecovery.1';status='preparing';createdUtc=[DateTime]::UtcNow.ToString('o');context=$context;original=$original;candidate=$candidate;originalSha256=$originalHash;candidateSha256=$candidateHash;length=21380;profileBefore=$snapshot;originalAcl=(Get-Acl -LiteralPath $original).Sddl;privacy='Private local recovery; original corruption retained for forensics.'}
  Copy-PreparationFile $original (Join-Path $directory 'original-corrupt.ini') $originalHash 21380
  Copy-PreparationFile $candidate (Join-Path $directory 'candidate.ini') $candidateHash 21380
  Assert-PreparationTree $snapshot
  Assert-PreparationClosed @($context.application,$context.managed)
  $prepared=Join-Path $directory 'prepared.json'
  Write-Record $prepared @{schema=1;owner='OpenNavX.ExactZeroProfileRecovery.1';status='prepared';intentSha256=(Get-Digest $intent);applicationLaunched=$false;profileModified=$false}
  [pscustomobject]@{status='prepared';record=$prepared;recordSha256=(Get-Digest $prepared);profileModified=$false;applicationLaunched=$false} | ConvertTo-Json
  return
}
$record=Assert-LocalPath $Record
if ($ExpectedRecordSha256 -cnotmatch '^[a-f0-9]{64}$' -or (Get-Digest $record) -cne $ExpectedRecordSha256) { throw 'Exact prepared-record hash required.' }
$directory=[IO.Path]::GetDirectoryName($record)
if (-not $directory.StartsWith((Join-Path $context.workspace 'runs')+'\',[StringComparison]::OrdinalIgnoreCase)) { throw 'Recovery record must be in this workspace run directory.' }
$prepared=Read-Record $record
if ($prepared.schema -ne 1 -or $prepared.owner -cne 'OpenNavX.ExactZeroProfileRecovery.1' -or $prepared.status -cne 'prepared') { throw 'Unrecognized prepared recovery record.' }
$intentPath=Join-Path $directory 'intent.json'
if ((Get-Digest $intentPath) -cne $prepared.intentSha256) { throw 'Recovery intent changed.' }
$intent=Read-Record $intentPath
if ($intent.owner -cne $prepared.owner -or $intent.context.sid -cne $context.sid -or $intent.context.profile -ine $context.profile -or $intent.original -ine $original -or
    $intent.context.application -ine $context.application -or $intent.context.managed -ine $context.managed -or $intent.context.workspace -ine $context.workspace -or
    $intent.profileBefore.root -ine $context.profile -or $intent.profileBefore.exists -isnot [bool] -or -not $intent.profileBefore.exists -or
    $intent.originalSha256 -cne $originalHash -or $intent.candidateSha256 -cne $candidateHash -or $intent.length -ne 21380) { throw 'Recovery intent does not match this exact authorization/context.' }
$saved=Join-Path $directory 'candidate.ini';$corrupt=Join-Path $directory 'original-corrupt.ini'
if ((Get-Digest $saved) -cne $candidateHash -or (Get-Digest $corrupt) -cne $originalHash) { throw 'Recovery backups changed.' }
if ($Action -ceq 'Apply') {
  Assert-AuthorizedRecovery $original $intent.candidate
  Assert-PreparationTree $intent.profileBefore
  Assert-PreparationAcl $intent.originalAcl (Get-Acl -LiteralPath $original).Sddl
  $apply=Join-Path $directory 'applying.json'
  Write-Record $apply @{owner=$prepared.owner;status='applying';preparedSha256=$ExpectedRecordSha256;createdUtc=[DateTime]::UtcNow.ToString('o');originalSha256=$originalHash;replacementSha256=$candidateHash;rollbackToCorruption=$false;applicationLaunched=$false}
  Assert-PreparationClosed @($context.application,$context.managed)
  Publish-PreparedProfile $original $saved $originalHash $candidateHash 21380 $apply
}
if ((Get-Digest $original) -cne $candidateHash -or (Get-Digest $intent.candidate) -cne $candidateHash) { throw 'Exact recovered-profile/TMP postcondition failed.' }
$recoveredAcl=(Get-Acl -LiteralPath $original).Sddl
Assert-PreparationAcl $intent.originalAcl $recoveredAcl -AllowDaclAutoInherited
$expected=$intent.profileBefore
$entry=@($expected.entries | Where-Object { $_.path -ceq 'opencpn.ini' })
if ($entry.Count -ne 1) { throw 'Prepared profile inventory is ambiguous.' }
$entry[0].sha256=$candidateHash
Assert-PreparationTree $expected
$null=Read-ProfileForAudit $original
Assert-PreparationClosed @($context.application,$context.managed)
$verified=Join-Path $directory ('verified-'+[guid]::NewGuid().ToString('N')+'.json')
Write-Record $verified @{owner=$prepared.owner;status='recovered';preparedSha256=$ExpectedRecordSha256;createdUtc=[DateTime]::UtcNow.ToString('o');profileSha256=$candidateHash;workingUndoBaseline=$candidateHash;connectionsChanged=$false;navigationDataChanged=$false;applicationLaunched=$false;launchAuditStillRequired=$true;corruptOriginalPreserved=$true;originalAcl=$intent.originalAcl;recoveredAcl=$recoveredAcl;aclComparison='Exact serialized security descriptor except DACL AutoInherited control flag (0x0400)';aclWrittenByRecovery=$false}
[pscustomobject]@{status='recovered';verification=$verified;verificationSha256=(Get-Digest $verified);profileSha256=$candidateHash;applicationLaunched=$false;launchAuditStillRequired=$true} | ConvertTo-Json

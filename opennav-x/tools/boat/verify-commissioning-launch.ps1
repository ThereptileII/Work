# Verification only, immediately before a separately authorized normal launch.
# No application, helper, transport, profile or transaction mutation occurs here.
[CmdletBinding(DefaultParameterSetName='Installed')]
param(
  [Parameter(Mandatory=$true)][string]$Workspace,
  [Parameter(Mandatory=$true)]$Audit,
  [Parameter(Mandatory=$true,ParameterSetName='Installed')]$Installed,
  [Parameter(Mandatory=$true,ParameterSetName='Stock')][switch]$Stock
)
. (Join-Path $PSScriptRoot 'Commissioning.ps1')

function Read-LaunchRecord([string]$Path,[string]$Hash,[string]$Owner) {
  if ($Hash -cnotmatch '^[a-f0-9]{64}$' -or (Get-Digest $Path) -cne $Hash) { throw 'Commissioning launch record missing or changed.' }
  $record=Read-Record $Path
  if ($record.schema -ne 1 -or $record.owner -cne $Owner) { throw 'Unknown commissioning launch record.' }
  return $record
}
function Assert-LaunchFreshReview([string]$Time) {
  $at=[DateTime]::Parse($Time).ToUniversalTime()
  if ($at -gt [DateTime]::UtcNow -or ([DateTime]::UtcNow-$at).TotalHours -gt 24) { throw 'Commissioning source review expired; no launch.' }
}
$context=Get-CommissioningContext $Workspace
if ($PSCmdlet.ParameterSetName -ceq 'Stock') {
  if (-not $Stock) { throw 'Stock verification must be explicitly requested.' }
  Assert-StockAuditIdentity $context.installation (Get-Digest $context.executable) $Audit
  $launchExecutable=$context.executable
} else {
  if (-not $context.installation -or $context.installation.executable -ine $Installed.executable -or
      $context.installation.commit -cne $Installed.ownership.commit) { throw 'Commissioning does not cover this installed executable.' }
  $launchExecutable=$Installed.executable
}
$binding=$Audit.commissioning
$recordPath=Assert-LocalPath $binding.record
$separator=[IO.Path]::DirectorySeparatorChar
$runs=Join-Path $context.workspace 'runs'
if (-not $recordPath.StartsWith($runs+$separator,[StringComparison]::OrdinalIgnoreCase) -or
    [IO.Path]::GetFileName($recordPath) -cne 'prepared.json') { throw 'Launch must bind the private prepared commissioning record.' }
$directory=[IO.Path]::GetDirectoryName($recordPath)
$prepared=Read-LaunchRecord $recordPath $binding.recordSha256 $script:CommissioningOwner
$baselineInfo=Get-PreparedCommissioningBaseline $prepared $directory $context.workspace
Assert-CommissioningContext $prepared.context $context
$active=Read-Record (Join-Path $context.workspace 'commissioning-active.json')
if ($active.schema -ne 1 -or $active.owner -cne $script:CommissioningOwner -or
    $active.record -ine $recordPath -or $active.recordSha256 -cne $binding.recordSha256) { throw 'Commissioning is not the active owned transaction.' }
if ((Test-Path -LiteralPath (Join-Path $directory 'restored.json')) -or
    @(Get-ChildItem -LiteralPath $directory -Filter 'restored-*.json' -Force).Count) { throw 'Commissioning was restored; no launch.' }
if (@(Get-ChildItem -LiteralPath $directory -Filter 'restore-intent-*.json' -Force).Count) { throw 'Commissioning restoration has started; no launch.' }
$applied=Read-LaunchRecord (Join-Path $directory 'applied.json') $binding.appliedSha256 $script:CommissioningOwner
if ($applied.status -cne 'input-only-prepared' -or $applied.recordSha256 -cne $binding.recordSha256 -or
    $applied.profileSha256 -cne $prepared.inputSha256) { throw 'Complete applied commissioning evidence required; partial Apply cannot launch.' }
$plan=Read-LaunchRecord (Join-Path $directory 'review-plan.json') $prepared.planSha256 'OpenNavX.ReadOnlyCommissioning.Plan.1'
Assert-LaunchFreshReview $plan.reviewedUtc
$inventory=Read-LaunchRecord (Join-Path $directory 'inventory.json') $prepared.inventorySha256 'OpenNavX.ReadOnlyCommissioning.Inventory.1'
if ($plan.inventorySha256 -cne $prepared.inventorySha256 -or $inventory.profileSha256 -cne $baselineInfo.sha256) { throw 'Source plan and cold inventory disagree.' }
Assert-CommissioningInventory $inventory $context
$original=Join-Path $directory 'baseline.ini'
$inputProfile=Join-Path $directory 'input-only.ini'
if ((Get-Digest $original) -cne $baselineInfo.sha256 -or (Get-Digest $inputProfile) -cne $prepared.inputSha256 -or
    (Get-CommissioningHash (Get-CommissioningInputBytes ([IO.File]::ReadAllBytes($original)))) -cne $prepared.inputSha256) { throw 'Prepared profile byte proof changed.' }
# Read the parsed array property directly. Windows PowerShell 5 returns a JSON
# array as one pipeline object, whereas PowerShell 7 enumerates its items; a
# serialization/pipeline round trip would wrap all decisions in one extra array.
# This plan object is local to verification; replacing evidencePath below only
# selects the immutable prepared copy and never writes the plan file.
$decisions=@($plan.plugins)
if (@($prepared.evidence).Count -ne $decisions.Count) { throw 'Incomplete saved source-review chain.' }
for ($i=0;$i -lt $decisions.Count;$i++) {
  $saved=Join-Path $directory ('review-'+($i+1)+'.txt')
  $evidence=$prepared.evidence[$i]
  if ($evidence.path -ine $saved -or $evidence.sha256 -cne $decisions[$i].evidenceSha256 -or
      (Get-Digest $saved) -cne $evidence.sha256) { throw 'Copied source-review evidence changed or was substituted.' }
  # The immutable prepared copy is authoritative; original workstation paths
  # need not remain present after preparation.
  $decisions[$i].evidencePath=$saved
}
$candidates=@(Get-CommissioningCandidates $inventory.trees)
Assert-CommissioningReview $candidates $decisions
$planned=@($decisions | Where-Object {$_.decision -ceq 'quarantine'})
if (@($prepared.quarantine).Count -ne $planned.Count) { throw 'Applied quarantine differs from source review.' }
$seen=@{}
$quarantineDirectory=Join-Path $directory 'quarantine'
if ((Assert-LocalPath $context.launchEnvironment.workingDirectory) -ine [IO.Path]::GetDirectoryName($launchExecutable)) { throw 'Commissioning working directory differs from the exact launch executable.' }
Assert-CommissioningQuarantine $quarantineDirectory $context.pluginRoots $context.launchEnvironment.path
foreach ($move in @($prepared.quarantine)) {
  $path=Assert-LocalPath $move.path
  $destination=Assert-LocalPath $move.destination
  $backup=Assert-LocalPath $move.backup
  $match=@($planned | Where-Object {$_.path -ieq $path -and $_.sha256 -ceq $move.sha256})
  if ($match.Count -ne 1 -or $seen.ContainsKey($path) -or [IO.Path]::GetDirectoryName($destination) -ine $quarantineDirectory -or
      [IO.Path]::GetDirectoryName($backup) -ine $directory) { throw 'Unknown or duplicated quarantine ownership.' }
  if (Test-Path -LiteralPath $path) { throw 'A quarantined plugin has returned to a loader root; no launch.' }
  if ((Get-Digest $destination) -cne $move.sha256 -or (Get-Digest $backup) -cne $move.sha256) { throw 'Quarantined plugin or recovery bytes changed.' }
  $seen[$path]=$true
}
# Unlike the per-DLL audit, this also checks every helper executable, runtime
# library, resource and recursive directory against the reviewed cold inventory.
Assert-CommissioningTrees $inventory.trees $prepared.quarantine -AllowMoved
$retained=@($decisions | Where-Object {$_.decision -ceq 'retain'})
$remaining=@(Get-AuditPluginCandidates $context.pluginRoots)
Assert-PluginAudit $remaining $retained
if ($applied.remainingPluginCount -ne $remaining.Count) { throw 'Applied plugin count no longer matches.' }
$ini=Join-Path $context.profile 'opencpn.ini'
if ((Get-Digest $ini) -cne $Audit.profileIniSha256) { throw 'Current launch profile differs from the independent read-only audit.' }
Assert-InputOnlyProfile (Read-ProfileForAudit $ini)
Assert-CommissioningRestoreIni $inputProfile $ini
Assert-LaunchFreshReview $Audit.reviewedUtc
if ($PSCmdlet.ParameterSetName -ceq 'Installed' -and $Audit.buildCommit -cne $Installed.ownership.commit) { throw 'Launch audit belongs to another build.' }
Assert-CommissioningContext $context (Get-CommissioningContext $Workspace)
if ((Get-Digest $ini) -cne $Audit.profileIniSha256) { throw 'Profile changed while verifying commissioning.' }
# Only the exact verified child environment is returned. Caller must assign it
# on ProcessStartInfo without changing the user's or system's environment.
return $context.launchEnvironment

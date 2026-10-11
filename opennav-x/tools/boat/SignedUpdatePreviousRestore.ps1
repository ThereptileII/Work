# Concrete offline PreviousRestored operation, deliberately NOT product-wired.
# Executes the existing commissioning engine; never installs, launches, closes or
# terminates a process. Returned evidence/journal events are NOT launch authority.
# Native authentication, continuous namespace/loader custody and signed broker
# integration remain required. The engine uses DOS paths and its own profile
# mutex: pre/post physical-root checks are not continuous alias-retarget custody.
. (Join-Path $PSScriptRoot 'SignedUpdateSessionStore.ps1')
. (Join-Path $PSScriptRoot 'Commissioning.ps1')

function Assert-SignedPreviousRootBinding($Store) {
  $barrier=$Store.installationBarrier
  if(-not $barrier -or -not $barrier.ContainsKey('physicalRoot') -or -not $barrier.physicalRoot){throw 'Held physical installed-root custody is required.'}
  $pins=[SignedUpdateStoreNative]::PinDirectories($barrier.root)
  try {
    if([SignedUpdateStoreNative]::PhysicalDirectory($pins) -cne $barrier.physicalRoot){throw 'Installed DOS spelling no longer selects the held physical root.'}
  } finally {foreach($pin in $pins){$pin.Dispose()}}
}
function Read-SignedPreviousRecord([string]$Path,[string]$Hash,[string]$Owner) {
  Assert-SignedUpdateHex $Hash 64
  $path=Assert-LocalPath $Path
  $stream=[IO.File]::Open($path,[IO.FileMode]::Open,[IO.FileAccess]::Read,[IO.FileShare]::Read)
  try {
    Assert-SignedInstallationHandle $stream
    # Match the existing commissioning record bound; the engine creates these
    # records, and its full lineage/review checks remain authoritative.
    if($stream.Length -lt 1 -or $stream.Length -gt 4194304){throw 'Commissioning evidence exceeds its bound.'}
    $bytes=[byte[]]::new([int]$stream.Length);$offset=0
    while($offset -lt $bytes.Length){$n=$stream.Read($bytes,$offset,$bytes.Length-$offset);if($n -le 0){throw 'Incomplete commissioning evidence.'};$offset+=$n}
    if((Get-SignedStoreDigest $bytes) -cne $Hash){throw 'Exact retained commissioning evidence changed.'}
    $record=([Text.UTF8Encoding]::new($false,$true).GetString($bytes)|ConvertFrom-Json)
    if($record.schema -ne 1 -or $record.owner -cne $Owner){throw 'Commissioning evidence ownership differs.'}
    return $record
  } finally {$stream.Dispose()}
}
function Get-SignedPreviousSelection($Store) {
  $null=Assert-SignedStoreCurrent $Store
  Assert-SignedPreviousRootBinding $Store
  $installed=Read-InstalledIdentity $Store.installationBarrier.root
  $identity=[pscustomobject][ordered]@{generation=$installed.state.current;commit=$installed.ownership.commit;
    packageSha256=$installed.ownership.packageSha256;executableSha256=(Get-Digest $installed.executable)}
  Assert-SignedUpdateIdentity $identity
  if(-not(Test-SignedUpdateIdentity $identity $Store.session.binding.previous)){throw 'The exact previous generation must remain selected.'}
  return [pscustomobject]@{installed=$installed;identity=$identity;
    stateSha256=(Get-Digest (Join-Path $installed.root 'state.json'));
    ownershipSha256=(Get-Digest (Join-Path $installed.generation 'ownership.json'))}
}
function Get-SignedPreviousInputs($Store,[string]$Workspace,[string]$Record,[string]$ExpectedRecordSha256) {
  $null=Assert-SignedStoreCurrent $Store
  if($null -eq $Store.operations -or @($Store.operations.events).Count -ne 0 -or @($Store.session.receipts).Count -ne 0){throw 'Previous restoration requires fresh live operation-event custody.'}
  if($ExpectedRecordSha256 -cne $Store.session.binding.previousCommissioningSha256){throw 'Prepared transaction is not the session previous commissioning.'}
  $context=Get-CommissioningContext $Workspace
  $record=Assert-LocalPath $Record;$directory=[IO.Path]::GetDirectoryName($record)
  if([IO.Path]::GetFileName($record) -cne 'prepared.json' -or
     [IO.Path]::GetDirectoryName($directory) -ine (Join-Path $context.workspace 'runs')){throw 'Exact private previous transaction required.'}
  $prepared=Read-SignedPreviousRecord $record $ExpectedRecordSha256 $script:CommissioningOwner
  $null=Get-PreparedCommissioningBaseline $prepared $directory $context.workspace
  Assert-CommissioningContext $prepared.context $context
  $selection=Get-SignedPreviousSelection $Store
  $i=$context.installation
  if(-not $i -or $context.sid -cne $Store.owner.sid -or $context.session -ne $Store.owner.sessionId -or
     $i.root -ine $selection.installed.root -or $i.generation -ine $selection.installed.generation -or
     $i.commit -cne $selection.identity.commit -or $i.executableSha256 -cne $selection.identity.executableSha256 -or
     $i.stateSha256 -cne $selection.stateSha256 -or $i.ownershipSha256 -cne $selection.ownershipSha256){throw 'Previous commissioning context differs from current owned installation/session.'}
  return [pscustomobject]@{context=$context;record=$record;directory=$directory;prepared=$prepared;selection=$selection}
}
function Assert-SignedPreviousSelectionUnchanged($Store,$Before) {
  $after=Get-SignedPreviousSelection $Store
  if($after.stateSha256 -cne $Before.stateSha256 -or $after.ownershipSha256 -cne $Before.ownershipSha256){throw 'Selected state/ownership changed during previous restoration.'}
}
function Inspect-SignedUpdatePreviousRestore {
  [CmdletBinding()]
  param([Parameter(Mandatory=$true)][string]$StoreHandle,[Parameter(Mandatory=$true)][string]$Workspace,
    [Parameter(Mandatory=$true)][string]$Record,[Parameter(Mandatory=$true)][string]$ExpectedRecordSha256)
  $store=Get-SignedStoreLive $StoreHandle
  $null=Assert-SignedStoreCurrent $store;Assert-SignedPreviousRootBinding $store
  $lock=Open-SignedInstallationLock $store.installationBarrier.physicalRoot
  try {
    $selectedInput=Get-SignedPreviousInputs $store $Workspace $Record $ExpectedRecordSha256
    # Engine acquires/relinquishes its own profile lock; do not take it recursively.
    $result=(& (Join-Path $PSScriptRoot 'commission-read-only.ps1') -Action InspectRestore -Workspace $Workspace -Record $Record -ExpectedRecordSha256 $ExpectedRecordSha256)|ConvertFrom-Json
    if($result.status -cne 'review-required' -or $result.applicationLaunched -isnot [bool] -or $result.applicationLaunched){throw 'Real closed-session inspection did not complete.'}
    $inspectionRecord=Read-SignedPreviousRecord $result.inspection $result.inspectionSha256 'OpenNavX.ReadOnlyCommissioning.RestoreInspection.1'
    if($inspectionRecord.recordSha256 -cne $ExpectedRecordSha256 -or
       (Get-Digest $inspectionRecord.savedIni) -cne $inspectionRecord.currentIniSha256){throw 'Retained inspection is not the exact previous transaction.'}
    Assert-SignedPreviousSelectionUnchanged $store $selectedInput.selection
    return [pscustomobject]@{status='review-required';inspection=$result.inspection;inspectionSha256=$result.inspectionSha256;
      currentIniSha256=$inspectionRecord.currentIniSha256;applicationLaunched=$false;launchAuthority=$false;
      scope='Actual engine inspection only; independently review current settings before Restore.'}
  } finally {$lock.Dispose()}
}
function Invoke-SignedUpdatePreviousRestore {
  [CmdletBinding()]
  param([Parameter(Mandatory=$true)][string]$StoreHandle,[Parameter(Mandatory=$true)][string]$Workspace,
    [Parameter(Mandatory=$true)][string]$Record,[Parameter(Mandatory=$true)][string]$ExpectedRecordSha256,
    [Parameter(Mandatory=$true)][string]$Inspection,[Parameter(Mandatory=$true)][string]$ExpectedInspectionSha256,
    [Parameter(Mandatory=$true)][string]$ReviewedCurrentIniSha256,
    [string]$PreservationProposal='',[string]$ExpectedPreservationSha256='',
    [Parameter(Mandatory=$true)][string]$RequestNonce)
  Assert-SignedUpdateHex $RequestNonce 64;Assert-SignedUpdateHex $ReviewedCurrentIniSha256 64
  $store=Get-SignedStoreLive $StoreHandle
  $null=Assert-SignedStoreCurrent $store;Assert-SignedPreviousRootBinding $store
  $lock=Open-SignedInstallationLock $store.installationBarrier.physicalRoot
  $profileLock=$null;$operationStarted=$false;$primaryFailure=$null;$completed=$null
  try {
    $selectedInput=Get-SignedPreviousInputs $store $Workspace $Record $ExpectedRecordSha256
    $inspectionRecord=Read-SignedPreviousRecord $Inspection $ExpectedInspectionSha256 'OpenNavX.ReadOnlyCommissioning.RestoreInspection.1'
    if([IO.Path]::GetDirectoryName((Assert-LocalPath $Inspection)) -ine $selectedInput.directory -or
       $inspectionRecord.recordSha256 -cne $ExpectedRecordSha256 -or $inspectionRecord.currentIniSha256 -cne $ReviewedCurrentIniSha256 -or
       (Get-Digest $inspectionRecord.savedIni) -cne $ReviewedCurrentIniSha256){throw 'Exact operator-reviewed previous inspection required.'}
    Assert-CommissioningContext $inspectionRecord.context $selectedInput.context
    $ini=Join-Path $selectedInput.context.profile 'opencpn.ini'
    $inventoryPath=Join-Path $selectedInput.directory 'inventory.json'
    $inventory=Read-SignedPreviousRecord $inventoryPath $selectedInput.prepared.inventorySha256 'OpenNavX.ReadOnlyCommissioning.Inventory.1'
    Assert-CommissioningInventory $inventory $selectedInput.context
    $proof=$null;$restoreHash=$selectedInput.prepared.baselineSha256
    $restoreBytes=(Get-Item -LiteralPath (Join-Path $selectedInput.directory 'baseline.ini')).Length
    if($PreservationProposal -or $ExpectedPreservationSha256) {
      if(-not $PreservationProposal -or -not $ExpectedPreservationSha256){throw 'Exact preservation proposal and hash are both required.'}
      $proof=Read-SessionPreservationProposal $Workspace $PreservationProposal $ExpectedPreservationSha256 $Record $ExpectedRecordSha256
      if($proof.value.inspection -ine $Inspection -or $proof.value.inspectionSha256 -cne $ExpectedInspectionSha256){throw 'Preservation review belongs to another inspection.'}
      Assert-SessionPreservationLiveState $proof $selectedInput.context
      $restoreHash=$proof.value.baselineSha256;$restoreBytes=$proof.value.baselineBytes
    } elseif($ReviewedCurrentIniSha256 -cnotin @($selectedInput.prepared.inputSha256,$selectedInput.prepared.baselineSha256)) {
      throw 'Changed user settings require the exact independent preservation proposal; no automatic erasure.'
    }
    $acls=Get-SessionPreservationAcls (@($inspectionRecord.profileBeforeRestore)+@($inventory.trees)) $selectedInput.prepared.quarantine
    $expected=Copy-SignedUpdateValue $inspectionRecord.profileBeforeRestore
    $entry=@($expected.entries|Where-Object {$_.path -ceq 'opencpn.ini'})
    if($entry.Count -ne 1){throw 'Ambiguous restored profile snapshot.'}
    $entry[0].sha256=$restoreHash;$entry[0].bytes=$restoreBytes
    $arguments=@{Action='Restore';Workspace=$Workspace;Record=$Record;ExpectedRecordSha256=$ExpectedRecordSha256;
      Inspection=$Inspection;ExpectedInspectionSha256=$ExpectedInspectionSha256;ReviewedCurrentIniSha256=$ReviewedCurrentIniSha256}
    if($proof){$arguments.PreservationProposal=$PreservationProposal;$arguments.ExpectedPreservationSha256=$ExpectedPreservationSha256}
    Assert-SignedPreviousSelectionUnchanged $store $selectedInput.selection
    $operationStarted=$true
    $result=(& (Join-Path $PSScriptRoot 'commission-read-only.ps1') @arguments)|ConvertFrom-Json
    if($result.status -cne 'restored' -or $result.applicationLaunched -isnot [bool] -or $result.applicationLaunched -or
       $result.doNotAutoLaunch -isnot [bool] -or -not $result.doNotAutoLaunch){throw 'Real previous Restore did not complete.'}
    # Reacquire only AFTER the engine releases its lock. Any intervening mutation
    # must satisfy the complete postconditions below; no continuous custody claim.
    $profileLock=Open-CommissioningProfileLock $selectedInput.context.profile
    $completion=Read-SignedPreviousRecord $result.verification $result.verificationSha256 $script:CommissioningOwner
    if([IO.Path]::GetDirectoryName((Assert-LocalPath $result.verification)) -ine $selectedInput.directory -or
       $completion.status -cne 'restored' -or $completion.recordSha256 -cne $ExpectedRecordSha256 -or
       $completion.inspectionSha256 -cne $ExpectedInspectionSha256 -or $completion.profileSha256 -cne $restoreHash -or
       $completion.pluginInventoryRestored -isnot [bool] -or -not $completion.pluginInventoryRestored -or
       $completion.otherProfileFilesPreserved -isnot [bool] -or -not $completion.otherProfileFilesPreserved -or
       $completion.originalOutputConfigurationRestored -isnot [bool] -or -not $completion.originalOutputConfigurationRestored -or
       $completion.doNotAutoLaunch -isnot [bool] -or -not $completion.doNotAutoLaunch -or
       $completion.applicationLaunched -isnot [bool] -or $completion.applicationLaunched){throw 'Actual restore completion does not match the selected transaction/target.'}
    if($proof) {
      if($completion.preservationProposalSha256 -cne $proof.sha256 -or -not $result.baselineRecord){throw 'Completed preservation lineage missing.'}
      $baseline=Read-CompletedSessionPreservation $Workspace $result.baselineRecord $result.baselineRecordSha256 0
      if($baseline.sha256 -cne $restoreHash){throw 'Completed preservation baseline differs.'}
      Assert-SessionPreservationLiveState $proof $selectedInput.context
    }
    Assert-PreparationTree $expected
    Assert-CommissioningTrees $inventory.trees $selectedInput.prepared.quarantine
    if((Get-Digest $ini) -cne $restoreHash){throw 'Actual restored profile differs.'}
    Assert-PreparationAcl $selectedInput.prepared.originalAcl (Get-PreparationAccessAcl $ini) -AllowDaclAutoInherited
    $actualAcls=Get-SessionPreservationAcls (@($expected)+@($inventory.trees)) $selectedInput.prepared.quarantine
    Assert-SessionPreservationAcls $acls $actualAcls $ini
    # Get-ChildItem must succeed: a failed lookup is not marker absence.
    if(@(Get-ChildItem -LiteralPath $selectedInput.context.workspace -Force -ErrorAction Stop|Where-Object {$_.Name -ieq 'commissioning-active.json'}).Count){throw 'Active commissioning marker remains after restoration.'}
    Assert-CommissioningContext $selectedInput.context (Get-CommissioningContext $Workspace)
    Assert-SignedPreviousSelectionUnchanged $store $selectedInput.selection
    $trees=[pscustomobject][ordered]@{profile=(Get-PreparationTree $selectedInput.context.profile);
      plugins=@(Get-CommissioningTrees $selectedInput.context.pluginRoots);acls=$actualAcls}
    $evidence=[pscustomobject][ordered]@{identity=$selectedInput.selection.identity;stateSha256=$selectedInput.selection.stateSha256;
      ownershipSha256=$selectedInput.selection.ownershipSha256;preparedSha256=$ExpectedRecordSha256;
      restoreCompletionSha256=$result.verificationSha256;profileSha256=(Get-Digest $ini);
      treesSha256=(Get-SignedUpdateReceiptHash $trees);inspectionSha256=$ExpectedInspectionSha256}
    $request=[pscustomobject]@{session=$store.session.binding.session;sequence=1;action='PreviousRestored';
      requestNonce=$RequestNonce;previousEventSha256=$store.operations.headSha256;evidence=$evidence}
    $event=Add-SignedUpdateStoreEvent $StoreHandle (Get-SignedStoreBytes $request)
    $completed=[pscustomobject]@{status='previous-restored';event=$event;inspection=$Inspection;completion=$result.verification;
      preservationBaseline=$result.baselineRecord;applicationLaunched=$false;launchAuthority=$false;
      scope='Actual offline restoration evidence consumed by live store; no installer/spawn authorization or enabled signed broker.'}
  } catch {$primaryFailure=$_;if($operationStarted){$store.poisoned=$true};throw}
  finally {
    $cleanupFailure=$null
    try {if($profileLock){$profileLock.Dispose()}} catch {$cleanupFailure=$_}
    try {$lock.Dispose()} catch {if(-not $cleanupFailure){$cleanupFailure=$_}}
    if($cleanupFailure){
      $store.poisoned=$true
      # Keep any consumed event and denial marker; neither can be rolled back
      # into permission to retry. A primary operation error retains its cause.
      if(-not $primaryFailure){throw $cleanupFailure}
    }
  }
  # Do not emit success until both independent lock releases have completed.
  return $completed
}

# Cold recovery for an exact chart decoder left after an unrecorded parent exit.
# This never authorizes an application launch, profile restore or equipment I/O.
. (Join-Path $PSScriptRoot 'ChartHelperShutdown.ps1')
function Assert-OrphanChartHelper($Helper,$Context,[int]$ExpectedPid,[int]$ExpectedParent,[long]$ExpectedTicks,[string]$Hash,[datetime]$PreparedAt,[datetime]$Now) {
  $path=$Context.managed.TrimEnd('\')+'\oexserverd.exe'
  $started=[datetime]::Parse($Helper.startedUtc).ToUniversalTime()
  if($ExpectedPid -le 0 -or $ExpectedParent -le 0 -or $ExpectedPid -eq $ExpectedParent -or
      $Helper.pid -ne $ExpectedPid -or $Helper.parentPid -ne $ExpectedParent -or
      $Helper.sessionId -ne $Context.session -or $Helper.sid -cne $Context.sid -or
      $Context.session -le 0 -or $Context.sid -cnotmatch '^S-1-5-[0-9-]+$' -or
      $started.Ticks -ne $ExpectedTicks -or $started -lt $PreparedAt -or $started -gt $Now -or
      $Helper.path -ine $path -or $Hash -cne $script:ChartHelperHash) {
    throw 'Exact orphan PID, absent parent, creation, user/session, managed path and known chart-decoder bytes required.'
  }
  $pipe='OCPN'+($ExpectedParent%10000).ToString('D4',[Globalization.CultureInfo]::InvariantCulture)
  if($Helper.commandLine -cne ('"'+$path+'" -p '+$pipe)){throw 'Unknown chart-helper arguments refused.'}
}
function Read-OrphanChartHelper([string]$Workspace,[string]$Record,[string]$RecordHash,[int]$HelperPid,[int]$ParentPid,[long]$StartedTicks) {
  $record=Assert-LocalPath $Record;$workspace=Assert-LocalPath $Workspace
  $cold=[IO.Path]::GetDirectoryName($record)
  if([IO.Path]::GetFileName($record) -cne 'prepared.json' -or
      [IO.Path]::GetDirectoryName($cold) -ine (Join-Path $workspace 'runs') -or
      [IO.Path]::GetFileName($cold) -cnotmatch '^\d{8}-\d{6}-read-only-commissioning-[a-f0-9]{8}$' -or
      $RecordHash -cnotmatch '^[a-f0-9]{64}$' -or (Get-Digest $record) -cne $RecordHash){throw 'Exact owned cold transaction required.'}
  $prepared=Read-Record $record
  $active=Read-Record (Join-Path $workspace 'commissioning-active.json')
  if($prepared.owner -cne $script:CommissioningOwner -or $prepared.status -cne 'prepared' -or
      $active.owner -cne $script:CommissioningOwner -or $active.record -ine $record -or
      $active.recordSha256 -cne $RecordHash -or @(Get-ChildItem $cold -Filter 'restore*.json').Count){throw 'Transaction is inactive, restored or changed.'}
  $context=Get-CommissioningIdentityContext $workspace
  Assert-CommissioningContext $prepared.context $context
  $null=Get-PreparedCommissioningBaseline $prepared $cold $workspace
  $inventoryPath=Join-Path $cold 'inventory.json'
  if((Get-Digest $inventoryPath) -cne $prepared.inventorySha256){throw 'Cold plugin inventory changed.'}
  $inventory=Read-Record $inventoryPath
  Assert-CommissioningInventory $inventory $context
  Assert-CommissioningTrees $inventory.trees $prepared.quarantine -AllowMoved
  $path=Assert-LocalPath (Join-Path $context.managed 'oexserverd.exe')
  $matches=@(foreach($tree in $inventory.trees){foreach($entry in $tree.entries){
    if(-not $entry.directory -and (Join-Path $tree.root $entry.path) -ieq $path){$entry}
  }})
  if($matches.Count -ne 1 -or $matches[0].sha256 -cne $script:ChartHelperHash){throw 'Helper is not uniquely pinned in the cold inventory.'}
  $all=@(Get-CimInstance Win32_Process)
  if(@($all|Where-Object {$_.ProcessId -eq $ParentPid}).Count){throw 'Parent PID still exists; no orphan recovery, including a reused PID.'}
  $helper=@($all|Where-Object {$_.ProcessId -eq $HelperPid})
  if($helper.Count -ne 1){throw 'Exact orphan helper no longer exists.'}
  Assert-PreparationProcesses @($all|Where-Object {$_.ProcessId -ne $HelperPid}) (@($context.application,$context.managed)+@($context.pluginRoots))
  $owner=Invoke-CimMethod -InputObject $helper[0] -MethodName GetOwnerSid
  if($owner.ReturnValue -ne 0){throw 'Cannot verify orphan helper user.'}
  $process=Get-Process -Id $HelperPid -ErrorAction Stop
  try {
    $value=[pscustomobject]@{pid=$process.Id;parentPid=$helper[0].ParentProcessId;sessionId=$process.SessionId;
      sid=$owner.Sid;startedUtc=$process.StartTime.ToUniversalTime().ToString('o');path=(Assert-LocalPath $process.Path);commandLine=$helper[0].CommandLine}
    if($process.HasExited -or $helper[0].ExecutablePath -ine $value.path){throw 'Orphan helper changed during inventory.'}
    Assert-OrphanChartHelper $value $context $HelperPid $ParentPid $StartedTicks (Get-Digest $path) ([datetime]::Parse($prepared.createdUtc).ToUniversalTime()) ([datetime]::UtcNow)
  } finally {$process.Dispose()}
  return [pscustomobject]@{context=$context;helper=$value;cold=$cold}
}

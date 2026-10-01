# One normal local chart-decoder shutdown after its exact stock parent exited.
# No installed-generation bypass, automatic launch, retry or forced termination.
. (Join-Path $PSScriptRoot 'StockReview.ps1')
$script:ChartHelperHash='ec27c947fc9ba4ae09345961780b885deddc2e4fcf094805a14cdf19504e0adb'
function Ensure-ChartHelperGlobalLedger([string]$Ledger,[string]$Sid) {
  $ledger=Assert-LocalPath $Ledger
  if($Sid -cnotmatch '^S-1-5-[0-9-]+$'){throw 'Exact current account SID required for chart-helper ledger.'}
  $allowed=@($Sid,'S-1-5-18','S-1-5-32-544')
  if(-not (Test-Path -LiteralPath $ledger)){
    $null=New-Item -ItemType Directory -Path $ledger
    $acl=New-Object Security.AccessControl.DirectorySecurity
    $acl.SetAccessRuleProtection($true,$false)
    foreach($value in $allowed){
      $identity=New-Object Security.Principal.SecurityIdentifier($value)
      $acl.AddAccessRule((New-Object Security.AccessControl.FileSystemAccessRule($identity,'FullControl','ContainerInherit, ObjectInherit','None','Allow')))
    }
    Set-Acl -LiteralPath $ledger -AclObject $acl
  }
  $item=Get-Item -LiteralPath $ledger -Force
  if(-not $item.PSIsContainer -or ($item.Attributes -band [IO.FileAttributes]::ReparsePoint)){throw 'Chart-helper ledger must be a local directory.'}
  $acl=Get-Acl -LiteralPath $ledger
  if(-not $acl.AreAccessRulesProtected -or $acl.GetOwner([Security.Principal.SecurityIdentifier]).Value -cnotin $allowed){throw 'Chart-helper ledger ACL/owner changed.'}
  foreach($rule in $acl.GetAccessRules($true,$true,[Security.Principal.SecurityIdentifier])){
    if($rule.IdentityReference.Value -cnotin $allowed -or $rule.AccessControlType -ne [Security.AccessControl.AccessControlType]::Allow){throw 'Chart-helper ledger has unreviewed access.'}
  }
}
function New-ChartHelperGlobalLocator([string]$Workspace,[string]$Sid,[int]$HelperPid,[long]$StartedTicks,[string]$Intent,[string]$Owner) {
  # A fresh cold capture or another active transaction cannot authorize a
  # second attempt after delivery became uncertain. Old transaction locators
  # are checked by exact basename without reading unrelated private evidence.
  $workspace=Assert-LocalPath $Workspace
  if($HelperPid -le 0 -or $StartedTicks -le 0 -or $Owner -cnotin @('OpenNavX.ChartHelperShutdown.1','OpenNavX.OrphanChartHelperShutdown.1','OpenNavX.ColdChartHelper.1')){
    throw 'Invalid chart-helper identity or locator owner.'
  }
  $intent=Assert-LocalPath $Intent
  if(-not $intent.StartsWith((Join-Path $workspace 'runs')+'\',[StringComparison]::OrdinalIgnoreCase)) {throw 'Intent must be private workspace evidence.'}
  $name='chart-helper-shutdown-'+$HelperPid+'-'+$StartedTicks+'.json'
  $runs=Assert-LocalPath (Join-Path $workspace 'runs')
  $runItems=@(Get-ChildItem -LiteralPath $runs -Directory -Force -ErrorAction Stop)
  if($runItems.Count -gt 10000){throw 'Run history exceeds chart-helper attempt scan bound.'}
  foreach($run in $runItems){
    if($run.Attributes -band [IO.FileAttributes]::ReparsePoint){throw 'Redirected run evidence refused.'}
    if(Test-Path -LiteralPath (Join-Path $run.FullName $name)){throw 'Previous chart-helper shutdown attempt exists.'}
  }
  $ledger=Assert-LocalPath (Join-Path $workspace 'chart-helper-attempts')
  Ensure-ChartHelperGlobalLedger $ledger $Sid
  $locator=Join-Path $ledger $name
  Write-Record $locator @{owner=$Owner;intent=$intent;intentSha256=(Get-Digest $intent)}
  return $locator
}
function Assert-ChartHelperIdentity($Helper,$Launch,[int]$ExpectedPid,[long]$ExpectedStartedTicks,[string]$ExpectedPath,[string]$ActualHash) {
  $started=[datetime]::Parse($Helper.startedUtc).ToUniversalTime()
  if($ExpectedPid -le 0 -or $Helper.pid -ne $ExpectedPid -or $Helper.parentPid -ne $Launch.pid -or
      $Helper.sessionId -ne $Launch.sessionId -or $Helper.sid -cne $Launch.sid -or
      $started.Ticks -ne $ExpectedStartedTicks -or $started -lt [datetime]::Parse($Launch.processStartedUtc).ToUniversalTime() -or
      $started -gt [datetime]::UtcNow -or $Helper.path -ine $ExpectedPath -or $ActualHash -cne $script:ChartHelperHash) {
    throw 'Exact remaining stock chart-helper parent, PID, creation, SID, session, path and hash required.'
  }
  $pipe='OCPN'+([int]$Launch.pid%10000).ToString('D4',[Globalization.CultureInfo]::InvariantCulture)
  if($Helper.commandLine -cne ('"'+$ExpectedPath+'" -p '+$pipe)) {throw 'Only the exact source-generated chart-helper pipe arguments are supported.'}
}
function Read-ChartHelperContext([string]$Workspace,[string]$LaunchResult,[string]$LaunchSha256,[string]$RequestSha256,[int]$HelperPid,[long]$StartedTicks) {
  $config=Get-StockTarget $Workspace;$path=Assert-LocalPath $LaunchResult;$launch=Read-Record $path
  # Reuse only the read-only proof reader; no Capture action is dispatched.
  $proof=[pscustomobject]@{action='ReviewStock';reviewAction='Capture';workspace=(Assert-LocalPath $Workspace);
    executable=$config.stockExecutable;executableSha256=(Get-Digest $config.stockExecutable);processId=$launch.pid;
    launchResult=$path;launchResultSha256=$LaunchSha256;launchRequestSha256=$RequestSha256;
    reviewHelperSha256=(Get-Digest (Join-Path $PSScriptRoot 'StockReview.ps1'));
    nativeHelperSha256=(Get-Digest (Join-Path $PSScriptRoot 'StockReviewNative.cs'))}
  $review=Read-StockReview $proof
  if([Security.Principal.WindowsIdentity]::GetCurrent().User.Value -cne $review.launch.sid){throw 'Only the reviewed stock user may stop its chart helper.'}
  $prepared=Read-Record $config.stockReadOnlyAudit.commissioning.record
  $helperPath=Assert-LocalPath (Join-Path $prepared.context.managed 'oexserverd.exe')
  $cold=[IO.Path]::GetDirectoryName($config.stockReadOnlyAudit.commissioning.record)
  $inventory=Read-Record (Join-Path $cold 'inventory.json')
  $matchesHelper=@(foreach($tree in $inventory.trees){foreach($entry in $tree.entries){
    if(-not $entry.directory -and (Join-Path $tree.root $entry.path) -ieq $helperPath){$entry}
  }})
  if($matchesHelper.Count -ne 1 -or $matchesHelper[0].sha256 -cne $script:ChartHelperHash){throw 'Exact helper is not uniquely pinned by the active cold inventory.'}
  $all=@(Get-CimInstance Win32_Process)
  if(@($all|Where-Object {$_.Name -ieq 'opencpn.exe' -or $_.ProcessId -eq $review.launch.pid}).Count){throw 'The reviewed stock parent and all OpenCPN processes must be gone before helper cleanup.'}
  $helper=@($all|Where-Object {$_.ProcessId -eq $HelperPid})
  if($helper.Count -ne 1){throw 'One exact remaining helper required.'}
  # Every other application/managed-tree process still blocks, including another
  # chart helper. The exception is this exact explicitly bound process only.
  Assert-PreparationProcesses @($all|Where-Object {$_.ProcessId -ne $HelperPid}) @($prepared.context.pluginRoots+[IO.Path]::GetDirectoryName($config.stockExecutable))
  $owner=Invoke-CimMethod -InputObject $helper[0] -MethodName GetOwnerSid
  if($owner.ReturnValue -ne 0){throw 'Cannot establish remaining helper owner.'}
  $process=Get-Process -Id $HelperPid -ErrorAction Stop
  try {
    $value=[pscustomobject]@{pid=$process.Id;parentPid=$helper[0].ParentProcessId;sessionId=$process.SessionId;
      sid=$owner.Sid;startedUtc=$process.StartTime.ToUniversalTime().ToString('o');path=(Assert-LocalPath $process.Path);commandLine=$helper[0].CommandLine}
    if($process.HasExited -or $helper[0].ExecutablePath -ine $value.path){throw 'Helper changed during process inventory.'}
    Assert-ChartHelperIdentity $value $review.launch $HelperPid $StartedTicks $helperPath (Get-Digest $helperPath)
  } finally {$process.Dispose()}
  return [pscustomobject]@{helper=$value;launch=$review.launch;prepared=$prepared;helperPath=$helperPath;cold=$cold}
}

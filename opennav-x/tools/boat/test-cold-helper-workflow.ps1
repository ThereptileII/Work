# Disposable Capture/Close orchestration. A temporary source copy substitutes
# only OS/process/ACL observations and the native dispatch; production has no
# test switch and no alternate chart-helper transport.
[CmdletBinding()]
param([switch]$PortableContracts)
. (Join-Path $PSScriptRoot 'ColdChartHelper.ps1')
$checks=New-Object 'Collections.Generic.List[string]'
function Check($Condition,[string]$Name){if(-not $Condition){throw $Name};$checks.Add($Name)}
function Refuse([scriptblock]$Action,[string]$Name){$failed=$false;try{& $Action|Out-Null}catch{$failed=$true};Check $failed $Name}
function Assert-LocalPath([string]$Path){if([string]::IsNullOrWhiteSpace($Path)){throw 'Missing test path'};return [IO.Path]::GetFullPath($Path)}
function Get-CommissioningIdentityContext([string]$Workspace){return $global:fixtureContext}
function New-PreparationDirectory($Context,[string]$Purpose){
  $global:runNumber++
  $path=Join-Path $Context.workspace ('runs/20261001-120000-'+$Purpose+'-'+$global:runNumber.ToString('x8'))
  $null=New-Item -ItemType Directory -Path $path -Force
  return $path
}
function Get-ColdHelperSet($Context){return @($global:liveHelpers)}
function Get-CimInstance {return @($global:liveHelpers|ForEach-Object {[pscustomobject]@{Name='oexserverd.exe';ProcessId=$_.pid;ExecutablePath=$_.path}})}
function Get-Acl {return [pscustomobject]@{Sddl='fixture-acl'}}
function Assert-PreparationAcl([string]$Expected,[string]$Actual){if($Expected -cne $Actual){throw 'Fixture ACL drift.'}}
function Assert-ColdPrivateEvidence([string]$Directory,[string]$Sid){if(-not (Test-Path -LiteralPath $Directory)){throw 'Missing private fixture evidence.'}}
function New-ChartHelperGlobalLocator([string]$Workspace,[string]$Sid,[int]$HelperPid,[long]$StartedTicks,[string]$Intent,[string]$Owner){
  $bound=@($global:liveHelpers|Where-Object {$_.pid -eq $HelperPid})
  if($bound.Count -ne 1 -or $StartedTicks -ne ([datetime]($bound[0].startedUtc)).ToUniversalTime().Ticks){throw 'Fixture locator lost fractional-second creation identity.'}
  $ledger=Join-Path $Workspace 'chart-helper-attempts';$null=New-Item -ItemType Directory -Path $ledger -Force
  $path=Join-Path $ledger ('chart-helper-shutdown-'+$HelperPid+'-'+$StartedTicks+'.json')
  Write-Record $path @{owner=$Owner;intent=$Intent;intentSha256=(Get-Digest $Intent)}
  return $path
}
function Invoke-TestShutdown($Helper,[long]$StartedTicks){
  if($StartedTicks -ne ([datetime]($Helper.startedUtc)).ToUniversalTime().Ticks){throw 'Fixture dispatch lost fractional-second creation identity.'}
  $global:observedTicks.Add($StartedTicks)
  $global:nativeCalls++
  if($global:failAt -eq $global:nativeCalls){return [pscustomobject]@{Succeeded=$false;WriteAttempted=$true;Failure='fixture uncertain delivery'}}
  $global:liveHelpers=@($global:liveHelpers|Where-Object {$_.pid -ne $Helper.pid})
  if($global:nativeCalls -eq 1){
    if($global:driftMode -ceq 'new-helper'){
      $global:liveHelpers+=([pscustomobject]@{pid=3003;parentPid=1003;sessionId=3;sid='S-1-5-21-223';
        startedUtc=[datetime]::UtcNow.ToString('o');path=(Join-Path $global:fixtureContext.managed 'oexserverd.exe');
        sha256=$script:ChartHelperHash;commandLine='fixture-source-shape'})
    } elseif($global:driftMode -ceq 'active-marker'){
      Write-Record (Join-Path $global:fixtureContext.workspace 'commissioning-active.json') @{owner='fixture'}
    } elseif($global:driftMode -ceq 'installed-context'){
      $global:fixtureContext.installation.stateSha256='b'*64
    }
  }
  return [pscustomobject]@{Succeeded=$true;WriteAttempted=$true;ExitObserved=$true;ExitCodeKnown=$true;ExitCode=0}
}
$source=[IO.File]::ReadAllText((Join-Path $PSScriptRoot 'recover-cold-chart-helpers.ps1'))
$source=$source.Replace(". (Join-Path `$PSScriptRoot 'ColdChartHelper.ps1')","`$script:ColdHelperOwner='OpenNavX.ColdChartHelper.1' # fixture uses loaded production policy")
$source=$source.Replace("if([Environment]::OSVersion.Platform -ne 'Win32NT'){throw 'Native Windows is required.'}",'# isolated fixture platform')
$source=$source.Replace("Add-Type -Path (Join-Path `$PSScriptRoot 'ChartHelperShutdownNative.cs')",'# isolated fixture transport')
$source=[regex]::Replace($source,'(?m)^\s*\$result=\[OpenNavX\.ChartHelperShutdownNative\]::Shutdown\([^\r\n]*\)$','    $result=Invoke-TestShutdown $helper $ticks')
if($source -match '\[OpenNavX\.ChartHelperShutdownNative\]::Shutdown'){throw 'Native call replacement failed; fixture refused.'}
$sandbox=Join-Path ([IO.Path]::GetTempPath()) ('cold-helper-workflow-'+[guid]::NewGuid().ToString('N'))
$null=New-Item -ItemType Directory -Path $sandbox
$fixtureScript=Join-Path $sandbox 'fixture-recovery.ps1'
[IO.File]::WriteAllText($fixtureScript,$source)
Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'ChartHelperShutdownNative.cs') -Destination (Join-Path $sandbox 'ChartHelperShutdownNative.cs')
function New-Fixture {
  $global:runNumber=0;$global:nativeCalls=0;$global:failAt=0;$global:driftMode=''
  $global:observedTicks=New-Object 'Collections.Generic.List[long]'
  $root=Join-Path $sandbox ([guid]::NewGuid().ToString('N'))
  $profile=Join-Path $root 'profile';$managed=Join-Path $root 'managed';$installed=Join-Path $root 'installed-plugins'
  foreach($directory in @($profile,$managed,$installed,(Join-Path $root 'runs'))){$null=New-Item -ItemType Directory -Path $directory -Force}
  [IO.File]::WriteAllText((Join-Path $profile 'opencpn.ini'),"[Settings]`nFixture=unchanged`n")
  [IO.File]::WriteAllText((Join-Path $managed 'oexserverd.exe'),'fixture-only')
  [IO.File]::WriteAllText((Join-Path $installed 'fixture_pi.dll'),'fixture-only')
  $global:fixtureContext=[pscustomobject]@{sid='S-1-5-21-223';session=3;workspace=$root;profile=$profile;
    managed=$managed;application=(Join-Path $root 'application');pluginRoots=@($managed,$installed);
    installation=[pscustomobject]@{generation=(Join-Path $root 'owned-generation');stateSha256=('a'*64)}}
  $now=[datetime]::UtcNow
  $global:liveHelpers=@(1..2|ForEach-Object {
    $parent=1000+$_
    [pscustomobject]@{pid=(2000+$_);parentPid=$parent;sessionId=3;sid='S-1-5-21-223';
      startedUtc=$now.AddMinutes(-$_).ToString('o');path=(Join-Path $managed 'oexserverd.exe');
      sha256=$script:ChartHelperHash;commandLine='fixture-source-shape'}
  })
  return $root
}
function Get-Capture([string]$Workspace){return ((& $fixtureScript -Action Capture -Workspace $Workspace)|ConvertFrom-Json)}
function Get-Review($Capture,[string]$Workspace){
  $path=Join-Path $Workspace ('review-'+[guid]::NewGuid().ToString('N')+'.json')
  Write-Record $path @{schema=1;owner='OpenNavX.ColdChartHelperReview.1';captureSha256=$Capture.captureSha256;
    reviewedUtc=[datetime]::UtcNow.ToString('o');decision='approve-exact-cmd-exit-once';reason='Fixture exact candidates';candidates=@($Capture.helpers)}
  return [pscustomobject]@{path=$path;sha256=(Get-Digest $path)}
}
function Invoke-Close($Capture,$Review,[string]$Workspace){
  & $fixtureScript -Action Close -Workspace $Workspace -CaptureRecord $Capture.captureRecord -ExpectedCaptureSha256 $Capture.captureSha256 -Review $Review.path -ExpectedReviewSha256 $Review.sha256
}
try {
  $w=New-Fixture;$capture=Get-Capture $w;$review=Get-Review $capture $w
  Check (@($capture.helpers).Count -eq 2 -and $fixtureContext.installation.generation) 'Captures both helpers with existing installed generation identity'
  $closed=Invoke-Close $capture $review $w|ConvertFrom-Json
  Check ($closed.status -ceq 'closed-cold-capture-required' -and $global:nativeCalls -eq 2 -and @($global:liveHelpers).Count -eq 0) 'Closes two exact helpers once each'
  Check ($global:observedTicks.Count -eq 2 -and
    $global:observedTicks[0] -eq ([datetime]($capture.helpers[0].startedUtc)).ToUniversalTime().Ticks -and
    $global:observedTicks[1] -eq ([datetime]($capture.helpers[1].startedUtc)).ToUniversalTime().Ticks) 'Fractional-second UTC creation ticks survive capture, locator and dispatch'
  Check ((Test-Path -LiteralPath (Join-Path ([IO.Path]::GetDirectoryName($capture.captureRecord)) 'closed.json')) -and
    [IO.File]::ReadAllText((Join-Path $w 'profile/opencpn.ini')) -ceq "[Settings]`nFixture=unchanged`n") 'Publishes closed evidence without changing profile bytes'
  Refuse {Invoke-Close $capture $review $w} 'Completed capture cannot replay'
  Check ($global:nativeCalls -eq 2) 'Completed replay sends no command'

  $w=New-Fixture;$capture=Get-Capture $w;$review=Get-Review $capture $w
  [IO.File]::AppendAllText((Join-Path $w 'profile/opencpn.ini'),'drift')
  Refuse {Invoke-Close $capture $review $w} 'Profile byte drift blocks closure'
  Check ($global:nativeCalls -eq 0) 'Profile drift sends no command'

  $w=New-Fixture;$capture=Get-Capture $w;$review=Get-Review $capture $w
  [IO.File]::AppendAllText((Join-Path $w 'installed-plugins/fixture_pi.dll'),'drift')
  Refuse {Invoke-Close $capture $review $w} 'Installed plugin byte drift blocks closure'
  Check ($global:nativeCalls -eq 0) 'Plugin drift sends no command'

  $w=New-Fixture;$capture=Get-Capture $w;$review=Get-Review $capture $w
  [IO.File]::AppendAllText((Join-Path ([IO.Path]::GetDirectoryName($capture.captureRecord)) 'profile-backup/opencpn.ini'),'drift')
  Refuse {Invoke-Close $capture $review $w} 'Private backup byte drift blocks closure'
  Check ($global:nativeCalls -eq 0) 'Backup drift sends no command'

  $w=New-Fixture;$capture=Get-Capture $w;$review=Get-Review $capture $w
  Write-Record (Join-Path $w 'commissioning-active.json') @{owner='fixture'}
  Refuse {Invoke-Close $capture $review $w} 'Active transaction marker blocks closure'
  Check ($global:nativeCalls -eq 0) 'Active marker sends no command'

  $w=New-Fixture;$capture=Get-Capture $w;$review=Get-Review $capture $w;$global:failAt=2
  Refuse {Invoke-Close $capture $review $w} 'Second helper uncertain delivery stops sequence'
  $evidence=[IO.Path]::GetDirectoryName($capture.captureRecord)
  Check ($global:nativeCalls -eq 2 -and @($global:liveHelpers).Count -eq 1 -and
    (Test-Path -LiteralPath (Join-Path $evidence 'result-2002.json')) -and
    (Test-Path -LiteralPath (Join-Path $evidence 'failure-2002.json')) -and
    -not (Test-Path -LiteralPath (Join-Path $evidence 'closed.json'))) 'Partial results and failure persist without false completion'
  $fresh=Get-Capture $w;$freshReview=Get-Review $fresh $w;$global:failAt=0
  Refuse {Invoke-Close $fresh $freshReview $w} 'Fresh capture cannot retry uncertain same process identity'
  Check ($global:nativeCalls -eq 2) 'Uncertain delivery never dispatches a second command'

  foreach($mode in @('new-helper','active-marker','installed-context')){
    $w=New-Fixture;$capture=Get-Capture $w;$review=Get-Review $capture $w;$global:driftMode=$mode
    Refuse {Invoke-Close $capture $review $w} ('Drift after first exit is refused: '+$mode)
    Check ($global:nativeCalls -eq 1 -and -not (Test-Path -LiteralPath (Join-Path ([IO.Path]::GetDirectoryName($capture.captureRecord)) 'closed.json'))) ('No second command or false completion after '+$mode)
  }
  foreach($mode in @('duplicate-helper','wrong-profile-root','missing-installed-plugin-tree')){
    $w=New-Fixture;$capture=Get-Capture $w;$data=Read-Record $capture.captureRecord
    if($mode -ceq 'duplicate-helper'){$data.helpers[1]=$data.helpers[0]}
    elseif($mode -ceq 'wrong-profile-root'){$data.profile.root=Join-Path $w 'other-profile'}
    else {$data.pluginTrees=@($data.pluginTrees|Where-Object {$_.root -ne (Join-Path $w 'installed-plugins')})}
    [IO.File]::WriteAllText($capture.captureRecord,($data|ConvertTo-Json -Depth 16))
    $capture.captureSha256=Get-Digest $capture.captureRecord
    $capture.helpers=@($data.helpers)
    $review=Get-Review $capture $w
    Refuse {Invoke-Close $capture $review $w} ('Caller-rehashed malformed capture refused: '+$mode)
    Check ($global:nativeCalls -eq 0) ('Malformed capture sends no command: '+$mode)
  }
  [pscustomobject]@{status='passed';count=$checks.Count;checks=@($checks);isolatedSourceCopy=$true;
    substituted=@('process observations','ACL observations','native transport','ledger write');
    actualNativeTransport=$false;vendorHelperExecuted=$false;boatAccess=$false}|ConvertTo-Json -Depth 5
} finally {Remove-Item -LiteralPath $sandbox -Recurse -Force -ErrorAction SilentlyContinue}

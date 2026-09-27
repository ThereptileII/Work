# Opt-in commissioning restart broker primitives. Not imported by normal launch.
. (Join-Path $PSScriptRoot 'Common.ps1')
. (Join-Path $PSScriptRoot 'Preparation.ps1')
. (Join-Path $PSScriptRoot 'RestartCommissioningPolicy.ps1')
$script:RestartOwner='OpenNavX.ReadOnlyRestart.Session.1'
$script:RestartDependencies=@('Common.ps1','InteractiveJob.ps1','run-mode.ps1','Preparation.ps1','Commissioning.ps1','CommissioningBaseline.ps1','verify-commissioning-launch.ps1','RestartCommissioningPolicy.ps1','RestartAuiPersistence.ps1','RestartDashboardPersistence.ps1','RestartCommissioning.ps1','RestartCommissioningNative.cs','RestartCommissioningPrepare.ps1','RestartCommissioningBroker.ps1','RestartCommissioningArm.ps1')
function Initialize-RestartNative {
  if(-not ('OpenNavX.RestartCommissioningNative' -as [type])){Add-Type -Path (Join-Path $PSScriptRoot 'RestartCommissioningNative.cs')}
}
function New-RestartToken {
  $bytes=New-Object byte[] 32;$random=[Security.Cryptography.RandomNumberGenerator]::Create()
  try{$random.GetBytes($bytes);return ([BitConverter]::ToString($bytes)).Replace('-','').ToLowerInvariant()}finally{$random.Dispose()}
}
function Get-RestartBytesHash([byte[]]$Bytes) {
  $hash=[Security.Cryptography.SHA256]::Create()
  try{return ([BitConverter]::ToString($hash.ComputeHash($Bytes))).Replace('-','').ToLowerInvariant()}finally{$hash.Dispose()}
}
function Assert-RestartPrivateDirectory([string]$Path,[string]$Sid) {
  $acl=Get-Acl -LiteralPath (Assert-LocalPath $Path)
  if(-not $acl.AreAccessRulesProtected){throw 'Restart evidence directory must have a protected private ACL.'}
  $allowed=@($Sid,'S-1-5-18','S-1-5-32-544')
  if($acl.GetOwner([Security.Principal.SecurityIdentifier]).Value -cnotin $allowed){throw 'Restart evidence ownership changed.'}
  foreach($rule in $acl.GetAccessRules($true,$true,[Security.Principal.SecurityIdentifier])) {
    if($rule.IdentityReference.Value -cnotin $allowed){throw 'Restart evidence has an unreviewed principal.'}
  }
}
function Read-RestartSession([string]$Record,[string]$ExpectedSha256) {
  $record=Assert-LocalPath $Record
  if($ExpectedSha256 -cnotmatch '^[a-f0-9]{64}$' -or [IO.Path]::GetFileName($record) -cne 'session.json' -or (Get-Digest $record) -cne $ExpectedSha256){throw 'Exact immutable cold-session record required.'}
  $session=Read-Record $record
  if($session.schema -ne 1 -or $session.owner -cne $script:RestartOwner -or $session.session -cnotmatch '^[a-f0-9]{64}$'){throw 'Unknown restart session.'}
  $directory=[IO.Path]::GetDirectoryName($record)
  if([IO.Path]::GetDirectoryName($directory) -ine (Join-Path (Assert-LocalPath $session.workspace) 'runs') -or [IO.Path]::GetFileName($directory) -cnotmatch '^\d{8}-\d{6}-restart-session-[a-f0-9]{8}$'){throw 'Restart session escaped owned workspace.'}
  $sid=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value
  if($session.sid -cne $sid){throw 'Restart session belongs to another account.'}
  Assert-RestartPrivateDirectory $directory $sid
  $now=[DateTime]::UtcNow;$created=[DateTime]::Parse($session.createdUtc).ToUniversalTime();$expires=[DateTime]::Parse($session.expiresUtc).ToUniversalTime()
  if($created -gt $now -or $expires -le $now -or $expires -le $created -or ($expires-$created).TotalHours -gt 4){throw 'Cold restart session expired or invalid.'}
  if($session.toolDirectory -ine $PSScriptRoot -or @($session.scripts).Count -ne $script:RestartDependencies.Count){throw 'Cold restart tool inventory differs.'}
  foreach($name in $script:RestartDependencies) {
    $entry=@($session.scripts | Where-Object {$_.name -ceq $name})
    if($entry.Count -ne 1 -or $entry[0].sha256 -cnotmatch '^[a-f0-9]{64}$' -or (Get-Digest (Join-Path $PSScriptRoot $name)) -cne $entry[0].sha256){throw 'Commissioning verifier changed after cold review.'}
  }
  $installed=Get-Installed
  if($installed.state.current -cne $session.generation -or $installed.ownership.commit -cne $session.buildCommit -or $installed.executable -cne $session.executable -or
     (Get-Digest $session.executable) -cne $session.executableSha256 -or (Get-Digest $session.helper) -cne $session.helperSha256){throw 'Installed restart generation changed.'}
  $productBuild=Join-Path $installed.generation 'docs\PRODUCT_BUILD.json'
  if($session.productBuild -cne $productBuild -or (Get-Digest $productBuild) -cne $session.productBuildSha256){throw 'Guard-capable build evidence changed.'}
  Assert-RestartBuild (Read-Record $productBuild) $session.buildCommit $session.executableSha256 $session.helperSha256
  if($session.helper -cne (Join-Path ([IO.Path]::GetDirectoryName($session.executable)) 'opennav-restart.exe') -or $session.workingDirectory -cne [IO.Path]::GetDirectoryName($session.executable)){throw 'Unexpected helper/working directory.'}
  if($session.beforeIni -cne (Join-Path $directory 'before.ini') -or (Get-Digest $session.beforeIni) -cne $session.beforeIniSha256 -or $session.audit.profileIniSha256 -cne $session.beforeIniSha256){throw 'Cold pre-session profile proof changed.'}
  if((Get-Digest (Join-Path $session.workspace 'boat-target.json')) -cne $session.targetSha256){throw 'Independent boat-target audit changed; prepare a new cold session.'}
  $normal=Join-Path ([Environment]::GetFolderPath('CommonApplicationData')) 'opencpn\opencpn.ini'
  if($session.profile -cne $normal){throw 'Only the reviewed real profile is permitted.'}
  $shutdown=Join-Path $directory 'shutdown-review.json'
  if($session.shutdownReview -cne $shutdown -or (Get-Digest $shutdown) -cne $session.shutdownReviewSha256){throw 'Retained plugin shutdown review changed.'}
  $prepared=Read-Record $session.audit.commissioning.record
  $plan=Read-Record (Join-Path ([IO.Path]::GetDirectoryName($session.audit.commissioning.record)) 'review-plan.json')
  Assert-RestartShutdownReview (Read-Record $shutdown) @($plan.plugins | Where-Object {$_.decision -ceq 'retain'})
  $session | Add-Member -NotePropertyName recordSha256 -NotePropertyValue $ExpectedSha256
  return $session
}
function Test-RestartImagePath([string]$Observed,[string]$Expected) {
  return (-not [string]::IsNullOrEmpty($Observed) -and -not [string]::IsNullOrEmpty($Expected) -and
    [StringComparer]::OrdinalIgnoreCase.Equals($Observed,$Expected))
}
function Get-RestartProcess([uint32]$ProcessId,$Session,[string]$ExpectedImage) {
  $process=Get-Process -Id $ProcessId -ErrorAction Stop
  try {
    $native=@(Get-CimInstance Win32_Process -Filter ('ProcessId='+$ProcessId))
    if($native.Count -ne 1){throw 'Exact process disappeared.'}
    $owner=Invoke-CimMethod -InputObject $native[0] -MethodName GetOwnerSid
    # Windows reports System32/system32 and WINDOWS/Windows inconsistently even
    # for the same native PowerShell image. Only path letter case may vary;
    # verify the bytes reached by both spellings as well. Identity/time/session
    # fields retain their existing strict comparisons.
    $imageMatches=Test-RestartImagePath $process.Path $ExpectedImage
    $imageHashMatches=$false
    if($imageMatches -and -not $process.HasExited){$imageHashMatches=(Get-Digest $process.Path) -ceq (Get-Digest $ExpectedImage)}
    if($owner.ReturnValue -ne 0 -or $owner.Sid -cne $Session.sid -or $process.HasExited -or -not $imageMatches -or -not $imageHashMatches -or
       $process.SessionId.ToString() -cne $Session.windowsSessionId){
      # Private commissioning evidence records each exact disagreement. Do not
      # normalize/relax an identity check based only on a platform hypothesis.
      $diagnostic=@{pid=$ProcessId.ToString();ownerResult=$owner.ReturnValue;observedSid=$owner.Sid;expectedSid=$Session.sid;
        hasExited=$process.HasExited;observedImage=$process.Path;expectedImage=$ExpectedImage;imageHashesMatch=$imageHashMatches;
        observedSession=$process.SessionId.ToString();expectedSession=$Session.windowsSessionId}
      throw ('Process owner/image/session differs: '+($diagnostic|ConvertTo-Json -Compress))
    }
    return [pscustomobject]@{pid=$ProcessId.ToString();createdFiletime=$process.StartTime.ToUniversalTime().ToFileTimeUtc().ToString();image=$process.Path;session=$process.SessionId.ToString();commandLine=$native[0].CommandLine}
  } finally {$process.Dispose()}
}
function Assert-RestartReceipt($Receipt,$Request,[string]$RequestSha256,[string]$PermitId) {
  $keys=@('protocol','kind','session','recordSha256','nonce','requestSha256','permitId','status','childPid','childCreatedFiletime','win32Error')
  if($Receipt.Count -ne $keys.Count -or @($keys|Where-Object {-not $Receipt.ContainsKey($_)}).Count -or $Receipt['protocol'] -isnot [int] -or $Receipt['protocol'] -ne 1 -or $Receipt['kind'] -cne 'receipt'){throw 'Invalid child receipt schema.'}
  foreach($key in @('session','recordSha256','nonce')){if($Receipt[$key] -cne $Request[$key]){throw 'Child receipt request identity differs.'}}
  if($Receipt['requestSha256'] -cne $RequestSha256 -or $Receipt['permitId'] -cne $PermitId -or $Receipt['status'] -cnotin @('started','failed')){throw 'Child receipt permit identity differs.'}
  foreach($key in @('childPid','childCreatedFiletime','win32Error')){Assert-RestartDecimal $Receipt[$key] $key $true}
  if($Receipt['status'] -ceq 'started' -and ($Receipt['childPid'] -ceq '0' -or $Receipt['childCreatedFiletime'] -ceq '0' -or $Receipt['win32Error'] -cne '0')){throw 'Invalid started-child receipt.'}
  if($Receipt['status'] -ceq 'failed' -and ($Receipt['childPid'] -cne '0' -or $Receipt['childCreatedFiletime'] -cne '0')){throw 'Failure receipt ambiguously claims a child.'}
}
function Publish-RestartPermit([string]$Directory,$Permit) {
  # A parent cannot obtain another permit after an ALLOW or an uncertain send.
  # Create/flush/rename is durable and exclusive through Write-Record.
  $path=Join-Path $Directory 'permit-consumed.json'
  Write-Record $path $Permit
  return $path
}
function Read-RestartIni([string]$Path) {
  # Retain each value exactly, including whitespace; the ordinary audit reader
  # intentionally normalizes lines for OpenCPN compatibility. The restart delta
  # must not normalize a change in an output/connection/plugin value away.
  $path=Assert-LocalPath $Path;$file=Get-Item -LiteralPath $path
  if($file.Length -le 0 -or $file.Length -gt 4194304){throw 'Restart profile exceeds bounds.'}
  $text=(New-Object Text.UTF8Encoding($false,$true)).GetString([IO.File]::ReadAllBytes($path)).TrimStart([char]0xfeff)
  if($text -match '[\x00-\x08\x0b\x0c\x0e-\x1f]'){throw 'Invalid restart profile text.'}
  $values=[Collections.Generic.Dictionary[string,string]]::new([StringComparer]::Ordinal)
  $seen=New-Object 'Collections.Generic.HashSet[string]' ([StringComparer]::OrdinalIgnoreCase)
  $sections=New-Object 'Collections.Generic.HashSet[string]' ([StringComparer]::OrdinalIgnoreCase)
  $section=''
  foreach($raw in ($text -split '\r?\n')) {
    $line=$raw.Trim()
    if(-not $line -or $line.StartsWith('#') -or $line.StartsWith(';')){continue}
    if($line -match '^\[([^\[\]]+)\]$'){$section=$Matches[1];if(-not $sections.Add($section)){throw 'Ambiguous restart profile section.'};continue}
    $at=$raw.IndexOf('=')
    if(-not $section -or $at -le 0){throw 'Invalid restart profile syntax.'}
    $key=$section+'/'+$raw.Substring(0,$at).Trim()
    if(-not $seen.Add($key)){throw 'Ambiguous restart profile key.'}
    $values.Add($key,$raw.Substring($at+1))
  }
  if(-not $sections.Contains('Settings')){throw 'Missing OpenCPN settings.'}
  return ,$values
}
function Get-RestartBaseline($Session,[string]$Record,$Parent,[switch]$ReviewOnly,[string]$PendingDirectory='') {
  if($PendingDirectory -and -not $ReviewOnly){throw 'Pending transition inspection is read-only only.'}
  $directory=[IO.Path]::GetDirectoryName($Record)
  $cold=Read-Record (Join-Path $directory 'cold-child.json')
  $launch=Join-Path $directory 'cold-launch-consumed.json'
  if($cold.owner -cne $script:RestartOwner -or $cold.session -cne $Session.session -or $cold.recordSha256 -cne $Session.recordSha256 -or
     (Get-Digest $launch) -cne $cold.launchSha256){throw 'Exact successfully started cold child proof required.'}
  $launchRecord=Read-Record $launch
  if($launchRecord.owner -cne $script:RestartOwner -or $launchRecord.session -cne $Session.session -or $launchRecord.recordSha256 -cne $Session.recordSha256 -or
     $launchRecord.status -cne 'consumed-before-start' -or $launchRecord.mode -cnotin @('--xnav','--legacy','--safe-mode')){throw 'Cold launch journal identity differs.'}
  Assert-RestartDecimal $cold.pid 'cold child';Assert-RestartDecimal $cold.createdFiletime 'cold child creation'
  $baseline=$Session.beforeIni;$hash=$Session.beforeIniSha256;$expectedChild=$cold;$index=0
  $mode=$launchRecord.mode;$completionPath=$null;$completionHash=$null;$pending=$false
  foreach($transition in @(Get-ChildItem -LiteralPath $directory -Directory -Filter 'transition-*' | Sort-Object Name)) {
    $index++
    if($index -gt 16 -or $transition.Name -cne ('transition-{0:d4}' -f $index)){throw 'Restart chain is ambiguous or exceeds the session bound.'}
    $path=Assert-LocalPath $transition.FullName
    if($pending){throw 'An armed pending transition cannot precede another transition.'}
    if($PendingDirectory -and $path -ceq (Assert-LocalPath $PendingDirectory)) {
      if(Test-Path -LiteralPath (Join-Path $path 'completion.json')){throw 'Expected a listening transition, not a completed one.'}
      if(@(Get-ChildItem -LiteralPath $path -Force | Where-Object {$_.Name -cnotin @('ready.json','ui-intent-consumed.json')}).Count){throw 'Pending transition already has an outcome or unexpected files.'}
      $ready=Read-Record (Join-Path $path 'ready.json')
      if($ready.owner -cne $script:RestartOwner -or $ready.session -cne $Session.session -or $ready.recordSha256 -cne $Session.recordSha256 -or
         $ready.parent.pid -cne $expectedChild.pid -or $ready.parent.createdFiletime -cne $expectedChild.createdFiletime -or
         $ready.beforeSha256 -cne $hash){throw 'Armed pending transition is not bound to the verified last child and baseline.'}
      $pending=$true;continue
    }
    $completion=Read-Record (Join-Path $path 'completion.json')
    $permitFile=Join-Path $path 'permit-consumed.json';$receiptFile=Join-Path $path 'receipt.json';$requestFile=Join-Path $path 'request.json'
    if($completion.owner -cne $script:RestartOwner -or $completion.status -cne 'child-identity-verified' -or $completion.session -cne $Session.session -or $completion.recordSha256 -cne $Session.recordSha256 -or
       (Get-Digest $permitFile) -cne $completion.permitSha256 -or (Get-Digest $receiptFile) -cne $completion.receiptSha256 -or
       (Get-Digest $requestFile) -cne $completion.requestSha256){throw 'Previous restart completion proof changed or is incomplete.'}
    $permit=Read-Record $permitFile
    $request=[OpenNavX.RestartCommissioningNative]::Message([IO.File]::ReadAllBytes($requestFile))
    $receipt=[OpenNavX.RestartCommissioningNative]::Message([IO.File]::ReadAllBytes($receiptFile))
    if($permit.owner -cne $script:RestartOwner -or $permit.session -cne $Session.session -or $permit.recordSha256 -cne $Session.recordSha256 -or $permit.beforeSha256 -cne $hash -or
       $permit.requestSha256 -cne $completion.requestSha256 -or $permit.status -cne 'consumed-before-allow' -or
       ($expectedChild -and ($request['parentPid'] -cne $expectedChild.pid -or $request['parentCreatedFiletime'] -cne $expectedChild.createdFiletime))){throw 'Previous restart chain identity differs.'}
    $identity=[pscustomobject]@{pid=$request['parentPid'];createdFiletime=$request['parentCreatedFiletime']}
    Assert-RestartRequest $request $Session $identity $permit.mode $request['helperPid']
    Assert-RestartReceipt $receipt $request $completion.requestSha256 $permit.permitId
    if($receipt['status'] -cne 'started'){throw 'Previous restart did not produce a verified child; cold review required.'}
    $next=Join-Path $path 'post-close.ini'
    if((Get-Digest $next) -cne $permit.profileSha256){throw 'Previous post-close profile proof changed.'}
    $null=Assert-RestartIniDelta (Read-RestartIni $baseline) (Read-RestartIni $next) $permit.mode
    $baseline=$next;$hash=$permit.profileSha256
    $expectedChild=[pscustomobject]@{pid=$receipt['childPid'];createdFiletime=$receipt['childCreatedFiletime']}
    if($ReviewOnly -and ($completion.child.pid -cne $expectedChild.pid -or $completion.child.createdFiletime -cne $expectedChild.createdFiletime)){throw 'Completed child identity differs from the native receipt.'}
    $mode=$permit.mode;$completionPath=Join-Path $path 'completion.json';$completionHash=Get-Digest $completionPath
  }
  if($PendingDirectory -and -not $pending){throw 'Expected armed pending transition was not found.'}
  if($index -ge 16 -and -not $ReviewOnly){throw 'Restart session transition limit reached; cold review required.'}
  if($expectedChild -and ($Parent.pid -cne $expectedChild.pid -or $Parent.createdFiletime -cne $expectedChild.createdFiletime)){throw 'Only the last verified child can continue this restart session.'}
  return [pscustomobject]@{path=$baseline;sha256=$hash;nextDirectory=(Join-Path $directory ('transition-{0:d4}' -f ($index+1)));
    mode=$mode;child=$expectedChild;completionPath=$completionPath;completionSha256=$completionHash;completedTransitions=($index-[int]$pending)}
}
function Save-RestartWire([string]$Path,[byte[]]$Bytes) {
  $path=Assert-LocalPath $Path
  $stream=New-Object IO.FileStream($path,[IO.FileMode]::CreateNew,[IO.FileAccess]::Write,[IO.FileShare]::None)
  try{$stream.Write($Bytes,0,$Bytes.Length);$stream.Flush($true)}finally{$stream.Dispose()}
}

function Assert-RestartShutdownReview($Review,$Retained) {
  if($Review.schema -ne 1 -or $Review.physicalCommands -ne 0 -or @($Review.plugins).Count -ne @($Retained).Count){throw 'Shutdown review must cover every exact retained plugin.'}
  $when=[DateTime]::Parse($Review.reviewedUtc).ToUniversalTime()
  if($when -gt [DateTime]::UtcNow -or ([DateTime]::UtcNow-$when).TotalHours -gt 24){throw 'Shutdown source review expired.'}
  $seen=New-Object 'Collections.Generic.HashSet[string]' ([StringComparer]::Ordinal)
  foreach($plugin in @($Retained)) {
    $name=[IO.Path]::GetFileNameWithoutExtension($plugin.path)
    if(-not $seen.Add($name)){throw 'Retained plugin basename is ambiguous.'}
    $entry=@($Review.plugins | Where-Object {$_.plugin -ceq $name})
    if($entry.Count -ne 1 -or $entry[0].revision -cne $plugin.sourceRevision -or $entry[0].revision -cnotmatch '^[a-f0-9]{40}$' -or
       $entry[0].sourceSha256 -cnotmatch '^[a-f0-9]{64}$' -or -not $entry[0].shutdownBoundary){throw 'Retained shutdown source identity/review differs.'}
    if($name -ceq 'o-charts_pi' -and (-not $entry[0].limitation -or $entry[0].streamSourceSha256 -cnotmatch '^[a-f0-9]{64}$')){throw 'The closed vendor/helper trust boundary must be retained explicitly.'}
  }
}
function Get-RestartLaunchBinding([string]$Record,[string]$Hash,$Installed,$Config,$Environment) {
  $session=Read-RestartSession $Record $Hash
  if($Installed.executable -cne $session.executable -or $Installed.ownership.commit -cne $session.buildCommit -or
     $Config.profileDirectory -cne [IO.Path]::GetDirectoryName($session.profile) -or
     $Config.readOnlyAudit.profileIniSha256 -cne $session.beforeIniSha256 -or (Get-Digest $session.profile) -cne $session.beforeIniSha256 -or
     $Environment.path -cne $session.path -or $Environment.workingDirectory -cne $session.workingDirectory){throw 'Cold restart binding differs from the full current launch audit.'}
  $directory=[IO.Path]::GetDirectoryName($Record)
  if((Test-Path -LiteralPath (Join-Path $directory 'cold-launch-consumed.json')) -or @(Get-ChildItem -LiteralPath $directory -Directory -Filter 'transition-*').Count){throw 'Cold restart session has already been consumed.'}
  return [pscustomobject]@{session=$session;record=$Record;recordSha256=$Hash;directory=$directory}
}
function Set-RestartLaunchBinding($Start,$Binding) {
  # Called only after both full audit boundaries and immediately before the
  # existing sole Process.Start. A failed start consumes this cold session.
  if($Start.FileName -cne $Binding.session.executable -or $Start.WorkingDirectory -cne $Binding.session.workingDirectory -or
     $Start.EnvironmentVariables['PATH'] -cne $Binding.session.path -or $Start.Arguments -cnotin @('--xnav','--legacy','--safe-mode') -or $Start.UseShellExecute){throw 'ProcessStartInfo differs from the verified cold session.'}
  Write-Record (Join-Path $Binding.directory 'cold-launch-consumed.json') @{owner=$script:RestartOwner;session=$Binding.session.session;recordSha256=$Binding.recordSha256;mode=$Start.Arguments;status='consumed-before-start';at=[DateTime]::UtcNow.ToString('o')}
  $Start.EnvironmentVariables['OPENNAV_COMMISSIONING_RESTART_SESSION']=$Binding.session.session
  $Start.EnvironmentVariables['OPENNAV_COMMISSIONING_RESTART_RECORD_SHA256']=$Binding.recordSha256
}
function Save-RestartColdChild($Process,$Binding) {
  if($Process.Path -cne $Binding.session.executable -or $Process.SessionId.ToString() -cne $Binding.session.windowsSessionId){throw 'Cold child image/session differs.'}
  Write-Record (Join-Path $Binding.directory 'cold-child.json') @{owner=$script:RestartOwner;session=$Binding.session.session;recordSha256=$Binding.recordSha256;pid=$Process.Id.ToString();createdFiletime=$Process.StartTime.ToUniversalTime().ToFileTimeUtc().ToString();launchSha256=(Get-Digest (Join-Path $Binding.directory 'cold-launch-consumed.json'))}
}

function Assert-RestartBuild($Build,[string]$Commit,[string]$ExecutableSha256,[string]$HelperSha256) {
  if($ExecutableSha256 -cnotmatch '^[a-f0-9]{64}$' -or $HelperSha256 -cnotmatch '^[a-f0-9]{64}$' -or
     $Build.executable_sha256 -cne $ExecutableSha256 -or $Build.restart_helper_sha256 -cne $HelperSha256){throw 'Guard capability must describe these exact installed binaries.'}
  if($Build.PSObject.Properties.Name -cnotcontains 'commissioning_restart_protocol' -or
     ($Build.commissioning_restart_protocol -isnot [int] -and $Build.commissioning_restart_protocol -isnot [long]) -or $Build.commissioning_restart_protocol -ne 1){throw 'Installed generation is not qualified for commissioning restart protocol 1; use separately reviewed cold launch.'}
  if($Build.test_fixtures -isnot [bool] -or $Build.test_fixtures -or $Build.build_purpose -cne 'INSTALLED PRODUCT' -or $Build.commit -cne $Commit){throw 'Only the exact fixture-free installed product may be armed.'}
}

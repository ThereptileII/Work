# Inert filesystem fixtures only. No COM ports, scheduled tasks or applications.
[CmdletBinding()]
param([switch]$PortableContracts,[switch]$IsolatedLocal)
Set-StrictMode -Version Latest;$ErrorActionPreference='Stop'
$native=[Environment]::OSVersion.Platform -eq 'Win32NT'
if(-not $native -and -not $PortableContracts){throw 'Explicit portable fixture mode required.'}
if($native -and -not $IsolatedLocal -and $env:GITHUB_ACTIONS -cne 'true'){throw 'Use disposable CI or explicit isolated fixture mode.'}
. (Join-Path $PSScriptRoot 'ManualPilotCommissioning.ps1')
$fixtureRoot=Join-Path ([IO.Path]::GetTempPath()) ('manual-pilot-'+[guid]::NewGuid().ToString('N'))
$null=New-Item -ItemType Directory $fixtureRoot
$checks=0
function Check([scriptblock]$Call){$null=& $Call;$script:checks++}
function Refuse([scriptblock]$Call){$failed=$false;try{$null=& $Call}catch{$failed=$true};if(-not $failed){throw 'Unsafe manual operation accepted'};$script:checks++}
function Same($A,$B){if($A -cne $B){throw 'Fixture equality failed'}}
function Clone($V){return $V|ConvertTo-Json -Depth 32|ConvertFrom-Json}
function Assert-LocalPath([string]$Path){$p=[IO.Path]::GetFullPath($Path);if(-not $p.StartsWith($fixtureRoot+[IO.Path]::DirectorySeparatorChar,[StringComparison]::Ordinal)){throw 'Escaped inert fixture root'};return $p}
# Strictly isolate native identity probes, signature checks and original trees.
# Production publication/metadata, ownership checks, parsing and inverse remain real.
$script:closed=$true
function Assert-PreparationClosed([string[]]$Roots){if(-not $script:closed){throw 'Fixture process is still open'}}
function Get-CommissioningIdentityContext([string]$Workspace){Same $Workspace $script:context.workspace;return Clone $script:context}
function Get-CommissioningContext([string]$Workspace){Assert-PreparationClosed @();return Get-CommissioningIdentityContext $Workspace}
function Get-Installed{return $script:installed}
function Get-Target([string]$Workspace){return $script:config}
function Get-ManualPilotEngine{return [pscustomobject]@{path=$script:exe;sha256=(Get-Digest $script:exe)}}
function Assert-ReadOnlyAudit($Config,$Installed,[string]$Workspace){
 if(Test-Path (Join-Path $Workspace 'manual-pilot-active.json')){throw 'Child active'}
 Same (Get-Digest $script:ini) $script:baselineHash
 Assert-InputOnlyProfile (Read-ProfileForAudit $script:ini)
 if(Test-Path $script:plugin){throw 'Plugin returned'}
}
$realTree=${function:Get-PreparationTree}
function Get-PreparationTree([string]$Root){if($Root -eq $PSScriptRoot){$Root=$script:fakeTools};return & $script:realTree $Root}
if(-not $native){
 function Get-Acl {param([string]$LiteralPath) return [pscustomobject]@{Sddl='fixture-acl'}}
 function Assert-PreparationAcl([string]$Expected,[string]$Actual,[switch]$AllowDaclAutoInherited){Same $Expected $Actual}
 function New-PreparationDirectory($Context,[string]$Purpose){return New-RunDirectory $Context.workspace $Purpose}
}
try {
 foreach($f in @('ManualPilotCommissioning.ps1','commission-manual-pilot.ps1','Common.ps1','InteractiveJob.ps1','commission-read-only.ps1')){
  $t=$null;$e=$null;$null=[Management.Automation.Language.Parser]::ParseFile((Join-Path $PSScriptRoot $f),[ref]$t,[ref]$e)
  Check {if($e.Count){throw ($e|Out-String)}}
 }
 $alpha='OpenNavXSettings 1\n"battery" "existing source"\n"curve" "opaque calibration"\n'
 $bound=$alpha+'"pilot.interface" "COM8"\n"pilot.name" "c0508700e76004d2"\n"pilot.permission" "display-only"\n'
 Check {Same (Assert-ManualPilotDelta $alpha $bound).name 'c0508700e76004d2'}
 foreach($bad in @($bound.Replace('opaque calibration','changed'),$bound.Replace('COM8','COM9'),$bound.Replace('c0508700e76004d2','0000000000000000'),$bound.Replace('display-only','manual'),($bound+'"pilot.name" "c0508700e76004d2"\n'),($bound+'"pilot.other" "x"\n'),$bound.Replace('"COM8"','"C\\OM8"'))){Refuse {Assert-ManualPilotDelta $alpha $bad}}
 $now=[datetime]::UtcNow
 $diag=[pscustomobject]@{build_commit=('b'*40);build_purpose='INSTALLED PRODUCT';data_mode='OPENCPN selected navigation';xnav_hardware_output_policy='manual-commissioning';xnav_manual_control_contract=1;runtime=[pscustomobject]@{pilot=[pscustomobject]@{enabled=$false;serial_session_enabled=$false;configured_permission=$false;simulated=$false;track_capability=$false;wind_capability=$false};display=[pscustomobject]@{route_creation_active=$false};replay=[pscustomobject]@{active=$false}}}
 $diag | Add-Member publication_clock 'live monotonic clock'
 $diag | Add-Member publication_monotonic_ms '10000'
 $diag | Add-Member route ([pscustomobject]@{state='NoActiveRoute';id='';waypoint='';waypoint_count=0;source='OpenCPN 5.12.4 normal route progress: active range + subsequent stored legs; cross-track error (NM)';revision='1';revision_scope='SKAGER session 123456789';observed_monotonic_ms='9900';quality='Unavailable'})
 Check {Assert-ManualPilotStartupDiagnostics $diag ('b'*40) $now $now.AddSeconds(-1) $now}
 foreach($field in @('enabled','serial_session_enabled','configured_permission','simulated','track_capability','wind_capability')){
  $bad=Clone $diag;$bad.runtime.pilot.$field=$true;Refuse {Assert-ManualPilotStartupDiagnostics $bad ('b'*40) $now $now.AddSeconds(-1) $now}
 }
 Refuse {Assert-ManualPilotStartupDiagnostics $diag ('c'*40) $now $now.AddSeconds(-1) $now}
 Refuse {Assert-ManualPilotStartupDiagnostics $diag ('b'*40) $now.AddSeconds(-10) $now.AddSeconds(-1) $now}
 # The route state is explicit; unavailable/missing/active data never proves
 # inactivity. NoActiveRoute quality is legitimately Unavailable.
 foreach($state in @('Valid','MissingPosition','AwaitingProgress','InvalidRoute','StalePosition')) {
  $bad=Clone $diag;$bad.route.state=$state;Refuse {Assert-ManualPilotStartupDiagnostics $bad ('b'*40) $now $now.AddSeconds(-1) $now}
 }
 foreach($field in @('id','waypoint','source','revision_scope')) {
  $bad=Clone $diag;$bad.route.$field='unexpected';Refuse {Assert-ManualPilotStartupDiagnostics $bad ('b'*40) $now $now.AddSeconds(-1) $now}
 }
 foreach($value in @($null,'0','10001','4999','18446744073709551616',10000)) {
  $bad=Clone $diag;$bad.route.observed_monotonic_ms=$value;Refuse {Assert-ManualPilotStartupDiagnostics $bad ('b'*40) $now $now.AddSeconds(-1) $now}
 }
 $bad=Clone $diag;$bad.route=$null;Refuse {Assert-ManualPilotStartupDiagnostics $bad ('b'*40) $now $now.AddSeconds(-1) $now}
 $bad=Clone $diag;$bad.route.id=$null;Refuse {Assert-ManualPilotStartupDiagnostics $bad ('b'*40) $now $now.AddSeconds(-1) $now}
 $bad=Clone $diag;$bad.route.waypoint_count=1;Refuse {Assert-ManualPilotStartupDiagnostics $bad ('b'*40) $now $now.AddSeconds(-1) $now}
 $bad=Clone $diag;$bad.route.revision='0';Refuse {Assert-ManualPilotStartupDiagnostics $bad ('b'*40) $now $now.AddSeconds(-1) $now}
 $bad=Clone $diag;$bad.publication_clock='recorded session clock';Refuse {Assert-ManualPilotStartupDiagnostics $bad ('b'*40) $now $now.AddSeconds(-1) $now}
 # Simulate a UI read republishing unchanged retained route state six seconds
 # later. Its fresh file date and new publication time must not renew the pass.
 $bad=Clone $diag;$bad.publication_monotonic_ms='16000'
 Refuse {Assert-ManualPilotStartupDiagnostics $bad ('b'*40) $now.AddSeconds(6) $now.AddSeconds(-1) $now.AddSeconds(6)}
 # File age and age-at-publication both count toward the five-second bound.
 $bad=Clone $diag;$bad.route.observed_monotonic_ms='6000'
 Refuse {Assert-ManualPilotStartupDiagnostics $bad ('b'*40) $now.AddSeconds(-2) $now.AddSeconds(-10) $now}
 # Exercise the production bounded reader, parser, hash and handle lifetime.
 # The Unix-only adapter reads metadata from that SAME held handle; Win32 uses
 # the actual GetFileInformationByHandle implementation with no metadata shim.
 Initialize-ManualPilotDiagnosticsNative
 if(-not $native) {
  function Get-ManualPilotDiagnosticsMetadata([IO.FileStream]$Stream) {
   return [pscustomobject]@{Length=$Stream.Length;Attributes=0x80;WrittenUtc=[IO.File]::GetLastWriteTimeUtc($Stream.SafeFileHandle)}
  }
 }
 $realDiagnosticMetadata=${function:Get-ManualPilotDiagnosticsMetadata}
 $script:diagnosticPath=Join-Path $fixtureRoot 'diagnostics.json'
 $script:diagnosticStage=Join-Path $fixtureRoot 'diagnostics.pending'
 $utf8=New-Object Text.UTF8Encoding($false)
 $oldBytes=$utf8.GetBytes(($diag|ConvertTo-Json -Depth 32))
 [IO.File]::WriteAllBytes($diagnosticPath,$oldBytes)
 [IO.File]::SetLastWriteTimeUtc($diagnosticPath,$now.AddMinutes(-30))
 $oldWritten=[IO.File]::GetLastWriteTimeUtc($diagnosticPath)
 $newDiag=Clone $diag;$newDiag.route.state='Valid';$newDiag.route.id='active-route'
 $newBytes=$utf8.GetBytes(($newDiag|ConvertTo-Json -Depth 32))
 [IO.File]::WriteAllBytes($diagnosticStage,$newBytes)
 [IO.File]::SetLastWriteTimeUtc($diagnosticStage,$now)
 if($native){Initialize-PreparationNative}
 $script:replacementAttempted=$false;$script:replacementDenied=$false;$script:replacementErrorCode=$null
 function Get-ManualPilotDiagnosticsMetadata([IO.FileStream]$Stream) {
  # Deterministic interleaving: exact old bytes have been read, and the publisher
  # tries replacing the path before metadata is fetched. No timers or sleeps.
  $script:replacementAttempted=$true
  if($native) {
   try{[OpenNavX.PreparationNative]::Publish($diagnosticStage,$diagnosticPath)}catch{
    $cause=$_.Exception.GetBaseException()
    $code=if($cause -is [ComponentModel.Win32Exception]){$cause.NativeErrorCode}else{'not-Win32Exception'}
    # MoveFileEx(REPLACE_EXISTING|WRITE_THROUGH) can report ACCESS_DENIED (5)
    # or SHARING_VIOLATION (32) for the destination held without delete sharing.
    # Require that actual native denial AND both exact files to survive; the
    # same publication must then succeed after the reader releases its handle.
    if($cause -isnot [ComponentModel.Win32Exception] -or $code -notin @(5,32)){
     throw ('Unexpected held-publication refusal: nativeCode='+$code+'; type='+$cause.GetType().FullName+'; message='+$cause.Message)
    }
    try {
     Same (Get-Digest $diagnosticPath) (Get-CommissioningHash $oldBytes)
     Same (Get-Digest $diagnosticStage) (Get-CommissioningHash $newBytes)
     Same ([IO.File]::GetLastWriteTimeUtc($diagnosticPath)) $oldWritten
    } catch {throw ('Held-publication refusal did not preserve original/stage: nativeCode='+$code+'; '+$_.Exception.Message)}
    $script:replacementErrorCode=$code;$script:replacementDenied=$true
   }
   Refuse {$writer=[IO.File]::Open($diagnosticPath,[IO.FileMode]::Open,[IO.FileAccess]::Write,[IO.FileShare]::ReadWrite);$writer.Dispose()}
  } else {[IO.File]::Move($diagnosticStage,$diagnosticPath,$true)}
  return & $script:realDiagnosticMetadata $Stream
 }
 try{$held=Read-ManualPilotDiagnosticsSnapshot $diagnosticPath}finally{Set-Item Function:Get-ManualPilotDiagnosticsMetadata $realDiagnosticMetadata}
 Check {
  Same $replacementAttempted $true
  Same $held.data.route.state 'NoActiveRoute';Same $held.sha256 (Get-CommissioningHash $oldBytes)
  Same $held.writtenUtc $oldWritten;Same $held.bytes $oldBytes.Length
  if($native){Same $replacementDenied $true}
 }
 # The stale/pre-process observation cannot borrow the new publication's date.
 Refuse {Assert-ManualPilotStartupDiagnostics $held.data ('b'*40) $held.writtenUtc $now.AddSeconds(-1) $now}
 if($native){
  try{[OpenNavX.PreparationNative]::Publish($diagnosticStage,$diagnosticPath)}catch{
   throw ('Publication must succeed after the read handle is released; held nativeCode='+$replacementErrorCode+'; '+$_.Exception.GetBaseException().Message)
  }
  Check {if(Test-Path -LiteralPath $diagnosticStage){throw 'Successful rename retained the stage unexpectedly'}}
 }
 $next=Read-ManualPilotDiagnosticsSnapshot $diagnosticPath
 Check {Same $next.data.route.state 'Valid';Same $next.sha256 (Get-CommissioningHash $newBytes);Same $next.writtenUtc ([IO.File]::GetLastWriteTimeUtc($diagnosticPath))}
 Refuse {Assert-ManualPilotStartupDiagnostics $next.data ('b'*40) $next.writtenUtc $now.AddSeconds(-1) $now}
 if($native) {
  $linked=Join-Path $fixtureRoot 'diagnostics-link.json'
  $null=New-Item -ItemType HardLink -Path $linked -Target $diagnosticPath
  try{Refuse {Read-ManualPilotDiagnosticsSnapshot $diagnosticPath}}finally{Remove-Item -LiteralPath $linked}
  Check {Same (Read-ManualPilotDiagnosticsSnapshot $diagnosticPath).sha256 (Get-CommissioningHash $newBytes)}
 }
 foreach($invalid in @([byte[]]@(),[byte[]]@(0xff),$utf8.GetBytes('{'),(New-Object byte[] 4194305))) {
  [IO.File]::WriteAllBytes($diagnosticPath,$invalid)
  Refuse {Read-ManualPilotDiagnosticsSnapshot $diagnosticPath}
  # Both successful and failed reads must release the exclusive read handle.
  Check {$writer=[IO.File]::Open($diagnosticPath,[IO.FileMode]::Open,[IO.FileAccess]::Write,[IO.FileShare]::None);$writer.Dispose()}
 }
 $script:workspace=Join-Path $fixtureRoot 'workspace';$profile=Join-Path $fixtureRoot 'profile';$generation=Join-Path $fixtureRoot 'generation';$plugins=Join-Path $generation 'app/plugins';$script:fakeTools=Join-Path $fixtureRoot 'tools'
 foreach($d in @($workspace,$profile,$generation,$plugins,$fakeTools)){ $null=New-Item -ItemType Directory $d -Force }
 [IO.File]::WriteAllText((Join-Path $fakeTools 'helper.txt'),'inert tool identity')
 $script:ini=Join-Path $profile 'opencpn.ini';$null=New-Item -ItemType Directory (Join-Path $generation 'app') -Force;$script:exe=Join-Path $generation 'app/opencpn.exe'
 [IO.File]::WriteAllText($exe,'inert, never executed')
 $connection='0;0;;0;1;COM8;115200;0;0;0;;0;;0;0;1;0;1;Gateway;0;;0'
 $text="[Settings]`r`nPersistActiveRoute=0`r`nActiveRoute=12345678-90AB-cdef-1234-567890abcdef`r`n[Settings/NMEADataSource]`r`nDataConnections=$connection`r`n[Settings/GlobalState]`r`nFrameWinX=1024`r`n[OpenNav]`r`nAlphaSettings=$alpha`r`n"
 [IO.File]::WriteAllText($ini,$text,(New-Object Text.UTF8Encoding($false)));$script:baselineHash=Get-Digest $ini
 Check {Assert-ManualPilotProfile (Read-ProfileForAudit $ini) $false}
 Check {$empty=Read-ProfileForAudit $ini;$empty['Settings/ActiveRoute']='';Assert-ManualPilotProfile $empty $false}
 foreach($bad in @($text.Replace('PersistActiveRoute=0','PersistActiveRoute=1'),$text.Replace('ActiveRoute=','ActiveRoute=some-route'),$text.Replace('COM8','COM9'),$text.Replace($connection,$connection+'|'+$connection),$text.Replace($connection,$connection+'|'+$connection.Replace('COM8','COM9').Replace(';0;0;0;;',';0;1;0;;')))){
  $badIni=Join-Path $fixtureRoot 'bad.ini';[IO.File]::WriteAllText($badIni,$bad);Refuse {Assert-ManualPilotProfile (Read-ProfileForAudit $badIni) $false}
 }
 $script:plugin=Join-Path $plugins 'autotrack_pi.dll';[IO.File]::WriteAllText($plugin,'inert plugin')
 $inventory=@{trees=@(Get-CommissioningTrees @($plugins))}
 $parent=Join-Path $workspace 'parent';$quarantine=Join-Path $parent 'quarantine';$null=New-Item -ItemType Directory $quarantine -Force
 $backup=Join-Path $parent 'plugin.bin';[IO.File]::Copy($plugin,$backup);$destination=Join-Path $quarantine 'plugin.bin'
 $acl=(Get-Acl -LiteralPath $plugin).Sddl;$pluginHash=Get-Digest $plugin;[IO.File]::Move($plugin,$destination)
 $parentRecord=Join-Path $parent 'prepared.json'
 Write-Record $parentRecord @{quarantine=@(@{path=$plugin;backup=$backup;destination=$destination;sha256=$pluginHash;acl=$acl})}
 Write-Record (Join-Path $parent 'inventory.json') $inventory
 Write-Record (Join-Path $parent 'review-plan.json') @{plugins=@()}
 Write-Record (Join-Path $workspace 'commissioning-active.json') @{record=$parentRecord}
 Write-Record (Join-Path $workspace 'boat-target.json') @{fixture=$true}
 $script:installed=[pscustomobject]@{root=$workspace;executable=$exe;generation=$generation;state=[pscustomobject]@{current=('a'*32)};ownership=[pscustomobject]@{commit=('b'*40);packageSha256=('c'*64);xnavHardwareOutputPolicy='manual-commissioning';xnavManualControlContract=1;files=@([pscustomobject]@{path='app/opencpn.exe';sha256=(Get-Digest $exe)});managedFiles=@([pscustomobject]@{path='app/opencpn.exe';sha256=(Get-Digest $exe)})}}
 $runtime=Join-Path $generation 'app/wxbase32u_vc14x.dll';[IO.File]::WriteAllText($runtime,'inert owned runtime')
 $installed.ownership.files+=@([pscustomobject]@{path='app/wxbase32u_vc14x.dll';sha256=(Get-Digest $runtime)},[pscustomobject]@{path='app/plugins/autotrack_pi.dll';sha256=$pluginHash})
 $installed.ownership.managedFiles+=@([pscustomobject]@{path='app/wxbase32u_vc14x.dll';sha256=(Get-Digest $runtime)})
 Write-Record (Join-Path $generation 'ownership.json') $installed.ownership
 $candidate=[pscustomobject]@{generation=('a'*32);commit=('b'*40);packageSha256=('c'*64);executableSha256=(Get-Digest $exe);ownershipSha256=(Get-Digest (Join-Path $generation 'ownership.json'))}
 $sid=if($native){[Security.Principal.WindowsIdentity]::GetCurrent().User.Value}else{'S-1-5-21-1-2-3-1000'}
 $script:context=[pscustomobject]@{workspace=$workspace;profile=$profile;sid=$sid;session=1;application=$generation;managed=$plugins;pluginRoots=@($plugins);installation=[pscustomobject]@{root=$workspace;executable=$exe};launchEnvironment=[pscustomobject]@{workingDirectory=$generation;path=$generation}}
 $script:config=[pscustomobject]@{readOnlyAudit=[pscustomobject]@{commissioning=[pscustomobject]@{record=$parentRecord}}}
 $qualification=Join-Path $fixtureRoot 'qualification.json'
 Write-Record $qualification @{owner='OpenNavX.ManualPilotQualification.1';commit=$candidate.commit;packageSha256=$candidate.packageSha256;executableSha256=$candidate.executableSha256;nativeSerialGatePassed=$true;defaultOffDiagnosticsPassed=$true;retainedPluginsReviewedForBidirectional=$true;reviewedUtc=[datetime]::UtcNow.ToString('o');shutdown=@{schema=2;physicalCommands=0;reviewedUtc=[datetime]::UtcNow.ToString('o');plugins=@()}}
 foreach($field in @('generation','commit','packageSha256','executableSha256','ownershipSha256')){$bad=Clone $candidate;$bad.$field='d'*$bad.$field.Length;Refuse {Assert-ManualPilotCandidate $installed $bad}}
 $installed.ownership.xnavManualControlContract='1';Refuse {Assert-ManualPilotCandidate $installed $candidate};$installed.ownership.xnavManualControlContract=1
 $installed.ownership.xnavHardwareOutputPolicy='status-only';Refuse {Assert-ManualPilotCandidate $installed $candidate};$installed.ownership.xnavHardwareOutputPolicy='manual-commissioning'
 $ownedQuarantine=@((Read-Record $parentRecord).quarantine)
 Check {Assert-ManualPilotCandidate $installed $candidate $ownedQuarantine}
 [IO.File]::WriteAllText($runtime,'modified owned non-plugin runtime')
 Refuse {Assert-ManualPilotCandidate $installed $candidate $ownedQuarantine}
 [IO.File]::WriteAllText($runtime,'inert owned runtime')
 $unowned=Join-Path $generation 'app/unowned.dll';[IO.File]::WriteAllText($unowned,'inert unowned runtime')
 Refuse {Assert-ManualPilotCandidate $installed $candidate $ownedQuarantine};[IO.File]::Delete($unowned)
 $originalExe=[IO.File]::ReadAllText($exe);[IO.File]::WriteAllText($exe,'changed runtime')
 Refuse {Assert-ManualPilotCandidate $installed $candidate};[IO.File]::WriteAllText($exe,$originalExe)
 Write-Record (Join-Path $workspace 'update-pending.json') @{fixture='pending'}
 Refuse {New-ManualPilot $workspace $candidate $qualification (Get-Digest $qualification)}
 Remove-Item -LiteralPath (Join-Path $workspace 'update-pending.json')
 $prepared=New-ManualPilot $workspace $candidate $qualification (Get-Digest $qualification)
 Check {Same (Get-Digest $ini) $baselineHash; if(-not (Test-Path (Join-Path $workspace 'manual-pilot-active.json'))){throw 'Prepared child ownership missing'}}
 Refuse {New-ManualPilot $workspace $candidate $qualification (Get-Digest $qualification)}
 $script:closed=$false;Refuse {Apply-ManualPilot $workspace $prepared.record $prepared.recordSha256};$script:closed=$true
 $applied=Apply-ManualPilot $workspace $prepared.record $prepared.recordSha256
 Check {Assert-ManualPilotProfile (Read-ProfileForAudit $ini) $true;Same $applied.profileSha256 (Get-Digest $ini);if(Test-Path $plugin){throw 'Plugin restored'}}
 Refuse {Apply-ManualPilot $workspace $prepared.record $prepared.recordSha256}
 Check {Read-ManualPilot $workspace $prepared.record $prepared.recordSha256 -Active -Launching}
 Write-Record (Join-Path $workspace 'update-pending.json') @{fixture='pending launch'}
 Refuse {Read-ManualPilot $workspace $prepared.record $prepared.recordSha256 -Active -Launching}
 Remove-Item -LiteralPath (Join-Path $workspace 'update-pending.json')
 [IO.File]::Copy($destination,$plugin);Refuse {Read-ManualPilot $workspace $prepared.record $prepared.recordSha256 -Active};[IO.File]::Delete($plugin)
 [IO.File]::WriteAllText((Join-Path $fakeTools 'helper.txt'),'changed');Refuse {Read-ManualPilot $workspace $prepared.record $prepared.recordSha256 -Active};[IO.File]::WriteAllText((Join-Path $fakeTools 'helper.txt'),'inert tool identity')
 $childDir=[IO.Path]::GetDirectoryName($prepared.record)
 Write-Record (Join-Path $childDir 'launch-intent.json') @{fixture='ambiguous start'}
 Refuse {Read-ManualPilot $workspace $prepared.record $prepared.recordSha256 -Active -Launching}
 # UI's explicit binding save, still display-only. No unrelated field changes.
 [IO.File]::WriteAllText($ini,([IO.File]::ReadAllText($ini).Replace($alpha,$bound).Replace('FrameWinX=1024','FrameWinX=1280')),(New-Object Text.UTF8Encoding($false)))
 foreach($badText in @(([IO.File]::ReadAllText($ini).Replace('opaque calibration','different calibration')),([IO.File]::ReadAllText($ini).Replace('display-only','manual')),([IO.File]::ReadAllText($ini)+"`r`nUnexpectedManualSetting=1`r`n"))) {
  $badFile=Join-Path $fixtureRoot 'bad-delta.ini';[IO.File]::WriteAllText($badFile,$badText)
  Refuse {Assert-ManualPilotProfileDelta (Join-Path $childDir 'output.ini') $badFile}
 }
 $inspection=Inspect-ManualPilot $workspace $prepared.record $prepared.recordSha256
 Refuse {Rollback-ManualPilot $workspace $prepared.record $prepared.recordSha256 $inspection.inspection $inspection.inspectionSha256 ('0'*64)}
 # Inject interruption AFTER actual atomic rollback publication but BEFORE completion.
 $realPublish=${function:Publish-PreparedProfile};$script:interrupt=$true
 function Publish-PreparedProfile([string]$Original,[string]$SavedCandidate,[string]$OriginalHash,[string]$CandidateHash,[int]$Length,[string]$Journal){
  & $script:realPublish $Original $SavedCandidate $OriginalHash $CandidateHash $Length $Journal
  if($script:interrupt -and [IO.Path]::GetFileName($Journal) -ceq 'rollback-intent.json'){throw 'Injected post-publication interruption'}
 }
 Refuse {Rollback-ManualPilot $workspace $prepared.record $prepared.recordSha256 $inspection.inspection $inspection.inspectionSha256 $inspection.currentIniSha256}
 Check {Assert-ManualPilotProfile (Read-ProfileForAudit $ini) $false;if(-not(Test-Path (Join-Path $workspace 'manual-pilot-active.json'))){throw 'Interruption lost ownership'}}
 $script:interrupt=$false
 $rolled=Rollback-ManualPilot $workspace $prepared.record $prepared.recordSha256 $inspection.inspection $inspection.inspectionSha256 $inspection.currentIniSha256
 Check {Same $rolled.status 'rolled-back';Same ((Read-ProfileForAudit $ini)['OpenNav/AlphaSettings']) $bound;Same ((Read-ProfileForAudit $ini)['Settings/ActiveRoute']) '12345678-90AB-cdef-1234-567890abcdef';if(Test-Path $plugin){throw 'Quarantine lost'};if(-not(Test-Path (Join-Path $workspace 'commissioning-active.json'))){throw 'Parent ownership lost'}}
 Refuse {Read-ManualPilot $workspace $prepared.record $prepared.recordSha256 -Active -Launching}
 # Apply interruption before publication still owns the parent and can roll back.
 $script:baselineHash=Get-Digest $ini
 $again=New-ManualPilot $workspace $candidate $qualification (Get-Digest $qualification)
 function Publish-PreparedProfile([string]$Original,[string]$SavedCandidate,[string]$OriginalHash,[string]$CandidateHash,[int]$Length,[string]$Journal){
  if([IO.Path]::GetFileName($Journal) -ceq 'apply-intent.json'){throw 'Injected before Apply publication'}
  & $script:realPublish $Original $SavedCandidate $OriginalHash $CandidateHash $Length $Journal
 }
 Refuse {Apply-ManualPilot $workspace $again.record $again.recordSha256}
 Check {Same (Get-Digest $ini) $script:baselineHash;if(-not(Test-Path (Join-Path $workspace 'manual-pilot-active.json'))){throw 'Failed Apply lost ownership'}}
 $beforeApplyInspection=Inspect-ManualPilot $workspace $again.record $again.recordSha256
 Check {Rollback-ManualPilot $workspace $again.record $again.recordSha256 $beforeApplyInspection.inspection $beforeApplyInspection.inspectionSha256 $beforeApplyInspection.currentIniSha256}
 # A parent that passed its entry check must recheck AFTER obtaining the lock.
 Assert-NoManualPilotChild $workspace
 Write-Record (Join-Path $workspace 'manual-pilot-active.json') @{fixture='child prepared between entry check and lock'}
 $parentSource=[IO.File]::ReadAllText((Join-Path $PSScriptRoot 'commission-read-only.ps1'))
 $start=$parentSource.IndexOf('$restoreLock=Open-CommissioningRestoreLock $directory')
 $end=$parentSource.IndexOf('$activeRecord=Read-Record $active',$start)
 if($start -lt 0 -or $end -le $start){throw 'Parent source boundaries changed'}
 $parentGuard=[scriptblock]::Create($parentSource.Substring($start,$end-$start)+'}finally{$restoreLock.Dispose()}')
 $directory=$parent;$Workspace=$workspace;$beforeGuardHash=Get-Digest $ini
 Refuse {& $parentGuard}
 Check {Same (Get-Digest $ini) $beforeGuardHash;if(Test-Path $plugin){throw 'Parent guard allowed plugin restoration'}}
 Remove-Item -LiteralPath (Join-Path $workspace 'manual-pilot-active.json')
 [pscustomobject]@{status='passed';checks=$checks;nativeMetadata=$native;heldPublicationNativeError=$replacementErrorCode;scope='Inert real transaction/file publication plus mocked OS identity/signature/process discovery; no actual launch, close, task, port, boat or plugin execution'}|ConvertTo-Json
}finally{Remove-Item -LiteralPath $fixtureRoot -Recurse -Force}

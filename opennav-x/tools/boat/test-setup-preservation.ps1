# Disposable exact-profile contracts. No application, equipment or real profile.
[CmdletBinding()]
param([switch]$PortableContracts,[switch]$IsolatedLocal)
. (Join-Path $PSScriptRoot 'Commissioning.ps1')
$native=[Environment]::OSVersion.Platform -eq 'Win32NT'
if(-not $native -and -not $PortableContracts){throw 'Explicit portable contracts required.'}
if($native -and -not $IsolatedLocal -and $env:GITHUB_ACTIONS -cne 'true'){throw 'Disposable CI or explicit isolated tests required.'}
$root=Join-Path ([IO.Path]::GetTempPath()) ('setup-preservation-'+[guid]::NewGuid().ToString('N'))
$null=New-Item -ItemType Directory $root
function Assert-LocalPath([string]$Path){$p=[IO.Path]::GetFullPath($Path);if(-not $p.StartsWith($root+[IO.Path]::DirectorySeparatorChar,[StringComparison]::Ordinal)){throw 'Escaped fixture root'};return $p}
$checks=0
function Check([scriptblock]$Call){$null=& $Call;$script:checks++}
function Refuse([scriptblock]$Call){$failed=$false;try{$null=& $Call}catch{$failed=$true};if(-not $failed){throw 'Unexpected unsafe preservation admission'};$script:checks++}
function Review($before,$after){return [pscustomobject]@{schema=1;owner='OpenNavX.SessionPreservationReview.1';beforeSha256=(Get-Digest $before);afterSha256=(Get-Digest $after);reviewedUtc=[datetime]::UtcNow.ToString('o');provenance='current-user-state;origin-unverified';preservationOnly=$true;launchPermission=$false;changes=@(Get-CommissioningIniDiff $before $after|ForEach-Object{[pscustomobject]@{key=$_.key;before=$_.before;after=$_.after;decision='preserve-current';origin='unverified';reason='Explicit inert setup preservation test'}})}}
$encoding=New-Object Text.UTF8Encoding($false,$true)
try{
 foreach($value in @('v1|pending','v1|complete','v1|existing')){Check {Assert-PreservedSetupPreference 'OpenNav/BoatSetupV1' $value}}
 foreach($value in @('v2|complete','complete','v1|COMPLETE','v1|pending|extra','')){Refuse {Assert-PreservedSetupPreference 'OpenNav/BoatSetupV1' $value}}
 foreach($scale in @(100,125,150)){foreach($layout in @('balanced','chart','instruments')){Check {Assert-PreservedSetupPreference 'OpenNav/DisplayPreferencesV1' ('v1|'+$scale+'|'+$layout)}}}
 foreach($value in @('v1|200|balanced','v1|125|chart|extra','v2|100|chart','v1|100|balanced ')){Refuse {Assert-PreservedSetupPreference 'OpenNav/DisplayPreferencesV1' $value}}
 foreach($value in @('Boat',(' M/S '+[char]0xc5+'land '),'A\B','Boat "name"','"foo"','')){
  $wire=$value.Replace('\','\\');if($value.Length -gt 0 -and ($value.StartsWith('"') -or [char]::IsWhiteSpace($value[0]) -or [char]::IsWhiteSpace($value[$value.Length-1]))){$wire='"'+$wire.Replace('"','\"')+'"'}
  Check {Assert-PreservedVesselName $wire}
 }
 foreach($value in @('A\nB','A\tB','A\qB','"missing-end','" "',('x'*129),(([string][char]0xc5)*65))){Refuse {Assert-PreservedVesselName $value}}
 foreach($value in @('0','3.5','1e6','-0')){Check {Assert-PreservedSetupPreference 'Settings/GlobalState/S52_MAR_SAFETY_CONTOUR' $value}}
 foreach($value in @('-1','1000001','NaN','inf','1,5','1e999','3 m','')){Refuse {Assert-PreservedSetupPreference 'Settings/GlobalState/S52_MAR_SAFETY_CONTOUR' $value}}
 Refuse {Assert-PreservedSetupPreference 'OpenNav/OtherKey' 'v1|complete'}
 $alpha='OpenNavXSettings 1\n"battery" ""\n"capacity" "20"\n"consumption" "measured"\n"corridor" "50"\n"current" "unconfigured"\n"display.instruments" "sog,depth"\n"display.rail" "sog,heading"\n"draft" "1"\n"efficiency" ""\n"hotel" ""\n"margin" "1"\n"minimum_speed" "1"\n"model_source" ""\n"reserve" "20"\n'
 $configured=$alpha+'"pilot.interface" "existing"\n"pilot.name" "c0508700e76004d2"\n"pilot.permission" "manual"\n"source.depth" "existing source"\n"curve" "opaque existing calibration"\n'
 $changed=$configured.Replace('"capacity" "20"','"capacity" "24"').Replace('"draft" "1"','"draft" "2"')
 Check {Assert-PreservedSetupSettingsDelta $configured $changed}
 Check {Assert-PreservedSetupSettingsDelta $configured ($changed.Replace('"pilot.permission" "manual"','"pilot.permission" "display-only"'))}
 Refuse {Assert-PreservedSetupSettingsDelta ($configured.Replace('"manual"','"display-only"')) $changed}
 foreach($pair in @(@('existing source','new source'),@('existing"','new identity"'),@('opaque existing calibration','different calibration'),@('"current" "unconfigured"','"current" "charge"'),@('c0508700e76004d2','c0508700e76004d3'))){Refuse {Assert-PreservedSetupSettingsDelta $configured ($changed.Replace($pair[0],$pair[1]))}}
 foreach($value in @('-1','100001','NaN','1e999','20,5')){Refuse {Assert-PreservedSetupSettingsDelta $configured ($changed.Replace('"capacity" "24"',('"capacity" "'+$value+'"')))}}
 Refuse {Assert-PreservedSetupSettingsDelta $configured ($changed+'"capacity" "24"\n')}
 Refuse {Assert-PreservedSetupSettingsDelta $configured ($changed+'"pilot.permission" "display-only"\n')}
 Refuse {Assert-PreservedSetupSettingsDelta $configured ($changed+'"model_source" ""\n')}
 Refuse {Assert-PreservedSetupSettingsDelta $configured ($changed.Replace('OpenNavXSettings 1','OpenNavXSettings 2'))}
 $provenance='User-configured usable battery energy and reserve / OpenCPN profile'
 Check {Assert-PreservedAlphaSettings ($alpha.Replace('"model_source" ""',('"model_source" "'+$provenance+'"')))}
 Check {Assert-PreservedSetupSettingsDelta $configured ($changed.Replace('"model_source" ""',('"model_source" "'+$provenance+'"')))}
 Refuse {Assert-PreservedSetupSettingsDelta $configured ($changed.Replace('"model_source" ""','"model_source" "invented source"'))}
 $before=Join-Path $root 'before.ini';$after=Join-Path $root 'after.ini'
 $prefix="[Settings]`r`nPersistActiveRoute=0`r`n[Settings/NMEADataSource]`r`nDataConnections=0;0;;0;1;COM8;115200;0;0;0;;0;;0;0;1;0;1;Fixture;0;;0`r`n"
 $original=$prefix+"[Settings/GlobalState]`r`nS52_MAR_SAFETY_CONTOUR=3`r`n[OpenNav]`r`nAlphaSettings=$configured`r`n"
 $updated=$prefix+"[Settings/GlobalState]`r`nS52_MAR_SAFETY_CONTOUR=4`r`n[OpenNav]`r`nAlphaSettings=$changed`r`nBoatSetupV1=v1|complete`r`nDisplayPreferencesV1=v1|125|chart`r`nVesselName=M/S Test`r`n"
 [IO.File]::WriteAllText($before,$original,$encoding);[IO.File]::WriteAllText($after,$updated,$encoding)
 $review=Review $before $after
 Check {$null=Assert-SessionPreservationReview $before $after $review}
 # Positive result is only preservation, and the one-byte inverse still holds.
 Check {$bytes=[IO.File]::ReadAllBytes($after);$out=Get-CommissioningOutputBytes $bytes;if((Get-CommissioningHash (Get-CommissioningInputBytes $out)) -cne (Get-Digest $after)){throw 'Settings changed during direction inverse'}}
 $review.launchPermission=$true;Refuse {Assert-SessionPreservationReview $before $after $review};$review.launchPermission=$false
 $review.changes=$review.changes[1..($review.changes.Count-1)];Refuse {Assert-SessionPreservationReview $before $after $review}
 [IO.File]::WriteAllText($after,$updated.Replace('BoatSetupV1=v1|complete','BoatSetupV1=v2|complete'),$encoding)
 Refuse {Assert-SessionPreservationReview $before $after (Review $before $after)}
 [IO.File]::WriteAllText($before,$updated,$encoding);[IO.File]::WriteAllText($after,$updated.Replace("BoatSetupV1=v1|complete`r`n",''),$encoding)
 Refuse {Assert-SessionPreservationReview $before $after (Review $before $after)}
 Write-Output ('Setup/profile preservation: '+$checks+' focused checks passed; no launch permission.')
}finally{Remove-Item -LiteralPath $root -Recurse -Force}

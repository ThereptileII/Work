# TEST ONLY identity adaptation for a native scheduled task, which intentionally
# does not inherit worker-process environment variables. No production entrypoint
# or task/process/pipe/audit function body is changed by this helper.
function Enable-ScheduledBrokerFixture($Fixture,[string]$SourceTools) {
 if($env:GITHUB_ACTIONS -cne 'true' -or [Environment]::OSVersion.Platform -ne 'Win32NT'){throw 'Scheduled broker fixture is native CI only.'}
 $root=Assert-LocalPath $Fixture.root
 $temp=[IO.Path]::GetFullPath([IO.Path]::GetTempPath()).TrimEnd('\')
 if(-not $root.StartsWith($temp+'\OpenNav broker fixture ',[StringComparison]::OrdinalIgnoreCase)){throw 'Scheduled fixture escaped marked TEMP tree.'}
 $identity=Join-Path $Fixture.scripts 'fixture.identity.json'
 if((Get-Digest $identity) -cne $Fixture.identityHash){throw 'Scheduled fixture identity changed.'}
 $adapter=Join-Path $Fixture.scripts 'BrokerFixtureIdentity.ps1'
 $body=[IO.File]::ReadAllText($adapter)
 # This is a copied TEST adapter, not a production bypass. Its immutable digest
 # and all temporary paths are verified again on every native task invocation.
 foreach($pair in @(@('$env:GITHUB_ACTIONS',"'true'"),@('$env:OPENNAV_BROKER_FIXTURE_IDENTITY_SHA256',("'"+$Fixture.identityHash+"'")))) {
  if(-not $body.Contains($pair[0])){throw 'Expected test adapter identity boundary changed.'}
  $body=$body.Replace($pair[0],$pair[1])
 }
 [IO.File]::WriteAllText($adapter,$body,(New-Object Text.UTF8Encoding($false)))
 $common=Join-Path $Fixture.scripts 'Common.ps1';$body=[IO.File]::ReadAllText($common)
 if([regex]::Matches($body,[regex]::Escape('$env:LOCALAPPDATA')).Count -ne 2){throw 'Expected copied known-folder identity check changed.'}
 $body=$body.Replace('$env:LOCALAPPDATA','$script:BrokerFixture.localData')
 [IO.File]::WriteAllText($common,$body,(New-Object Text.UTF8Encoding($false)))
 $proof=@()
 foreach($name in @('RestartCommissioningPrepare.ps1','RestartCommissioningArm.ps1','RestartCommissioningBroker.ps1')) {
  $original=Get-Digest (Join-Path $SourceTools $name);$copied=Get-Digest (Join-Path $Fixture.scripts $name)
  if($original -cne $copied){throw 'Actual Prepare/Arm/Collect/Broker source was modified.'}
  $proof+=@{name=$name;sourceSha256=$original;copiedSha256=$copied}
 }
 return ,$proof
}

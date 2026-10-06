# Inert temporary records only; no installed application, equipment or network.
[CmdletBinding()]
param()
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'
. (Join-Path (Split-Path $PSScriptRoot -Parent) 'installer/windows/UpdateSupervisor.ps1')
function Check([bool]$Value,[string]$Message){if(-not $Value){throw $Message}}
function Reject([scriptblock]$Action){$denied=$false;try{& $Action}catch{$denied=$true};Check $denied 'Expected refusal.'}
function Json([string]$Path,$Value){[IO.File]::WriteAllText($Path,($Value|ConvertTo-Json))}
$root=Join-Path ([IO.Path]::GetTempPath()) ('bootstrap-expectation-'+[guid]::NewGuid().ToString('N'))
$held=$null
try {
 $id='a'*32;$expected=$id+':absent'
 $null=New-Item -ItemType Directory -Path (Join-Path $root ('generations/'+$id+'/app')) -Force
 $null=New-Item -ItemType Directory -Path (Join-Path $root 'known-good')
 Json (Join-Path $root 'owner.json') @{owner='OpenNavX.Alpha1.SideBySide.1'}
 $state=@{owner='OpenNavX.Alpha1.SideBySide.1';schema=1;current=$id;previous=''}
 Json (Join-Path $root 'state.json') $state
 Reject {Assert-UpdateBootstrapExpectation $root $expected $null}
 $held=[IO.File]::Open((Join-Path $root 'transaction.lock'),[IO.FileMode]::OpenOrCreate,[IO.FileAccess]::ReadWrite,[IO.FileShare]::None)
 Assert-UpdateBootstrapExpectation $root $expected $held
 foreach($bad in @('', $id,($id+':'),($id+':ABSENT'),($id+':'+('b'*64)))){Reject {Assert-UpdateBootstrapExpectation $root $bad $held}}
 foreach($relative in @('update-pending.json',('known-good/'+$id+'.receipt'),('generations/'+$id+'/app/update-trust.json'))){
  $path=Join-Path $root $relative
  [IO.File]::WriteAllText($path,'appeared during handoff; malformed still refuses')
  Reject {Assert-UpdateBootstrapExpectation $root $expected $held}
  Remove-Item -LiteralPath $path
 }
 $state.current='b'*32;Json (Join-Path $root 'state.json') $state
 Reject {Assert-UpdateBootstrapExpectation $root $expected $held}
 $state.current=$id;Json (Join-Path $root 'state.json') $state
 Assert-UpdateBootstrapExpectation $root $expected $held
 $held.Dispose();$held=$null
 Reject {Invoke-UpdateSupervision $root 'QualifyCurrent' -BootstrapExpectation ''}
 Reject {Invoke-UpdateSupervision $root 'RecoverPending' -BootstrapExpectation $expected}
 Write-Output 'PASS bootstrap expectation: real exclusive lock, unchanged absence, receipt/trust/pending creation, generation drift and malformed request refusal.'
 if([Environment]::OSVersion.Platform -eq [PlatformID]::Win32NT){
  # Execute the real supervisor orchestration. Only health/session and spawn are
  # inert doubles; mutate records AFTER generation resolution/lock acquisition.
  $script:starts=0
  function Get-SupervisedGeneration($Root,$Generation){return @{identity=@{generation=$Generation}}}
  function Start-SupervisedGeneration($Generation,$Session){$script:starts++;throw 'No application may start in a refusal fixture.'}
  function New-UpdateHealthSession($Identity){
   & $script:race
   return @{server=(New-Object IO.MemoryStream)}
  }
  foreach($relative in @('update-pending.json',('known-good/'+$id+'.receipt'),('generations/'+$id+'/app/update-trust.json'))){
   $script:race={ [IO.File]::WriteAllText((Join-Path $root $relative),'late mutation') }
   Reject {Invoke-UpdateSupervision $root 'QualifyCurrent' -BootstrapExpectation $expected}
   Check ($script:starts -eq 0) 'Changed expectation reached application creation.'
   Remove-Item -LiteralPath (Join-Path $root $relative)
  }
  $script:race={$state.current='b'*32;Json (Join-Path $root 'state.json') $state}
  Reject {Invoke-UpdateSupervision $root 'QualifyCurrent' -BootstrapExpectation $expected}
  Check ($script:starts -eq 0) 'Changed generation reached application creation.'
  Write-Output 'PASS native supervisor handoff: late receipt/trust/pending/generation mutations refuse before spawn.'
 } else {Write-Output 'PENDING native Windows supervisor orchestration; portable contracts do not qualify process creation.'}
} finally {
 if($held){$held.Dispose()}
 if(Test-Path -LiteralPath $root){Remove-Item -LiteralPath $root -Recurse -Force}
}

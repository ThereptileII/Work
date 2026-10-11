# Inert owned TEMP installation only; never opens a real profile or application.
[CmdletBinding()]
param([switch]$IsolatedLocal)
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'
$native=[Environment]::OSVersion.Platform -eq [PlatformID]::Win32NT
if($native -and $env:GITHUB_ACTIONS -cne 'true' -and -not $IsolatedLocal){throw 'Native execution requires CI or explicit isolated local testing.'}
. (Join-Path $PSScriptRoot 'SignedUpdateSessionStore.ps1')
$checks=0;$fixture=Join-Path ([IO.Path]::GetTempPath()) ('signed-installed-store-'+[guid]::NewGuid().ToString('N'))
$null=[IO.Directory]::CreateDirectory($fixture)
function Check([bool]$Value,[string]$Message){if(-not $Value){throw $Message};$script:checks++}
function Reject([scriptblock]$Call,[string]$Message){$failed=$false;try{$null=& $Call}catch{$failed=$true};Check $failed $Message}
function Hash([int]$N){return $N.ToString('x64')}
function Get-SignedStoreNow {return 1000L}
$binding=[pscustomobject]@{session=(Hash 1);previous=[pscustomobject]@{generation=('a'*32);commit=('a'*40);packageSha256=('a'*64);executableSha256=('c'*64)};release=[pscustomobject]@{commit=('b'*40);packageSha256=('b'*64);executableSha256=('d'*64);policySha256=(Hash 2);sourceReviewSha256=(Hash 3)};previousCommissioningSha256=(Hash 4);fallbackSourceReviewSha256=(Hash 5);createdUnix=1000;expiresUnix=4000}
function WriteJson([string]$Path,$Value){[IO.File]::WriteAllBytes($Path,(Get-SignedStoreBytes $Value))}
function Fresh {
  $root=Join-Path $script:fixture ([guid]::NewGuid().ToString('N'))
  $null=[IO.Directory]::CreateDirectory($root)
  WriteJson (Join-Path $root 'owner.json') @{owner='OpenNavX.Alpha1.SideBySide.1'}
  WriteJson (Join-Path $root 'state.json') @{schema=1;owner='OpenNavX.Alpha1.SideBySide.1';current=$script:binding.previous.generation}
  $journal=Join-Path $root 'private-journal';$null=[IO.Directory]::CreateDirectory($journal)
  if($script:native){
    $sid=[Security.Principal.WindowsIdentity]::GetCurrent().User
    $acl=[Security.AccessControl.DirectorySecurity]::new();$acl.SetOwner($sid);$acl.SetAccessRuleProtection($true,$false)
    foreach($principal in @($sid,[Security.Principal.SecurityIdentifier]::new('S-1-5-18'))){
      $acl.AddAccessRule([Security.AccessControl.FileSystemAccessRule]::new($principal,[Security.AccessControl.FileSystemRights]::FullControl,([Security.AccessControl.InheritanceFlags]::ContainerInherit -bor [Security.AccessControl.InheritanceFlags]::ObjectInherit),[Security.AccessControl.PropagationFlags]::None,[Security.AccessControl.AccessControlType]::Allow))
    }
    Set-Acl -LiteralPath $journal -AclObject $acl
  }
  return @{root=$root;journal=$journal;marker=(Join-Path $root 'signed-update-armed.jsonl')}
}
$write=${function:Write-SignedStoreFrame}
try {
  if(-not $native){
    $f=Fresh
    Reject {New-SignedUpdateStore $f.journal (Get-SignedStoreBytes $binding) -InstallationRoot $f.root} 'Native owner required before installed arming'
    Check (-not [IO.File]::Exists($f.marker)) 'Portable production refusal leaves root unchanged'
    # Only unsupported Windows handle/ACL operations are substituted. The real
    # parser, lock sharing, durable writes, ordering and store lifecycle execute.
    function Get-SignedStoreOwner {return [pscustomobject]@{sid='fixture';pid=$PID;createdFiletime='fixture';sessionId=1;image='inert';imageSha256=(Hash 99)}}
    function Enter-SignedInstallationRoot([string]$Root){
      if(-not $Root.StartsWith($script:fixture+[IO.Path]::DirectorySeparatorChar,[StringComparison]::Ordinal) -or -not [IO.Directory]::Exists($Root)){throw 'Fixture root escaped'}
      return ,@()
    }
    function Open-SignedInstallationLock([string]$Root){return [IO.File]::Open((Join-Path $Root 'transaction.lock'),[IO.FileMode]::OpenOrCreate,[IO.FileAccess]::ReadWrite,[IO.FileShare]::None)}
    function Assert-SignedInstallationHandle($Stream,[string]$Sid=''){if($Stream.SafeFileHandle.IsClosed){throw 'Closed fixture handle'}}
    function Enter-SignedStoreDirectory([string]$Directory,$Owner){if(-not [IO.Directory]::Exists($Directory)){throw 'Missing private fixture directory'};return ,@()}
    function New-SignedStoreFile([string]$Path,[string]$Sid){return [IO.FileStream]::new($Path,[IO.FileMode]::CreateNew,[IO.FileAccess]::ReadWrite,[IO.FileShare]::None,4096,[IO.FileOptions]::WriteThrough)}
    function Assert-SignedStoreNative($Store){if($Store.stream.SafeFileHandle.IsClosed){throw 'Closed fixture journal'};if($Store.installationBarrier){Assert-SignedInstallationBarrier $Store.installationBarrier}}
  }
  $f=Fresh;$h=New-SignedUpdateStore $f.journal (Get-SignedStoreBytes $binding) -InstallationRoot $f.root -OperationEvents
  Check ([IO.File]::Exists($f.marker)) 'Installed arming publishes root denial marker'
  Check ([IO.File]::Exists((Join-Path $f.journal 'signed-update-armed.jsonl'))) 'Private journal remains separate'
  $s=Get-SignedStoreLive $h
  Check ($null -ne $s.operations -and $s.installationBarrier.root -ceq $f.root) 'Live owner retains installed barrier and operation state'
  $null=Assert-SignedStoreCurrent $s;Check $true 'Current journal also verifies held installed marker'
  $lock=Open-SignedInstallationLock $f.root;$lock.Dispose();Check $true 'Publication releases maintenance lock after durable arming'
  Reject {New-SignedUpdateStore $f.journal (Get-SignedStoreBytes $binding) -InstallationRoot $f.root} 'Live installed marker cannot be reopened'
  Close-SignedUpdateStore $h
  Check ([IO.File]::Exists($f.marker)) 'Owner close leaves installed denial marker'
  Reject {New-SignedUpdateStore $f.journal (Get-SignedStoreBytes $binding) -InstallationRoot $f.root} 'Abandoned marker cannot be resumed'
  foreach($contents in @('','corrupt','{"status":"healthy"}')){
    $f=Fresh;[IO.File]::WriteAllText($f.marker,$contents)
    Reject {New-SignedUpdateStore $f.journal (Get-SignedStoreBytes $binding) -InstallationRoot $f.root} 'Any preexisting marker refuses without interpreting contents'
    Check ([IO.File]::ReadAllText($f.marker) -ceq $contents) 'Preexisting marker bytes preserved'
    Check (-not [IO.File]::Exists((Join-Path $f.journal 'signed-update-armed.jsonl'))) 'Marker refusal precedes journal creation'
  }
  $f=Fresh;$null=[IO.Directory]::CreateDirectory($f.marker)
  Reject {New-SignedUpdateStore $f.journal (Get-SignedStoreBytes $binding) -InstallationRoot $f.root} 'Directory marker also refuses'
  $f=Fresh;$lock=Open-SignedInstallationLock $f.root
  try {Reject {New-SignedUpdateStore $f.journal (Get-SignedStoreBytes $binding) -InstallationRoot $f.root} 'Existing maintenance lock prevents arming'} finally {$lock.Dispose()}
  Check (-not [IO.File]::Exists($f.marker)) 'Locked refusal creates no marker'
  $f=Fresh;WriteJson (Join-Path $f.root 'state.json') @{schema=1;owner='OpenNavX.Alpha1.SideBySide.1';current=('b'*32)}
  Reject {New-SignedUpdateStore $f.journal (Get-SignedStoreBytes $binding) -InstallationRoot $f.root} 'Different selected generation refuses arming'
  Check (-not [IO.File]::Exists($f.marker)) 'Invalid installation stays unarmed'
  $f=Fresh;[IO.File]::WriteAllText((Join-Path $f.root 'owner.json'),'{"owner":"bad","Owner":"OpenNavX.Alpha1.SideBySide.1"}')
  Reject {New-SignedUpdateStore $f.journal (Get-SignedStoreBytes $binding) -InstallationRoot $f.root} 'Ambiguous installed owner JSON refuses'
  Check (-not [IO.File]::Exists($f.marker)) 'Ambiguous identity creates no marker'
  $f=Fresh
  function Write-SignedStoreFrame($Store,[byte[]]$Bytes){if($Store.path -ceq $script:f.marker){throw 'Injected root marker write failure'};& $script:write $Store $Bytes}
  Reject {New-SignedUpdateStore $f.journal (Get-SignedStoreBytes $binding) -InstallationRoot $f.root} 'Root write failure refuses'
  Check ([IO.File]::Exists($f.marker)) 'Partial root marker retained after write failure'
  Check (-not [IO.File]::Exists((Join-Path $f.journal 'signed-update-armed.jsonl'))) 'Root barrier must persist before journal creation'
  ${function:Write-SignedStoreFrame}=$write
  $f=Fresh
  function Write-SignedStoreFrame($Store,[byte[]]$Bytes){if($Store.path -cne $script:f.marker){throw 'Injected journal write failure'};& $script:write $Store $Bytes}
  Reject {New-SignedUpdateStore $f.journal (Get-SignedStoreBytes $binding) -InstallationRoot $f.root} 'Private journal write failure refuses'
  Check ([IO.File]::Exists($f.marker)) 'Installed denial retained after failed journal arming'
  Check ([IO.File]::Exists((Join-Path $f.journal 'signed-update-armed.jsonl'))) 'Failed private journal retained'
  $lock=Open-SignedInstallationLock $f.root;$lock.Dispose();Check $true 'Failed arming releases transaction lock without erasing denial'
  ${function:Write-SignedStoreFrame}=$write
  Check ($script:SignedUpdateStores.Count -eq 0) 'Failed/closed sessions retain no live handle authority'
  [pscustomobject]@{status='passed';count=$checks;native=$native;scope='inert installed barrier and private store only';applicationLaunched=$false;profileAccess=$false} | ConvertTo-Json -Compress
} finally {
  ${function:Write-SignedStoreFrame}=$write
  foreach($handle in @($script:SignedUpdateStores.Keys)){try{Close-SignedUpdateStore $handle}catch{}}
  Remove-Item -LiteralPath $fixture -Recurse -Force
}

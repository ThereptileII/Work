# Disposable byte/filesystem contracts. Native CI is authoritative for Windows.
[CmdletBinding()]
param([switch]$PortableContracts,[switch]$IsolatedLocal)
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'
$native=[Environment]::OSVersion.Platform -eq 'Win32NT'
if (-not $native -and -not $PortableContracts) { throw 'Native Windows required unless explicitly running portable contracts.' }
if ($native -and -not $IsolatedLocal -and $env:GITHUB_ACTIONS -ne 'true') { throw 'Use disposable CI or explicitly choose isolated temporary-file tests.' }
. (Join-Path $PSScriptRoot 'Preparation.ps1')
$testRoot=Join-Path ([IO.Path]::GetTempPath()) ('OpenNav preparation '+[guid]::NewGuid().ToString('N'))
$null=New-Item -ItemType Directory -Path $testRoot
if (-not $native) {
  # Only path syntax is substituted. The actual hashing, parsing, copy, flush,
  # journal and atomic-replace implementations run unchanged. Never a release gate.
  function Assert-LocalPath([string]$Path) {
    $full=[IO.Path]::GetFullPath($Path)
    if ($full -ne $testRoot -and -not $full.StartsWith($testRoot+'/',[StringComparison]::Ordinal)) { throw 'Portable test path escaped its disposable root.' }
    $walk=$full
    while ($walk -ne $testRoot) {
      if ((Test-Path -LiteralPath $walk) -and ((Get-Item -LiteralPath $walk -Force).Attributes -band [IO.FileAttributes]::ReparsePoint)) { throw 'Reparse path refused.' }
      $walk=[IO.Path]::GetDirectoryName($walk)
    }
    return $full
  }
}
$checks=New-Object 'Collections.Generic.List[string]'
$ini=$null;$aclBefore=$null;$sddl=$null;$metadataBefore=$null
function Reject([scriptblock]$Action,[string]$Reason) {
  $rejected=$false
  try { $null=& $Action } catch { $rejected=$true }
  if (-not $rejected) { throw ('Unsafe operation accepted: '+$Reason) }
}
function HashBytes([byte[]]$Bytes) {
  $sha=[Security.Cryptography.SHA256]::Create()
  try { return ([BitConverter]::ToString($sha.ComputeHash($Bytes))).Replace('-','').ToLowerInvariant() }
  finally { $sha.Dispose() }
}
try {
  # Parse all new entry points on both platforms; never invoke real environment APIs.
  foreach ($name in @('Preparation.ps1','backup-managed-plugins.ps1','recover-zero-profile.ps1')) {
    $tokens=$null;$errors=$null
    $null=[Management.Automation.Language.Parser]::ParseFile((Join-Path $PSScriptRoot $name),[ref]$tokens,[ref]$errors)
    if ($errors.Count) { throw ($errors | Out-String) }
  }
  $checks.Add('Both native entry points and shared primitives parse without executing environment changes')
  $unknown=Join-Path $testRoot 'unknown.exe';[IO.File]::WriteAllBytes($unknown,[byte[]]@(77,90,0,0))
  Reject { Assert-PreparationStock ([pscustomobject]@{executable=$unknown}) } 'unknown stock build'
  $checks.Add('Unknown stock binaries fail before any preparation mutation')
  $source=Join-Path $testRoot 'plugin library';$copy=Join-Path $testRoot 'backup'
  $null=New-Item -ItemType Directory -Path $source,(Join-Path $source 'empty'),(Join-Path $source 'nested'),(Join-Path $source 'cache/tarballs'),(Join-Path $source 'install_data')
  [IO.File]::WriteAllBytes((Join-Path $source 'nested/control_pi.dll'),[byte[]]@(1,2,3))
  [IO.File]::WriteAllText((Join-Path $source 'cache/tarballs/private & cache.tar.gz'),'archive fixture')
  [IO.File]::WriteAllText((Join-Path $source 'install_data/plugin.version'),'2.0')
  [IO.File]::WriteAllBytes((Join-Path $source 'empty.dat'),[byte[]]@())
  $snapshot=Get-PreparationTree $source
  Copy-PreparationTree $snapshot $copy
  if (@($snapshot.entries | Where-Object {-not $_.directory}).Count -ne 4 -or -not (Test-Path -LiteralPath (Join-Path $copy 'empty'))) { throw 'Complete nested/private/cache/empty tree copy failed.' }
  Assert-PreparationTree $snapshot
  $checks.Add('Complete recursive plugin/helper/cache/metadata copy preserves bytes, empty files and empty directories')
  Reject { Copy-PreparationTree $snapshot $copy } 'existing backup overwrite'
  Reject { Copy-PreparationTree $snapshot (Join-Path $source 'nested-backup') } 'nested copy'
  Reject { Assert-PreparationSeparate $source @((Join-Path $source 'nested')) } 'parent source overlap'
  $checks.Add('Cold copy refuses overlapping roots and existing destinations')
  $missing=Get-PreparationTree (Join-Path $testRoot 'missing')
  Copy-PreparationTree $missing (Join-Path $testRoot 'missing-copy')
  if ($missing.exists -or (Test-Path -LiteralPath (Join-Path $testRoot 'missing-copy'))) { throw 'Absent tree was fabricated.' }
  $checks.Add('Absent managed roots stay explicitly absent')
  [IO.File]::AppendAllText((Join-Path $source 'install_data/plugin.version'),' changed')
  Reject { Assert-PreparationTree $snapshot } 'changed content'
  Reject { Copy-PreparationTree $snapshot (Join-Path $testRoot 'changed-copy') } 'changed source'
  if (Test-Path -LiteralPath (Join-Path $testRoot 'changed-copy')) { throw 'Changed source created backup content.' }
  $checks.Add('Source changes invalidate the entire inventory before publication')
  $snapshot=Get-PreparationTree $source
  $extra=Join-Path $source 'new.dat';[IO.File]::WriteAllText($extra,'new')
  Reject { Assert-PreparationTree $snapshot } 'new file'
  Remove-Item -LiteralPath $extra
  Remove-Item -LiteralPath (Join-Path $source 'empty')
  Reject { Assert-PreparationTree $snapshot } 'removed directory'
  $checks.Add('Before/after inventories detect added files and removed directories')
  $outside=Join-Path $testRoot 'outside';$null=New-Item -ItemType Directory -Path $outside
  $link=Join-Path $source 'redirect'
  if ($native) { $null=New-Item -ItemType Junction -Path $link -Target $outside }
  else { $null=New-Item -ItemType SymbolicLink -Path $link -Target $outside }
  Reject { Get-PreparationTree $source } 'redirected child'
  Reject { Get-PreparationTree $link } 'redirected root'
  if ($native) { [IO.Directory]::Delete($link) } else { Remove-Item -LiteralPath $link }
  $checks.Add('Reparse directories are refused before recursive enumeration')
  $local=Join-Path $testRoot 'local';$null=New-Item -ItemType Directory -Path $local
  $sid='S-1-5-21-1-2-3-1001';$explorer=[pscustomobject]@{sid=$sid;session=1}
  Assert-PreparationIdentity $sid $local $local @($explorer)
  Reject { Assert-PreparationIdentity $sid $local $local @() } 'no interactive owner'
  Reject { Assert-PreparationIdentity $sid $local $local @($explorer,$explorer) } 'multiple desktops'
  Reject { Assert-PreparationIdentity 'S-1-5-21-1-2-3-1002' $local $local @($explorer) } 'other SID'
  Reject { Assert-PreparationIdentity $sid $local (Join-Path $testRoot 'other') @($explorer) } 'different environment'
  Reject { Assert-PreparationIdentity $sid $local $local @([pscustomobject]@{sid=$sid;session=0}) } 'noninteractive session'
  $checks.Add('Interactive SID, unique desktop, session and LOCALAPPDATA identity must agree')
  Assert-PreparationProcesses @([pscustomobject]@{Name='explorer.exe';ExecutablePath='C:\Windows\explorer.exe'}) @('C:\plugins')
  foreach ($name in @('opencpn.exe','oeserverd.exe','rtl_ais.exe','AIS-catcher.exe')) {
    Reject { Assert-PreparationProcesses @([pscustomobject]@{Name=$name;ExecutablePath=$null}) @('C:\plugins') } 'known live helper'
  }
  Reject { Assert-PreparationProcesses @([pscustomobject]@{Name='renamed.exe';ExecutablePath='C:\plugins\vendor\renamed.exe'}) @('C:\plugins') } 'unfamiliar helper below plugin root'
  $checks.Add('OpenCPN and known or path-owned plugin helpers must be closed; no process is terminated')
  $valid=[Text.Encoding]::UTF8.GetBytes("[Settings]`nPersistActiveRoute=0`n[Settings/NMEADataSource]`nDataConnections=preserved-output-setting`n")
  $zeros=New-Object byte[] $valid.Length
  $zh=HashBytes $zeros;$vh=HashBytes $valid
  Assert-ExactRecoveryBytes $zeros $valid $zh $vh $valid.Length
  Reject { Assert-ExactRecoveryBytes $zeros $valid $zh ('0'*64) $valid.Length } 'candidate hash'
  Reject { Assert-ExactRecoveryBytes $valid $valid $vh $vh $valid.Length } 'nonzero original'
  Reject { Assert-ExactRecoveryBytes $zeros $valid $zh $vh ($valid.Length+1) } 'length'
  $checks.Add('Recovery validates both hashes, exact lengths and every original zero byte')
  $ini=Join-Path $testRoot 'opencpn.ini';$tmp=Join-Path $testRoot 'opencpn.ini~fixture.TMP'
  [IO.File]::WriteAllBytes($ini,$zeros);[IO.File]::WriteAllBytes($tmp,$valid)
  Reject { Assert-AuthorizedRecovery $ini $tmp } 'synthetic pair cannot bypass the hard-coded real authorization'
  $checks.Add('Public recovery policy refuses alternative fixture hashes and lengths')
  $parsed=Read-ProfileForAudit $tmp
  if ($parsed['Settings/NMEADataSource/DataConnections'] -cne 'preserved-output-setting') { throw 'Parser altered connection values.' }
  foreach ($bad in @("[Settings]`na=1`na=2", "[Settings]`na=1`n[Settings]`nb=2", "[Settings]`na=`0", "orphan=1", "[Other]`na=1")) {
    [IO.File]::WriteAllText($tmp,$bad)
    Reject { Read-ProfileForAudit $tmp } 'ambiguous/corrupt candidate'
  }
  [IO.File]::WriteAllBytes($tmp,[byte[]]@(0xff,0xfe,0xff))
  Reject { Read-ProfileForAudit $tmp } 'malformed UTF8'
  [IO.File]::WriteAllBytes($tmp,$valid)
  $checks.Add('Strict candidate parser rejects ambiguous/corrupt syntax and retains connection values unchanged')
  $saved=Join-Path $testRoot 'candidate-saved.ini';$originalSaved=Join-Path $testRoot 'original-saved.ini'
  Copy-PreparationFile $tmp $saved $vh $valid.Length
  Copy-PreparationFile $ini $originalSaved $zh $valid.Length
  $journal=Join-Path $testRoot 'applying.json'
  Reject { Publish-PreparedProfile $ini $saved $zh $vh $valid.Length $journal } 'missing durable journal'
  if ((Get-Digest $ini) -cne $zh) { throw 'Missing journal mutated original.' }
  Write-Record $journal @{owner='OpenNavX.Preparation.ContractTest';status='applying';sourceSha256=$zh;targetSha256=$vh}
  Reject { Write-Record $journal @{status='overwrite'} } 'journal overwrite'
  $checks.Add('Replacement requires a durable, non-overwritable apply journal and preserves both source backups')
  $stageName=$ini+'.opennav-recovery-'+('a'*32)+'.partial'
  Assert-PreparationStagePath $ini $stageName
  foreach($wrong in @($ini,$saved,$tmp,($ini+'.partial'),($ini+'.opennav-recovery-not-a-guid.partial'),(Join-Path $source ([IO.Path]::GetFileName($stageName))))) {
    Reject {Assert-PreparationStagePath $ini $wrong} 'only exact owned same-directory staging path may receive permissions'
  }
  $checks.Add('Staging permission writes cannot target original, saved candidate, TMP or another directory/name')
  Initialize-PreparationNative
  if(-not ('OpenNavX.PreparationNative' -as [type])){throw 'Native publication helper did not compile.'}
  $checks.Add('Native rename interop compiles; only Windows tests invoke its fixed replace-existing/write-through flags')
  foreach($attributes in @(2,4,32,128,8192,0x2026)){Assert-PreparationAttributes $attributes}
  foreach($attributes in @(0,1,3,0x10,0x40,0x82,0x100,0x200,0x400,0x800,0x1000,0x4000,0x8000,0x10000,-1)) {
    Reject {Assert-PreparationAttributes $attributes} 'read-only/directory/specialized/unrecognized metadata'
  }
  $checks.Add('Only ordinary profile attributes are supported; read-only and specialized metadata fail closed')
  # Valid self-relative fixture: owner BA, group BU, one inherited SY allow ACE.
  # Tests use no real SID, ACL mutation or Windows resource API on Linux.
  [byte[]]$descriptor=@(
    1,0,4,128,20,0,0,0,36,0,0,0,0,0,0,0,52,0,0,0,
    1,2,0,0,0,0,0,5,32,0,0,0,32,2,0,0,
    1,2,0,0,0,0,0,5,32,0,0,0,33,2,0,0,
    2,0,28,0,1,0,0,0,0,16,20,0,255,1,31,0,
    1,1,0,0,0,0,0,5,18,0,0,0)
  Assert-PreparationAclBytes $descriptor $descriptor
  $marked=[byte[]]$descriptor.Clone();$marked[3]=$marked[3] -bor 4
  Assert-PreparationAclBytes $descriptor $marked -AllowDaclAutoInherited
  Reject {Assert-PreparationAclBytes $descriptor $marked} 'pre-apply does not ignore any marker'
  $checks.Add('Post-replacement comparison permits only DACL AutoInherited metadata; pre-apply comparison is strict')
  for($index=0;$index -lt $descriptor.Length;$index++) {
    foreach($bit in @(1,2,4,8,16,32,64,128)) {
      if($index -eq 3 -and $bit -eq 4){continue}
      $changed=[byte[]]$descriptor.Clone();$changed[$index]=$changed[$index] -bxor $bit
      Reject {Assert-PreparationAclBytes $descriptor $changed -AllowDaclAutoInherited} 'every non-exempt owner/group/header/control/ACL/ACE bit'
    }
  }
  Reject {Assert-PreparationAclBytes $descriptor $descriptor[0..18] -AllowDaclAutoInherited} 'truncated descriptor'
  $checks.Add('All 639 other descriptor bits, including owner/group/protection/ACE rights and inheritance, remain exact')
  if($native) {
    $sddl='O:BAG:BUD:(A;ID;FA;;;SY)(A;ID;FA;;;BA)'
    Assert-PreparationAcl $sddl ($sddl.Replace('D:','D:AI')) -AllowDaclAutoInherited
    Reject {Assert-PreparationAcl $sddl ($sddl.Replace('D:','D:AI'))} 'native strict precondition'
    Assert-PreparationAcl $sddl ($sddl.Replace('O:BA','O:S-1-5-32-544').Replace(';FA;',';0x1f01ff;'))
    $checks.Add('Native SDDL parsing accepts identical SID/rights spellings and observed D to D:AI metadata normalization')
    foreach($changed in @(
      'O:SYG:BUD:AI(A;ID;FA;;;SY)(A;ID;FA;;;BA)',
      'O:BAG:SYD:AI(A;ID;FA;;;SY)(A;ID;FA;;;BA)',
      'O:BAG:BUD:PAI(A;ID;FA;;;SY)(A;ID;FA;;;BA)',
      'O:BAG:BUD:ARAI(A;ID;FA;;;SY)(A;ID;FA;;;BA)',
      'O:BAG:BUD:AI(A;ID;FR;;;SY)(A;ID;FA;;;BA)',
      'O:BAG:BUD:AI(D;ID;FA;;;SY)(A;ID;FA;;;BA)',
      'O:BAG:BUD:AI(A;CIID;FA;;;SY)(A;ID;FA;;;BA)',
      'O:BAG:BUD:AI(A;;FA;;;SY)(A;ID;FA;;;BA)',
      'O:BAG:BUD:AI(A;ID;FA;;;BA)(A;ID;FA;;;SY)',
      'O:BAG:BUD:AI(A;ID;FA;;;SY)',
      'O:BAG:BUD:AI(A;ID;FA;;;SY)(A;ID;FA;;;BA)(A;ID;FR;;;BU)',
      'O:BAG:BUD:AI(A;ID;FA;;;SY)(A;ID;FA;;;BA)S:(AU;SA;FA;;;WD)')) {
      $null=Convert-PreparationAcl $changed # Each negative must be valid SDDL.
      Reject {Assert-PreparationAcl $sddl $changed -AllowDaclAutoInherited} 'changed owner/group/protection/required inheritance/ACE type/order/rights/flags/count or SACL'
    }
    Reject {Assert-PreparationAcl $sddl 'not an SDDL' -AllowDaclAutoInherited} 'invalid SDDL'
    $checks.Add('Native semantic ACL comparison rejects owner/group/protection, ordered ACE and SACL changes')
  }
  if ($native) {
    $aclBefore=(Get-Acl -LiteralPath $ini).Sddl
    $metadataBefore=Get-PreparationNativeMetadata $ini
    # Reproduce the decisive Server behavior even on desktop Windows: the audit
    # query reports DACL ACEs without ID, but the ordinary query and actual file
    # retain ID. Audit DACLs must never become a permission-copy baseline.
    $scopeStage=$ini+'.opennav-recovery-'+[guid]::NewGuid().ToString('N')+'.partial'
    Copy-PreparationFile $saved $scopeStage $vh $valid.Length
    $ordinaryDescriptor=New-Object Security.AccessControl.RawSecurityDescriptor($aclBefore)
    # RawAcl's pipeline adapter differs on Windows PowerShell 5.1. Read its
    # indexed entries and compare integer flags without enumerator conversion.
    $inheritedFlag=[int]([Security.AccessControl.AceFlags]::Inherited)
    $inheritedCount=0
    for ($aceIndex=0;$aceIndex -lt $ordinaryDescriptor.DiscretionaryAcl.Count;$aceIndex++) {
      $entryFlags=[int]($ordinaryDescriptor.DiscretionaryAcl[$aceIndex].AceFlags)
      if (($entryFlags -band $inheritedFlag) -ne 0) { $inheritedCount++ }
    }
    if ($inheritedCount -eq 0) { throw 'Regression fixture requires actual inherited access entries.' }
    function Get-Acl {
      param([string]$LiteralPath,[switch]$Audit)
      if (-not $Audit) { return Microsoft.PowerShell.Security\Get-Acl -LiteralPath $LiteralPath }
      $actual=Microsoft.PowerShell.Security\Get-Acl -LiteralPath $LiteralPath -Audit
      $view=New-Object Security.AccessControl.RawSecurityDescriptor($actual.Sddl)
      $inheritedFlag=[int]([Security.AccessControl.AceFlags]::Inherited)
      for ($aceIndex=0;$aceIndex -lt $view.DiscretionaryAcl.Count;$aceIndex++) {
        $ace=$view.DiscretionaryAcl[$aceIndex]
        $entryFlags=[int]($ace.AceFlags)
        $ace.AceFlags=[Security.AccessControl.AceFlags]($entryFlags -band (-bnot $inheritedFlag))
        $view.DiscretionaryAcl[$aceIndex]=$ace
      }
      return [pscustomobject]@{Sddl=$view.GetSddlForm([Security.AccessControl.AccessControlSections]::All)}
    }
    try {
      $separated=Get-PreparationNativeMetadata $ini
      Assert-PreparationAcl $aclBefore $separated.acl
      Reject {Assert-PreparationAcl $separated.acl $separated.auditAcl} 'transformed audit DACL is not equivalent to actual inherited access entries'
      Set-PreparationStageAcl $ini $scopeStage $separated.acl
    } finally { Remove-Item Function:Get-Acl }
    Assert-PreparationAcl $metadataBefore.acl (Get-PreparationAccessAcl $ini)
    Assert-PreparationAcl $metadataBefore.acl (Get-PreparationAccessAcl $scopeStage) -AllowDaclAutoInherited
    if ((Get-Digest $ini) -cne $zh -or (Get-Digest $scopeStage) -cne $vh) { throw 'Separated security-view fixture modified original/stage contents.' }
    $checks.Add('Native staging preserves actual inherited ACEs when audit retrieval presents them as explicit; no ACE flags/order/count are normalized')
    $wrong=New-Object Security.AccessControl.RawSecurityDescriptor($metadataBefore.acl)
    $otherOwner=if ($wrong.Owner.Value -ceq 'S-1-5-18') {'S-1-5-32-544'} else {'S-1-5-18'}
    $wrong.Owner=New-Object Security.Principal.SecurityIdentifier($otherOwner)
    $stageAcl=Get-PreparationAccessAcl $scopeStage
    Reject { Set-PreparationStageAcl $ini $scopeStage ($wrong.GetSddlForm([Security.AccessControl.AccessControlSections]::All)) } 'changed ordinary access owner before staging'
    Assert-PreparationAcl $metadataBefore.acl (Get-PreparationAccessAcl $ini)
    Assert-PreparationAcl $stageAcl (Get-PreparationAccessAcl $scopeStage)
    $checks.Add('Changed ordinary owner is refused before any staging permission write; both original and stage remain exact')
    Remove-Item -LiteralPath $scopeStage
    if ($env:GITHUB_ACTIONS -eq 'true') {
      Write-Host (([pscustomobject]@{scope='disposable-ci-fixture-before-publication';ordinarySddl=$aclBefore;auditedSddl=$metadataBefore.auditAcl;ordinaryFlags=[int](New-Object Security.AccessControl.RawSecurityDescriptor($aclBefore)).ControlFlags;auditedFlags=[int](New-Object Security.AccessControl.RawSecurityDescriptor($metadataBefore.auditAcl)).ControlFlags}) | ConvertTo-Json -Compress)
    }
  }
  Publish-PreparedProfile $ini $saved $zh $vh $valid.Length $journal
  if ((Get-Digest $ini) -cne $vh -or (Get-Digest $tmp) -cne $vh -or (Get-Digest $originalSaved) -cne $zh) { throw 'Atomic replacement did not preserve exact sources.' }
  if ($native) {
    $aclAfter=(Get-Acl -LiteralPath $ini).Sddl
    Assert-PreparationAcl $aclBefore $aclAfter -AllowDaclAutoInherited
    $checks.Add('Windows atomic replacement preserves every permission while allowing only DACL AutoInherited metadata normalization')
    # Exercise a different owner/group/explicit protected DACL from the parent.
    # These objects exist only in this fresh disposable test directory.
    $owned=Join-Path $testRoot 'owned-profile.ini';[IO.File]::WriteAllBytes($owned,$zeros)
    $sid=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value
    $ownedSddl='O:'+$sid+'G:BUD:P(A;;FA;;;'+$sid+')(A;;FA;;;SY)(A;;FR;;;BU)'
    $ownedSecurity=New-Object Security.AccessControl.FileSecurity
    $ownedSections=[Security.AccessControl.AccessControlSections]::Owner -bor [Security.AccessControl.AccessControlSections]::Group -bor [Security.AccessControl.AccessControlSections]::Access
    $ownedSecurity.SetSecurityDescriptorSddlForm($ownedSddl,$ownedSections)
    # FileSecurity orders explicit ACEs canonically before persisting them.
    # Assert the requested principals/rights separately, then use that exact
    # native canonical order as the fixture's immutable replacement baseline.
    $ownedExpected=$ownedSecurity.GetSecurityDescriptorSddlForm($ownedSections)
    $expectedDescriptor=New-Object Security.AccessControl.RawSecurityDescriptor($ownedExpected)
    if($expectedDescriptor.Owner.Value -cne $sid -or $expectedDescriptor.Group.Value -cne 'S-1-5-32-545' -or
        ($expectedDescriptor.ControlFlags -band [Security.AccessControl.ControlFlags]::DiscretionaryAclProtected) -eq 0 -or
        $expectedDescriptor.DiscretionaryAcl.Count -ne 3){throw 'Protected ACL fixture lost its intended owner/group/protection/count.'}
    $desiredRights=@{$sid=0x1f01ff;'S-1-5-18'=0x1f01ff;'S-1-5-32-545'=0x120089}
    foreach($ace in $expectedDescriptor.DiscretionaryAcl) {
      $aceSid=$ace.SecurityIdentifier.Value
      if($ace.AceType -ne [Security.AccessControl.AceType]::AccessAllowed -or $ace.AceFlags -ne [Security.AccessControl.AceFlags]::None -or
          -not $desiredRights.ContainsKey($aceSid) -or $ace.AccessMask -ne $desiredRights[$aceSid]){throw 'Protected ACL fixture changed requested ACE semantics.'}
      $desiredRights.Remove($aceSid)
    }
    if($desiredRights.Count){throw 'Protected ACL fixture omitted an intended principal.'}
    Set-Acl -LiteralPath $owned -AclObject $ownedSecurity
    $ownedBefore=(Get-Acl -LiteralPath $owned).Sddl
    Assert-PreparationAcl $ownedExpected $ownedBefore -AllowDaclAutoInherited
    $creation=[datetime]::SpecifyKind([datetime]'2020-02-03T04:05:06',[DateTimeKind]::Utc)
    [IO.File]::SetCreationTimeUtc($owned,$creation)
    [IO.File]::SetAttributes($owned,[IO.FileAttributes]0x2026)
    $ownedJournal=Join-Path $testRoot 'owned-applying.json';Write-Record $ownedJournal @{owner='OpenNavX.Preparation.ContractTest';status='applying'}
    Publish-PreparedProfile $owned $saved $zh $vh $valid.Length $ownedJournal
    Assert-PreparationAcl $ownedBefore (Get-Acl -LiteralPath $owned).Sddl -AllowDaclAutoInherited
    if((Get-Digest $owned) -cne $vh){throw 'Explicit-permission fixture did not publish exact bytes.'}
    $checks.Add('Native staging preserves a different explicit owner/group and protected ordered permission list')
    if([IO.File]::GetCreationTimeUtc($owned).Ticks -ne $creation.Ticks -or [int][IO.File]::GetAttributes($owned) -ne 0x2026){throw 'Native rename lost creation time or supported file attributes.'}
    $checks.Add('Native replacement preserves original creation UTC and hidden/system/archive/not-content-indexed attributes')
    # A previous run may have legitimately saved a different-length INI. The
    # same helper must support reviewed restoration without broadening hashes.
    $longer=Join-Path $testRoot 'longer-profile.ini';[IO.File]::WriteAllBytes($longer,($zeros+[byte[]]@(0,0,0)))
    $longerHash=Get-Digest $longer
    Publish-PreparedProfile $longer $saved $longerHash $vh $valid.Length $journal
    if((Get-Digest $longer) -cne $vh -or (Get-Item -LiteralPath $longer).Length -ne $valid.Length){throw 'Different-length reviewed restoration failed.'}
    $checks.Add('Reviewed different-length current profile can be atomically restored to exact candidate bytes')
    $special=Join-Path $testRoot 'special-profile.ini';[IO.File]::WriteAllBytes($special,$zeros)
    [IO.File]::SetAttributes($special,[IO.FileAttributes]::ReadOnly)
    try {Reject {Publish-PreparedProfile $special $saved $zh $vh $valid.Length $journal} 'read-only original'}
    finally {[IO.File]::SetAttributes($special,[IO.FileAttributes]::Archive)}
    if((Get-Digest $special) -cne $zh){throw 'Read-only profile was overwritten.'}
    # .NET Framework path validation rejects colon syntax for ADS. Exercise the
    # native filesystem provider on both PowerShell 5.1 and current PowerShell.
    Set-Content -LiteralPath $special -Stream 'retained-metadata' -Value 'metadata-fixture' -Encoding Ascii -NoNewline
    Reject {Publish-PreparedProfile $special $saved $zh $vh $valid.Length $journal} 'named stream original'
    if((Get-Digest $special) -cne $zh -or (Get-Content -LiteralPath $special -Stream 'retained-metadata' -Raw -Encoding Ascii) -cne 'metadata-fixture'){throw 'Refused named stream profile changed.'}
    Remove-Item -LiteralPath $special -Stream 'retained-metadata'
    $hardLink=Join-Path $testRoot 'linked-profile.ini';$null=New-Item -ItemType HardLink -Path $hardLink -Target $special
    Reject {Publish-PreparedProfile $special $saved $zh $vh $valid.Length $journal} 'multiply linked original'
    if((Get-Digest $special) -cne $zh -or (Get-Digest $hardLink) -cne $zh){throw 'Refused hard links changed.'}
    Remove-Item -LiteralPath $hardLink
    $checks.Add('Native read-only, named-stream and hard-linked profiles are refused with all original data intact')
    $auditSecurity=Get-Acl -LiteralPath $special -Audit
    $auditRule=New-Object Security.AccessControl.FileSystemAuditRule((New-Object Security.Principal.SecurityIdentifier('S-1-5-18')),'ReadData','Success')
    $auditSecurity.AddAuditRule($auditRule)
    Set-Acl -LiteralPath $special -AclObject $auditSecurity
    $auditBefore=(Get-Acl -LiteralPath $special -Audit).Sddl
    Reject {Publish-PreparedProfile $special $saved $zh $vh $valid.Length $journal} 'audit metadata needs separate review'
    Assert-PreparationAcl $auditBefore (Get-Acl -LiteralPath $special -Audit).Sddl
    if((Get-Digest $special) -cne $zh){throw 'Audited profile was overwritten.'}
    $checks.Add('Native auditing metadata is refused and retained rather than dropped during rename')
  }
  Reject { Publish-PreparedProfile $ini $saved $zh $vh $valid.Length $journal } 'repeat replacement of recovered state'
  if ((Get-Digest $ini) -cne $vh) { throw 'Repeat recovery rolled back working state.' }
  $checks.Add('Atomic replacement publishes exact candidate, retains corruption for forensics and refuses replay')
  $unexpected=Join-Path $testRoot 'unexpected.ini';[IO.File]::WriteAllText($unexpected,'user change')
  $unexpectedHash=Get-Digest $unexpected
  Reject { Publish-PreparedProfile $unexpected $saved $zh $vh $valid.Length $journal } 'intervening user edit'
  if ((Get-Digest $unexpected) -cne $unexpectedHash) { throw 'Intervening user edit was overwritten.' }
  if ($native) {
    $locked=Join-Path $testRoot 'locked.ini';[IO.File]::WriteAllBytes($locked,$zeros)
    $lock=[IO.File]::Open($locked,[IO.FileMode]::Open,[IO.FileAccess]::Read,[IO.FileShare]::None)
    try { Reject { Publish-PreparedProfile $locked $saved $zh $vh $valid.Length $journal } 'locked profile' }
    finally { $lock.Dispose() }
    if ((Get-Digest $locked) -cne $zh) { throw 'Locked original changed.' }
    $checks.Add('Windows exclusive file lock refuses replacement without changing the original')
    # Unlike the exclusive lock, this permits all preflight reads and stages the
    # candidate, but must make the final rename fail with sharing violation.
    $renameLocked=Join-Path $testRoot 'rename-locked.ini';[IO.File]::WriteAllBytes($renameLocked,$zeros)
    $renameAcl=(Get-Acl -LiteralPath $renameLocked).Sddl
    $lock=[IO.File]::Open($renameLocked,[IO.FileMode]::Open,[IO.FileAccess]::Read,[IO.FileShare]::Read)
    try {Reject {Publish-PreparedProfile $renameLocked $saved $zh $vh $valid.Length $journal} 'read-sharing handle without delete sharing'}
    finally {$lock.Dispose()}
    $retainedStages=@(Get-ChildItem -LiteralPath $testRoot -File -Force | Where-Object {$_.Name -like 'rename-locked.ini.opennav-recovery-*.partial'})
    if($retainedStages.Count -ne 1 -or (Get-Digest $retainedStages[0].FullName) -cne $vh -or (Get-Digest $renameLocked) -cne $zh){throw 'Rename lock did not retain original and exact staged candidate.'}
    Assert-PreparationAcl $renameAcl (Get-Acl -LiteralPath $renameLocked).Sddl
    $checks.Add('Native final rename respects delete-sharing locks and retains original permissions, bytes, journal and exact stage')
  }
  $checks.Add('Intervening user edits fail without overwriting existing data')
  if ($native) {
    $nativeSid=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value
    $private=New-PreparationDirectory ([pscustomobject]@{workspace=$testRoot;sid=$nativeSid}) 'test-private'
    $acl=Get-Acl -LiteralPath $private
    if (-not $acl.AreAccessRulesProtected) { throw 'Recovery folder inherited public access.' }
    $allowed=@($nativeSid,'S-1-5-18','S-1-5-32-544')
    foreach ($rule in $acl.GetAccessRules($true,$true,[Security.Principal.SecurityIdentifier])) {
      if ($rule.IdentityReference.Value -notin $allowed) { throw 'Unexpected recovery access principal.' }
    }
    $checks.Add('Native backup ACL grants only the current SID, administrators and SYSTEM')
  }
  [pscustomobject]@{status='passed';environment=$(if($native){'native-windows-disposable-filesystem'}else{'linux-powershell-portable-contracts'});count=$checks.Count;checks=@($checks);boatAccess=$false;applicationLaunched=$false;nativeFileSystemContracts=$native;productOrBoatAcceptance=$false} | ConvertTo-Json -Depth 5
} catch {
  # Only disposable CI fixture descriptors are printed publicly. Local boat
  # tests retain private account descriptors outside this script's output.
  $failure=@{status='failed';error=$_.Exception.Message;scriptStack=$_.ScriptStackTrace;completedChecks=@($checks)}
  if($native -and $env:GITHUB_ACTIONS -eq 'true') {
    $failure.syntheticSddl=$sddl;$failure.beforeSddl=$aclBefore;$failure.nativeMetadataBefore=$metadataBefore
    $failure.fixtureAcls=@(Get-ChildItem -LiteralPath $testRoot -File -Force | Where-Object {$_.Name -match '\.ini|\.partial$'} | ForEach-Object {
      $ordinary=(Get-Acl -LiteralPath $_.FullName).Sddl
      $audited=Get-PreparationAuditedAcl $_.FullName
      @{name=$_.Name;sddl=$ordinary;auditedSddl=$audited;ordinaryFlags=[int](New-Object Security.AccessControl.RawSecurityDescriptor($ordinary)).ControlFlags;auditedFlags=[int](New-Object Security.AccessControl.RawSecurityDescriptor($audited)).ControlFlags}
    })
  }
  Write-Host ($failure | ConvertTo-Json -Depth 6)
  throw
} finally { Remove-Item -LiteralPath $testRoot -Recurse -Force }

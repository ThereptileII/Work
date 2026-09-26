# Private cold-copy/recovery primitives. No launch, transport or privilege changes.
. (Join-Path $PSScriptRoot 'Common.ps1')

function Assert-PreparationIdentity([string]$CurrentSid,[string]$LocalAppData,[string]$EnvironmentLocal,$Explorers) {
  if ($CurrentSid -cnotmatch '^S-1-5-[0-9-]+$' -or @($Explorers).Count -ne 1 -or
      $Explorers[0].sid -cne $CurrentSid -or $Explorers[0].session -le 0) { throw 'One interactive desktop owned by the current SID is required.' }
  if (-not $EnvironmentLocal -or (Assert-LocalPath $LocalAppData) -ine (Assert-LocalPath $EnvironmentLocal)) { throw 'Interactive user plugin environment is ambiguous.' }
}
function Assert-PreparationSeparate([string]$Destination,[string[]]$Sources) {
  $destination=Assert-LocalPath $Destination
  foreach ($source in $Sources) {
    $source=Assert-LocalPath $source
    if ($destination -ieq $source -or $destination.StartsWith($source+[IO.Path]::DirectorySeparatorChar,[StringComparison]::OrdinalIgnoreCase) -or
        $source.StartsWith($destination+[IO.Path]::DirectorySeparatorChar,[StringComparison]::OrdinalIgnoreCase)) { throw 'Recovery workspace must be separate from every original tree.' }
  }
}
function Get-PreparationContext([string]$Workspace) {
  if ([Environment]::OSVersion.Platform -ne 'Win32NT') { throw 'Native Windows preparation only.' }
  $sid=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value
  $explorers=@(Get-CimInstance Win32_Process -Filter "Name='explorer.exe'" | ForEach-Object {
    $owner=Invoke-CimMethod -InputObject $_ -MethodName GetOwnerSid
    if ($owner.ReturnValue -ne 0) { throw 'Cannot verify interactive desktop ownership.' }
    [pscustomobject]@{sid=$owner.Sid;session=$_.SessionId}
  })
  $local=[Environment]::GetFolderPath('LocalApplicationData')
  Assert-PreparationIdentity $sid $local $env:LOCALAPPDATA $explorers
  $profile=Assert-LocalPath (Join-Path ([Environment]::GetFolderPath('CommonApplicationData')) 'opencpn')
  $application=Assert-LocalPath (Join-Path ${env:ProgramFiles(x86)} 'OpenCPN')
  $managed=Assert-LocalPath (Join-Path $local 'opencpn\plugins')
  $workspace=Assert-LocalPath $Workspace
  Assert-PreparationSeparate $workspace @($profile,$application,$managed)
  $context=[pscustomobject]@{sid=$sid;session=$explorers[0].session;localAppData=$local;profile=$profile;application=$application;managed=$managed;workspace=$workspace;executable=(Join-Path $application 'opencpn.exe')}
  Assert-PreparationStock $context
  Assert-PreparationClosed @($application,$managed)
  return $context
}
function Assert-PreparationStock($Context) {
  if ((Get-Digest $Context.executable) -cne '7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c') { throw 'Only the validated official OpenCPN 5.12.4 executable is supported.' }
  $stream=[IO.File]::OpenRead($Context.executable);$reader=New-Object IO.BinaryReader($stream)
  try {
    if ($stream.Length -lt 128 -or $reader.ReadUInt16() -ne 0x5a4d) { throw 'Expected native PE executable.' }
    $stream.Position=60;$offset=$reader.ReadUInt32()
    if ($offset -gt $stream.Length-24) { throw 'Invalid PE offset.' }
    $stream.Position=$offset
    if ($reader.ReadUInt32() -ne 0x4550 -or $reader.ReadUInt16() -ne 0x14c) { throw 'Expected supported Win32 OpenCPN ABI.' }
  } finally { $reader.Dispose();$stream.Dispose() }
  $version=[Diagnostics.FileVersionInfo]::GetVersionInfo($Context.executable)
  if ($version.FileMajorPart -ne 5 -or $version.FileMinorPart -ne 12 -or $version.FileBuildPart -ne 4) { throw 'Unexpected stock executable version.' }
}
function Assert-PreparationProcesses($Processes,[string[]]$Roots) {
  foreach ($process in @($Processes)) {
    if ($process.Name -match '^(?i:opencpn|oeserver\w*|oexserver\w*|rtl_ais|rtl_fm|aisdecoder|ais-?catcher|hackrf_transfer)\.exe$') { throw 'Close OpenCPN/XNav and plugin helper processes normally before preparation.' }
    if (-not $process.ExecutablePath) { continue }
    foreach ($root in $Roots) {
      if ($process.ExecutablePath.StartsWith($root+'\',[StringComparison]::OrdinalIgnoreCase)) { throw 'A process is running from an application/plugin tree.' }
    }
  }
}
function Assert-PreparationClosed([string[]]$Roots) {
  Assert-PreparationProcesses @(Get-CimInstance Win32_Process) $Roots
}
function New-PreparationDirectory($Context,[string]$Purpose) {
  $directory=New-RunDirectory $Context.workspace $Purpose
  # These backups may include license/user metadata. Do not inherit broad read access.
  $acl=New-Object Security.AccessControl.DirectorySecurity
  $acl.SetAccessRuleProtection($true,$false)
  foreach ($sid in @($Context.sid,'S-1-5-18','S-1-5-32-544')) {
    $identity=New-Object Security.Principal.SecurityIdentifier($sid)
    $rule=New-Object Security.AccessControl.FileSystemAccessRule($identity,'FullControl','ContainerInherit, ObjectInherit','None','Allow')
    $acl.AddAccessRule($rule)
  }
  Set-Acl -LiteralPath $directory -AclObject $acl
  return $directory
}
function Get-PreparationTree([string]$Root) {
  $root=Assert-LocalPath $Root
  $entries=New-Object 'Collections.Generic.List[object]'
  if (-not (Test-Path -LiteralPath $root)) { return [pscustomobject]@{root=$root;exists=$false;entries=@()} }
  $item=Get-Item -LiteralPath $root -Force
  if (-not $item.PSIsContainer) { throw 'Expected a directory tree.' }
  $pending=New-Object 'Collections.Generic.Queue[string]';$pending.Enqueue($root)
  while ($pending.Count) {
    foreach ($entry in Get-ChildItem -LiteralPath $pending.Dequeue() -Force) {
      if ($entry.Attributes -band [IO.FileAttributes]::ReparsePoint) { throw 'Redirected preparation tree refused before recursion.' }
      if ($entries.Count -ge 30000) { throw 'Preparation inventory exceeds its bound.' }
      $relative=$entry.FullName.Substring($root.Length+1)
      if ($entry.PSIsContainer) {
        $entries.Add([pscustomobject]@{path=$relative;directory=$true;bytes=0;sha256=''});$pending.Enqueue($entry.FullName)
      } else {
        $entries.Add([pscustomobject]@{path=$relative;directory=$false;bytes=$entry.Length;sha256=(Get-Digest $entry.FullName)})
      }
    }
  }
  return [pscustomobject]@{root=$root;exists=$true;entries=@($entries | Sort-Object path)}
}
function Assert-PreparationTree($Expected) {
  $actual=Get-PreparationTree $Expected.root
  if (($actual | ConvertTo-Json -Depth 8 -Compress) -cne ($Expected | ConvertTo-Json -Depth 8 -Compress)) { throw 'Source tree changed during preparation.' }
}
function Copy-PreparationFile([string]$Source,[string]$Destination,[string]$Sha256,[long]$Length) {
  $source=Assert-LocalPath $Source;$destination=Assert-LocalPath $Destination
  if ($Sha256 -cnotmatch '^[a-f0-9]{64}$' -or $Length -lt 0 -or (Test-Path -LiteralPath $destination)) { throw 'Invalid or existing preparation destination.' }
  if ((Get-Digest $source) -cne $Sha256 -or (Get-Item -LiteralPath $source).Length -ne $Length) { throw 'Source changed before cold copy.' }
  $null=New-Item -ItemType Directory -Path ([IO.Path]::GetDirectoryName($destination)) -Force
  $sourceStream=[IO.File]::Open($source,[IO.FileMode]::Open,[IO.FileAccess]::Read,[IO.FileShare]::Read)
  $copyStream=$null
  try {
    $copyStream=New-Object IO.FileStream($destination,[IO.FileMode]::CreateNew,[IO.FileAccess]::Write,[IO.FileShare]::None)
    $sourceStream.CopyTo($copyStream);$copyStream.Flush($true)
  } finally { if ($copyStream) {$copyStream.Dispose()};$sourceStream.Dispose() }
  if ((Get-Digest $destination) -cne $Sha256 -or (Get-Item -LiteralPath $destination).Length -ne $Length -or (Get-Digest $source) -cne $Sha256) { throw 'Cold-copy verification failed; retain partial journal.' }
}
function Copy-PreparationTree($Snapshot,[string]$Destination) {
  $destination=Assert-LocalPath $Destination
  Assert-PreparationSeparate $destination @($Snapshot.root)
  if (Test-Path -LiteralPath $destination) { throw 'A new cold-copy directory is required.' }
  Assert-PreparationTree $Snapshot
  if (-not $Snapshot.exists) { return }
  $null=New-Item -ItemType Directory -Path $destination
  foreach ($entry in $Snapshot.entries) {
    # Entries came from local enumeration, never an imported path list.
    $target=Assert-LocalPath (Join-Path $destination $entry.path)
    if ($entry.directory) { $null=New-Item -ItemType Directory -Path $target -Force }
    else { Copy-PreparationFile (Join-Path $Snapshot.root $entry.path) $target $entry.sha256 $entry.bytes }
  }
  Assert-PreparationTree $Snapshot
  $copy=Get-PreparationTree $destination
  if (($copy.entries | ConvertTo-Json -Depth 6 -Compress) -cne ($Snapshot.entries | ConvertTo-Json -Depth 6 -Compress)) { throw 'Complete copied inventory differs from source.' }
}
function Assert-ExactRecoveryBytes([byte[]]$Original,[byte[]]$Candidate,[string]$OriginalHash,[string]$CandidateHash,[int]$Length) {
  if ($Length -le 0 -or $Original.Length -ne $Length -or $Candidate.Length -ne $Length) { throw 'Exact recovery lengths required.' }
  foreach ($byte in $Original) { if ($byte -ne 0) { throw 'Original INI must be entirely zero-filled.' } }
  $sha=[Security.Cryptography.SHA256]::Create()
  try {
    foreach ($pair in @(@($Original,$OriginalHash),@($Candidate,$CandidateHash))) {
      $hash=([BitConverter]::ToString($sha.ComputeHash([byte[]]$pair[0]))).Replace('-','').ToLowerInvariant()
      if ($hash -cne $pair[1]) { throw 'Recovery bytes differ from the authorized hash.' }
    }
  } finally { $sha.Dispose() }
}
function Assert-AuthorizedRecovery([string]$Original,[string]$Candidate) {
  $original=Assert-LocalPath $Original;$candidate=Assert-LocalPath $Candidate
  if ([IO.Path]::GetFileName($original) -cne 'opencpn.ini' -or
      [IO.Path]::GetDirectoryName($candidate) -ine [IO.Path]::GetDirectoryName($original) -or
      [IO.Path]::GetFileName($candidate) -cnotmatch '^opencpn\.ini~[A-Za-z0-9]+\.TMP$') { throw 'Expected the original profile and its same-directory recovery TMP.' }
  foreach ($file in @($original,$candidate)) { if ((Get-Item -LiteralPath $file).Length -ne 21380) { throw 'Only the inspected 21,380-byte recovery is authorized.' } }
  Assert-ExactRecoveryBytes ([IO.File]::ReadAllBytes($original)) ([IO.File]::ReadAllBytes($candidate)) 'f84114bfa5c456c4b8f5f073141abb95537155f9f75d78dedc12db3ed5ab3867' 'a2e416b7d35d6d2f82dcf6b3a97136a8fba5a133435c2047e07e09022426aef9' 21380
  $values=Read-ProfileForAudit $candidate
  if ($values['Directories/pluginInstallDir']) { throw 'Custom plugin root is outside this narrowly reviewed recovery.' }
  # Output settings deliberately remain untouched. Recovery is NOT a launch audit.
}
function Assert-PreparationAclBytes([byte[]]$Expected,[byte[]]$Actual,[switch]$AllowDaclAutoInherited) {
  # SECURITY_DESCRIPTOR_RELATIVE: revision, reserved, 16-bit control, offsets.
  # Native SDDL parsing below validates structure before this exact comparison.
  # Only the observed DACL AutoInherited metadata bit (0x0400) may differ after
  # File.Replace. Do not mask protection, auto-inherit-required, SACL or ACE bits.
  foreach($descriptor in @($Expected,$Actual)) {
    if($descriptor.Length -lt 20 -or $descriptor.Length -gt 1048576 -or $descriptor[0] -ne 1 -or
        ($descriptor[3] -band 0x80) -eq 0){throw 'Expected bounded self-relative security descriptor.'}
  }
  if($Expected.Length -ne $Actual.Length){throw 'Profile security descriptor size changed.'}
  for($index=0;$index -lt $Expected.Length;$index++) {
    $before=[int]$Expected[$index];$after=[int]$Actual[$index]
    if($AllowDaclAutoInherited -and $index -eq 3){$before=$before -band 0xfb;$after=$after -band 0xfb}
    if($before -ne $after){throw 'Profile owner, group, control flags or ordered ACL entries changed.'}
  }
}
function Convert-PreparationAcl([string]$Sddl) {
  if([string]::IsNullOrEmpty($Sddl) -or $Sddl.Length -gt 262144){throw 'Expected bounded recorded SDDL.'}
  $descriptor=New-Object Security.AccessControl.RawSecurityDescriptor($Sddl)
  $bytes=New-Object byte[] $descriptor.BinaryLength
  $descriptor.GetBinaryForm($bytes,0)
  return ,$bytes
}
function Assert-PreparationAcl([string]$Expected,[string]$Actual,[switch]$AllowDaclAutoInherited) {
  # RawSecurityDescriptor canonicalizes SID/rights spellings, never sorts or
  # merges ACEs. Comparison and serialization do not write the file's ACL.
  $before=Convert-PreparationAcl $Expected;$after=Convert-PreparationAcl $Actual
  Assert-PreparationAclBytes $before $after -AllowDaclAutoInherited:$AllowDaclAutoInherited
}
function Publish-PreparedProfile([string]$Original,[string]$SavedCandidate,[string]$OriginalHash,[string]$CandidateHash,[int]$Length,[string]$Journal) {
  $original=Assert-LocalPath $Original;$savedCandidate=Assert-LocalPath $SavedCandidate
  $journal=Assert-LocalPath $Journal
  if (-not [IO.File]::Exists($journal)) { throw 'Durable apply journal required before replacement.' }
  if ((Get-Digest $original) -cne $OriginalHash) { throw 'Original changed before staging; no replacement attempted.' }
  $temporary=$original+'.opennav-recovery-'+[guid]::NewGuid().ToString('N')+'.partial'
  Copy-PreparationFile $savedCandidate $temporary $CandidateHash $Length
  if ((Get-Digest $original) -cne $OriginalHash) { throw 'Original changed immediately before atomic replacement; staged file retained.' }
  # Same-directory replacement is atomic and retains destination permissions.
  # Windows may materialize the DACL AutoInherited marker; the caller compares
  # all permission data and retains raw before/after SDDL in private evidence.
  # The original is already durably backed up. Never roll back to corrupt zeros.
  [IO.File]::Replace($temporary,$original,[NullString]::Value)
  if ((Get-Digest $original) -cne $CandidateHash) { throw 'Recovery postcondition failed; inspect the durable journal without automatic rollback.' }
}

# Cold recovery copy only. Never launches OpenCPN or communicates with vessel equipment.
[CmdletBinding()]
param(
  [string]$Workspace = 'C:\XNav',
  [Parameter(Mandatory=$true)][string]$OpenCpnDirectory,
  [Parameter(Mandatory=$true)][string]$ProfileDirectory
)
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
function Plain([string]$Path) {
  if ($Path -notmatch '^[A-Za-z]:\\' -or $Path -match '[\x00-\x1f"]') { throw 'Use an absolute local Windows path.' }
  $full = [IO.Path]::GetFullPath($Path).TrimEnd('\')
  if ($full.Length -lt 4) { throw 'A drive root is not a backup location.' }
  $walk = $full
  while ($walk.Length -gt 3) {
    if ((Test-Path -LiteralPath $walk) -and ((Get-Item -LiteralPath $walk -Force).Attributes -band [IO.FileAttributes]::ReparsePoint)) { throw "Reparse path refused: $walk" }
    $walk = [IO.Path]::GetDirectoryName($walk)
  }
  return $full
}
function Digest([string]$Path) {
  $sha=[Security.Cryptography.SHA256]::Create(); $stream=[IO.File]::OpenRead($Path)
  try { return ([BitConverter]::ToString($sha.ComputeHash($stream))).Replace('-','').ToLowerInvariant() }
  finally { $stream.Dispose(); $sha.Dispose() }
}
function Inventory([string]$Directory) {
  $result = @()
  foreach ($item in Get-ChildItem -LiteralPath $Directory -Force -Recurse) {
    if ($item.Attributes -band [IO.FileAttributes]::ReparsePoint) { throw "Redirected source refused: $($item.FullName)" }
    if (-not $item.PSIsContainer) {
      $result += [pscustomobject]@{path=$item.FullName.Substring($Directory.Length+1);bytes=$item.Length;sha256=(Digest $item.FullName)}
    }
  }
  return @($result | Sort-Object path)
}
function AssertClosed {
  if (@(Get-Process -Name opencpn -ErrorAction SilentlyContinue).Count) { throw 'Close OpenCPN/XNav normally before making a cold recovery backup.' }
}
AssertClosed
$Workspace=Plain $Workspace; $OpenCpnDirectory=Plain $OpenCpnDirectory; $ProfileDirectory=Plain $ProfileDirectory
foreach ($source in @($OpenCpnDirectory,$ProfileDirectory)) {
  if (-not [IO.Directory]::Exists($source)) { throw 'Both original application and actual profile must exist.' }
  if ($Workspace.StartsWith($source+'\',[StringComparison]::OrdinalIgnoreCase) -or $source.StartsWith($Workspace+'\',[StringComparison]::OrdinalIgnoreCase) -or $Workspace -ieq $source) { throw 'Backup workspace must be separate from the application and user profile.' }
}
if (-not [IO.File]::Exists((Join-Path $OpenCpnDirectory 'opencpn.exe'))) { throw 'Original OpenCPN application missing.' }
$application=@(Inventory $OpenCpnDirectory); $profile=@(Inventory $ProfileDirectory)
$size=[long]0; foreach ($entry in @($application)+@($profile)) { $size += $entry.bytes }
$drive=New-Object IO.DriveInfo([IO.Path]::GetPathRoot($Workspace))
if ($drive.AvailableFreeSpace -lt ($size + 1073741824)) { throw 'Insufficient free space for verified backup and working margin.' }
$null=New-Item -ItemType Directory -Path $Workspace -Force
$recovery=Plain (Join-Path $Workspace 'recovery'); $null=New-Item -ItemType Directory -Path $recovery -Force
$id='pre-beta2-'+[DateTime]::UtcNow.ToString('yyyyMMdd-HHmmss')+'-'+[guid]::NewGuid().ToString('N').Substring(0,8)
$stage=Join-Path $recovery ($id+'.partial'); $target=Join-Path $recovery $id
$null=New-Item -ItemType Directory -Path $stage
foreach ($source in @(@('opencpn',$OpenCpnDirectory,$application),@('profile',$ProfileDirectory,$profile))) {
  $destination=Join-Path $stage $source[0]; $null=New-Item -ItemType Directory -Path $destination
  foreach ($entry in $source[2]) {
    AssertClosed
    $original=Plain (Join-Path $source[1] $entry.path)
    if ((Digest $original) -cne $entry.sha256) { throw 'Source changed during backup; partial copy retained without acceptance marker.' }
    $copy=Join-Path $destination $entry.path
    $null=New-Item -ItemType Directory -Path ([IO.Path]::GetDirectoryName($copy)) -Force
    Copy-Item -LiteralPath $original -Destination $copy
    $copiedHash=Digest $copy; $copiedBytes=(Get-Item -LiteralPath $copy -Force).Length
    if ($copiedHash -cne $entry.sha256 -or $copiedBytes -ne $entry.bytes) {
      $detail=@{tree=$source[0];path=$entry.path;expectedBytes=$entry.bytes;actualBytes=$copiedBytes;expectedSha256=$entry.sha256;actualSha256=$copiedHash;currentSourceSha256=(Digest $original)}
      [IO.File]::WriteAllText((Join-Path $stage 'copy-failure.json'),($detail | ConvertTo-Json -Depth 4),(New-Object Text.UTF8Encoding($false)))
      throw 'Backup copy verification failed. Private relative path and hashes are recorded in copy-failure.json inside the retained .partial set.'
    }
  }
  $after=@(Inventory $source[1])
  if (($after | ConvertTo-Json -Depth 4 -Compress) -cne ($source[2] | ConvertTo-Json -Depth 4 -Compress)) { throw 'Source inventory changed during backup.' }
}
AssertClosed
$record=@{schema=1;owner='OpenNavX.BoatRecovery.1';createdUtc=[DateTime]::UtcNow.ToString('o');sourceApplication=$OpenCpnDirectory;sourceProfile=$ProfileDirectory;application=$application;profile=$profile;bytes=$size;status='verified';privacy='Local recovery data; contains private navigation/profile files. Do not upload.'}
[IO.File]::WriteAllText((Join-Path $stage 'recovery.json'),($record | ConvertTo-Json -Depth 8),(New-Object Text.UTF8Encoding($false)))
[IO.Directory]::Move($stage,$target)
[pscustomobject]@{status='verified';backup=$target;applicationFiles=$application.Count;profileFiles=$profile.Count;bytes=$size;originalExecutableSha256=(Digest (Join-Path $OpenCpnDirectory 'opencpn.exe'));navigationDataUploaded=$false} | ConvertTo-Json

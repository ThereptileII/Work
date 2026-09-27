# Stage an exact qualified review-tool copy without changing the source checkout,
# active restart dependencies, application, profile or remote-access services.
[CmdletBinding()]
param(
 [string]$Workspace='C:\XNav',
 [Parameter(Mandatory=$true)][string]$Archive,
 [Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{64}$')][string]$ArchiveSha256,
 [Parameter(Mandatory=$true)][string]$Manifest,
 [Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{64}$')][string]$ManifestSha256,
 [Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{40}$')][string]$Commit
)
. (Join-Path $PSScriptRoot 'Common.ps1')
$workspace=Assert-LocalPath $Workspace
if((Get-Digest $Archive) -cne $ArchiveSha256 -or (Get-Digest $Manifest) -cne $ManifestSha256){throw 'Review-tool transport hash differs.'}
$record=Read-Record $Manifest
if($record.schema -ne 1 -or $record.commit -cne $Commit -or @($record.files).Count -lt 1 -or @($record.files).Count -gt 200){throw 'Unexpected tool manifest.'}
$expected=[Collections.Generic.Dictionary[string,string]]::new([StringComparer]::OrdinalIgnoreCase)
foreach($file in $record.files) {
 if($file.name -cnotmatch '^[A-Za-z0-9][A-Za-z0-9._-]*\.(ps1|cs|py|json)$' -or $file.sha256 -cnotmatch '^[a-f0-9]{64}$' -or $expected.ContainsKey($file.name)){throw 'Ambiguous or redirected tool entry.'}
 $expected.Add($file.name,$file.sha256)
}
$scripts=Assert-LocalPath (Join-Path $workspace 'scripts')
if(-not [IO.Directory]::Exists($scripts)){throw 'Existing owned scripts directory required.'}
$destination=Assert-LocalPath (Join-Path $scripts ('review-'+$Commit))
if(Test-Path -LiteralPath $destination){throw 'Existing tool copy is preserved; inspect its completion record.'}
Add-Type -AssemblyName System.IO.Compression.FileSystem
$zip=[IO.Compression.ZipFile]::OpenRead((Assert-LocalPath $Archive))
try {
 if($zip.Entries.Count -ne $expected.Count){throw 'Archive/manifest entry count differs.'}
 $seen=New-Object 'Collections.Generic.HashSet[string]' ([StringComparer]::OrdinalIgnoreCase)
 foreach($entry in $zip.Entries) {
  if(-not $expected.ContainsKey($entry.FullName) -or -not $seen.Add($entry.FullName) -or $entry.Length -le 0 -or $entry.Length -gt 4194304){throw 'Unexpected archive entry.'}
 }
 $null=New-Item -ItemType Directory -Path $destination
 foreach($entry in $zip.Entries) {
  $path=Join-Path $destination $entry.FullName
  [IO.Compression.ZipFileExtensions]::ExtractToFile($entry,$path,$false)
  if((Get-Digest $path) -cne $expected[$entry.FullName]){throw 'Extracted tool hash differs; retain incomplete copy for inspection.'}
 }
 $result=@{schema=1;owner='OpenNavX.QualifiedReviewCopy.1';status='staged-only';commit=$Commit;utc=[datetime]::UtcNow.ToString('o');
  directory=$destination;archiveSha256=$ArchiveSha256;manifestSha256=$ManifestSha256;files=$record.files;
  sourceCheckoutChanged=$false;applicationChanged=$false;profileChanged=$false;toolsExecuted=$false}
 Write-Record (Join-Path $destination 'staging-complete.json') $result
 [pscustomobject]@{status='staged-only';directory=$destination;commit=$Commit;files=$expected.Count;
  completionSha256=(Get-Digest (Join-Path $destination 'staging-complete.json'))}|ConvertTo-Json
}finally{$zip.Dispose()}

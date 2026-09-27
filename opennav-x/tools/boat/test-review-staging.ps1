# Actual archive/filesystem boundary in disposable Windows directories only.
[CmdletBinding()]
param([switch]$IsolatedLocal)
Set-StrictMode -Version Latest;$ErrorActionPreference='Stop'
if([Environment]::OSVersion.Platform -ne 'Win32NT' -or ($env:GITHUB_ACTIONS -cne 'true' -and -not $IsolatedLocal)){throw 'Native CI or explicit disposable Windows test required.'}
. (Join-Path $PSScriptRoot 'Common.ps1')
Add-Type -AssemblyName System.IO.Compression,System.IO.Compression.FileSystem
$root=Join-Path ([IO.Path]::GetTempPath()) ('OpenNav review staging & '+[guid]::NewGuid().ToString('N'))
$null=New-Item -ItemType Directory -Path $root
$checks=New-Object 'Collections.Generic.List[string]'
$stage=Join-Path $PSScriptRoot 'stage-review-tools.ps1'
try {
 foreach($case in @('valid','archive-hash','manifest-hash','commit','file-hash','archive-traversal','manifest-traversal','duplicate','existing','missing-scripts','extra-entry','empty-entry')) {
  $work=Join-Path $root $case;$null=New-Item -ItemType Directory -Path $work
  $scripts=Join-Path $work 'scripts'
  if($case -cne 'missing-scripts'){$null=New-Item -ItemType Directory -Path $scripts}
  $source=Join-Path $work 'source';$profileFixture=Join-Path $work 'profile'
  $null=New-Item -ItemType Directory -Path $source;$null=New-Item -ItemType Directory -Path $profileFixture
  [IO.File]::WriteAllText((Join-Path $source 'active-helper.ps1'),'unchanged active restart dependency')
  [IO.File]::WriteAllText((Join-Path $profileFixture 'settings.ini'),'unchanged user settings')
  $sourceHash=Get-Digest (Join-Path $source 'active-helper.ps1');$profileHash=Get-Digest (Join-Path $profileFixture 'settings.ini')
  $payload=Join-Path $work 'inert.ps1';[IO.File]::WriteAllText($payload,"throw 'An archive entry must never execute'")
  $entry=if($case -ceq 'archive-traversal'){'../escaped.ps1'}else{'inert.ps1'}
  $zipPath=Join-Path $work 'tools.zip';$zip=[IO.Compression.ZipFile]::Open($zipPath,[IO.Compression.ZipArchiveMode]::Create)
  try {
   if($case -ceq 'empty-entry'){$null=$zip.CreateEntry($entry)}
   else{$null=[IO.Compression.ZipFileExtensions]::CreateEntryFromFile($zip,$payload,$entry)}
   if($case -ceq 'extra-entry'){$null=[IO.Compression.ZipFileExtensions]::CreateEntryFromFile($zip,$payload,'extra.ps1')}
  }finally{$zip.Dispose()}
  $commit='a'*40
  $files=@(@{name=$(if($case -ceq 'manifest-traversal'){'../escaped.ps1'}else{'inert.ps1'});sha256=$(if($case -ceq 'file-hash'){'b'*64}else{Get-Digest $payload})})
  if($case -ceq 'duplicate'){$files+=@{name='INERT.ps1';sha256=(Get-Digest $payload)}}
  $manifest=Join-Path $work 'manifest.json';Write-Record $manifest @{schema=1;commit=$commit;files=$files}
  $destination=Join-Path $scripts ('review-'+$commit)
  if($case -ceq 'existing') {
   $null=New-Item -ItemType Directory -Path $destination
   [IO.File]::WriteAllText((Join-Path $destination 'preserve.txt'),'existing tool copy')
  }
  $parameters=@{Workspace=$work;Archive=$zipPath;ArchiveSha256=$(if($case -ceq 'archive-hash'){'b'*64}else{Get-Digest $zipPath});
    Manifest=$manifest;ManifestSha256=$(if($case -ceq 'manifest-hash'){'b'*64}else{Get-Digest $manifest});Commit=$(if($case -ceq 'commit'){'b'*40}else{$commit})}
  $refused=$false;$result=$null
  try{$result=(& $stage @parameters)|ConvertFrom-Json}catch{$refused=$true}
  if((Get-Digest (Join-Path $source 'active-helper.ps1')) -cne $sourceHash -or (Get-Digest (Join-Path $profileFixture 'settings.ini')) -cne $profileHash){throw 'Staging changed active dependencies or profile.'}
  if(Test-Path -LiteralPath (Join-Path $scripts 'escaped.ps1')){throw 'Archive path escaped destination.'}
  if($case -ceq 'valid') {
   if($refused -or $result.status -cne 'staged-only' -or $result.files -ne 1 -or
     (Get-Digest (Join-Path $destination 'inert.ps1')) -cne (Get-Digest $payload)){throw 'Exact staging failed.'}
   $proof=Read-Record (Join-Path $destination 'staging-complete.json')
   if($proof.toolsExecuted -ne $false -or $proof.profileChanged -ne $false -or $proof.sourceCheckoutChanged -ne $false){throw 'Invalid staging evidence.'}
  } else {
   if(-not $refused -or (Test-Path -LiteralPath (Join-Path $destination 'staging-complete.json'))){throw ('Unsafe staging accepted: '+$case)}
   if($case -ceq 'existing' -and [IO.File]::ReadAllText((Join-Path $destination 'preserve.txt')) -cne 'existing tool copy'){throw 'Existing tool copy was overwritten.'}
   if($case -cnotin @('file-hash','existing') -and (Test-Path -LiteralPath $destination)){throw ('Refused preflight created a destination: '+$case)}
  }
  $checks.Add($case)
 }
 [pscustomobject]@{status='passed';count=$checks.Count;checks=$checks.ToArray();boatAccess=$false;applicationLaunched=$false;hardwareCommands=$false}|ConvertTo-Json
}finally{Remove-Item -LiteralPath $root -Recurse -Force}

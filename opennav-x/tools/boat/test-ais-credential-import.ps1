[CmdletBinding()]
param([switch]$PortableContracts,[switch]$IsolatedLocal)
$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest
$native=[Environment]::OSVersion.Platform -eq 'Win32NT'
if(-not $native -and -not $PortableContracts){throw 'Select portable contracts outside Windows.'}
if($native -and -not $IsolatedLocal -and $env:GITHUB_ACTIONS -ne 'true'){throw 'Only explicit isolated tests or disposable CI.'}
if(-not ('XNavAisCredentialImport' -as [type])){Add-Type -Path (Join-Path $PSScriptRoot 'AisCredentialImport.cs')}
$checks=0
function Check($ok,[string]$why){if(-not $ok){throw $why};$script:checks++}
function Payload([byte[]]$bytes) {
  $stream=New-Object IO.MemoryStream
  $header=[BitConverter]::GetBytes([uint32]$bytes.Length)
  $stream.Write($header,0,4);$stream.Write($bytes,0,$bytes.Length);$stream.Position=0
  return ,$stream
}
foreach($bad in @([byte[]]@(),[byte[]]@(0,0,0,0),[byte[]]@(1,2,0,0),[byte[]]@(2,0,0,0,65),[byte[]]@(1,0,0,0,32),[byte[]]@(1,0,0,0,127),[byte[]]@(1,0,0,0,65,66),[byte[]]@(255,255,255,255))) {
  $s=New-Object IO.MemoryStream(,$bad)
  try {Check ($null -eq [XNavAisCredentialImport]::ReadPayload($s)) 'Malformed frame accepted'} finally {$s.Dispose()}
}
foreach($length in @(1,512)) {
  $bytes=New-Object byte[] $length;for($i=0;$i -lt $length;$i++){$bytes[$i]=65}
  $s=Payload $bytes
  try {$read=[XNavAisCredentialImport]::ReadPayload($s);Check ($read.Length -eq $length) 'Valid frame lost';[Array]::Clear($read,0,$read.Length)} finally {$s.Dispose()}
}
foreach($name in @('import-ais-credential.ps1','import-ais-desktop-credential.ps1','probe-online-ais-desktop.ps1')) {
  $tokens=$null;$errors=$null;$null=[Management.Automation.Language.Parser]::ParseFile((Join-Path $PSScriptRoot $name),[ref]$tokens,[ref]$errors)
  Check ($errors.Count -eq 0) 'Import/probe entry point does not parse'
}
if($native) {
  $id=[guid]::NewGuid().ToString('N')
  try {
    foreach($test in @(@('DISPOSABLE-FAKE-KEY','stored-and-verified'),@('DISPOSABLE-FAKE-KEY','already-stored-and-verified'),@('OTHER-FAKE-KEY','existing-different-key-preserved'),@('DISPOSABLE-FAKE-KEY','already-stored-and-verified'))) {
      $s=Payload ([Text.Encoding]::ASCII.GetBytes($test[0]))
      try {Check ([XNavAisCredentialImport]::StoreForTest($s,$id) -ceq $test[1]) 'Native protected import roundtrip/conflict failed'} finally {$s.Dispose()}
    }
  } finally {[XNavAisCredentialImport]::RemoveForTest($id)}
}
[pscustomobject]@{passed=$true;checks=$checks;native=$native;productionCredentialAccessed=$false} | ConvertTo-Json -Compress

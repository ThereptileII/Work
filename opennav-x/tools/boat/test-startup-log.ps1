# Synthetic temporary bytes only; no application, profile, network or device.
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'StartupLog.ps1')
$script:checks=0
function Bytes([string]$Value) { return ,([Text.Encoding]::GetEncoding(28591).GetBytes($Value)) }
function Check([bool]$Value,[string]$Label) { if (-not $Value) { throw ('FAILED: '+$Label) };$script:checks++ }
$start='------- OpenCPN version 5.12.4'
$ready='OnInitTimer...Finalize Canvases'
$old=$start+"`n"+$ready+"`n"
Check (-not (Test-StartupInitializedSince (Bytes '') (Bytes ''))) 'empty log is not ready'
Check (Test-StartupInitializedSince (Bytes '') (Bytes $old)) 'first complete startup'
Check (-not (Test-StartupInitializedSince (Bytes $old) (Bytes $old))) 'retained complete log is not a fresh startup'
Check (-not (Test-StartupInitializedSince (Bytes $old) (Bytes ($old+"more text`n"+$ready)))) 'new finalization without new startup is refused'
Check (Test-StartupInitializedSince (Bytes $old) (Bytes ($old+$old))) 'normal append'
Check (-not (Test-StartupInitializedSince (Bytes $old) (Bytes ($old+$start)))) 'new incomplete startup'
Check (-not (Test-StartupInitializedSince (Bytes '') (Bytes ($old+$start)))) 'latest startup must finish'
Check (Test-StartupInitializedSince (Bytes ($old+'old tail')) (Bytes $old)) 'rotation replaces longer prior file'
Check (Test-StartupInitializedSince (Bytes ('prefix'+$old)) (Bytes ($old+'suffix'))) 'same length replacement'
Check (-not (Test-StartupInitializedSince (Bytes $old) (Bytes $ready))) 'rotation with only finalization is refused'
Check (-not (Test-StartupInitializedSince (Bytes '') (Bytes ($ready+$start)))) 'finalization before startup is refused'
$raw=[byte[]](@(255,254,0)+(Bytes $old))
Check (Test-StartupInitializedSince $raw ([byte[]](@($raw)+(Bytes $old)))) 'non-UTF8 bytes retain exact append boundary'
Check (-not (Test-StartupInitializedSince $raw $raw)) 'non-UTF8 retained content stays old'
$directory=Join-Path ([IO.Path]::GetTempPath()) ('OpenNav-startup-log-'+[guid]::NewGuid().ToString('N'))
$null=New-Item -ItemType Directory -Path $directory
try {
  $file=Join-Path $directory 'synthetic.log'
  Check ((Read-StartupLogBytes $file).Length -eq 0) 'missing file'
  [IO.File]::WriteAllBytes($file,(Bytes $old))
  $writer=New-Object IO.FileStream($file,[IO.FileMode]::Open,[IO.FileAccess]::ReadWrite,[IO.FileShare]::ReadWrite)
  try { Check ((Test-StartupInitializedSince (Bytes '') (Read-StartupLogBytes $file))) 'read while writer handle remains open' }
  finally { $writer.Dispose() }
  [IO.File]::WriteAllBytes($file,(New-Object byte[] (4*1024*1024+1)))
  $refused=$false;try { $null=Read-StartupLogBytes $file } catch { $refused=$true }
  Check $refused 'oversized log refused'
} finally { Remove-Item -LiteralPath $directory -Recurse -Force }
[pscustomobject]@{suite='fresh startup across append and rotation';checks=$script:checks;result='passed';rawMarineDataRead=$false;applicationLaunched=$false}|ConvertTo-Json

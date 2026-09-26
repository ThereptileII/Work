[CmdletBinding()]
param([string]$Workspace='C:\XNav',[Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{40}$')][string]$Commit)
. (Join-Path $PSScriptRoot 'Common.ps1')
$null=Get-Target $Workspace
$directory=Assert-LocalPath (Join-Path $Workspace 'source')
$origin='https://github.com/ThereptileII/Work.git'
if (-not (Test-Path -LiteralPath $directory)) {
  & git clone --no-checkout --filter=blob:none $origin $directory
  if ($LASTEXITCODE -ne 0) {throw 'Source clone failed.'}
}
Push-Location -LiteralPath $directory
try {
  if ((& git remote get-url origin) -cne $origin -or $LASTEXITCODE -ne 0) {throw 'Existing source checkout has another origin; preserve it.'}
  if (@(& git status --porcelain).Count) {throw 'Source checkout has local changes; preserve them.'}
  & git fetch --no-tags origin $Commit
  if ($LASTEXITCODE -ne 0) {throw 'Exact commit fetch failed.'}
  & git checkout --detach $Commit
  if ($LASTEXITCODE -ne 0 -or (& git rev-parse HEAD) -cne $Commit) {throw 'Source commit verification failed.'}
} finally {Pop-Location}
[pscustomobject]@{status='passed';commit=$Commit;source=$directory;applicationChanged=$false} | ConvertTo-Json

[CmdletBinding()]
param([string]$Workspace='C:\XNav',[Parameter(Mandatory=$true)][ValidateSet('Repair','Rollback','Uninstall','Diagnostics')][string]$Action)
. (Join-Path $PSScriptRoot 'Common.ps1')
$config=Get-Target $Workspace;$installed=Get-Installed
if (@(Get-Process -Name opencpn -ErrorAction SilentlyContinue).Count -and $Action -ne 'Diagnostics') { throw 'Close OpenCPN normally first.' }
$directory=New-RunDirectory $Workspace $Action.ToLowerInvariant();$report=Join-Path $directory 'maintenance.json'
$script=Assert-LocalPath (Join-Path $installed.generation 'Lifecycle.ps1')
$owned=@($installed.ownership.managedFiles | Where-Object { $_.path -ceq 'Lifecycle.ps1' })
if ($owned.Count -ne 1 -or (Get-Digest $script) -cne $owned[0].sha256) { throw 'Maintenance engine changed; use the verified original Setup.' }
& (Join-Path $env:WINDIR 'System32\WindowsPowerShell\v1.0\powershell.exe') -NoProfile -NonInteractive -ExecutionPolicy Bypass -File $script -Action $Action -Report $report
if ($LASTEXITCODE -ne 0) { throw 'Maintenance failed; inspect retained local report.' }
if ($Action -ne 'Diagnostics' -and (Read-Record $report).status -cne 'passed') { throw 'Maintenance completion not confirmed.' }
if ((Get-Digest $config.stockExecutable) -cne '7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c') { throw 'Unexpected original OpenCPN hash change.' }
[pscustomobject]@{status='passed';action=$Action;report=$report;originalOpenCpn='unchanged';navigationProfile='preserved'} | ConvertTo-Json

# Optional on-target build for an explicitly provisioned native MSVC build tree.
# Normal boat deployment consumes the exact already-qualified CI artifact.
[CmdletBinding()]
param([string]$Workspace='C:\XNav',[Parameter(Mandatory=$true)][string]$BuildDirectory)
. (Join-Path $PSScriptRoot 'Common.ps1')
$null=Get-Target $Workspace
$directory=Assert-LocalPath $BuildDirectory
if (-not $directory.StartsWith((Assert-LocalPath $Workspace)+'\',[StringComparison]::OrdinalIgnoreCase)) {throw 'Build tree must be inside the owned boat workspace.'}
$cache=Get-Content -LiteralPath (Join-Path $directory 'CMakeCache.txt') -Raw
if ($cache -notmatch '(?m)^XNAV_ENABLE_TEST_FIXTURES:BOOL=OFF\r?$' -or $cache -notmatch '(?m)^CMAKE_GENERATOR_PLATFORM:INTERNAL=Win32\r?$') {throw 'Use a configured native Win32 MSVC product tree with test fixtures OFF.'}
& cmake --build $directory --config Release --parallel 2
if ($LASTEXITCODE -ne 0) {throw 'Native build failed.'}
& ctest --test-dir $directory --build-config Release --output-on-failure
if ($LASTEXITCODE -ne 0) {throw 'Native tests failed; do not deploy.'}
Write-Output 'Build/test complete. This does not qualify an unreviewed on-target binary for installation.'

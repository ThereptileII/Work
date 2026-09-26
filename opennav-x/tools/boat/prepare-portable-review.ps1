[CmdletBinding()]
param(
  [string]$Workspace='C:\XNav',
  [Parameter(Mandatory=$true)][string]$Archive,
  [Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{64}$')][string]$ExpectedArchiveSha256,
  [Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{40}$')][string]$ExpectedCommit
)
. (Join-Path $PSScriptRoot 'PortableReview.ps1')
if ([Environment]::OSVersion.Platform -ne 'Win32NT') {throw 'Native Windows display-review preparation required.'}
if (@(Get-Process -Name opencpn -ErrorAction SilentlyContinue).Count) {throw 'Close OpenCPN normally before preparing an isolated review.'}
$Workspace=Assert-LocalPath $Workspace;$Archive=Assert-LocalPath $Archive
if ((Get-Item -LiteralPath $Archive).Length -gt 536870912 -or (Get-Digest $Archive) -cne $ExpectedArchiveSha256) {throw 'Archive does not match the accepted CI artifact hash.'}
$roots=@(Get-NormalOpenCpnRoots)
foreach ($root in $roots) {
  if ($Workspace -ieq $root -or $Workspace.StartsWith($root+'\',[StringComparison]::OrdinalIgnoreCase)) {throw 'Review workspace must not overlap normal OpenCPN data.'}
}
$before=@(Get-ProtectedInventory $roots)
$directory=New-RunDirectory $Workspace 'portable-review'
$package=Expand-ReviewArchive $Archive (Join-Path $directory 'extracted')
if ((Get-Digest $Archive) -cne $ExpectedArchiveSha256) {throw 'Archive changed during extraction; no executable started.'}
$manifest=Get-Digest (Join-Path $package 'FILE_SHA256.json')
$product=Assert-ReviewPackage $package $manifest $ExpectedCommit $true
# Actual executable self-test exits before profile initialization and plugin load.
# Suppress native missing-DLL dialogs just for this child creation, as in setup.
if (-not ('OpenNavReviewErrorMode' -as [type])) {
  $framework=[Runtime.InteropServices.RuntimeEnvironment]::GetRuntimeDirectory()
  $previous=[Environment]::CurrentDirectory;Push-Location -LiteralPath $framework
  try {
    [Environment]::CurrentDirectory=$framework
    Add-Type -TypeDefinition @'
using System.Runtime.InteropServices;
public static class OpenNavReviewErrorMode {
 [DllImport("kernel32.dll")] public static extern uint SetErrorMode(uint mode);
}
'@
  } finally {[Environment]::CurrentDirectory=$previous;Pop-Location}
}
$selftest=Join-Path $directory 'loader.json'
$start=New-Object Diagnostics.ProcessStartInfo
$start.FileName=$product.executable;$start.Arguments='--opennav-self-test "'+$selftest+'"'
$start.WorkingDirectory=[IO.Path]::GetDirectoryName($product.executable);$start.UseShellExecute=$false;$start.CreateNoWindow=$true
$start.EnvironmentVariables['PATH']=$env:WINDIR+'\System32;'+$env:WINDIR
$old=[OpenNavReviewErrorMode]::SetErrorMode(0x8003)
try {$process=[Diagnostics.Process]::Start($start)} finally {$null=[OpenNavReviewErrorMode]::SetErrorMode($old)}
try {
  if (-not $process.WaitForExit(30000)) {throw 'Loader self-test timed out; inspect it before continuing. No force termination performed.'}
  if ($process.ExitCode -ne 0) {throw 'Recovery executable self-test failed.'}
} finally {$process.Dispose()}
$identity=Read-Record $selftest
Assert-TrueBoolean $identity.passed 'Actual loader check passed'
foreach ($name in @('test_fixtures','profile_initialized','plugins_loaded')) {
  if ($identity.$name -isnot [bool] -or $identity.$name -ne $false) {throw 'Recovery self-test did not prove isolation.'}
}
if ($identity.commit -cne $ExpectedCommit -or $identity.version -cne '0.4.0-beta2' -or $identity.build_purpose -cne 'INSTALLED PRODUCT') {throw 'Unexpected recovery executable identity.'}
# Only the fresh, hash-verified recovery profile is adjusted. Existing boat data
# and original/plugin settings are never copied here or changed.
$ini=Join-Path $product.profile 'opencpn.conf';$values=Read-ProfileForAudit $ini
Assert-InputOnlyProfile $values
if ($values['OpenNav/AlphaSettings'] -or $values['Settings/NMEADataSource/DataConnections'] -or $values['Settings/ActiveRoute']) {throw 'Recovery seed must have no configured vessel or connection.'}
$addition="`n"
foreach ($plugin in @('chartdldr_pi.dll','dashboard_pi.dll','grib_pi.dll','wmm_pi.dll')) {
  if ($values.ContainsKey('PlugIns/'+$plugin+'/bEnabled')) {throw 'Unexpected preconfigured plugin seed; review package before preparation.'}
  $addition+='[PlugIns/'+$plugin+"]`nbEnabled=0`n"
}
[IO.File]::AppendAllText($ini,$addition,(New-Object Text.UTF8Encoding($false)))
Assert-ReviewProfile $product.profile
Assert-ProtectedInventory $roots $before
$recordPath=Join-Path $directory 'review.json'
Write-Record $recordPath @{owner='OpenNavX.PortableDisplayReview.1';purpose='DISPLAY ONLY; NO INSTALLED, CHART OR HARDWARE ACCEPTANCE';preparedUtc=[DateTime]::UtcNow.ToString('o');workspace=$Workspace;package=$package;commit=$ExpectedCommit;archiveSha256=$ExpectedArchiveSha256;manifestSha256=$manifest;protectedRoots=$roots;protectedFiles=$before;privacy='Private local paths and profile hashes; do not publish this record.'}
[pscustomobject]@{status='prepared';record=$recordPath;recordSha256=(Get-Digest $recordPath);commit=$ExpectedCommit;normalFilesVerified=$before.Count;purpose='Preliminary isolated display review only; original unsupported OpenCPN remains unchanged'} | ConvertTo-Json

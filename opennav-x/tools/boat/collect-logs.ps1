# Export a small allowlist of operational state. No profile, chart paths,
# coordinates, AIS identities, source IDs, credentials or raw bus logs.
[CmdletBinding()]
param([string]$Workspace='C:\XNav')
. (Join-Path $PSScriptRoot 'Common.ps1')
$config=Get-Target $Workspace;$installed=Get-Installed
$directory=New-RunDirectory $Workspace 'diagnostics'
$report=@{schema=1;utc=[DateTime]::UtcNow.ToString('o');version=$installed.ownership.version;commit=$installed.ownership.commit;executableSha256=(Get-Digest $installed.executable);stockSha256=(Get-Digest $config.stockExecutable);os=[Environment]::OSVersion.VersionString;processes=@();privacy='Metadata only. Use in-app explicit Export Diagnostic Bundle for selected detailed reports.'}
foreach ($process in @(Get-Process -Name opencpn -ErrorAction SilentlyContinue)) {
  try {
    if ($process.Path -ieq $installed.executable) {
      $report.processes+=@{pid=$process.Id;workingSetBytes=$process.WorkingSet64;privateBytes=$process.PrivateMemorySize64;handles=$process.HandleCount;cpuSeconds=$process.TotalProcessorTime.TotalSeconds;responding=$process.Responding}
    }
  } finally {$process.Dispose()}
}
$path=Join-Path $directory 'deployment-health.json';Write-Record $path $report
[pscustomobject]@{status='passed';report=$path;sha256=(Get-Digest $path);rawLogsIncluded=$false} | ConvertTo-Json

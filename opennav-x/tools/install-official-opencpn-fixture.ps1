# Only the official prerequisite installer needs elevation. Alpha remains user-level.
param([string]$Setup, [string]$Directory, [string]$Report)
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
if ($env:GITHUB_ACTIONS -ne 'true') { throw 'Disposable native CI fixture only.' }
if ((Get-FileHash -LiteralPath $Setup -Algorithm SHA256).Hash.ToLowerInvariant() -ne 'e949f55de57611afe2fc0dad5a8ac33795c46ba488cb40ca07b65f639a07b8aa') { throw 'Unexpected official setup hash.' }
if ($Directory.Contains('"') -or -not [IO.Path]::IsPathRooted($Directory)) { throw 'Invalid disposable installation path.' }
$Identity = [Security.Principal.WindowsIdentity]::GetCurrent()
$Principal = New-Object Security.Principal.WindowsPrincipal($Identity)
$Result = @{callerAdministrator=$Principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator); powershell=$PSVersionTable.PSVersion.ToString(); status='running'; invocation='Official stock setup only: ShellExecute RunAs, /S, final unquoted /D=path'}
try {
  # Start-Process uses ShellExecute for RunAs, honoring the stock installer's
  # requested elevation. Do not weaken UAC policy or alter the stock executable.
  $Process = Start-Process -FilePath $Setup -ArgumentList ('/S /D=' + $Directory) -Verb RunAs -PassThru -Wait
  $Result.exitCode = $Process.ExitCode
  if ($Process.ExitCode -ne 0) { throw "Official prerequisite installer failed: $($Process.ExitCode)" }
  $Exe = Join-Path $Directory 'opencpn.exe'
  $Result.executableSha256 = (Get-FileHash -LiteralPath $Exe -Algorithm SHA256).Hash.ToLowerInvariant()
  if ($Result.executableSha256 -ne '7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c') { throw 'Installed stock executable mismatch.' }
  $Result.status = 'passed'
} catch {
  $Result.status = 'failed'; $Result.error = $_.Exception.Message
} finally {
  [IO.File]::WriteAllText($Report, ($Result | ConvertTo-Json -Depth 4), (New-Object Text.UTF8Encoding($false)))
}
if ($Result.status -ne 'passed') { throw $Result.error }

# Shared producer-parent setup. Preserve prefix order and inherited PATH bytes:
# native Perl first, then verified Gettext. Do not normalize or deduplicate PATH.
# Dot-source this file in the calling process; a child cannot repair its parent.
function Initialize-WindowsNativePerl {
    if (-not (Test-Path -LiteralPath $env:SKAGER_NATIVE_PERL -PathType Leaf)) {
        throw 'The native OpenSSL build Perl was not selected before MSYS2 setup'
    }
    $NativePerl = (Resolve-Path -LiteralPath $env:SKAGER_NATIVE_PERL).Path
    $env:PATH = "$(Split-Path $NativePerl -Parent);$env:PATH"
    if ((Get-Command perl.exe -CommandType Application -ErrorAction Stop | Select-Object -First 1).Source -ine $NativePerl) {
        throw 'OpenSSL build Perl differs from the preselected native tool'
    }
    if (-not (Test-Path -LiteralPath $env:SKAGER_CURL_TEST_PERL -PathType Leaf)) {
        throw 'MSYS2 curl test Perl was not selected'
    }
}

function Initialize-WindowsGettext {
    param(
        [Parameter(Mandatory=$true)][string]$Python,
        [Parameter(Mandatory=$true)][string]$Receipt,
        [ValidateSet('Ensure','Verify')][string]$Mode = 'Verify',
        [string]$Log
    )
    $Arguments = @((Join-Path $PSScriptRoot 'windows_gettext.py'), $Mode.ToLowerInvariant(), '--receipt', $Receipt)
    # Only the original dependency-build caller may request acquisition.
    if ($Mode -eq 'Ensure') { $Arguments += '--allow-install' }
    if ($Log) {
        & $Python @Arguments 2>&1 | Tee-Object -FilePath $Log -Append | Out-Host
    } else {
        & $Python @Arguments 2>&1 | Out-Host
    }
    if ($LASTEXITCODE -ne 0) { throw "Gettext $Mode failed with exit code $LASTEXITCODE" }
    $Facts = Get-Content -LiteralPath $Receipt -Raw | ConvertFrom-Json
    $Gettext = [string]$Facts.directory
    $env:PATH = "$Gettext;$env:PATH"
    return $Gettext
}

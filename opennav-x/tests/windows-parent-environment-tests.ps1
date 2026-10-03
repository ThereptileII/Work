param([Parameter(Mandatory=$true)][string]$Python)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
$Root = Split-Path $PSScriptRoot -Parent
$Temp = Join-Path ([IO.Path]::GetTempPath()) ('xnav-parent-test-' + [guid]::NewGuid().ToString('N'))
$Before = @{ PATH=$env:PATH; Native=$env:SKAGER_NATIVE_PERL; Curl=$env:SKAGER_CURL_TEST_PERL }
$Checks = 0
function Check([bool]$Value, [string]$Label) {
    if (-not $Value) { throw $Label }; $script:Checks++
}
function Refuses([scriptblock]$Action, [string]$Expected) {
    $Message = ''
    try { & $Action | Out-Null } catch { $Message = $_.Exception.Message }
    Check ($Message.Contains($Expected)) "Expected refusal: $Expected; observed: $Message"
}
try {
    New-Item -ItemType Directory $Temp | Out-Null
    Copy-Item (Join-Path $Root 'tools/windows-parent-environment.ps1') $Temp
    # Only the external tool-discovery/probe boundaries are fixture-controlled.
    # The production parent initialization functions run unchanged.
    @'
import json, pathlib, sys
p=pathlib.Path(__file__).parent
(p/'arguments.json').write_text(json.dumps(sys.argv[1:]))
if (p/'fail').exists(): sys.exit(19)
print('fixture gettext probe completed')
'@ | Set-Content (Join-Path $Temp 'windows_gettext.py') -Encoding utf8
    . (Join-Path $Temp 'windows-parent-environment.ps1')
    $NativeDirectory = Join-Path $Temp 'native'
    New-Item -ItemType Directory $NativeDirectory | Out-Null
    $env:SKAGER_NATIVE_PERL = Join-Path $NativeDirectory 'perl.exe'
    $env:SKAGER_CURL_TEST_PERL = Join-Path $Temp 'curl-perl.exe'
    Set-Content $env:SKAGER_NATIVE_PERL 'fixture native Perl'
    Set-Content $env:SKAGER_CURL_TEST_PERL 'fixture curl Perl'
    $script:SelectedPerl = $env:SKAGER_NATIVE_PERL
    function Get-Command {
        param($Name, $CommandType, $ErrorAction)
        if ($Name -cne 'perl.exe' -or $CommandType -cne 'Application') { throw 'Unexpected discovery' }
        [pscustomobject]@{ Source=$script:SelectedPerl }
    }
    $Inherited = 'Existing;MiXeD;;Existing;'
    $env:PATH = $Inherited
    Initialize-WindowsNativePerl
    Check ($env:PATH -ceq "$NativeDirectory;$Inherited") 'Native prefix altered inherited PATH'
    $Receipt = Join-Path $Temp 'gettext.json'
    $Gettext = Join-Path $Temp 'gettext'
    @{directory=$Gettext} | ConvertTo-Json | Set-Content $Receipt -Encoding utf8
    $ReceiptHash = (Get-FileHash $Receipt).Hash
    $Selected = Initialize-WindowsGettext -Python $Python -Receipt $Receipt -Mode Verify
    Check ($Selected -ceq $Gettext) 'Gettext directory return is not a single string'
    Check ($env:PATH -ceq "$Gettext;$NativeDirectory;$Inherited") 'Parent prefix order or inherited bytes changed'
    $ArgsSeen = @(Get-Content (Join-Path $Temp 'arguments.json') -Raw | ConvertFrom-Json)
    Check (($ArgsSeen -join '|') -ceq "verify|--receipt|$Receipt") 'Verify requested acquisition or wrong receipt'
    Check ((Get-FileHash $Receipt).Hash -ceq $ReceiptHash) 'Verify refreshed original receipt'
    $env:PATH = $Inherited
    $null = Initialize-WindowsGettext -Python $Python -Receipt $Receipt -Mode Ensure -Log (Join-Path $Temp 'probe.log')
    $ArgsSeen = @(Get-Content (Join-Path $Temp 'arguments.json') -Raw | ConvertFrom-Json)
    Check (($ArgsSeen -join '|') -ceq "ensure|--receipt|$Receipt|--allow-install") 'Original build acquisition contract changed'
    Set-Content (Join-Path $Temp 'fail') 'fail'
    $env:PATH = $Inherited
    Refuses { Initialize-WindowsGettext -Python $Python -Receipt $Receipt -Mode Verify } 'exit code 19'
    Check ($env:PATH -ceq $Inherited) 'Failed Gettext probe modified PATH'
    $script:SelectedPerl = Join-Path $Temp 'wrong-perl.exe'
    Refuses { Initialize-WindowsNativePerl } 'differs from the preselected'
    $script:SelectedPerl = $env:SKAGER_NATIVE_PERL
    Remove-Item $env:SKAGER_CURL_TEST_PERL
    Refuses { Initialize-WindowsNativePerl } 'MSYS2 curl test Perl was not selected'
    Remove-Item $env:SKAGER_NATIVE_PERL
    Refuses { Initialize-WindowsNativePerl } 'was not selected before MSYS2 setup'
    # Parse changed production scripts; no build or native tool is executed.
    foreach ($Name in @('windows-parent-environment.ps1','build-pristine-windows.ps1')) {
        $Tokens=$null; $Errors=$null
        $null=[Management.Automation.Language.Parser]::ParseFile((Join-Path $Root "tools/$Name"),[ref]$Tokens,[ref]$Errors)
        Check ($Errors.Count -eq 0) "PowerShell parse failed: $Name"
    }
    Write-Output "$Checks parent initialization checks passed (mocked discovery/probe; not native proof)"
} finally {
    $env:PATH=$Before.PATH; $env:SKAGER_NATIVE_PERL=$Before.Native; $env:SKAGER_CURL_TEST_PERL=$Before.Curl
    Remove-Item Function:Get-Command -ErrorAction SilentlyContinue
    if (Test-Path $Temp) { Remove-Item $Temp -Recurse -Force }
}

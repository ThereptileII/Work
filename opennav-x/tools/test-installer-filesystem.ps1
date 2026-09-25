# Exercise the actual engine's filesystem helpers on disposable native NTFS.
# Import only function AST nodes; never execute the installation entry point.
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
if ($env:GITHUB_ACTIONS -ne 'true' -or $env:OS -ne 'Windows_NT') { throw 'Disposable Windows CI only.' }
$Source = Join-Path $PSScriptRoot '../installer/windows/Lifecycle.ps1'
$ParseErrors = $null
$Ast = [System.Management.Automation.Language.Parser]::ParseFile($Source, [ref]$null, [ref]$ParseErrors)
if ($ParseErrors) { throw ($ParseErrors | Out-String) }
foreach ($Name in @('Log','Hash','PlainPath','RelativePath','ReadJson','AtomicJson','PeArchitecture','FileRecords','VerifyFiles','SelfTest')) {
  $Definitions = @($Ast.FindAll({ param($Node) $Node -is [System.Management.Automation.Language.FunctionDefinitionAst] -and $Node.Name -eq $Name }, $true))
  if ($Definitions.Count -ne 1) { throw "Expected one actual engine function: $Name" }
  . ([scriptblock]::Create($Definitions[0].Extent.Text))
}
$Utf8 = New-Object System.Text.UTF8Encoding($false)
$SessionLog = New-Object System.Collections.Generic.List[string]
$Fixture = Join-Path ([IO.Path]::GetTempPath()) ('OpenNav filesystem ' + [guid]::NewGuid().ToString('N'))
$null = New-Item -ItemType Directory -Path $Fixture
$Checks = 0
$Junction = $null
function Check([bool]$Ok, [string]$Name) {
  if (-not $Ok) { throw "FAILED: $Name" }
  $script:Checks++; Write-Host "PASS: $Name"
}
function Refuses([scriptblock]$Operation, [string]$Name) {
  $Failed = $false
  try { & $Operation | Out-Null } catch { $Failed = $true }
  Check $Failed $Name
}
try {
  $Vector = Join-Path $Fixture 'sha256-vector.txt'
  [IO.File]::WriteAllText($Vector,'abc',$Utf8)
  Check ((Hash $Vector) -ceq 'ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad') 'Known SHA-256 vector through the actual engine'
  $Path = RelativePath $Fixture 'app/uidata/markicons/Service-Wine&Dine.svg'
  Check ($Path.EndsWith('Service-Wine&Dine.svg')) 'Literal upstream ampersand filename'
  foreach ($Invalid in @('../escape','app/../escape','/absolute','app\escape','app//empty','app/name.','app/name ','app/CON.svg','app/NUL','app/C:escape','app/a*','app/a?')) {
    Refuses { RelativePath $Fixture $Invalid } ('Unsafe relative path: '+$Invalid)
  }
  Refuses { PlainPath '\\server\share\file' } 'UNC path refused'
  Refuses { PlainPath 'C:\' } 'Drive root refused'
  $Target = Join-Path $Fixture 'target'; $null = New-Item -ItemType Directory -Path $Target
  $Junction = Join-Path $Fixture 'redirect'; $null = New-Item -ItemType Junction -Path $Junction -Value $Target
  Refuses { RelativePath $Fixture 'redirect/child' } 'Nested reparse point refused'
  $Record = Join-Path $Fixture 'record.json'
  AtomicJson $Record @{epoch=1;meaning='original'}
  Check ((ReadJson $Record).epoch -eq 1) 'Atomic first publication'
  AtomicJson $Record @{epoch=2;meaning='replacement'}
  Check ((ReadJson $Record).epoch -eq 2) 'Atomic replacement on native filesystem'
  Check (@(Get-ChildItem -LiteralPath $Fixture -Filter '*.tmp').Count -eq 0) 'No temporary publication residue'
  Refuses { ReadJson $Record 2 } 'Oversized JSON refused'
  $Bundle = Join-Path $Fixture 'bundle'; $null = New-Item -ItemType Directory -Path $Bundle
  [IO.File]::WriteAllText((Join-Path $Bundle 'file&name.txt'),'original',$Utf8)
  $Files = @(FileRecords $Bundle)
  VerifyFiles $Bundle $Files
  Check ($Files.Count -eq 1) 'Owned inventory hash verification'
  [IO.File]::WriteAllText((Join-Path $Bundle 'file&name.txt'),'corrupt',$Utf8)
  Refuses { VerifyFiles $Bundle $Files } 'Corrupt owned file refused'
  $Dll = Join-Path ([Environment]::GetFolderPath('SystemX86')) 'kernel32.dll'
  Check ((PeArchitecture $Dll) -eq 'x86') 'Native PE i386 architecture'
  Refuses { PeArchitecture $Record } 'Non-PE data refused'
  # This explicit test-only executable exercises the real process/identity
  # wrapper, independently of the much slower integrated OpenCPN build.
  # Its JSON is a fixture, not a claim of real chart/DLL/profile validation.
  $LoaderStage = Join-Path $Fixture 'loader'; $LoaderApp = Join-Path $LoaderStage 'app'
  $null = New-Item -ItemType Directory -Path $LoaderApp -Force
  $LoaderSource = Join-Path $Fixture 'LoaderFixture.cs'
  $LoaderExe = Join-Path $LoaderApp 'opencpn.exe'
  $Code = @'
using System;
using System.IO;
class OpenNavLoaderContractFixture {
  static int Main(string[] args) {
    if (args.Length != 2 || args[0] != "--opennav-self-test") return 64;
    File.WriteAllText(args[1], "{\"passed\":true,\"commit\":\"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\",\"version\":\"loader-fixture\",\"profile_initialized\":false,\"plugins_loaded\":false}");
    return 0;
  }
}
'@
  [IO.File]::WriteAllText($LoaderSource,$Code,$Utf8)
  $Compiler = Join-Path ([Runtime.InteropServices.RuntimeEnvironment]::GetRuntimeDirectory()) 'csc.exe'
  & $Compiler /nologo /target:exe /platform:x86 ('/out:'+$LoaderExe) $LoaderSource
  if ($LASTEXITCODE -ne 0) { throw 'Could not compile native loader contract fixture.' }
  SelfTest $LoaderStage 'aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa' 'loader-fixture'
  Check (@(Get-ChildItem -LiteralPath $LoaderStage -Filter 'loader-*.json').Count -eq 0) 'Actual loader wrapper waits for exit and consumes its verified fixture report'
  Refuses { SelfTest $LoaderStage 'bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb' 'loader-fixture' } 'Loader wrapper refuses another executable identity'
  Write-Host "$Checks native filesystem checks passed in PowerShell $($PSVersionTable.PSVersion), $([IntPtr]::Size * 8)-bit host."
} finally {
  if ($Junction -and (Test-Path -LiteralPath $Junction)) { [IO.Directory]::Delete($Junction) }
  Remove-Item -LiteralPath $Fixture -Recurse -Force
}

# Exercise the actual engine's filesystem helpers on disposable native NTFS.
# Import only function AST nodes; never execute the installation entry point.
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
if ($env:GITHUB_ACTIONS -ne 'true' -or $env:OS -ne 'Windows_NT') { throw 'Disposable Windows CI only.' }
$Source = Join-Path $PSScriptRoot '../installer/windows/Lifecycle.ps1'
$ParseErrors = $null
$Ast = [System.Management.Automation.Language.Parser]::ParseFile($Source, [ref]$null, [ref]$ParseErrors)
if ($ParseErrors) { throw ($ParseErrors | Out-String) }
foreach ($Name in @('Log','Hash','PlainPath','RelativePath','ReadJson','AtomicJson','PeArchitecture','FileRecords','VerifyFiles','SelfTest','ExtractPayload','Failure')) {
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
$FailurePoint = ''
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
  # Qualify the permission fixture before the expensive installer matrix.
  # Retain the old provider's diagnostic if a pwsh-inherited module path broke it.
  try { $null = Get-Acl -LiteralPath $Fixture; Write-Host 'ACL provider probe: available' }
  catch { Write-Host ('ACL provider probe: ' + $_.Exception.Message) }
  $Denied = Join-Path $Fixture 'denied staging'; $null = [IO.Directory]::CreateDirectory($Denied)
  $SavedAcl = Join-Path $Fixture 'original-acl.txt'
  $AclTool = Join-Path $PSScriptRoot 'installer-deny-directory.ps1'
  try {
    & $AclTool -Directory $Denied -Saved $SavedAcl
    Refuses { [IO.Directory]::CreateDirectory((Join-Path $Denied 'new generation')) } 'Native NTFS staging denial is actually enforced'
  } finally {
    if ([IO.File]::Exists($SavedAcl)) { & $AclTool -Directory $Denied -Saved $SavedAcl -Restore }
  }
  $null = [IO.Directory]::CreateDirectory((Join-Path $Denied 'restored generation'))
  $RestoredSddl = [IO.Directory]::GetAccessControl($Denied,[Security.AccessControl.AccessControlSections]::Access).GetSecurityDescriptorSddlForm([Security.AccessControl.AccessControlSections]::Access)
  $SavedSddl = [IO.File]::ReadAllText($SavedAcl)
  Write-Host ('Original disposable DACL: '+$SavedSddl)
  Write-Host ('Restored disposable DACL: '+$RestoredSddl)
  # SetNamedSecurityInfo records SE_DACL_AUTO_INHERITED after applying the
  # unchanged inherited entries. This bookkeeping flag is not an ACE or an
  # inheritance-protection change. Compare exact ACL bytes and every other flag.
  $BeforeAcl = New-Object System.Security.AccessControl.RawSecurityDescriptor($SavedSddl)
  $AfterAcl = New-Object System.Security.AccessControl.RawSecurityDescriptor($RestoredSddl)
  $BeforeBytes = New-Object byte[] $BeforeAcl.DiscretionaryAcl.BinaryLength
  $AfterBytes = New-Object byte[] $AfterAcl.DiscretionaryAcl.BinaryLength
  $BeforeAcl.DiscretionaryAcl.GetBinaryForm($BeforeBytes,0)
  $AfterAcl.DiscretionaryAcl.GetBinaryForm($AfterBytes,0)
  $Mask = -bnot [int][Security.AccessControl.ControlFlags]::DiscretionaryAclAutoInherited
  Check (([Convert]::ToBase64String($BeforeBytes) -ceq [Convert]::ToBase64String($AfterBytes)) -and
         (([int]$BeforeAcl.ControlFlags -band $Mask) -eq ([int]$AfterAcl.ControlFlags -band $Mask))) 'Permission fixture restores exact DACL entries, inheritance protection and directory creation'
  $Vector = Join-Path $Fixture 'sha256-vector.txt'
  [IO.File]::WriteAllText($Vector,'abc',$Utf8)
  Check ((Hash $Vector) -ceq 'ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad') 'Known SHA-256 vector through the actual engine'
  $Path = RelativePath $Fixture 'app/uidata/markicons/Service-Wine&Dine.svg'
  Check ($Path.EndsWith('Service-Wine&Dine.svg')) 'Literal upstream ampersand filename'
  $LocalePath = RelativePath $Fixture 'app/share/locale/ca@valencia/LC_MESSAGES/wxstd.mo'
  Check ($LocalePath.Contains('ca@valencia')) 'Pinned upstream wx locale modifier remains a literal safe filename'
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
  $Before = Hash $Record
  $Lock = [IO.File]::Open($Record, [IO.FileMode]::Open, [IO.FileAccess]::Read, [IO.FileShare]::Read)
  try { Refuses { AtomicJson $Record @{epoch=3} } 'Locked atomic record refuses replacement' }
  finally { $Lock.Dispose() }
  Check ((Hash $Record) -ceq $Before) 'Locked publication preserves exact previous durable state'
  Check (@(Get-ChildItem -LiteralPath $Fixture -Filter '*.tmp').Count -eq 0) 'Failed locked publication cleans only its temporary file'
  Refuses { ReadJson $Record 2 } 'Oversized JSON refused'
  $Bundle = Join-Path $Fixture 'bundle'; $null = New-Item -ItemType Directory -Path $Bundle
  [IO.File]::WriteAllText((Join-Path $Bundle 'file&name.txt'),'original',$Utf8)
  $Files = @(FileRecords $Bundle)
  VerifyFiles $Bundle $Files
  Check ($Files.Count -eq 1) 'Owned inventory hash verification'
  [IO.File]::WriteAllText((Join-Path $Bundle 'file&name.txt'),'corrupt',$Utf8)
  Refuses { VerifyFiles $Bundle $Files } 'Corrupt owned file refused'
  Add-Type -AssemblyName System.IO.Compression.FileSystem
  $Zip = Join-Path $Fixture 'payload.zip'
  [IO.Compression.ZipFile]::CreateFromDirectory($Bundle, $Zip)
  $Extract = Join-Path $Fixture 'extracted'
  Refuses { ExtractPayload $Zip $Extract $Files } 'Corrupt ZIP content refused against manifest hashes'
  Remove-Item -LiteralPath $Extract -Recurse -Force
  $Files = @(FileRecords $Bundle)
  ExtractPayload $Zip $Extract $Files
  Check ((Hash (Join-Path $Extract 'file&name.txt')) -ceq $Files[0].sha256) 'Valid bounded extraction checks every content hash'
  Refuses { ExtractPayload $Zip (Join-Path $Fixture 'wrong-inventory') @() } 'ZIP inventory mismatch rejected before publication'
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
using System.Runtime.InteropServices;
class OpenNavLoaderContractFixture {
  [DllImport("kernel32.dll")] static extern uint GetErrorMode();
  static int Main(string[] args) {
    if (args.Length != 2 || args[0] != "--opennav-self-test") return 64;
    if ((GetErrorMode() & 0x8003) != 0x8003) return 65;
    File.WriteAllText(args[1], "{\"passed\":true,\"commit\":\"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\",\"version\":\"loader-fixture\",\"profile_initialized\":false,\"plugins_loaded\":false}");
    return 0;
  }
}
'@
  [IO.File]::WriteAllText($LoaderSource,$Code,$Utf8)
  $Compiler = Join-Path ([Runtime.InteropServices.RuntimeEnvironment]::GetRuntimeDirectory()) 'csc.exe'
  & $Compiler /nologo /target:exe /platform:x86 ('/out:'+$LoaderExe) $LoaderSource
  if ($LASTEXITCODE -ne 0) { throw 'Could not compile native loader contract fixture.' }
  # NSIS's native System.dll must never become Add-Type's .NET reference.
  Copy-Item -LiteralPath $Dll -Destination (Join-Path $Fixture 'System.dll')
  $BeforeDirectory = [Environment]::CurrentDirectory
  Push-Location -LiteralPath $Fixture
  try {
    [Environment]::CurrentDirectory = $Fixture
    SelfTest $LoaderStage 'aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa' 'loader-fixture'
    Check (((Get-Location).Path -eq $Fixture) -and ([Environment]::CurrentDirectory -eq $Fixture)) 'Loader interop ignores native System.dll shadow and restores caller directories'
  } finally {
    [Environment]::CurrentDirectory = $BeforeDirectory
    Pop-Location
  }
  Check (@(Get-ChildItem -LiteralPath $LoaderStage -Filter 'loader-*.json').Count -eq 0) 'Actual loader wrapper waits for exit and consumes its verified fixture report'
  $BeforeErrorMode = [OpenNav.InstallerErrorMode]::GetErrorMode()
  SelfTest $LoaderStage 'aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa' 'loader-fixture'
  Check ([OpenNav.InstallerErrorMode]::GetErrorMode() -eq $BeforeErrorMode) 'Loader child inherits noninteractive errors and successful launch restores parent mode'
  Refuses { SelfTest $LoaderStage 'bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb' 'loader-fixture' } 'Loader wrapper refuses another executable identity'
  Check ([OpenNav.InstallerErrorMode]::GetErrorMode() -eq $BeforeErrorMode) 'Rejected loader identity preserves parent error mode'
  Write-Host "$Checks native filesystem checks passed in PowerShell $($PSVersionTable.PSVersion), $([IntPtr]::Size * 8)-bit host."
} finally {
  if ($Junction -and (Test-Path -LiteralPath $Junction)) { [IO.Directory]::Delete($Junction) }
  Remove-Item -LiteralPath $Fixture -Recurse -Force
}

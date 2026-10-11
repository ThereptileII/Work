$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

# Load the exact installer functions without invoking the installer's top-level
# transaction. These fixtures never access a user installation.
$source = Join-Path (Split-Path $PSScriptRoot -Parent) 'installer/windows/Lifecycle.ps1'
$tokens = $null; $errors = $null
$ast = [Management.Automation.Language.Parser]::ParseFile($source, [ref]$tokens, [ref]$errors)
if ($errors.Count) { throw 'Lifecycle.ps1 does not parse.' }
$names = @('Hash','PlainPath','RelativePath','PeU16','PeU32','PeMachine','PeRvaOffset',
  'PeImportName','GetPeImports','GetCandidateSystemX86','AssertCandidateTlsRuntime')
$definitions = @($ast.FindAll({
  param($node)
  $node -is [Management.Automation.Language.FunctionDefinitionAst] -and $node.Name -in $names
}, $true))
if ($definitions.Count -ne $names.Count) { throw 'Installer PE gate functions are missing or duplicated.' }
foreach ($definition in $definitions) { . ([scriptblock]::Create($definition.Extent.Text)) }
# Exercise the production SHA-256 function for ordinary files. Known obsolete
# identities are injected only for named inert fixtures; no stock DLLs execute
# or need to be copied into this source test. The candidate walk and refusal
# remain the actual installer functions, including transitive import traversal.
$script:realHash = ${function:Hash}
$script:fixtureHashes = @{}
function Hash([string]$Path) {
  if ($script:fixtureHashes.ContainsKey($Path)) { return $script:fixtureHashes[$Path] }
  return (& $script:realHash $Path)
}
# Linux can execute the parser fixtures, while Windows CI exercises the real
# installer path guard and OS-provided SystemX86 resolver. Replace only these
# two platform-dependent functions on Linux; all PE logic is the actual AST.
if ([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT) {
  function RelativePath([string]$Base, [string]$Name) { Join-Path $Base $Name }
  function GetCandidateSystemX86 { return $script:fixtureSystem }
}
# The gate is exercised without the installer's top-level transaction, so it
# has no session log. Capture what it reports instead of writing one.
$script:fixtureLog = New-Object 'System.Collections.Generic.List[string]'
function Log([string]$Message) { $script:fixtureLog.Add($Message) }

function SetU16([byte[]]$Bytes, [int]$At, [int]$Value) {
  [Array]::Copy([BitConverter]::GetBytes([uint16]$Value), 0, $Bytes, $At, 2)
}
function SetU32([byte[]]$Bytes, [int]$At, [uint32]$Value) {
  [Array]::Copy([BitConverter]::GetBytes($Value), 0, $Bytes, $At, 4)
}
function NewPe([string]$Path, [string]$Import = '', [bool]$Delay = $false) {
  [byte[]]$bytes = New-Object byte[] 1536
  SetU16 $bytes 0 0x5a4d; SetU32 $bytes 60 0x80
  SetU32 $bytes 0x80 0x4550; SetU16 $bytes 0x84 0x14c
  SetU16 $bytes 0x86 1; SetU16 $bytes 0x94 224
  SetU16 $bytes 0x98 0x10b
  SetU32 $bytes 0xc4 0x400000
  SetU32 $bytes 0xf4 16
  # One .rdata section: RVA 0x1000 maps to file offset 0x200.
  SetU32 $bytes 0x180 0x400; SetU32 $bytes 0x184 0x1000
  SetU32 $bytes 0x188 0x400; SetU32 $bytes 0x18c 0x200
  if ($Import) {
    $entry = if ($Delay) { 0x98 + 96 + 13 * 8 } else { 0x98 + 96 + 8 }
    $table = if ($Delay) { 0x1140 } else { 0x1100 }
    $raw = if ($Delay) { 0x340 } else { 0x300 }
    SetU32 $bytes $entry $table
    SetU32 $bytes ($entry + 4) $(if ($Delay) { 64 } else { 40 })
    if ($Delay) {
      SetU32 $bytes $raw 1
      SetU32 $bytes ($raw + 4) 0x1180
    } else {
      SetU32 $bytes ($raw + 12) 0x1180
    }
    $name = [Text.Encoding]::ASCII.GetBytes($Import + [char]0)
    [Array]::Copy($name, 0, $bytes, 0x380, $name.Length)
  }
  [IO.File]::WriteAllBytes($Path, $bytes)
}
function RequireFailure([scriptblock]$Action, [string]$Pattern) {
  $failed = $false
  try { & $Action }
  catch {
    if ($_.Exception.Message -notmatch $Pattern) { throw }
    $failed = $true
  }
  if (-not $failed) { throw "Expected rejection: $Pattern" }
}
$temp = Join-Path ([IO.Path]::GetTempPath()) ('opennav-pe-' + [guid]::NewGuid().ToString('N'))
$app = Join-Path $temp 'stage/app'
$plugins = Join-Path $app 'plugins'
$system = Join-Path $temp 'Windows/SysWOW64'
try {
  $null = New-Item -ItemType Directory -Force -Path $plugins, $system
  $script:fixtureSystem = $system
  [IO.File]::WriteAllBytes((Join-Path $system 'kernel32.dll'), [byte[]]@(1))
  $exe = Join-Path $app 'opencpn.exe'
  $plugin = Join-Path $plugins 'retained_pi.dll'
  NewPe $exe 'kernel32.dll'
  NewPe $plugin 'libcurl.dll'
  NewPe (Join-Path $app 'libcurl.dll')
  $originalPluginHash = Hash $plugin
  foreach ($obsoleteHash in @(
      '2bede6f2840cfe0a4e88d5c3fa648c170509c9ceed438c7e8440340541ca0446',
      '05752a0550349337af8b7e25362d527f6a30261dd3d1e4db61bf707c9fac589d')) {
    # A renamed retained plugin-local DLL is still rejected by byte identity.
    $script:fixtureHashes[$plugin] = $obsoleteHash
    RequireFailure { AssertCandidateTlsRuntime (Join-Path $temp 'stage') } ('Known obsolete TLS runtime in candidate: .*' + $obsoleteHash)
    $script:fixtureHashes.Clear()
    if ((Hash $plugin) -cne $originalPluginHash) { throw 'TLS rejection changed the retained plugin bytes.' }
  }
  # Also inspect imported local modules whose extension is neither DLL nor EXE.
  $retainedHelper = Join-Path $plugins 'retained-tls.drv'
  NewPe $retainedHelper
  NewPe $plugin 'retained-tls.drv'
  $script:fixtureHashes[$retainedHelper] = '2bede6f2840cfe0a4e88d5c3fa648c170509c9ceed438c7e8440340541ca0446'
  RequireFailure { AssertCandidateTlsRuntime (Join-Path $temp 'stage') } 'Known obsolete TLS runtime in candidate: OpenSSL 3.0.5'
  $script:fixtureHashes.Clear()
  AssertCandidateTlsRuntime (Join-Path $temp 'stage')
  Remove-Item -LiteralPath $retainedHelper
  NewPe $plugin 'libcurl.dll'
  $controlRejected = $false
  try { RequireFailure { AssertCandidateTlsRuntime (Join-Path $temp 'stage') } 'MISSING' }
  catch { if ($_.Exception.Message -match 'Expected rejection') { $controlRejected = $true } else { throw } }
  if (-not $controlRejected) { throw 'RequireFailure accepted a passing action.' }
  AssertCandidateTlsRuntime (Join-Path $temp 'stage')
  Remove-Item -LiteralPath (Join-Path $app 'libcurl.dll')
  RequireFailure { AssertCandidateTlsRuntime (Join-Path $temp 'stage') } 'Missing app-local PE import libcurl.dll'
  NewPe $plugin 'libeay32.dll'
  RequireFailure { AssertCandidateTlsRuntime (Join-Path $temp 'stage') } 'Unsupported legacy TLS runtime dependency in candidate: import libeay32.dll'
  NewPe $plugin 'ssleay32.dll' $true
  RequireFailure { AssertCandidateTlsRuntime (Join-Path $temp 'stage') } 'Unsupported legacy TLS runtime dependency in candidate: import ssleay32.dll'
  NewPe $plugin 'missing.dll' $true
  RequireFailure { AssertCandidateTlsRuntime (Join-Path $temp 'stage') } 'Missing app-local PE import missing.dll'
  NewPe $plugin 'kernel32.dll' $true
  AssertCandidateTlsRuntime (Join-Path $temp 'stage')
  NewPe $plugin 'KERNEL32.DLL'
  AssertCandidateTlsRuntime (Join-Path $temp 'stage')
  NewPe $plugin 'WINSPOOL.DRV'
  if (@(GetPeImports $plugin) -notcontains 'winspool.drv') { throw 'Safe .DRV module name was rejected.' }
  NewPe $plugin 'opencpn.exe'
  AssertCandidateTlsRuntime (Join-Path $temp 'stage')
  NewPe $plugin 'helper.drv'
  NewPe (Join-Path $app 'helper.drv') 'libeay32.dll'
  RequireFailure { AssertCandidateTlsRuntime (Join-Path $temp 'stage') } 'Unsupported legacy TLS runtime dependency in candidate: import libeay32.dll'
  Remove-Item -LiteralPath (Join-Path $app 'helper.drv')
  # SCRUM-334: a retained third-party plugin binary that is not an x86 image
  # cannot be loaded by the 32-bit host, so it is skipped for import scanning
  # instead of aborting the installation. rtlsdr_pi ships such a file.
  NewPe $plugin 'kernel32.dll'
  [byte[]]$foreign = [IO.File]::ReadAllBytes($plugin)
  SetU16 $foreign 0x84 0x8664
  [IO.File]::WriteAllBytes($plugin, $foreign)
  if ((PeMachine $plugin) -ne 0x8664) { throw 'PeMachine did not report the foreign machine type.' }
  $script:fixtureLog.Clear()
  AssertCandidateTlsRuntime (Join-Path $temp 'stage')
  if (-not (@($script:fixtureLog) -match 'Skipped non-x86 binary')) { throw 'Skipping a foreign binary was not reported.' }
  # The skip does not depend on what the unloadable binary imports.
  NewPe $plugin 'libeay32.dll'
  [byte[]]$foreign = [IO.File]::ReadAllBytes($plugin)
  SetU16 $foreign 0x84 0x8664
  [IO.File]::WriteAllBytes($plugin, $foreign)
  AssertCandidateTlsRuntime (Join-Path $temp 'stage')
  # An x86 binary carrying the same import is still refused, so the gate has
  # not been weakened for anything the host can actually load.
  NewPe $plugin 'libeay32.dll'
  if ((PeMachine $plugin) -ne 0x14c) { throw 'PeMachine did not report the x86 machine type.' }
  RequireFailure { AssertCandidateTlsRuntime (Join-Path $temp 'stage') } 'Unsupported legacy TLS runtime dependency in candidate: import libeay32.dll'
  # Malformed x86 images keep failing with their exact diagnosis; the skip
  # covers only a cleanly parsed foreign machine type.
  NewPe $plugin 'kernel32.dll'
  [byte[]]$broken = [IO.File]::ReadAllBytes($plugin)
  SetU32 $broken 0x300 1; SetU32 $broken 0x30c 0x7fffffff
  [IO.File]::WriteAllBytes($plugin, $broken)
  RequireFailure { AssertCandidateTlsRuntime (Join-Path $temp 'stage') } 'PE RVA does not map'
  NewPe $plugin
  [byte[]]$broken = [IO.File]::ReadAllBytes($plugin)
  SetU32 $broken 0x188 0x1000
  [IO.File]::WriteAllBytes($plugin, $broken)
  RequireFailure { AssertCandidateTlsRuntime (Join-Path $temp 'stage') } 'Invalid PE section range'
  NewPe $plugin
  [byte[]]$broken = [IO.File]::ReadAllBytes($plugin)
  SetU16 $broken 0x86 2
  SetU32 $broken 0x1a8 0x80; SetU32 $broken 0x1ac 0x1200
  SetU32 $broken 0x1b0 0x80; SetU32 $broken 0x1b4 0x400
  [IO.File]::WriteAllBytes($plugin, $broken)
  RequireFailure { AssertCandidateTlsRuntime (Join-Path $temp 'stage') } 'Overlapping PE section RVAs'
  [IO.File]::WriteAllBytes($plugin, [byte[]]@(0x4d,0x5a))
  RequireFailure { AssertCandidateTlsRuntime (Join-Path $temp 'stage') } 'Invalid PE file size'
  Write-Output 'Installer staged PE import closure fixtures passed.'
} finally {
  Remove-Item -LiteralPath $temp -Recurse -Force -ErrorAction SilentlyContinue
}

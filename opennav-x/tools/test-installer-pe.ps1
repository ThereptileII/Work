param([string[]]$RetainedPeFiles = @())

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

# Load the exact installer functions without invoking the installer's top-level
# transaction. These fixtures never access a user installation.
$source = Join-Path (Split-Path $PSScriptRoot -Parent) 'installer/windows/Lifecycle.ps1'
$tokens = $null; $errors = $null
$ast = [Management.Automation.Language.Parser]::ParseFile($source, [ref]$tokens, [ref]$errors)
if ($errors.Count) { throw 'Lifecycle.ps1 does not parse.' }
$names = @('Hash','PlainPath','RelativePath','PeU16','PeU32','PeMachine','PeRvaOffset',
  'PeModuleName','PeForwarderModule','PeImportName','GetPeImports','GetCandidateSystemX86','AssertCandidateTlsRuntime')
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
function SetU64([byte[]]$Bytes, [int]$At, [uint64]$Value) {
  [Array]::Copy([BitConverter]::GetBytes($Value), 0, $Bytes, $At, 8)
}
function NewPe64([string]$Path, [string]$Import) {
  # Inert PE32+ image with its own complete AMD64 optional header, section,
  # import descriptor and 64-bit lookup/address thunks. Never execute it.
  [byte[]]$bytes = New-Object byte[] 1536
  SetU16 $bytes 0 0x5a4d; SetU32 $bytes 60 0x80
  SetU32 $bytes 0x80 0x4550; SetU16 $bytes 0x84 0x8664
  SetU16 $bytes 0x86 1; SetU16 $bytes 0x94 240
  SetU16 $bytes 0x96 0x22  # Executable, large-address-aware, not DLL.
  SetU16 $bytes 0x98 0x20b
  SetU32 $bytes 0x9c 0x400; SetU32 $bytes 0xa8 0x1000
  SetU32 $bytes 0xac 0x1000; SetU64 $bytes 0xb0 0x140000000
  SetU32 $bytes 0xb8 0x1000; SetU32 $bytes 0xbc 0x200
  SetU16 $bytes 0xc0 6; SetU16 $bytes 0xc8 6
  SetU32 $bytes 0xd0 0x2000; SetU32 $bytes 0xd4 0x200
  SetU16 $bytes 0xdc 3  # Console subsystem.
  SetU64 $bytes 0xe0 0x100000; SetU64 $bytes 0xe8 0x1000
  SetU64 $bytes 0xf0 0x100000; SetU64 $bytes 0xf8 0x1000
  SetU32 $bytes 0x104 16
  SetU32 $bytes 0x110 0x1100; SetU32 $bytes 0x114 40
  SetU32 $bytes 0x168 0x1160; SetU32 $bytes 0x16c 16
  [Array]::Copy([Text.Encoding]::ASCII.GetBytes('.text'), 0, $bytes, 0x188, 5)
  SetU32 $bytes 0x190 0x400; SetU32 $bytes 0x194 0x1000
  SetU32 $bytes 0x198 0x400; SetU32 $bytes 0x19c 0x200
  SetU32 $bytes 0x1ac 0x60000020
  $bytes[0x200] = 0xc3  # RET, never called by these parser fixtures.
  SetU32 $bytes 0x300 0x1140; SetU32 $bytes 0x30c 0x1180
  SetU32 $bytes 0x310 0x1160
  SetU64 $bytes 0x340 0x11c0; SetU64 $bytes 0x360 0x11c0
  $name = [Text.Encoding]::ASCII.GetBytes($Import + [char]0)
  [Array]::Copy($name, 0, $bytes, 0x380, $name.Length)
  [Array]::Copy([Text.Encoding]::ASCII.GetBytes('ExitProcess'), 0, $bytes, 0x3c2, 11)
  [IO.File]::WriteAllBytes($Path, $bytes)
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
function NewPeForwarders([string]$Path, [string[]]$Targets) {
  NewPe $Path
  [byte[]]$bytes = [IO.File]::ReadAllBytes($Path)
  # Independent inert export directory within .rdata. The EAT may be outside
  # the export directory in PE, but the forwarder string AND NUL must be inside.
  SetU32 $bytes 0xf8 0x1200; SetU32 $bytes 0xfc 0x200
  SetU32 $bytes 0x414 $Targets.Count; SetU32 $bytes 0x41c 0x1240
  $next = 0x480
  for ($i = 0; $i -lt $Targets.Count; $i++) {
    SetU32 $bytes (0x440 + 4 * $i) ($next + 0xe00)
    $raw = [Text.Encoding]::ASCII.GetBytes($Targets[$i] + [char]0)
    [Array]::Copy($raw, 0, $bytes, $next, $raw.Length)
    $next += $raw.Length
  }
  [IO.File]::WriteAllBytes($Path, $bytes)
}
function NewPeExportBudget([string]$Path, [int]$Entries, [string]$Target) {
  NewPeForwarders $Path @('kernel32.ExitProcess')
  [byte[]]$original = [IO.File]::ReadAllBytes($Path)
  [byte[]]$bytes = New-Object byte[] 0x10000
  [Array]::Copy($original, $bytes, $original.Length)
  SetU32 $bytes 0x180 0xfe00; SetU32 $bytes 0x188 0xfe00
  SetU32 $bytes 0xfc 0xfc00; SetU32 $bytes 0x414 $Entries
  for ($i = 0; $i -lt $Entries; $i++) { SetU32 $bytes (0x440 + 4 * $i) 0x8e00 }
  $raw = [Text.Encoding]::ASCII.GetBytes($Target + [char]0)
  [Array]::Copy($raw, 0, $bytes, 0x8000, $raw.Length)
  [IO.File]::WriteAllBytes($Path, $bytes)
}
function NewPeLargeExports([string]$Path, [string]$Target = '') {
  NewPe $Path
  [byte[]]$original = [IO.File]::ReadAllBytes($Path)
  # Directory size and EAT count from retained native wx core. Keep raw bytes
  # beyond the directory so directory-boundary tests cannot pass by checking EOF.
  $directorySize = 0x11af00
  $exportRva = 0x1100
  $rawSize = $exportRva - 0x1000 + $directorySize + 0x200
  [byte[]]$bytes = New-Object byte[] (0x200 + $rawSize)
  [Array]::Copy($original, $bytes, $original.Length)
  SetU32 $bytes 0x180 $rawSize; SetU32 $bytes 0x188 $rawSize
  SetU32 $bytes 0xf8 $exportRva; SetU32 $bytes 0xfc $directorySize
  SetU32 $bytes 0x314 19522; SetU32 $bytes 0x31c 0x1140
  if ($Target) {
    $targetRva = $exportRva + $directorySize - $Target.Length - 1
    SetU32 $bytes 0x340 $targetRva
    $raw = [Text.Encoding]::ASCII.GetBytes($Target + [char]0)
    [Array]::Copy($raw, 0, $bytes, 0x200 + $targetRva - 0x1000, $raw.Length)
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
  # Extensionless declarations resolve to .dll, for both import directories.
  # Keep an explicit .drv/.exe extension unchanged (covered above).
  $helperDll = Join-Path $plugins 'helper.dll'
  foreach ($delay in @($false, $true)) {
    NewPe $plugin 'HeLpEr' $delay
    NewPe $helperDll 'kernel32.dll'
    if (@(GetPeImports $plugin) -notcontains 'helper.dll') { throw 'Extensionless import was not normalized.' }
    AssertCandidateTlsRuntime (Join-Path $temp 'stage')
    NewPe $helperDll 'ssleay32' $delay
    RequireFailure { AssertCandidateTlsRuntime (Join-Path $temp 'stage') } 'Unsupported legacy TLS runtime dependency in candidate: import ssleay32.dll'
  }
  Remove-Item -LiteralPath $helperDll
  # Actual export-directory declarations, not merely direct parser calls:
  # maintained forwarders (including decorated symbols/ordinals) remain usable.
  foreach ($target in @('KERNEL32.ExitProcess', 'kernel32.#1', 'kernel32.#65535', 'kernel32.#00001', 'kernel32.?Decorated@@YAXXZ')) {
    NewPeForwarders $plugin @($target)
    if (@(GetPeImports $plugin) -notcontains 'kernel32.dll') { throw "Forwarder module was not normalized: $target" }
    AssertCandidateTlsRuntime (Join-Path $temp 'stage')
  }
  foreach ($target in @('libeay32.SSL_new', 'SSLEAY32.#1')) {
    NewPeForwarders $plugin @($target)
    RequireFailure { AssertCandidateTlsRuntime (Join-Path $temp 'stage') } 'Unsupported legacy TLS runtime dependency in candidate: import (libeay32|ssleay32).dll'
  }
  NewPeForwarders $plugin @('missing-forwarder.Run')
  RequireFailure { AssertCandidateTlsRuntime (Join-Path $temp 'stage') } 'Missing app-local PE import missing-forwarder.dll'
  # A plugin declares its helper through a forwarder; the full candidate walk
  # also queues DLLs independently. A non-DLL helper below is reached through
  # an ordinary executable import and itself declares a legacy forwarder.
  NewPeForwarders $plugin @('HeLpEr.Run')
  if (@(GetPeImports $plugin) -notcontains 'helper.dll') { throw 'Local helper forwarder was omitted.' }
  NewPe $helperDll 'libeay32.dll'
  $helperHash = Hash $helperDll
  RequireFailure { AssertCandidateTlsRuntime (Join-Path $temp 'stage') } 'Unsupported legacy TLS runtime dependency in candidate: import libeay32.dll'
  if ((Hash $helperDll) -cne $helperHash) { throw 'Forwarder rejection changed retained helper bytes.' }
  NewPeForwarders $helperDll @('kernel32.ExitProcess')
  AssertCandidateTlsRuntime (Join-Path $temp 'stage')
  $script:fixtureHashes[$helperDll] = '2bede6f2840cfe0a4e88d5c3fa648c170509c9ceed438c7e8440340541ca0446'
  RequireFailure { AssertCandidateTlsRuntime (Join-Path $temp 'stage') } 'Known obsolete TLS runtime in candidate: OpenSSL 3.0.5'
  $script:fixtureHashes.Clear()
  Remove-Item -LiteralPath $helperDll
  NewPe $exe 'helper.drv'
  NewPeForwarders (Join-Path $app 'helper.drv') @('libeay32.SSL_new')
  NewPe $plugin 'kernel32.dll'
  RequireFailure { AssertCandidateTlsRuntime (Join-Path $temp 'stage') } 'Unsupported legacy TLS runtime dependency in candidate: import libeay32.dll'
  Remove-Item -LiteralPath (Join-Path $app 'helper.drv')
  NewPe $exe 'kernel32.dll'
  # Ambiguous grammar is unsupported, just as in the original-plugin guard;
  # malformed ordinals and qualified modules must not reach filesystem lookup.
  foreach ($target in @('kernel32.dll.ExitProcess', 'kernel32.Symbol.More', 'kernel32', '.Symbol', 'kernel32.', 'kernel32.#', 'kernel32.#0', 'kernel32.#65536', 'kernel32.#-1', 'kernel32.#1x', 'C:\kernel32.Symbol', '../kernel32.Symbol')) {
    NewPeForwarders $plugin @($target)
    RequireFailure { GetPeImports $plugin } '(Unsupported PE export forwarder|Invalid PE import module name)'
  }
  # Holes and nonforwarded export RVAs declare no additional module.
  NewPeForwarders $plugin @('kernel32.ExitProcess', 'kernel32.ExitProcess')
  [byte[]]$exports = [IO.File]::ReadAllBytes($plugin)
  SetU32 $exports 0x440 0; SetU32 $exports 0x444 0x1000
  [IO.File]::WriteAllBytes($plugin, $exports)
  if (@(GetPeImports $plugin).Count -ne 0) { throw 'Ordinary exports were mistaken for dependencies.' }
  AssertCandidateTlsRuntime (Join-Path $temp 'stage')
  # The EAT itself need not be inside the export directory. Only a target RVA
  # within that directory denotes a forwarder, including its terminating NUL.
  NewPeForwarders $plugin @('kernel32.ExitProcess')
  [byte[]]$exports = [IO.File]::ReadAllBytes($plugin)
  SetU32 $exports 0x41c 0x1140; SetU32 $exports 0x340 0x1280
  [IO.File]::WriteAllBytes($plugin, $exports)
  AssertCandidateTlsRuntime (Join-Path $temp 'stage')
  # Refuse malformed directory/EAT extents without relaxing any prior import
  # fixtures. Mutation closures each start from an independent valid image.
  foreach ($case in @(
      @{message='Invalid PE export directory'; mutate={ param($b) SetU32 $b 0xf8 0 }},
      @{message='Invalid PE export directory'; mutate={ param($b) SetU32 $b 0xfc 39 }},
      @{message='PE RVA points beyond raw section data'; mutate={ param($b) SetU32 $b 0xfc 1048577 }},
      @{message='PE RVA points beyond raw section data'; mutate={ param($b) SetU32 $b 0xfc 0x201 }},
      @{message='PE export address table budget exceeded'; mutate={ param($b) SetU32 $b 0x414 65537 }},
      @{message='PE RVA does not map'; mutate={ param($b) SetU32 $b 0x41c 0x2000 }},
      @{message='PE RVA points beyond raw section data'; mutate={ param($b) SetU32 $b 0x414 2; SetU32 $b 0x41c 0x13fc }},
      @{message='Empty or oversized PE export forwarder'; mutate={ param($b) $b[0x480] = 0 }},
      @{message='Non-ASCII PE export forwarder'; mutate={ param($b) $b[0x480] = 0x80 }},
      @{message='PE export forwarder crosses directory boundary'; mutate={ param($b) SetU32 $b 0xfc (0x80 + 20) }}
    )) {
    NewPeForwarders $plugin @('kernel32.ExitProcess')
    [byte[]]$exports = [IO.File]::ReadAllBytes($plugin)
    & ($case.mutate) $exports
    [IO.File]::WriteAllBytes($plugin, $exports)
    RequireFailure { GetPeImports $plugin } $case.message
  }
  NewPeExportBudget $plugin 4097 'kernel32.ExitProcess'
  RequireFailure { GetPeImports $plugin } 'PE export forwarder budget exceeded'
  NewPeExportBudget $plugin 4096 ('kernel32.' + ('A' * 248))
  RequireFailure { GetPeImports $plugin } 'PE declaration name budget exceeded'
  NewPeExportBudget $plugin 1 ('kernel32.' + ('A' * 4087))
  RequireFailure { GetPeImports $plugin } 'Empty or oversized PE export forwarder'
  # Large extents do not increase EAT/name/forwarder work budgets or grant a
  # TLS exception. The final-byte NUL and complete raw backing remain required.
  NewPeLargeExports $plugin
  if (@(GetPeImports $plugin).Count -ne 0) { throw 'Large all-hole export table refused.' }
  [byte[]]$exports = [IO.File]::ReadAllBytes($plugin)
  SetU32 $exports 0x314 65536
  [IO.File]::WriteAllBytes($plugin, $exports)
  if (@(GetPeImports $plugin).Count -ne 0) { throw 'Bounded maximum all-hole EAT refused.' }
  # The shared mapper accepts a large extent, but import descriptor directories
  # retain their independent scan ceiling for both ordinary and delay imports.
  foreach ($entry in @(0x100, 0x160)) {
    NewPeLargeExports $plugin
    [byte[]]$exports = [IO.File]::ReadAllBytes($plugin)
    SetU32 $exports $entry 0x1100; SetU32 $exports ($entry + 4) 1048577
    [IO.File]::WriteAllBytes($plugin, $exports)
    RequireFailure { GetPeImports $plugin } 'Invalid PE import directory'
  }
  NewPeLargeExports $plugin 'kernel32.ExitProcess'
  if (@(GetPeImports $plugin) -notcontains 'kernel32.dll') { throw 'Large-directory far forwarder omitted.' }
  AssertCandidateTlsRuntime (Join-Path $temp 'stage')
  NewPeLargeExports $plugin 'libeay32.#209'
  RequireFailure { AssertCandidateTlsRuntime (Join-Path $temp 'stage') } 'Unsupported legacy TLS runtime dependency in candidate: import libeay32.dll'
  NewPeForwarders $plugin @('helper.Run')
  NewPeLargeExports $helperDll 'ssleay32.#1'
  RequireFailure { AssertCandidateTlsRuntime (Join-Path $temp 'stage') } 'Unsupported legacy TLS runtime dependency in candidate: import ssleay32.dll'
  Remove-Item -LiteralPath $helperDll
  foreach ($case in @(
      @{message='Invalid PE export directory'; mutate={ param($b) SetU32 $b 0xf8 0 }},
      @{message='Invalid PE export directory'; mutate={ param($b) SetU32 $b 0xfc 39 }},
      @{message='Invalid PE export directory'; mutate={ param($b) SetU32 $b 0xfc ([uint32]::MaxValue) }},
      @{message='PE RVA points beyond raw section data'; mutate={ param($b) SetU32 $b 0xfc 0x11b101 }},
      @{message='PE RVA points beyond raw section data'; mutate={ param($b) SetU32 $b 0x188 0x11afff }},
      @{message='PE RVA points beyond raw section data'; mutate={ param($b) SetU32 $b 0x31c 0x11bfff }},
      @{message='PE export address table budget exceeded'; mutate={ param($b) SetU32 $b 0x314 65537 }},
      @{message='PE export forwarder crosses directory boundary'; mutate={ param($b)
        $target = [Text.Encoding]::ASCII.GetBytes('kernel32.ExitProcess' + [char]0)
        SetU32 $b 0x340 (0x11c000 - $target.Length + 1)
        [Array]::Copy($target, 0, $b, 0x11b200 - $target.Length + 1, $target.Length)
      }}
    )) {
    NewPeLargeExports $plugin 'kernel32.ExitProcess'
    [byte[]]$exports = [IO.File]::ReadAllBytes($plugin)
    & ($case.mutate) $exports
    [IO.File]::WriteAllBytes($plugin, $exports)
    RequireFailure { GetPeImports $plugin } $case.message
  }
  NewPeLargeExports $plugin
  $truncated = [IO.File]::OpenWrite($plugin)
  try { $truncated.SetLength(0x11b1ff) } finally { $truncated.Dispose() }
  RequireFailure { GetPeImports $plugin } 'Invalid PE section range'
  foreach ($range in @(@(0, 1), @(0x100000000L, 1), @(0xffffffffL, 2), @(1, 0), @(1, -1))) {
    RequireFailure { PeRvaOffset ([byte[]]@(0)) @() $range[0] $range[1] } 'Invalid PE RVA range'
  }
  Write-Output 'Installer large raw-backed export extent and boundary fixtures passed.'
  # Import descriptor count is bounded before deduplication, too. Many repeated
  # declarations must not buy an unbounded parser walk or evade the name budget.
  NewPe $plugin
  [byte[]]$original = [IO.File]::ReadAllBytes($plugin)
  [byte[]]$manyImports = New-Object byte[] 0x20000
  [Array]::Copy($original, $manyImports, $original.Length)
  SetU32 $manyImports 0x180 0x1fe00; SetU32 $manyImports 0x188 0x1fe00
  SetU32 $manyImports 0x100 0x1100; SetU32 $manyImports 0x104 (4098 * 20)
  for ($i = 0; $i -lt 4097; $i++) { SetU32 $manyImports (0x300 + 20 * $i + 12) 0x18e00 }
  $rawName = [Text.Encoding]::ASCII.GetBytes('kernel32.dll' + [char]0)
  [Array]::Copy($rawName, 0, $manyImports, 0x18000, $rawName.Length)
  [IO.File]::WriteAllBytes($plugin, $manyImports)
  RequireFailure { GetPeImports $plugin } 'PE import descriptor budget exceeded'
  Write-Output 'Installer extensionless and export forwarder policy fixtures passed.'
  NewPe $plugin 'kernel32.dll'
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
  # A foreign EXE can run outside the x86 host. Refuse even a clean import;
  # the x86 scan cannot qualify its closure. Exercise a real PE32+ layout and
  # a case-varied extension, while leaving the auxiliary-DLL exception above.
  $auxiliary = Join-Path $plugins 'auxiliary.dll'
  NewPe64 $auxiliary 'libeay32.dll'
  [byte[]]$auxiliaryBytes = [IO.File]::ReadAllBytes($auxiliary)
  SetU16 $auxiliaryBytes 0x96 0x2022  # Same PE32+ layout, DLL characteristic.
  [IO.File]::WriteAllBytes($auxiliary, $auxiliaryBytes)
  AssertCandidateTlsRuntime (Join-Path $temp 'stage')
  Remove-Item -LiteralPath $auxiliary
  $helper = Join-Path $plugins 'retained-helper.ExE'
  foreach ($import in @('kernel32.dll', 'libeay32.dll', 'ssleay32.dll')) {
    NewPe64 $helper $import
    $helperHash = Hash $helper
    RequireFailure { AssertCandidateTlsRuntime (Join-Path $temp 'stage') } 'Unqualified non-x86 executable helper in candidate: .*retained-helper.ExE'
    if ((Hash $helper) -cne $helperHash) { throw 'Foreign executable refusal changed retained bytes.' }
  }
  # An unreadable PE header must still fail through the existing parser,
  # rather than becoming an accepted foreign-machine exception.
  [byte[]]$brokenHelper = [IO.File]::ReadAllBytes($helper)
  SetU32 $brokenHelper 0x80 0
  [IO.File]::WriteAllBytes($helper, $brokenHelper)
  if ((PeMachine $helper) -ne 0) { throw 'Malformed PE header reported a usable machine type.' }
  RequireFailure { AssertCandidateTlsRuntime (Join-Path $temp 'stage') } 'Invalid PE header:'
  Remove-Item -LiteralPath $helper
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

# Optional inert proof against bytes authenticated by the caller's retained
# artifact intake. This does not install, load or qualify those binaries.
foreach ($path in $RetainedPeFiles) {
  $before = Get-FileHash -LiteralPath $path -Algorithm SHA256
  $imports = @(GetPeImports $path)
  $after = Get-FileHash -LiteralPath $path -Algorithm SHA256
  if ($before.Hash -cne $after.Hash) { throw 'Retained PE changed during parsing.' }
  [pscustomobject]@{path=$before.Path;sha256=$before.Hash.ToLowerInvariant();
    bytes=(Get-Item -LiteralPath $path).Length;imports=$imports;
    powershell=$PSVersionTable.PSVersion.ToString();hostBits=([IntPtr]::Size * 8);
    scope='Inert installer PE parser only; caller owns artifact authentication'} | ConvertTo-Json -Depth 4 -Compress
}

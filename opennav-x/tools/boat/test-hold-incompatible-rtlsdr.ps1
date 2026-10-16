[CmdletBinding()]
param([Parameter(Mandatory=$true)][string]$Evidence,[switch]$PortableContracts,[switch]$IsolatedLocal)
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'
$native=[Environment]::OSVersion.Platform -eq 'Win32NT'
if (-not $native -and -not $PortableContracts) { throw 'Native Windows PowerShell 5.1 is required.' }
if ($native -and ($PSVersionTable.PSVersion.Major -ne 5 -or $PSVersionTable.PSVersion.Minor -ne 1)) { throw 'Native fixture requires Windows PowerShell 5.1.' }
if ($native -and -not $IsolatedLocal -and $env:GITHUB_ACTIONS -cne 'true') { throw 'Use disposable Windows CI, or explicitly select isolated local temporary-file tests.' }
. (Join-Path $PSScriptRoot 'Preparation.ps1')
$helper=Join-Path $PSScriptRoot 'hold-incompatible-rtlsdr.ps1'
$helperText=[IO.File]::ReadAllText($helper)
$tokens=$null;$parseErrors=$null
$ast=[Management.Automation.Language.Parser]::ParseFile($helper,[ref]$tokens,[ref]$parseErrors)
if (@($parseErrors).Count) { throw 'Production hold helper does not parse.' }
# Load the real functions, never the production command's real-user entrypoint.
$functions=@($ast.FindAll({param($n) $n -is [Management.Automation.Language.FunctionDefinitionAst]},$false))
foreach ($function in $functions) { . ([scriptblock]::Create($function.Extent.Text)) }
$checks=New-Object 'Collections.Generic.List[string]'
function Check([bool]$Ok,[string]$Name) { if (-not $Ok) { throw $Name };$checks.Add($Name) }
function Refuses([scriptblock]$Action,[string]$Pattern,[string]$Name) {
  $message='';try { $null=& $Action } catch { $message=$_.Exception.Message }
  Check (-not [string]::IsNullOrEmpty($message) -and $message -like $Pattern) $Name
}
$root=Join-Path ([IO.Path]::GetTempPath()) ('SKAGER-rtlsdr-fixture-'+[guid]::NewGuid().ToString('N'))
$null=New-Item -ItemType Directory -Path $root
$originalPath=$env:PATH
try {
  if (-not $native) {
    function Assert-LocalPath([string]$Path) {
      $full=[IO.Path]::GetFullPath($Path)
      if (-not $full.StartsWith($root+'/',[StringComparison]::Ordinal)) { throw 'Portable fixture escaped its temporary root.' }
      return $full
    }
  }
  $inert=Join-Path $root 'inert.bin';[IO.File]::WriteAllBytes($inert,(New-Object byte[] 208384))
  Refuses { Assert-RtlsdrBytes $inert } '*Only the exact reviewed*' 'Production pin rejects inert same-size DLL'
  $emptyProcess=[pscustomobject]@{Name='unrelated.exe';ExecutablePath='';ProcessId=0;CommandLine=''}
  $task=[pscustomobject]@{TaskName='unrelated';Actions=@([pscustomobject]@{Execute='unrelated.exe';Arguments=''})}
  Assert-RtlsdrActors @($emptyProcess) @($task) @('C:\protected')
  $checks.Add('Unrelated inert process and task observations are permitted')
  foreach ($name in @('opencpn.exe','rtl_ais.exe','skager-start.exe')) {
    $process=[pscustomobject]@{Name=$name;ExecutablePath='';ProcessId=0;CommandLine=''}
    Refuses { Assert-RtlsdrActors @($process) @() @('C:\protected') } '*Close *' ('Live '+$name+' blocks cold hold')
  }
  Refuses { Assert-RtlsdrActors @($emptyProcess) @([pscustomobject]@{TaskName='SKAGER-resume';Actions=@()}) @('C:\protected') } '*scheduled tasks*' 'Application scheduled task refuses even without actions'
  Refuses { Assert-RtlsdrActors @($emptyProcess) @([pscustomobject]@{TaskName='other';Actions=@([pscustomobject]@{Execute='powershell.exe';Arguments='C:\protected\run.ps1'})}) @('C:\protected') } '*Scheduled action*' 'Indirect scheduled protected-tree action refuses'
  Refuses { Assert-RtlsdrActors @([pscustomobject]@{Name='powershell.exe';ExecutablePath='';ProcessId=0;CommandLine='powershell -File UpdateSupervisor.ps1'}) @() @('C:\protected') } '*Concurrent maintenance*' 'Concurrent supervisor command refuses'
  if ($native) {
    # Fixture-only derivation changes exactly one immutable DLL hash. All file,
    # journal, path, ACL, lock, guard and move code remains the actual helper.
    $pin='404e69931034140fbda0179964d0a29077a46b78a441c05fc68a4ee0616a64e5'
    $definition=@($functions | Where-Object Name -ceq 'Assert-RtlsdrBytes')[0].Extent.Text
    if ([regex]::Matches($definition,[regex]::Escape($pin)).Count -ne 1) { throw 'Expected exactly one production byte pin.' }
    . ([scriptblock]::Create($definition.Replace($pin,(Get-Digest $inert))))
    # AST-loaded functions have no script filename; preload the same actual
    # native exclusion class that the production wrapper loads from its folder.
    Add-Type -Path (Join-Path $PSScriptRoot 'CommissioningProfileLock.cs')
    $script:fixtureProcesses=@();$script:fixtureTasks=@()
    function Get-CimInstance { return $script:fixtureProcesses }
    function Get-ScheduledTask { return $script:fixtureTasks }
    # Actual stock validation is separately required to reject these inert bytes.
    Refuses { Assert-PreparationStock ([pscustomobject]@{executable=$inert}) } '*Only the validated official*' 'Production stock validator rejects inert executable'
    function Assert-PreparationStock($Context) {
      if ((Get-Digest $Context.executable) -cne $script:stockFixtureHash) { throw 'Fixture stock identity changed.' }
    }
    $env:PATH='C:\Windows\System32'
    function Fixture {
      $base=Join-Path $root ([guid]::NewGuid().ToString('N'))
      $context=[pscustomobject]@{sid=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value;workspace=(Join-Path $base 'workspace');application=(Join-Path $base 'stock');profile=(Join-Path $base 'profile');managed=(Join-Path $base 'managed');executable=(Join-Path $base 'stock\opencpn.exe')}
      $install=Join-Path $base 'installation';$generation=Join-Path $install ('generations\'+('a'*32))
      foreach ($dir in @($context.workspace,$context.profile,$context.managed,(Join-Path $context.application 'plugins\rtlsdr_pi\bin'),(Join-Path $generation 'app\plugins'))) { $null=New-Item -ItemType Directory -Path $dir -Force }
      [IO.File]::WriteAllText($context.executable,'inert stock identity; never executed')
      $script:stockFixtureHash=Get-Digest $context.executable
      [IO.File]::WriteAllText((Join-Path $context.profile 'opencpn.ini'),"[Settings]`r`nKeep=unchanged`r`n")
      [IO.File]::WriteAllText((Join-Path $context.application 'plugins\rtlsdr_pi\bin\rtl_ais.exe'),'inert adjacent helper; never executed')
      [IO.File]::WriteAllBytes((Join-Path $context.application 'plugins\rtlsdr_pi.dll'),[IO.File]::ReadAllBytes($inert))
      $exe=Join-Path $generation 'app\opencpn.exe';[IO.File]::WriteAllText($exe,'inert installed executable')
      Write-Record (Join-Path $generation 'ownership.json') @{owner='OpenNavX.Alpha1.SideBySide.1';managedFiles=@(@{path='app/opencpn.exe';sha256=(Get-Digest $exe)})}
      Write-Record (Join-Path $install 'state.json') @{schema=1;owner='OpenNavX.Alpha1.SideBySide.1';current=('a'*32);stock=@{path=$context.executable}}
      [IO.File]::WriteAllText((Join-Path $install 'transaction.lock'),'')
      return [pscustomobject]@{context=$context;installed=(Read-InstalledIdentity $install);source=(Join-Path $context.application 'plugins\rtlsdr_pi.dll')}
    }
    function Inspect($Fixture) { return Invoke-RtlsdrLocked 'Inspect' $Fixture.context $Fixture.installed '' '' }
    function Phase($Fixture,$Plan,[string]$Action) { return Invoke-RtlsdrLocked $Action $Fixture.context $Fixture.installed $Plan.record $Plan.recordSha256 }
    $f=Fixture;$p=Inspect $f;$acl=Get-PreparationAccessAcl $f.source
    $h=Phase $f $p 'hOlD';$r=Phase $f $p 'rEsToRe'
    Check ($h.status -ceq 'held' -and $r.status -ceq 'restored' -and (Get-Digest $f.source) -ceq (Get-Digest $inert)) 'Actual locked Hold/Restore moves exact bytes once with mixed-case action spelling'
    Assert-PreparationAcl $acl (Get-PreparationAccessAcl $f.source)
    $checks.Add('Original DLL ACL and all protected tree digests survive round trip')
    Refuses { Phase $f $p 'Restore' } '*already completed*' 'Completed Restore cannot be repeated'
    $f=Fixture;$p=Inspect $f
    Refuses { Invoke-RtlsdrLocked 'Hold' $f.context $f.installed $p.record ('0'*64) } '*Exact private*' 'Wrong plan hash refuses before moving source'
    $null=Phase $f $p 'Hold';$directory=Split-Path $p.record
    Remove-Item -LiteralPath (Join-Path $directory 'held.json')
    $null=Phase $f $p 'Restore'
    Check (Test-Path -LiteralPath $f.source) 'Interrupted Hold after move restores from durable intent without held receipt'
    $f=Fixture;$p=Inspect $f
    $state=Read-RtlsdrPlan $f.context $f.installed $p.record $p.recordSha256
    Write-RtlsdrPhase $state 'hold-intent'
    $r=Phase $f $p 'Restore'
    Check ($r.recoveryMode -ceq 'hold-move-not-started' -and (Test-Path -LiteralPath $f.source)) 'Interrupted Hold before move restores as a verified no-op'
    $f=Fixture;$p=Inspect $f;$null=Phase $f $p 'Hold';$directory=Split-Path $p.record
    $state=Read-RtlsdrPlan $f.context $f.installed $p.record $p.recordSha256
    Write-RtlsdrPhase $state 'restore-intent';Move-RtlsdrExact $state.held $state.source $state.plan.acl
    $r=Phase $f $p 'Restore'
    Check ($r.status -ceq 'restored') 'Interrupted Restore after move completes only with exact restore intent'
    $f=Fixture;$p=Inspect $f;$null=Phase $f $p 'Hold'
    [IO.File]::WriteAllText($f.source,'occupied destination must survive')
    Refuses { Phase $f $p 'Restore' } '*destination is occupied*' 'Occupied stock destination is never overwritten'
    Check ([IO.File]::ReadAllText($f.source) -ceq 'occupied destination must survive') 'Occupied destination bytes remain unchanged'
    $f=Fixture;$p=Inspect $f;$null=Phase $f $p 'Hold'
    $new=Join-Path $f.installed.root ('generations\'+('b'*32));Copy-Item -LiteralPath $f.installed.generation -Destination $new -Recurse
    $statePath=Join-Path $f.installed.root 'state.json';$state=Read-Record $statePath;$state.current='b'*32
    [IO.File]::WriteAllText($statePath,($state | ConvertTo-Json -Depth 8))
    $r=Phase $f $p 'Restore'
    Check ($r.status -ceq 'restored') 'Legitimate newly selected owned generation permits stock restoration'
    foreach ($marker in @('commissioning-active.json','manual-pilot-active.json','update-pending.json','transaction.json','signed-update-armed.jsonl')) {
      $f=Fixture;$p=Inspect $f
      $where=if ($marker -in @('commissioning-active.json','manual-pilot-active.json')) {$f.context.workspace} else {$f.installed.root}
      [IO.File]::WriteAllText((Join-Path $where $marker),'incomplete still blocks')
      Refuses { Phase $f $p 'Hold' } '*' ('Cold marker refuses before moving: '+$marker)
      Check ((Test-Path -LiteralPath $f.source) -and -not (Test-Path -LiteralPath (Join-Path (Split-Path $p.record) 'hold-intent.json'))) ('Cold marker preserved original and prevented intent: '+$marker)
    }
    $f=Fixture;$p=Inspect $f;$lock=[IO.File]::Open((Join-Path $f.installed.root 'transaction.lock'),'Open','ReadWrite','None')
    try { Refuses { Phase $f $p 'Hold' } '*being used*' 'Busy actual transaction lock prevents hold' } finally { $lock.Dispose() }
    $f=Fixture;$p=Inspect $f;Remove-Item -LiteralPath (Join-Path $f.installed.root 'transaction.lock')
    Refuses { Phase $f $p 'Hold' } '*Could not find file*' 'Missing transaction lock is never created'
    foreach ($corrupt in @('backup.bin','held.bin')) {
      $f=Fixture;$p=Inspect $f;$null=Phase $f $p 'Hold'
      [IO.File]::WriteAllText((Join-Path (Split-Path $p.record) $corrupt),'corrupt')
      Refuses { Phase $f $p 'Restore' } '*Only the exact reviewed*' ('Corrupt '+$corrupt+' blocks restoration')
    }
    $f=Fixture;$p=Inspect $f;[IO.File]::AppendAllText((Join-Path $f.context.profile 'opencpn.ini'),'changed')
    Refuses { Phase $f $p 'Hold' } '*Protected tree changed*profile*' 'Profile mutation is detected before hold'
    $f=Fixture;$p=Inspect $f;$null=Phase $f $p 'Hold'
    [IO.File]::AppendAllText((Join-Path $f.context.application 'plugins\rtlsdr_pi\bin\rtl_ais.exe'),'changed')
    Refuses { Phase $f $p 'Restore' } '*Protected tree changed*stock*' 'Adjacent helper mutation prevents false preserved-tree receipt'
    $f=Fixture;$p=Inspect $f
    $null=New-Item -ItemType HardLink -Path (Join-Path $root 'hardlink.bin') -Target $f.source
    Refuses { Phase $f $p 'Hold' } '*Hard-linked*' 'Hard-linked entry DLL refuses mutation'
    $f=Fixture;$p=Inspect $f;$directory=Split-Path $p.record;$other=Join-Path $root 'redirected';Move-Item -LiteralPath $directory -Destination $other
    $null=New-Item -ItemType Junction -Path $directory -Target $other
    Refuses { Phase $f $p 'Hold' } '*Reparse paths*' 'Redirected private journal is refused'
    # Remove only the junction, never recursively follow it during fixture cleanup.
    [IO.Directory]::Delete($directory)
  }
  $result=[ordered]@{schema=1;status='passed';nativeObservation=$(if($native){'passed'}else{'not-run'});hostPowerShell=$PSVersionTable.PSVersion.ToString();hostBits=([IntPtr]::Size*8);helperSha256=(Get-FileHash -LiteralPath $helper -Algorithm SHA256).Hash.ToLowerInvariant();fixtureScope='Inert temporary bytes only; one test-only DLL pin and stock identity substitution; no vendor execution';checkCount=$checks.Count;checks=$checks.ToArray()}
  $json=$result | ConvertTo-Json -Depth 6
  if (Test-Path -LiteralPath $Evidence) { throw 'Evidence path must be fresh.' }
  [IO.File]::WriteAllText([IO.Path]::GetFullPath($Evidence),$json,(New-Object Text.UTF8Encoding($false)))
  $json
} finally {
  $env:PATH=$originalPath
  Remove-Item -LiteralPath $root -Recurse -Force
}

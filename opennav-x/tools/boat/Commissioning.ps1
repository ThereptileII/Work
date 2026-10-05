# Read-only commissioning transaction primitives. No application launch or output.
. (Join-Path $PSScriptRoot 'Preparation.ps1')
$script:CommissioningOwner='OpenNavX.ReadOnlyCommissioning.1'
$script:CommissioningBaseline='a2e416b7d35d6d2f82dcf6b3a97136a8fba5a133435c2047e07e09022426aef9'

function Get-CommissioningHash([byte[]]$Bytes) {
  $algorithm=[Security.Cryptography.SHA256]::Create()
  try { return ([BitConverter]::ToString($algorithm.ComputeHash($Bytes))).Replace('-','').ToLowerInvariant() }
  finally { $algorithm.Dispose() }
}
function Get-CommissioningInputBytes([byte[]]$Bytes) {
  # Decode losslessly, find the sole exact connection entry, then change one byte.
  # Pinned conn_params.cpp: type[0], protocol[4], port[5], IOSelect[8], enabled[17].
  $encoding=New-Object Text.UTF8Encoding($false,$true)
  $text=$encoding.GetString($Bytes)
  $keyMatches=[regex]::Matches($text,'(?m)^DataConnections=([^\r\n]*)\r?$')
  if ($keyMatches.Count -ne 1) { throw 'Exactly one unindented DataConnections entry is required.' }
  $sections=[regex]::Matches($text.Substring(0,$keyMatches[0].Index),'(?m)^\[([^\]\r\n]+)\]\r?$')
  if ($sections.Count -eq 0 -or $sections[$sections.Count-1].Groups[1].Value -cne 'Settings/NMEADataSource') { throw 'Connection entry must belong to the normal selected-source section.' }
  $value=$keyMatches[0].Groups[1];$offset=0;$selected=-1;$count=0
  foreach ($connection in $value.Value.Split('|')) {
    if ($connection) {
      $fields=$connection.Split(';')
      if ($fields.Count -lt 18 -or $fields[8] -cnotmatch '^[012]$' -or $fields[17] -cnotmatch '^[01]$') { throw 'Ambiguous connection encoding.' }
      if ($fields[5] -ceq 'COM8') {
        $count++
        # GarminUpload[15] is a stored upload preference, not Garmin driver
        # mode[14]. Preserve its actual 1 byte; Send-to-GPS remains prohibited.
        if ($fields[0] -cne '0' -or $fields[4] -cne '1' -or $fields[8] -cne '1' -or $fields[17] -cne '1' -or $fields[14] -cne '0' -or $fields[15] -cnotmatch '^[01]$') { throw 'COM8 must be the reviewed enabled serial NMEA2000 input/output connection without Garmin driver mode.' }
        $within=0;for ($i=0;$i -lt 8;$i++) { $within+=$fields[$i].Length+1 }
        $selected=$value.Index+$offset+$within
      } elseif ($fields[17] -ceq '1' -and $fields[8] -cne '0') { throw 'Another enabled output connection requires a separate review.' }
    }
    $offset+=$connection.Length+1
  }
  if ($count -ne 1 -or $selected -lt 0) { throw 'Exactly one reviewed COM8 connection is required.' }
  $byteOffset=$encoding.GetByteCount($text.Substring(0,$selected))
  if ($Bytes[$byteOffset] -ne 49) { throw 'Expected literal IOSelect 1 byte.' }
  $result=[byte[]]$Bytes.Clone();$result[$byteOffset]=48
  return ,$result
}
function Get-CommissioningIdentityContext([string]$Workspace) {
  # Identity only; every mutating commissioning operation uses the closed wrapper.
  $context=Get-PreparationIdentityContext $Workspace
  $roots=@($context.managed,(Join-Path $context.application 'plugins'))
  $state=Join-Path $context.localAppData 'OpenNavXAlpha1\state.json'
  $installation=$null
  if (Test-Path -LiteralPath $state) {
    $installed=Get-Installed
    $installation=[pscustomobject]@{root=$installed.root;generation=$installed.generation;executable=$installed.executable;commit=$installed.ownership.commit;stateSha256=(Get-Digest $state);ownershipSha256=(Get-Digest (Join-Path $installed.generation 'ownership.json'));executableSha256=(Get-Digest $installed.executable)}
    $roots+=(Join-Path ([IO.Path]::GetDirectoryName($installed.executable)) 'plugins')
  }
  $context | Add-Member -NotePropertyName installation -NotePropertyValue $installation
  $context | Add-Member -NotePropertyName pluginRoots -NotePropertyValue @($roots | Sort-Object -Unique)
  $applicationExecutable=if ($installation) {$installation.executable} else {$context.executable}
  $windowsDirectory=Assert-LocalPath ([Environment]::GetFolderPath('Windows'))
  if (-not $env:WINDIR -or (Assert-LocalPath $env:WINDIR) -ine $windowsDirectory) { throw 'Windows system-directory identity is ambiguous.' }
  $context | Add-Member -NotePropertyName launchEnvironment -NotePropertyValue (Get-CommissioningLaunchEnvironment $applicationExecutable $windowsDirectory)
  return $context
}
function Get-CommissioningContext([string]$Workspace) {
  $context=Get-CommissioningIdentityContext $Workspace
  Assert-PreparationClosed (@($context.application,$context.managed)+$context.pluginRoots)
  return $context
}
function Get-CommissioningLaunchEnvironment([string]$Executable,[string]$WindowsDirectory) {
  $workingDirectory=Assert-LocalPath ([IO.Path]::GetDirectoryName((Assert-LocalPath $Executable)))
  $windows=Assert-LocalPath $WindowsDirectory
  $system=Assert-LocalPath (Join-Path $windows 'System32')
  foreach ($directory in @($workingDirectory,$system,$windows)) {
    if (-not [IO.Directory]::Exists($directory)) { throw 'Expected launch or operating-system directory is missing.' }
  }
  # No ambient/empty/relative PATH entries. OpenCPN itself appends/prepends its
  # reviewed plugin directories. This changes only the explicitly launched child.
  return [pscustomobject]@{workingDirectory=$workingDirectory;path=(@($workingDirectory,$system,$windows) -join ';')}
}
function Assert-CommissioningContext($Expected,$Actual) {
  # Session/identity, installation generation and full loader-root set stay pinned.
  if (($Expected | ConvertTo-Json -Depth 8 -Compress) -cne ($Actual | ConvertTo-Json -Depth 8 -Compress)) { throw 'Commissioning account, installation or plugin environment changed.' }
}
function Get-CommissioningTrees([string[]]$Roots) {
  return @($Roots | ForEach-Object { Get-PreparationTree $_ })
}
function Get-CommissioningCandidates($Trees) {
  $items=New-Object 'Collections.Generic.List[object]'
  foreach ($tree in $Trees) {
    foreach ($entry in $tree.entries) {
      if (-not $entry.directory -and [IO.Path]::GetFileName($entry.path) -like '*_pi.dll') {
        if ($items.Count -ge 256) { throw 'Plugin inventory exceeds review bound.' }
        $items.Add([pscustomobject]@{path=(Assert-LocalPath (Join-Path $tree.root $entry.path));sha256=$entry.sha256;bytes=$entry.bytes})
      }
    }
  }
  return @($items | Sort-Object path)
}
function Assert-CommissioningInventory($Inventory,$Context) {
  Assert-CommissioningContext $Inventory.context $Context
  $seen=@{}
  foreach ($tree in @($Inventory.trees)) {
    $root=Assert-LocalPath $tree.root
    if ($root -inotin @($Context.pluginRoots) -or $seen.ContainsKey($root) -or $tree.exists -isnot [bool]) { throw 'Inventory must cover each actual loader root exactly once.' }
    $seen[$root]=$true
  }
  if ($seen.Count -ne @($Context.pluginRoots).Count) { throw 'Incomplete actual loader-root inventory.' }
  $expected=@(Get-CommissioningCandidates $Inventory.trees)
  if (($expected | ConvertTo-Json -Depth 6 -Compress) -cne (@($Inventory.plugins) | ConvertTo-Json -Depth 6 -Compress)) { throw 'Plugin list differs from complete tree inventory.' }
}
function Assert-CommissioningReview($Candidates,$Decisions) {
  $seen=@{}
  foreach ($decision in @($Decisions)) {
    $path=Assert-LocalPath $decision.path
    $candidate=@($Candidates | Where-Object {$_.path -ieq $path})
    if ($candidate.Count -ne 1 -or $seen.ContainsKey($path) -or $decision.sha256 -cne $candidate[0].sha256) { throw 'Review must identify each exact installed plugin once.' }
    if ($decision.decision -cnotin @('retain','quarantine') -or [string]::IsNullOrWhiteSpace($decision.reason) -or [string]::IsNullOrWhiteSpace($decision.sourceBoundary)) { throw 'Each plugin needs an explicit disposition and source-review boundary.' }
    if ($decision.decision -ceq 'retain') {
      if ($decision.sourceRevision -cnotmatch '^[a-f0-9]{40}$') { throw 'A pinned reviewed source revision is required for retained plugins.' }
      Assert-TrueBoolean $decision.startupAndIdleReadOnly 'Operator reviewed retained plugin startup and idle behavior'
    }
    elseif ($decision.startupAndIdleReadOnly -isnot [bool] -or $decision.startupAndIdleReadOnly) { throw 'Quarantined plugins cannot be presented as reviewed read-only candidates.' }
    elseif ($decision.sourceRevision -and $decision.sourceRevision -cnotmatch '^[a-f0-9]{40}$') { throw 'Quarantined source provenance must be a valid revision or explicitly unknown.' }
    if ($decision.evidenceSha256 -cnotmatch '^[a-f0-9]{64}$' -or (Get-Digest $decision.evidencePath) -cne $decision.evidenceSha256) { throw 'Source review evidence changed or is missing.' }
    $seen[$path]=$true
  }
  if ($seen.Count -ne @($Candidates).Count) { throw 'Incomplete plugin review, including disabled DLLs.' }
}
function Assert-CommissioningQuarantine([string]$Directory,[string[]]$Roots,[string]$SearchPath) {
  $directory=Assert-LocalPath $Directory
  Assert-PreparationSeparate $directory $Roots
  foreach ($entry in $SearchPath.Split(';')) {
    if (-not $entry.Trim()) { throw 'Empty PATH search entry is ambiguous for quarantine.' }
    $expanded=[Environment]::ExpandEnvironmentVariables($entry.Trim().Trim('"'))
    $path=Assert-LocalPath $expanded
    Assert-PreparationSeparate $directory @($path)
  }
}
function Assert-CommissioningTrees($Trees,$Quarantine,[switch]$AllowMoved) {
  # Exact whole-tree comparison, subtracting only individually verified moves.
  foreach ($tree in $Trees) {
    $expected=$tree | ConvertTo-Json -Depth 8 | ConvertFrom-Json
    $missing=New-Object 'Collections.Generic.List[string]'
    foreach ($entry in @($Quarantine)) {
      $sourceExists=[IO.File]::Exists($entry.path);$targetExists=[IO.File]::Exists($entry.destination)
      if ($sourceExists -eq $targetExists) { throw 'Quarantine file has ambiguous source/destination presence.' }
      $current=if ($sourceExists) {$entry.path} else {$entry.destination}
      if ((Get-Digest $current) -cne $entry.sha256) { throw 'Quarantined plugin bytes changed.' }
      if (-not $sourceExists) {
        if (-not $AllowMoved) { throw 'Plugin moved before the transaction began.' }
        if ($entry.path.StartsWith($tree.root+[IO.Path]::DirectorySeparatorChar,[StringComparison]::OrdinalIgnoreCase)) { $missing.Add($entry.path.Substring($tree.root.Length+1)) }
      }
    }
    $expected.entries=@($expected.entries | Where-Object { $_.path -cnotin $missing })
    Assert-PreparationTree $expected
  }
}
function Get-CommissioningIniDiff([string]$Before,[string]$After) {
  $beforeValues=Read-ProfileForAudit $Before;$afterValues=Read-ProfileForAudit $After
  $keys=@(@($beforeValues.Keys)+@($afterValues.Keys) | Sort-Object -Unique)
  return @($keys | Where-Object {$beforeValues[$_] -cne $afterValues[$_]} | ForEach-Object {
    [pscustomobject]@{key=$_;before=$beforeValues[$_];after=$afterValues[$_]}
  })
}
function Assert-CommissioningProtectedValues($Before,$After,[string]$InstalledBasemapDefault='',$WmmResourceProof=$null) {
  Assert-InputOnlyProfile $After
  foreach ($key in @(@($Before.Keys)+@($After.Keys) | Sort-Object -Unique)) {
    if ($key -match '^(Settings/NMEADataSource/|Directories/|ChartDirectories/)' -and $Before[$key] -cne $After[$key]) {
      # Only the installed resource selector may fill this existing empty
      # preference from its hash-bound stock locator. Custom selections, chart
      # directories and every connection remain exact. No path normalization.
      if ($key -ceq 'Directories/BaseShapefileDir' -and $Before.ContainsKey($key) -and
          $Before[$key] -ceq '' -and $InstalledBasemapDefault -and
          $After[$key] -ceq $InstalledBasemapDefault) { continue }
      if($key -ceq 'Directories/WMMDataLocation' -and $WmmResourceProof -and
          $Before[$key] -ceq $WmmResourceProof.stockLocation -and $After[$key] -ceq $WmmResourceProof.installedLocation){continue}
      throw 'Navigation, chart path or connection configuration changed; automatic baseline restore refused.'
    }
  }
}
function Assert-CommissioningRestoreIni([string]$InputOnly,[string]$Current) {
  $before=Read-ProfileForAudit $InputOnly;$after=Read-ProfileForAudit $Current
  Assert-CommissioningProtectedValues $before $after
}
. (Join-Path $PSScriptRoot 'InstalledResourceReview.ps1')
. (Join-Path $PSScriptRoot 'CommissioningBaseline.ps1')

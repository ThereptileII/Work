# Resource identity shared by running review and explicit cold adoption.
# No file modification, process launch or transport.
function Get-InstalledCommissioningBasemap($Installed) {
  # InstalledResources.cpp reads this managed marker and only supplies missing
  # defaults from the original supported application. Never accept a generation
  # resource path, another installation or a caller-provided replacement path.
  $stock=Assert-LocalPath $Installed.state.stock.path
  if ((Get-Digest $stock) -cne '7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c') { throw 'Installed resource default requires the exact supported stock executable.' }
  $marker=Assert-LocalPath (Join-Path $Installed.generation 'app/OPENNAV_INSTALLED_STOCK')
  $owned=@($Installed.ownership.managedFiles | Where-Object {$_.path -ceq 'app/OPENNAV_INSTALLED_STOCK'})
  if ($owned.Count -ne 1 -or $owned[0].sha256 -cnotmatch '^[a-f0-9]{64}$' -or
      (Get-Item -LiteralPath $marker).Length -gt 4096 -or (Get-Digest $marker) -cne $owned[0].sha256) { throw 'Exact owned stock resource locator required.' }
  $encoding=New-Object Text.UTF8Encoding($false,$true)
  if ($encoding.GetString([IO.File]::ReadAllBytes($marker)) -cne $stock) { throw 'Stock resource locator differs from the installation binding.' }
  $parent=[IO.Path]::GetDirectoryName($stock)
  foreach ($name in @('tcdata/harmonics-dwf-20210110-free.tcd','tcdata/HARMONICS_NO_US.IDX','tcdata/HARMONICS_NO_US','gshhs/poly-c-1.dat','basemap_shp/basemap_low.shp','sounds/2bells.wav')) {
    $path=Assert-LocalPath (Join-Path $parent $name)
    if (-not [IO.File]::Exists($path) -or (Get-Item -LiteralPath $path).Length -le 0) { throw 'Pinned installed resource selector prerequisites are missing.' }
  }
  # wxFileConfig escapes each Windows backslash on disk. Preserve exact bytes;
  # no relaxed slash, case, traversal, quote or alternate-path comparison.
  return (Join-Path $parent 'basemap_shp').Replace('\','\\')
}
function Assert-CommissioningResourceProof($Prepared,$Proof) {
  if (-not $Proof) { return '' }
  $context=$Prepared.context;$installed=$context.installation
  if (-not $installed -or $Proof.owner -cne 'OpenNavX.InstalledResourceReview.1' -or
      $Proof.generation -cne $installed.generation -or $Proof.commit -cne $installed.commit -or
      $Proof.ownershipSha256 -cne $installed.ownershipSha256 -or $Proof.stateSha256 -cne $installed.stateSha256 -or
      $Proof.stockPath -cne $context.executable -or $context.executable -cne (Join-Path $context.application 'opencpn.exe') -or
      $Proof.stockExecutableSha256 -cne '7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c' -or
      $Proof.markerSha256 -cne (Get-CommissioningHash ((New-Object Text.UTF8Encoding($false,$true)).GetBytes($Proof.stockPath))) -or
      $Proof.basemapDefault -cne (Join-Path $context.application 'basemap_shp').Replace('\','\\')) {
    throw 'Exact installed stock resource evidence and parent-generation binding required.'
  }
  return [string]$Proof.basemapDefault
}
function Get-CommissioningResourceProof($Prepared) {
  $expected=$Prepared.context.installation
  if (-not $expected) { throw 'Stock-only commissioning cannot fill an installed resource preference.' }
  $installed=Get-Installed
  if ($installed.generation -cne $expected.generation -or $installed.executable -cne $expected.executable -or
      $installed.ownership.commit -cne $expected.commit -or (Get-Digest $installed.executable) -cne $expected.executableSha256 -or
      (Get-Digest (Join-Path $installed.root 'state.json')) -cne $expected.stateSha256 -or
      (Get-Digest (Join-Path $installed.generation 'ownership.json')) -cne $expected.ownershipSha256) {
    throw 'Current installation differs from cold preparation.'
  }
  $default=Get-InstalledCommissioningBasemap $installed
  $proof=[pscustomobject]@{owner='OpenNavX.InstalledResourceReview.1';generation=$installed.generation;commit=$installed.ownership.commit;
    ownershipSha256=$expected.ownershipSha256;stateSha256=$expected.stateSha256;stockPath=$installed.state.stock.path;
    stockExecutableSha256=(Get-Digest $installed.state.stock.path);markerSha256=(Get-Digest (Join-Path $installed.generation 'app/OPENNAV_INSTALLED_STOCK'));
    basemapDefault=$default}
  $null=Assert-CommissioningResourceProof $Prepared $proof
  return $proof
}

# Pinned OpenCPN 37fd0cddb7334fe489e9f18aa163977a9c5c84f7 wmm_pi.cpp:
# LoadConfig ignores this preference and derives shared-data/plugins/wmm_pi/data/;
# SaveConfig (also called by DeInit) writes that derived location. This proof is
# for explicit preservation only. It never supplies a migration/launch permission.
# Exact Windows Git checkout bytes (core.autocrlf=true): WMM.COF and
# wmm_pi.svg use CRLF; wmm_live.svg has no line terminators. No runtime
# newline normalization or alternate LF hashes are accepted.
function Get-CommissioningWmmResourcePins {
  return [ordered]@{
    'WMM.COF'='b766a66b3438b91f01a037ab9cf24c3e48dd3bbf32b00ddc8328bf99291aa805'
    'wmm_live.svg'='044064c5a0af3fc3d41fb884155f8dc3a7638b6de375af722f7862546481267f'
    'wmm_pi.svg'='194f32ab7a0e257920500f67449ad244b4eaca13be6759d2cb4ed31646e0a617'
  }
}
function Get-CommissioningWmmLocation([string]$Application) {
  # wxFileConfig stores doubled backslashes, including the final separator.
  return ((Join-Path $Application 'plugins/wmm_pi/data')+[IO.Path]::DirectorySeparatorChar).Replace('\','\\')
}
function Assert-CommissioningWmmResourceProof($Prepared,$Proof) {
  if(-not $Proof){return $null}
  $context=$Prepared.context;$expected=$context.installation
  if(-not $expected -or $Proof.schema -ne 1 -or $Proof.owner -cne 'OpenNavX.InstalledWmmResourceReview.1' -or
      $Proof.generation -cne $expected.generation -or $Proof.commit -cne $expected.commit -or
      $Proof.stateSha256 -cne $expected.stateSha256 -or $Proof.ownershipSha256 -cne $expected.ownershipSha256 -or
      $Proof.stockLocation -cne (Get-CommissioningWmmLocation $context.application) -or
      $Proof.installedLocation -cne (Get-CommissioningWmmLocation (Join-Path $expected.generation 'app')) -or
      $Proof.ownershipBase64 -isnot [string] -or $Proof.ownershipBase64.Length -gt 1398104){throw 'Exact parent-generation WMM resource proof required.'}
  $raw=[Convert]::FromBase64String($Proof.ownershipBase64)
  if($raw.Length -le 0 -or $raw.Length -gt 1048576 -or (Get-CommissioningHash $raw) -cne $expected.ownershipSha256){throw 'Frozen WMM ownership bytes differ from the original generation.'}
  $ownership=(New-Object Text.UTF8Encoding($false,$true)).GetString($raw)|ConvertFrom-Json
  if($ownership.owner -cne 'OpenNavX.Alpha1.SideBySide.1' -or $ownership.commit -cne $expected.commit){throw 'Frozen WMM ownership identity differs.'}
  $pins=Get-CommissioningWmmResourcePins
  if(@($Proof.resources).Count -ne $pins.Count){throw 'Complete pinned WMM resource set required.'}
  foreach($name in $pins.Keys){
    $file=@($Proof.resources|Where-Object{$_.name -ceq $name})
    $relative='app/plugins/wmm_pi/data/'+$name
    $owned=@($ownership.managedFiles|Where-Object{$_.path -ieq $relative})
    if($file.Count -ne 1 -or $file[0].sha256 -cne $pins[$name] -or $file[0].bytes -le 0 -or $file[0].bytes -gt 1048576 -or
        $owned.Count -ne 1 -or $owned[0].path -cne $relative -or $owned[0].sha256 -cne $pins[$name]){throw 'WMM resource must match pinned source bytes and unique package ownership.'}
  }
  return $Proof
}
function Get-CommissioningWmmResourceProof($Prepared) {
  $expected=$Prepared.context.installation
  if(-not $expected){throw 'Stock-only commissioning has no installed WMM resource proof.'}
  $installed=Get-Installed
  $ownershipPath=Assert-LocalPath (Join-Path $installed.generation 'ownership.json')
  if($installed.generation -cne $expected.generation -or $installed.executable -cne $expected.executable -or
      $installed.ownership.commit -cne $expected.commit -or (Get-Digest $installed.executable) -cne $expected.executableSha256 -or
      (Get-Digest (Join-Path $installed.root 'state.json')) -cne $expected.stateSha256 -or
      (Get-Item -LiteralPath $ownershipPath).Length -gt 1048576 -or (Get-Digest $ownershipPath) -cne $expected.ownershipSha256){throw 'Current WMM installation differs from cold preparation.'}
  $pins=Get-CommissioningWmmResourcePins;$resources=@();$reference=$null
  foreach($application in @($Prepared.context.application,(Join-Path $expected.generation 'app'))){
    $directory=Assert-LocalPath (Join-Path $application 'plugins/wmm_pi/data')
    $items=@(Get-ChildItem -LiteralPath $directory -Force)
    if($items.Count -ne $pins.Count){throw 'Unexpected WMM resource directory contents.'}
    $resources=@()
    foreach($name in $pins.Keys){
      $found=@($items|Where-Object{$_.Name -ceq $name})
      if($found.Count -ne 1 -or $found[0].PSIsContainer -or $found[0].Length -le 0 -or $found[0].Length -gt 1048576){throw 'Unexpected WMM resource object.'}
      $file=Assert-LocalPath $found[0].FullName;$hash=Get-Digest $file
      if($hash -cne $pins[$name]){throw 'Installed/stock WMM resources differ from pinned source bytes.'}
      $resources+=([pscustomobject]@{name=$name;sha256=$hash;bytes=$found[0].Length})
    }
    if($reference -and ($reference|ConvertTo-Json -Compress) -cne ($resources|ConvertTo-Json -Compress)){throw 'Stock and installed WMM resources differ.'}
    $reference=$resources
  }
  $proof=[pscustomobject]@{schema=1;owner='OpenNavX.InstalledWmmResourceReview.1';generation=$expected.generation;commit=$expected.commit;
    stateSha256=$expected.stateSha256;ownershipSha256=$expected.ownershipSha256;ownershipBase64=[Convert]::ToBase64String([IO.File]::ReadAllBytes($ownershipPath));
    stockLocation=(Get-CommissioningWmmLocation $Prepared.context.application);installedLocation=(Get-CommissioningWmmLocation (Join-Path $expected.generation 'app'));resources=$resources}
  return Assert-CommissioningWmmResourceProof $Prepared $proof
}
function Assert-CommissioningWmmLiveProof($Prepared,$Proof) {
  if(-not $Proof){return}
  $null=Assert-CommissioningWmmResourceProof $Prepared $Proof
  $current=Get-CommissioningWmmResourceProof $Prepared
  if(($current|ConvertTo-Json -Depth 8 -Compress) -cne ($Proof|ConvertTo-Json -Depth 8 -Compress)){throw 'WMM resource evidence changed since cold inspection.'}
}

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

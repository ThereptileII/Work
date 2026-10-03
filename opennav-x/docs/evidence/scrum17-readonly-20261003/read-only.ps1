$ErrorActionPreference='Stop'
$ProgressPreference='SilentlyContinue'
function Identity([string]$Path) {
  if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) { return $null }
  $item=Get-Item -LiteralPath $Path
  return [ordered]@{bytes=$item.Length;sha256=(Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash.ToLowerInvariant()}
}
$result=[ordered]@{schema='SKAGER.BoatReadOnlyReadiness.1';timeUtc=[DateTime]::UtcNow.ToString('o');sshReachable=$true;errors=@()}
$profile=Join-Path $env:ProgramData 'opencpn\opencpn.ini'
$root=Join-Path $env:LOCALAPPDATA 'OpenNavXAlpha1'
$statePath=Join-Path $root 'state.json'
$profileBefore=Identity $profile
$stateBefore=Identity $statePath
$result.profile=$profileBefore
$result.installState=$stateBefore
try {
  $target=Get-Content -LiteralPath 'C:\XNav\boat-target.json' -Raw | ConvertFrom-Json
  if($target.owner -cne 'OpenNavX.BoatTarget.1' -or $target.schema -ne 1){throw 'Target identity'}
  $stock=Get-Item -LiteralPath $target.stockExecutable
  $result.stock=[ordered]@{identity=(Identity $stock.FullName);version=$stock.VersionInfo.FileVersion;
    exactSupportedIdentity=((Identity $stock.FullName).sha256 -ceq '7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c')}
  $result.profileTargetMatches=([IO.Path]::GetFullPath($target.profileDirectory).TrimEnd('\') -ieq [IO.Path]::GetDirectoryName($profile))
}catch{$result.errors+= 'stock-target-inspection'}
try {
  $state=Get-Content -LiteralPath $statePath -Raw | ConvertFrom-Json
  if($state.owner -cne 'OpenNavX.Alpha1.SideBySide.1' -or $state.current -cnotmatch '^[a-f0-9]{32}$'){throw 'State identity'}
  $generation=Join-Path (Join-Path $root 'generations') $state.current
  $ownerPath=Join-Path $generation 'ownership.json'
  $owner=Get-Content -LiteralPath $ownerPath -Raw | ConvertFrom-Json
  if($owner.owner -cne $state.owner -or $owner.commit -cnotmatch '^[a-f0-9]{40}$'){throw 'Generation identity'}
  $app=Identity (Join-Path $generation 'app\opencpn.exe')
  $record=@($owner.managedFiles | Where-Object {$_.path -ceq 'app/opencpn.exe'})
  $result.installed=[ordered]@{commit=$owner.commit;version=$owner.version;ownership=(Identity $ownerPath);executable=$app;
    executableMatchesOwnership=($record.Count -eq 1 -and $record[0].sha256 -ceq $app.sha256)}
}catch{$result.errors+='installed-generation-inspection'}
try {
  $all=@(Get-CimInstance Win32_Process)
  $result.navigationProcesses=@($all | Where-Object {$_.Name -match '^(opencpn|opennav.*|xnav.*|skager.*|oexserverd|oeserverd)\.exe$'} | ForEach-Object {
    [ordered]@{name=$_.Name;sessionId=$_.SessionId;identity=if($_.ExecutablePath){Identity $_.ExecutablePath}else{$null}}
  })
  $result.remoteServices=@(Get-Service | Where-Object {$_.Name -match '^(sshd|tailscale|rustdesk)$'} | ForEach-Object {
    [ordered]@{name=$_.Name;status=$_.Status.ToString();startType=$_.StartType.ToString()}
  })
  $result.remoteProcessCounts=[ordered]@{
    sshd=@($all | Where-Object {$_.Name -ieq 'sshd.exe'}).Count;
    tailscale=@($all | Where-Object {$_.Name -match '^tailscale.*\.exe$'}).Count;
    rustdesk=@($all | Where-Object {$_.Name -ieq 'rustdesk.exe'}).Count
  }
  $rustIds=@($all | Where-Object {$_.Name -ieq 'rustdesk.exe'} | Select-Object -ExpandProperty ProcessId)
  $result.rustdeskEstablishedTcp=@(Get-NetTCPConnection -State Established -ErrorAction SilentlyContinue | Where-Object {$_.OwningProcess -in $rustIds}).Count
}catch{$result.errors+='process-service-inspection'}
try {
  $result.display=@(Get-CimInstance Win32_VideoController | ForEach-Object {
    [ordered]@{width=$_.CurrentHorizontalResolution;height=$_.CurrentVerticalResolution;driverVersion=$_.DriverVersion}
  })
  $metrics=Get-ItemProperty -LiteralPath 'HKCU:\Control Panel\Desktop\WindowMetrics'
  $desktop=Get-ItemProperty -LiteralPath 'HKCU:\Control Panel\Desktop'
  $result.displayDpi=[ordered]@{appliedDpi=$metrics.AppliedDPI;logPixels=$desktop.LogPixels;source='Current-user desktop registry, not a rendered-window measurement'}
}catch{$result.errors+='display-inspection'}
try {
  $tailscale=Join-Path $env:ProgramFiles 'Tailscale\tailscale.exe'
  $status=(& $tailscale status --json 2>$null | Out-String | ConvertFrom-Json)
  if($LASTEXITCODE -ne 0){throw 'Tailscale query'}
  $result.tailscale=[ordered]@{backendState=$status.BackendState;selfOnline=$status.Self.Online;healthWarningCount=@($status.Health | Where-Object {$_}).Count}
}catch{$result.errors+='tailscale-status'}
$result.reviewToolsPresent=Test-Path -LiteralPath 'C:\XNav\scripts\review-ffa2de31ae02ca826ec5d3604f57a8c619035bb1' -PathType Container
$result.commissioningMarkerPresent=Test-Path -LiteralPath 'C:\XNav\commissioning-active.json'
$result.profileAfter=Identity $profile
$result.installStateAfter=Identity $statePath
$result.profileUnchanged=($profileBefore.sha256 -ceq $result.profileAfter.sha256 -and $profileBefore.bytes -eq $result.profileAfter.bytes)
$result.installStateUnchanged=($stateBefore.sha256 -ceq $result.installStateAfter.sha256 -and $stateBefore.bytes -eq $result.installStateAfter.bytes)
$result | ConvertTo-Json -Depth 8 -Compress

# Read-only boat target inventory. Output can contain private local paths;
# retain raw output privately and commit only a redacted qualification summary.
[CmdletBinding()]
param()
$ErrorActionPreference = 'Stop'
function BinaryInfo([string]$Path) {
  if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) { return $null }
  $item = Get-Item -LiteralPath $Path
  $bytes = [IO.File]::ReadAllBytes($item.FullName)
  $arch = 'unknown'
  if ($bytes.Length -gt 64) {
    $offset = [BitConverter]::ToInt32($bytes, 60)
    if ($offset -ge 0 -and $offset + 6 -le $bytes.Length -and
        [BitConverter]::ToUInt32($bytes, $offset) -eq 0x4550) {
      $machine = [BitConverter]::ToUInt16($bytes, $offset + 4)
      if ($machine -eq 0x14c) { $arch = 'x86' }
      elseif ($machine -eq 0x8664) { $arch = 'x64' }
    }
  }
  [ordered]@{ path=$item.FullName; version=$item.VersionInfo.FileVersion;
    architecture=$arch; sha256=(Get-FileHash -LiteralPath $item.FullName -Algorithm SHA256).Hash.ToLowerInvariant() }
}
$registrations = @()
foreach ($key in @('HKLM:\Software\Microsoft\Windows\CurrentVersion\Uninstall',
    'HKLM:\Software\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall',
    'HKCU:\Software\Microsoft\Windows\CurrentVersion\Uninstall')) {
  if (Test-Path $key) {
    $registrations += @(Get-ChildItem $key | ForEach-Object {
      $v = Get-ItemProperty $_.PSPath
      if ($v.DisplayName -match 'OpenCPN|OpenNav|XNav') {
        [ordered]@{name=$v.DisplayName;version=$v.DisplayVersion;location=$v.InstallLocation;
          uninstall=$v.UninstallString;key=$_.PSChildName}
      }
    })
  }
}
$roots = @('C:\XNav',(Join-Path $env:LOCALAPPDATA 'OpenNavXAlpha1'),
  (Join-Path $env:ProgramFiles 'OpenCPN'),(Join-Path ${env:ProgramFiles(x86)} 'OpenCPN'))
$rootStatus = @($roots | Select-Object -Unique | ForEach-Object {
  $exists=Test-Path -LiteralPath $_ -PathType Container
  [ordered]@{path=$_;exists=$exists;children=if($exists){@(Get-ChildItem -LiteralPath $_ -Force | Select-Object Name,PSIsContainer)}else{@()}}
})
$binaries = @($roots | ForEach-Object { BinaryInfo (Join-Path $_ 'opencpn.exe') } | Where-Object {$_})
$processes = @(Get-CimInstance Win32_Process -Filter "Name='opencpn.exe'" |
  Select-Object ProcessId,SessionId,ExecutablePath,CommandLine)
$profiles = @((Join-Path $env:ProgramData 'opencpn'),(Join-Path $env:APPDATA 'opencpn'))
$profileStatus = @($profiles | ForEach-Object {
  [ordered]@{path=$_;exists=(Test-Path -LiteralPath $_);files=if(Test-Path -LiteralPath $_){
    @(Get-ChildItem -LiteralPath $_ -File | Where-Object {$_.Name -match '^(opencpn\.(ini|conf)|navobj.*|chartlist\.dat)$'} | ForEach-Object {
      [ordered]@{name=$_.Name;bytes=$_.Length;sha256=(Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash.ToLowerInvariant()}
    })}else{@()}}
})
$desktop=[Environment]::GetFolderPath([Environment+SpecialFolder]::Desktop)
$downloads=Join-Path $env:USERPROFILE 'Downloads'
$oldCopies=@()
foreach($folder in @($desktop,$downloads)) {
 if(Test-Path -LiteralPath $folder) {
  $oldCopies+=@(Get-ChildItem -LiteralPath $folder -Force |
    Where-Object {$_.Name -match 'OpenNav|XNav|X-nav'} |
    Select-Object FullName,Name,Length,PSIsContainer)
 }
}
[ordered]@{schema='OpenNavX.BoatInspection.1';timeUtc=[DateTime]::UtcNow.ToString('o');
  os=(Get-CimInstance Win32_OperatingSystem | Select-Object Caption,Version,OSArchitecture);
  display=@(Get-CimInstance Win32_VideoController | Select-Object Name,CurrentHorizontalResolution,CurrentVerticalResolution,DriverVersion);
  services=@(Get-Service | Where-Object {$_.Name -match 'sshd|tailscale|rustdesk'} | Select-Object Name,Status,StartType);
  sessions=@(quser 2>$null);registrations=$registrations;roots=$rootStatus;binaries=$binaries;
  processes=$processes;profiles=$profileStatus;oldCopies=$oldCopies;
  disks=@(Get-CimInstance Win32_LogicalDisk -Filter 'DriveType=3' | Select-Object DeviceID,Size,FreeSpace)
} | ConvertTo-Json -Depth 8

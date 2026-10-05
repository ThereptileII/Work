# Read-only inventory of the prototype font stack. This does not launch OpenCPN
# or prove which face an application window actually selects.
[CmdletBinding()]
param()
$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Drawing
$fonts = New-Object System.Drawing.Text.InstalledFontCollection
try {
  $names = @($fonts.Families | ForEach-Object { $_.Name })
  $stack = @('Segoe UI Variable Display', 'Segoe UI', 'Arial')
  [ordered]@{
    schema = 'SKAGER.PrototypeFontInventory.1'
    timeUtc = [DateTime]::UtcNow.ToString('o')
    fonts = @($stack | ForEach-Object {
      [ordered]@{ family = $_; available = ($names -contains $_) }
    })
    scope = 'Installed family inventory only; application HDC and chart font qualification remain separate.'
  } | ConvertTo-Json -Depth 4
} finally {
  $fonts.Dispose()
}

$ErrorActionPreference='Stop'
Add-Type -AssemblyName System.Drawing
$fonts=New-Object System.Drawing.Text.InstalledFontCollection
try {
 $names=@($fonts.Families | ForEach-Object {$_.Name})
 $os=Get-CimInstance Win32_OperatingSystem | Select-Object Caption,Version,BuildNumber,OSArchitecture
 [ordered]@{schema='SKAGER.BoatFontReadOnly.1';timeUtc=[DateTime]::UtcNow.ToString('o');windows=$os;fonts=@(@('Segoe UI Variable Display','Segoe UI','Arial')|ForEach-Object{[ordered]@{family=$_;available=($names -contains $_)}});scope='Read-only OS version and installed font families; no application launch or font mutation'}|ConvertTo-Json -Depth 5
} finally {$fonts.Dispose()}

$ErrorActionPreference='Stop'
$vswhere='C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe'
Write-Output 'VSWHERE_ALL_PRODUCTS_BEGIN'
& $vswhere -products '*' -all -format json
Write-Output 'VSWHERE_ALL_PRODUCTS_END'
foreach($base in @('C:\Program Files\Microsoft Visual Studio','C:\Program Files (x86)\Microsoft Visual Studio')) {
 if(Test-Path -LiteralPath $base) {
  Get-ChildItem -LiteralPath $base -Filter vcvarsall.bat -Recurse -File | Select-Object -ExpandProperty FullName
  Get-ChildItem -LiteralPath $base -Filter cl.exe -Recurse -File | Select-Object -ExpandProperty FullName
 }
}
$p=Get-Process -Id 1852 -ErrorAction SilentlyContinue
[pscustomobject]@{applicationPid1852Present=($null -ne $p)} | ConvertTo-Json -Compress

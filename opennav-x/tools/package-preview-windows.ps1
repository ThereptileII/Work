param([switch]$DeferRuntimeQualification,[string]$UpdateTrustConfig,[string]$UpdateTrustValidator)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
if ($DeferRuntimeQualification -and $env:GITHUB_ACTIONS -cne 'true') {
    throw 'Deferred portable qualification requires the explicit CI build job'
}
$Root = Split-Path $PSScriptRoot -Parent
$Vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
$VsSelection = @(& $Vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -format json | ConvertFrom-Json)
if ($LASTEXITCODE -ne 0 -or $VsSelection.Count -ne 1) { throw 'Unique native MSVC toolchain selection failed' }
$VS = $VsSelection[0].installationPath
$VsVersion = $VsSelection[0].installationVersion
$ProductProperty = $VsSelection[0].PSObject.Properties['productId']
if (-not $VS -or -not $ProductProperty -or $ProductProperty.Value -isnot [string]) {
    throw 'Native MSVC toolchain product identity not found'
}
$VsProductId = $ProductProperty.Value
# Installation product/catalog identity is not licensed-edition entitlement.
$VsCatalog = [ordered]@{}
$CatalogProperty = $VsSelection[0].PSObject.Properties['catalog']
if ($CatalogProperty -and $null -ne $CatalogProperty.Value) {
    if ($CatalogProperty.Value -isnot [PSCustomObject]) { throw 'MSVC catalog identity is not an object' }
    foreach ($Name in @('id','buildVersion','productDisplayVersion','productLine',
        'productLineVersion','productName','productSemanticVersion','productMilestone',
        'productMilestoneIsPreRelease','productPatchVersion','productPreReleaseMilestoneSuffix',
        'requiredEngineVersion')) {
        $Property = $CatalogProperty.Value.PSObject.Properties[$Name]
        if ($Property) {
            if ($Property.Value -isnot [string]) { throw ('MSVC catalog identity is not text: ' + $Name) }
            $VsCatalog[$Name] = $Property.Value
        }
    }
}
# Base64 preserves JSON quotes through Windows PowerShell 5.1 and pwsh alike.
$VsCatalogBase64 = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes(
    (ConvertTo-Json -InputObject $VsCatalog -Compress)))
$Runtime = Get-ChildItem "$VS\VC\Redist\MSVC\*\x86\Microsoft.VC143.CRT" -Directory |
    Sort-Object FullName -Descending | Select-Object -First 1
if (-not $Runtime) { throw 'App-local x86 MSVC redistributable directory not found' }
$Output = Join-Path $Root 'build/developer-preview'
$TrustArguments = @()
if ($UpdateTrustConfig -or $UpdateTrustValidator) {
    if (-not $UpdateTrustConfig -or -not $UpdateTrustValidator) { throw 'Explicit public trust input and qualified validator are both required' }
    $ValidatorRecord = Get-Content -LiteralPath ([IO.Path]::ChangeExtension($UpdateTrustValidator, '.json')) -Raw | ConvertFrom-Json
    if ($ValidatorRecord.schema -ne 1 -or $ValidatorRecord.commit -cne $env:GITHUB_SHA -or
        $ValidatorRecord.sha256 -cne (Get-FileHash -LiteralPath $UpdateTrustValidator -Algorithm SHA256).Hash.ToLowerInvariant()) {
        throw 'Trust validator is not the exact same-commit native producer output'
    }
    $TrustArguments = @('--update-trust-config',$UpdateTrustConfig,'--update-trust-validator',$UpdateTrustValidator)
}
python (Join-Path $PSScriptRoot 'package-preview.py') --install "$Root/build/production-install" `
    --build "$Root/build/production-windows" --runtime $Runtime.FullName --output $Output `
    --runtime-vs-installation $VS --runtime-vs-version $VsVersion `
    --runtime-vs-product-id $VsProductId --runtime-vs-catalog-base64 $VsCatalogBase64 `
    --openssl-source-cache "$Root/build/dependency-downloads/openssl-3.5.9.tar.gz" `
    --dependency-source-cache "$Root/build/dependency-downloads" @TrustArguments
$AssemblyExit = $LASTEXITCODE
$Evidence = Join-Path $Root 'evidence/local'
$null = New-Item -ItemType Directory -Path $Evidence -Force
foreach ($Name in @('production-package-selftest.json', 'production-restart-selftest.json')) {
    $Report = Join-Path $Output $Name
    if (Test-Path -LiteralPath $Report -PathType Leaf) {
        Copy-Item -LiteralPath $Report -Destination (Join-Path $Evidence $Name)
    } elseif ($AssemblyExit -eq 0) {
        throw ('Successful assembly omitted executed capability evidence: ' + $Name)
    }
}
if ($AssemblyExit -ne 0) { throw 'Recovery assembly failed' }
python (Join-Path $PSScriptRoot 'verify-preview-pe.py') "$Output/SKAGER-Beta2-Portable-Recovery/app" `
    --report "$Root/evidence/local/preview-dll-audit.json"
if ($LASTEXITCODE -ne 0) { throw 'Recovery dependency closure failed' }
if (-not $DeferRuntimeQualification) {
    python (Join-Path $PSScriptRoot 'smoke-portable-production.py') --package "$Output/SKAGER-Beta2-Portable-Recovery.zip"
    if ($LASTEXITCODE -ne 0) { throw 'Extracted production recovery smoke test failed' }
}

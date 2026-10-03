param([string]$Evidence = '')
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
if (-not $IsWindows) { throw 'This fixture preflight requires native Windows PowerShell 7' }
$Root = Split-Path $PSScriptRoot -Parent
if (-not $Evidence) { $Evidence = Join-Path $Root 'evidence/local/downloader-certificate-fixtures' }
$Evidence = [IO.Path]::GetFullPath($Evidence)
if (Test-Path -LiteralPath $Evidence) { throw 'Fixture evidence directory must be new' }
$null = New-Item -ItemType Directory $Evidence
$null = Start-Transcript -LiteralPath (Join-Path $Evidence 'fixture-run.log')
$Source = Join-Path $PSScriptRoot 'test-downloader-trust-windows.ps1'
$Work = Join-Path ([IO.Path]::GetTempPath()) ('skager-cert-fixture-' + [guid]::NewGuid().ToString('N'))
$Fixtures = Join-Path $Work 'fixtures with spaces'
$null = New-Item -ItemType Directory -Path $Fixtures
$Report = [ordered]@{passed=$false;scope='Actual certificate helper fixture issuance only';trustStoreModified=$false;tlsAcceptance=$false;applicationAcceptance=$false;runId=$env:GITHUB_RUN_ID;commit=$env:GITHUB_SHA;checks=@()}
function Check([bool]$Condition,[string]$Name) {
  if (-not $Condition) { throw "Fixture check failed: $Name" }
  $Report.checks += $Name
}
function Identity([string]$Path) {
  [ordered]@{bytes=(Get-Item -LiteralPath $Path).Length;sha256=(Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash.ToLowerInvariant()}
}
try {
  $Report.source = Identity $Source
  $Report.preflight = Identity $PSCommandPath
  $Workflow = Join-Path $Root '.github/workflows/skager-certificate-fixtures.yml'
  if (-not (Test-Path -LiteralPath $Workflow)) { $Workflow = Join-Path (Split-Path $Root -Parent) '.github/workflows/skager-certificate-fixtures.yml' }
  $Report.workflow = Identity $Workflow
  $ProviderCandidates = @(Get-Command openssl.exe -CommandType Application -ErrorAction Stop)
  $Provider = $ProviderCandidates[0]
  $ProviderPath = $Provider.Source
  $Report.opensslPathCandidates = @($ProviderCandidates | ForEach-Object { $_.Source })
  $Report.openssl = [ordered]@{path=$ProviderPath;identity=(Identity $ProviderPath);version=(& $ProviderPath version | Out-String).Trim();details=(& $ProviderPath version -a | Out-String).Trim();maintainedProducerVersion='3.5.9';producerQualification=$false}
  if ($LASTEXITCODE -ne 0) { throw 'OpenSSL version query failed' }
  # Parse only these actual function definitions. Never dot-source the enclosing
  # trust-store script, bypass its CI guard, or import its trust/server helpers.
  $Tokens=$null; $Errors=$null
  $Ast=[Management.Automation.Language.Parser]::ParseFile($Source,[ref]$Tokens,[ref]$Errors)
  if ($Errors.Count) { throw ($Errors | Out-String) }
  $Definitions=[ordered]@{}
  foreach($Name in @('Run','OpenSsl','New-Ca','New-Leaf')) {
    $Nodes=@($Ast.FindAll({param($Node) $Node -is [Management.Automation.Language.FunctionDefinitionAst] -and $Node.Name -ceq $Name},$true))
    if ($Nodes.Count -ne 1) { throw "Expected exactly one actual $Name function" }
    $Definitions[$Name]=$Nodes[0].Extent.Text
    . ([scriptblock]::Create($Definitions[$Name]))
  }
  $Report.functions=@($Definitions.Keys)
  $Fixed='[IO.File]::WriteAllBytes($Index, [byte[]]::new(0))'
  $Original="Set-Content -LiteralPath `$Index -Value '' -Encoding ascii"
  if ([regex]::Matches($Definitions['New-Leaf'],[regex]::Escape($Fixed)).Count -ne 1) { throw 'Actual empty-index boundary changed' }
  $OldFunction=$Definitions['New-Leaf'].Replace($Fixed,$Original) -replace '^function New-Leaf\(', 'function New-Leaf-Original('
  . ([scriptblock]::Create($OldFunction))
  $Ca=New-Ca 'fixture-owned-ca'
  $NegativeLog=Join-Path $Evidence 'original-newline.log'
  $Failed=$false
  try { New-Leaf-Original 'original' $Ca[0] $Ca[1] 'localhost' -Expired *> $NegativeLog }
  catch { $Failed=$true; $Report.originalError=$_.Exception.Message }
  $OriginalBytes=[IO.File]::ReadAllBytes((Join-Path $Fixtures 'original-index.txt'))
  $Report.originalIndexHex=[Convert]::ToHexString($OriginalBytes)
  Check ($Failed -and (Get-Content -LiteralPath $NegativeLog -Raw) -match 'Problem with index file:') 'Original actual helper fails at index parser'
  Check ($Report.originalIndexHex -ceq '0D0A') 'Original Windows index is one CRLF blank line'
  $Valid=New-Leaf 'valid' $Ca[0] $Ca[1] 'localhost'
  $Expired=New-Leaf 'expired' $Ca[0] $Ca[1] 'localhost' -Expired
  # OpenSSL retains the pre-issuance database as .old: inspect the real helper's
  # zero-byte starting database, then the issued row in the current database.
  $OldIndex=Join-Path $Fixtures 'expired-index.txt.old'
  Check ((Get-Item -LiteralPath $OldIndex).Length -eq 0) 'Actual expired issuer began with zero-byte database'
  Check ((Get-Item -LiteralPath (Join-Path $Fixtures 'expired-index.txt')).Length -gt 0) 'Actual expired issuer wrote a certificate database row'
  $ValidOutput=& $ProviderPath verify -CAfile $Ca[1] -purpose sslserver -verify_hostname localhost $Valid[1] 2>&1 | Out-String
  $ValidExit=$LASTEXITCODE
  $ValidOutput | Set-Content -LiteralPath (Join-Path $Evidence 'valid-verify.log') -Encoding utf8
  Check ($ValidExit -eq 0 -and $ValidOutput -match ': OK') 'Issued valid localhost certificate verifies'
  $ExpiredOutput=& $ProviderPath verify -CAfile $Ca[1] -purpose sslserver -verify_hostname localhost $Expired[1] 2>&1 | Out-String
  $ExpiredExit=$LASTEXITCODE
  $ExpiredOutput | Set-Content -LiteralPath (Join-Path $Evidence 'expired-verify.log') -Encoding utf8
  Check ($ExpiredExit -eq 2 -and $ExpiredOutput -match 'error 10 at 0 depth lookup: certificate has expired') 'Issued expired localhost certificate rejected specifically for expiry'
  $Dates=& $ProviderPath x509 -in $Expired[1] -noout -dates 2>&1 | Out-String
  Check ($LASTEXITCODE -eq 0 -and $Dates -match 'notBefore=Jan\s+1 00:00:00 2020 GMT' -and $Dates -match 'notAfter=Jan\s+2 00:00:00 2020 GMT') 'Expired fixture retains original fixed 2020 validity interval'
  $Report.expiredDates=$Dates.Trim()
  foreach($Cert in @($Ca[1],$Valid[1],$Expired[1])) { Copy-Item -LiteralPath $Cert -Destination $Evidence }
  $Report.passed=$true
} catch {
  $Report.error=$_.Exception.Message
  throw
} finally {
  # No private key, trust-store operation or listening server is retained/run.
  if (Test-Path -LiteralPath $Work) { Remove-Item -LiteralPath $Work -Recurse -Force }
  $Report.cleanupVerified=-not (Test-Path -LiteralPath $Work)
  $Report | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath (Join-Path $Evidence 'report.json') -Encoding utf8
  $null = Stop-Transcript
}
Write-Host "$($Report.checks.Count) actual certificate fixture checks passed; no TLS or application acceptance"

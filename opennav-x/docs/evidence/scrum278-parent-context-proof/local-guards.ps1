$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest
$Path=Join-Path (Get-Location) 'tools/test-windows-parent-context.ps1'
$Tokens=$null; $Errors=$null
$Ast=[Management.Automation.Language.Parser]::ParseFile($Path,[ref]$Tokens,[ref]$Errors)
if($Errors.Count){ throw ($Errors | Out-String) }
foreach($Name in @('TextDigest','Get-ProducerSetup','Assert-OnlyPathDifference')) {
    $Node=$Ast.Find({param($N) $N -is [Management.Automation.Language.FunctionDefinitionAst] -and $N.Name -eq $Name},$true)
    . ([scriptblock]::Create($Node.Extent.Text))
}
$Checks=1
function Pass([bool]$Condition,[string]$Name) { if(-not $Condition){throw $Name}; $script:Checks++; Write-Output $Name }
function Refuse([scriptblock]$Action,[string]$Expected) {
    $Rejected=$false
    try { & $Action | Out-Null } catch { if($_.Exception.Message -cne $Expected){throw}; $Rejected=$true }
    Pass $Rejected $Expected
}
$Source=[IO.File]::ReadAllText((Join-Path (Get-Location) 'tools/build-openssl-windows.ps1'))
$Span=Get-ProducerSetup $Source
Pass ($Span.Length -eq 2273) 'Exact maintained span accepted'
Pass ((Get-ProducerSetup ($Source.Replace("`n","`r`n"))) -ceq $Span) 'CRLF checkout retains exact normalized span'
Refuse {Get-ProducerSetup ($Source.Replace('C:\Strawberry\perl\bin','C:\Other\perl\bin'))} 'Producer-local setup boundaries changed'
Refuse {Get-ProducerSetup ($Source.Replace('Pinned NASM executable missing','Changed NASM executable missing'))} 'Producer-local setup differs from the reviewed bounded span'
Refuse {Get-ProducerSetup ($Source+$Span)} 'Producer-local setup boundaries changed'
$Before='{"environment":{"PATHSha256":"'+('a'*64)+'","LIBSha256":null},"tools":{"perl":"C:\\real\\perl.exe"}}'
$After=$Before.Replace('a'*64,'b'*64)
Assert-OnlyPathDifference $Before $After
Pass $true 'Only PATH fingerprint difference accepted'
Refuse {Assert-OnlyPathDifference $Before $Before} 'Negative control did not change a valid PATHSha256'
Refuse {Assert-OnlyPathDifference $Before ($After.Replace('b'*64,'invalid'))} 'Negative control did not change a valid PATHSha256'
Refuse {Assert-OnlyPathDifference $Before ($After.Replace('real','other'))} 'Negative control changed facts besides PATHSha256'
Refuse {Assert-OnlyPathDifference $Before ($After.Replace('"LIBSha256":null','"LIBSha256":"changed"'))} 'Negative control changed facts besides PATHSha256'
Refuse {Assert-OnlyPathDifference $Before ($After.Replace('"LIBSha256":null','"LIBSha256":null,"PATHSha256":"'+('b'*64)+'"'))} 'Negative control changed facts besides PATHSha256'
# Confirm the native-only entry cannot run locally or replace any capture.
Refuse {& $Path} 'This bounded native parent-context proof requires a disposable Windows Actions runner'
Write-Output "$Checks local parser/comparison/source-boundary checks passed; no native tools exercised"

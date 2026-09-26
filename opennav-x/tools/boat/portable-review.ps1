# Explicit display-only actions for a prepared, fixture-OFF recovery package.
[CmdletBinding()]
param(
  [Parameter(Mandatory=$true)][string]$Record,
  [Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{64}$')][string]$ExpectedRecordSha256,
  [ValidateSet('Launch','Capture','Close','Verify')][string]$Action='Verify',
  [ValidateSet('XNav','Legacy','Safe')][string]$Mode='XNav',
  [int]$ProcessId=0,
  [ValidatePattern('^[a-z0-9-]+$')][string]$Name='display'
)
. (Join-Path $PSScriptRoot 'PortableReview.ps1')
$review=Read-PortableReview $Record $ExpectedRecordSha256 ($Action -in @('Launch','Verify'))
$product=$review.product;$workspace=Assert-LocalPath $review.record.workspace
if ($Action -eq 'Verify') {
  Assert-ProtectedInventory $review.record.protectedRoots $review.record.protectedFiles
  [pscustomobject]@{status='verified';commit=$product.commit;purpose=$review.record.purpose} | ConvertTo-Json
  exit
}
$job=[pscustomobject]@{action=$Action;executable=$product.executable;executableSha256=$product.executableSha256;processId=$ProcessId}
if ($Action -eq 'Launch') {
  Assert-ProtectedInventory $review.record.protectedRoots $review.record.protectedFiles
  $job.action='LaunchPortableReview'
  $job | Add-Member -NotePropertyName reviewRecord -NotePropertyValue (Assert-LocalPath $Record)
  $job | Add-Member -NotePropertyName reviewRecordSha256 -NotePropertyValue $ExpectedRecordSha256
  $job | Add-Member -NotePropertyName mode -NotePropertyValue (@{XNav='--xnav';Legacy='--legacy';Safe='--safe-mode'}[$Mode])
} else {
  if ($ProcessId -le 0) {throw 'The exact reviewed process ID is required.'}
  if ($Action -eq 'Capture') {
    $directory=New-RunDirectory $workspace 'portable-capture'
    $job | Add-Member -NotePropertyName imagePath -NotePropertyValue (Join-Path $directory ($Name+'.png'))
  }
}
$result=Invoke-InteractiveJob $workspace $job
if ($Action -eq 'Close') {Assert-ProtectedInventory $review.record.protectedRoots $review.record.protectedFiles}
$result | Add-Member -NotePropertyName buildCommit -NotePropertyValue $product.commit
$result | Add-Member -NotePropertyName acceptance -NotePropertyValue 'PRELIMINARY DISPLAY ONLY; original-profile/chart/hardware gates still blocked'
$result | ConvertTo-Json -Depth 8

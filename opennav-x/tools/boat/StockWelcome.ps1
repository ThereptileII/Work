# Source-specific stock first-start caution review. Called only after the full
# StockReview launch/profile/plugin proof; not an alternative launch authority.
function Initialize-StockWelcomeNative {
  if (-not ('OpenNavX.StockWelcomeNative' -as [type])) { Add-Type -Path (Join-Path $PSScriptRoot 'StockWelcomeNative.cs') }
}
function Assert-StockWelcomeInspection($Inspection,$Job,[datetime]$Now) {
  if ($Inspection.status -cne 'passed' -or $Inspection.action -cne 'ReviewStock' -or $Inspection.reviewAction -cne 'InspectWelcome' -or
      $Inspection.mode -cne 'StockLegacy' -or $Inspection.processId -ne $Job.processId -or
      $Inspection.executableSha256 -cne $Job.executableSha256 -or $Inspection.launchResultSha256 -cne $Job.launchResultSha256 -or
      $Inspection.launchRequestSha256 -cne $Job.launchRequestSha256 -or $Inspection.welcomeHelperSha256 -cne $Job.welcomeHelperSha256 -or
      $Inspection.welcomeNativeSha256 -cne $Job.welcomeNativeSha256 -or $Inspection.imageSha256 -cnotmatch '^[a-f0-9]{64}$') { throw 'Inspect this exact stock warning before acknowledging it.' }
  $at=[datetime]::Parse($Inspection.utc).ToUniversalTime()
  if ($at -gt $Now -or ($Now-$at).TotalMinutes -gt 30) { throw 'Warning inspection expired; capture and review it again.' }
  $info=$Inspection.nativeWindow
  if ($info.ProcessId -ne $Job.processId -or $info.Title -cne 'Welcome to OpenCPN' -or $info.ModalClass -cne '#32770' -or
      $info.AgreeText -cne 'Agree' -or $info.CancelText -cne 'Cancel' -or $info.AgreeId -ne 5100 -or $info.CancelId -ne 5101 -or
      $info.HtmlClass -cne 'wxWindowNR' -or $info.HtmlName -cne 'htmlWindow') { throw 'Captured warning is not the pinned English navigation caution.' }
}
function Save-StockWelcomeCapture([int]$ProcessId,$Info,[string]$Path) {
  $path=Assert-LocalPath $Path
  if (Test-Path -LiteralPath $path) { throw 'A new private warning image is required.' }
  Add-Type -AssemblyName System.Drawing
  $bitmap=New-Object Drawing.Bitmap($Info.Bounds.Width,$Info.Bounds.Height);$graphics=[Drawing.Graphics]::FromImage($bitmap)
  try {
    [OpenNavX.StockWelcomeNative]::AssertUnchanged($ProcessId,$Info)
    $graphics.CopyFromScreen($Info.Bounds.Left,$Info.Bounds.Top,0,0,$bitmap.Size)
    [OpenNavX.StockWelcomeNative]::AssertUnchanged($ProcessId,$Info)
    $bitmap.Save($path,[Drawing.Imaging.ImageFormat]::Png)
  } finally { $graphics.Dispose();$bitmap.Dispose() }
  return Get-Digest $path
}
function Invoke-StockWelcomeAgreement([int]$ProcessId,$Info,[string]$ExpectedImageHash,[string]$BeforeImage,[string]$IntentPath) {
  if ($ExpectedImageHash -cnotmatch '^[a-f0-9]{64}$') { throw 'Reviewed warning image hash required.' }
  $hash=Save-StockWelcomeCapture $ProcessId $Info $BeforeImage
  if ($hash -cne $ExpectedImageHash) { throw 'Warning pixels changed since inspection; no acknowledgement sent.' }
  # Durable exclusive one-use intent. Even uncertain delivery cannot be retried
  # using this inspection. Never confuse transmission with modal dismissal.
  Write-Record $IntentPath @{owner='OpenNavX.StockWelcome.1';status='agree-intent';utc=[datetime]::UtcNow.ToString('o');processId=$ProcessId;imageSha256=$ExpectedImageHash;nativeWindow=$Info}
  [OpenNavX.StockWelcomeNative]::Agree($ProcessId,$Info)
}
function Invoke-StockWelcomeReview($Job,$Review,$Process,[string]$Sid,[int]$Session) {
  foreach ($name in @('welcomeHelperSha256','welcomeNativeSha256')) { if ($Job.$name -cnotmatch '^[a-f0-9]{64}$') { throw 'Pinned warning helper hashes required.' } }
  if ((Get-Digest (Join-Path $PSScriptRoot 'StockWelcome.ps1')) -cne $Job.welcomeHelperSha256 -or
      (Get-Digest (Join-Path $PSScriptRoot 'StockWelcomeNative.cs')) -cne $Job.welcomeNativeSha256) { throw 'Warning review helper changed.' }
  $directory=Assert-StockImageDirectory $Job $Sid
  Initialize-StockWelcomeNative
  $result=@{status='passed';action='ReviewStock';reviewAction=$Job.reviewAction;utc=[datetime]::UtcNow.ToString('o');processId=$Process.Id;
    executableSha256=$Job.executableSha256;launchResultSha256=$Job.launchResultSha256;launchRequestSha256=$Job.launchRequestSha256;
    welcomeHelperSha256=$Job.welcomeHelperSha256;welcomeNativeSha256=$Job.welcomeNativeSha256;mode='StockLegacy';actuatorCommandsIssuedByTool=$false;
    physicalBusSilenceNotClaimed=$true;bodyTextAccessible=$false;source='OpenCPN 37fd0cddb7334fe489e9f18aa163977a9c5c84f7 ShowNavWarning -> AlertDialog';
    review='English GPL/no-warranty/navigation caution only. HTML body is verified by human review of the captured pixels; no text-accessibility claim.'}
  if ($Job.reviewAction -ceq 'InspectWelcome') {
    $info=[OpenNavX.StockWelcomeNative]::Inspect($Process.Id)
    $null=Read-StockReview $Job;$Process.Refresh();Assert-StockProcess $Process $Job $Review.launch $Sid $Session
    $image=Join-Path $directory 'welcome.png'
    $result.imageSha256=Save-StockWelcomeCapture $Process.Id $info $image
    $result.image=$image;$result.nativeWindow=$info;$result.acknowledgementSent=$false
    return $result
  }
  if ($Job.reviewAction -cne 'AcknowledgeWelcome' -or $Job.welcomeInspectionSha256 -cnotmatch '^[a-f0-9]{64}$') { throw 'A separately reviewed warning record is required.' }
  $inspectionPath=Assert-LocalPath $Job.welcomeInspection;$inspectionDir=[IO.Path]::GetDirectoryName($inspectionPath)
  if ([IO.Path]::GetFileName($inspectionPath) -cne 'review.json' -or [IO.Path]::GetDirectoryName($inspectionDir) -ine (Join-Path $Job.workspace 'runs') -or
      [IO.Path]::GetFileName($inspectionDir) -cnotmatch '^\d{8}-\d{6}-stock-review-[a-f0-9]{8}$' -or $inspectionDir -ieq $directory -or
      (Get-Digest $inspectionPath) -cne $Job.welcomeInspectionSha256) { throw 'Expected the owned captured warning review record.' }
  $inspection=Read-Record $inspectionPath
  Assert-StockWelcomeInspection $inspection $Job ([datetime]::UtcNow)
  if ($inspection.image -ine (Join-Path $inspectionDir 'welcome.png') -or (Get-Digest $inspection.image) -cne $inspection.imageSha256) { throw 'Reviewed warning capture changed.' }
  $info=[OpenNavX.StockWelcomeNative+NoticeInfo]$inspection.nativeWindow
  $intent=Join-Path $inspectionDir 'agree-intent.json'
  if (Test-Path -LiteralPath $intent) { throw 'This inspection already has an acknowledgement intent; inspect actual state, never retry it.' }
  $null=Read-StockReview $Job;$Process.Refresh();Assert-StockProcess $Process $Job $Review.launch $Sid $Session
  $before=Join-Path $directory 'welcome-before-agree.png'
  Invoke-StockWelcomeAgreement $Process.Id $info $inspection.imageSha256 $before $intent
  $result.acknowledgementSent=$true;$result.inspectionSha256=$Job.welcomeInspectionSha256;$result.before=$before;$result.beforeSha256=Get-Digest $before
  $result.nativeWindow=$info;$result.modalDismissalConfirmed=$false
  # No automatic retry, startup-dialog dismissals or force close. Preserve the
  # receipt if normal startup needs further attention (including another modal).
  $deadline=[datetime]::UtcNow.AddSeconds(20)
  do {
    Start-Sleep -Milliseconds 250;$Process.Refresh()
    if ($Process.HasExited) { break }
    try {
      Assert-StockProcess $Process $Job $Review.launch $Sid $Session
      $frame=$Process.MainWindowHandle
      [OpenNavX.StockReviewNative]::Foreground($frame,$Process.Id)
      $window=[OpenNavX.StockReviewNative]::AssertFrame($frame,$Process.Id)
      $result.modalDismissalConfirmed=$true
      $bitmap=New-Object Drawing.Bitmap($window.Bounds.Width,$window.Bounds.Height);$graphics=[Drawing.Graphics]::FromImage($bitmap)
      try {
        [OpenNavX.StockReviewNative]::AssertCapture($frame,$Process.Id,$window)
        $graphics.CopyFromScreen($window.Bounds.Left,$window.Bounds.Top,0,0,$bitmap.Size)
        [OpenNavX.StockReviewNative]::AssertCapture($frame,$Process.Id,$window)
        $after=Join-Path $directory 'after-agree.png';$bitmap.Save($after,[Drawing.Imaging.ImageFormat]::Png)
      } finally { $graphics.Dispose();$bitmap.Dispose() }
      $result.after=$after;$result.afterSha256=Get-Digest $after;$result.afterNativeWindow=$window
      break
    } catch { $result.attention=$_.Exception.Message }
  } while ([datetime]::UtcNow -lt $deadline)
  if (-not $result.modalDismissalConfirmed -or -not $result.ContainsKey('after')) { $result.status='attention';$result.review+=' Agreement delivery alone does not prove completed startup; inspect the application normally.' }
  return $result
}

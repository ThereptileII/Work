# Source-specific stock first-start caution review. Called only after the full
# StockReview launch/profile/plugin proof; not an alternative launch authority.
function Initialize-StockWelcomeNative {
  if (-not ('OpenNavX.StockWelcomeNative' -as [type])) { Add-Type -Path (Join-Path $PSScriptRoot 'StockWelcomeNative.cs') }
}
# Exact tuples from the qualified official English/Swedish catalogues. Keep this
# source ASCII for Windows PowerShell 5.1's BOM-less script decoding.
function Assert-StockWelcomeWindow($Info,[int]$ProcessId) {
  $english=$Info.Title -ceq 'Welcome to OpenCPN' -and $Info.AgreeText -ceq 'Agree' -and $Info.CancelText -ceq 'Cancel'
  $swedish=$Info.Title -ceq ('V'+[char]0x00e4+'lkommen till OpenCPN') -and $Info.AgreeText -ceq 'Acceptera' -and $Info.CancelText -ceq 'Avbryt'
  if ($ProcessId -le 0 -or $Info.ProcessId -ne $ProcessId -or (-not $english -and -not $swedish) -or
      $Info.ModalClass -cne '#32770' -or $Info.AgreeId -ne 5100 -or $Info.CancelId -ne 5101 -or
      $Info.HtmlClass -cne 'wxWindowNR' -or $Info.HtmlName -cne 'htmlWindow') {
    throw 'Captured warning is not the pinned English or Swedish navigation caution.'
  }
}
function Convert-StockWelcomeWindow($Info) {
  # JSON includes C# Rect's read-only Width/Height. Rebuild only writable fields,
  # and prove the serialized derived dimensions instead of discarding them.
  Initialize-StockWelcomeNative
  $names=@('Frame','Modal','Agree','Cancel','Html','ProcessId','AgreeId','CancelId','Dpi','Bounds','Title','ModalClass','HtmlClass','HtmlName','AgreeText','CancelText')
  if ($null -eq $Info -or ((@($Info.PSObject.Properties.Name | Sort-Object) -join '|') -cne (($names | Sort-Object) -join '|'))) { throw 'Captured warning field schema differs.' }
  $bounds=$Info.Bounds
  if ($null -eq $bounds -or ((@($bounds.PSObject.Properties.Name | Sort-Object) -join '|') -cne 'Bottom|Height|Left|Right|Top|Width')) { throw 'Captured warning rectangle schema differs.' }
  function Integer($Value,[long]$Minimum,[long]$Maximum) {
    if (($Value -isnot [int] -and $Value -isnot [long] -and $Value -isnot [uint32]) -or $Value -lt $Minimum -or $Value -gt $Maximum) { throw 'Captured warning integer type or bounds differ.' }
    return [long]$Value
  }
  $rectangle=New-Object OpenNavX.StockWelcomeNative+Rect
  foreach($name in @('Left','Top','Right','Bottom')) { $rectangle.$name=[int](Integer $bounds.$name ([int]::MinValue) ([int]::MaxValue)) }
  $width=Integer $bounds.Width 1 ([int]::MaxValue);$height=Integer $bounds.Height 1 ([int]::MaxValue)
  if (([long]$rectangle.Right-$rectangle.Left) -ne $width -or ([long]$rectangle.Bottom-$rectangle.Top) -ne $height) { throw 'Captured warning derived width or height differs.' }
  $result=New-Object OpenNavX.StockWelcomeNative+NoticeInfo
  foreach($name in @('Frame','Modal','Agree','Cancel','Html')) { $result.$name=Integer $Info.$name 1 ([long]::MaxValue) }
  foreach($name in @('ProcessId','AgreeId','CancelId')) { $result.$name=[int](Integer $Info.$name 1 ([int]::MaxValue)) }
  $result.Dpi=[uint32](Integer $Info.Dpi 72 384);$result.Bounds=$rectangle
  foreach($name in @('Title','ModalClass','HtmlClass','HtmlName','AgreeText','CancelText')) {
    if ($Info.$name -isnot [string]) { throw 'Captured warning text type differs.' };$result.$name=$Info.$name
  }
  Assert-StockWelcomeWindow $result $result.ProcessId
  return $result
}
function Assert-StockWelcomeInspection($Inspection,$Job,[datetime]$Now) {
  if ($Inspection.status -cne 'passed' -or $Inspection.action -cne 'ReviewStock' -or $Inspection.reviewAction -cne 'InspectWelcome' -or
      $Inspection.mode -cne 'StockLegacy' -or $Inspection.processId -ne $Job.processId -or
      $Inspection.executableSha256 -cne $Job.executableSha256 -or $Inspection.launchResultSha256 -cne $Job.launchResultSha256 -or
      $Inspection.launchRequestSha256 -cne $Job.launchRequestSha256 -or $Inspection.welcomeHelperSha256 -cne $Job.welcomeHelperSha256 -or
      $Inspection.welcomeNativeSha256 -cne $Job.welcomeNativeSha256 -or $Inspection.imageSha256 -cnotmatch '^[a-f0-9]{64}$') { throw 'Inspect this exact stock warning before acknowledging it.' }
  $at=[datetime]::Parse($Inspection.utc).ToUniversalTime()
  if ($at -gt $Now -or ($Now-$at).TotalMinutes -gt 30) { throw 'Warning inspection expired; capture and review it again.' }
  Assert-StockWelcomeWindow $Inspection.nativeWindow $Job.processId
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
function Restore-StockWelcomeAgreementForeground([int]$ProcessId,$Info) {
  # A separately scheduled acknowledgement can leave its own console foreground.
  # Use only the existing ordinary activation/rendezvous; no caption input.
  # Fresh discovery is never substituted for the saved reviewed observation.
  $null=[OpenNavX.StockWelcomeNative]::Inspect($ProcessId)
  [OpenNavX.StockWelcomeNative]::AssertUnchanged($ProcessId,$Info)
}
function Save-StockWelcomeSettledCapture([int]$ProcessId,$Info,[string]$ExpectedImageHash,[string]$BeforeImage) {
  if ($ExpectedImageHash -cnotmatch '^[a-f0-9]{64}$' -or (Test-Path -LiteralPath $BeforeImage)) { throw 'New image path and exact reviewed hash required.' }
  $watch=[Diagnostics.Stopwatch]::StartNew();$consecutive=0
  # DWM can finish painting a newly active title bar after WM_NULL completes.
  # Observe only: no focus/input retries, cropped pixels, or changed authority.
  for($attempt=0;$attempt -lt 20 -and $watch.ElapsedMilliseconds -lt 5000;$attempt++) {
    $candidate=$BeforeImage+'.settle-'+$attempt.ToString('00')+'.png'
    $hash=Save-StockWelcomeCapture $ProcessId $Info $candidate
    if ($watch.ElapsedMilliseconds -ge 5000) { break }
    if ($hash -ceq $ExpectedImageHash) { $consecutive++ } else { $consecutive=0 }
    if ($consecutive -eq 2) {
      # Preserve every observation and publish only the exact reviewed image.
      [IO.File]::Copy($candidate,$BeforeImage,$false)
      if ((Get-Digest $BeforeImage) -cne $ExpectedImageHash) { throw 'Settled warning copy changed before acknowledgement.' }
      return $ExpectedImageHash
    }
    if ($attempt -lt 19) { Start-Sleep -Milliseconds 150 }
  }
  throw 'Warning pixels did not settle to the exact reviewed image; no acknowledgement sent. Private full-image attempts retained.'
}
function Invoke-StockWelcomeAgreement([int]$ProcessId,$Info,[string]$ExpectedImageHash,[string]$BeforeImage,[string]$IntentPath) {
  if ($ExpectedImageHash -cnotmatch '^[a-f0-9]{64}$') { throw 'Reviewed warning image hash required.' }
  $Info=Convert-StockWelcomeWindow $Info
  if ($Info.ProcessId -ne $ProcessId) { throw 'Captured warning belongs to another process.' }
  Restore-StockWelcomeAgreementForeground $ProcessId $Info
  $hash=Save-StockWelcomeSettledCapture $ProcessId $Info $ExpectedImageHash $BeforeImage
  if ($hash -cne $ExpectedImageHash) { throw 'Warning pixels changed since inspection; no acknowledgement sent.' }
  # Durable exclusive one-use intent. Even uncertain delivery cannot be retried
  # using this inspection. Never confuse transmission with modal dismissal.
  Write-Record $IntentPath @{owner='OpenNavX.StockWelcome.1';status='agree-intent';utc=[datetime]::UtcNow.ToString('o');processId=$ProcessId;imageSha256=$ExpectedImageHash;nativeWindow=$Info}
  [OpenNavX.StockWelcomeNative]::Agree($ProcessId,$Info)
}
function Invoke-StockWelcomeFocus([int]$ProcessId,[long]$StartedUtcTicks,[string]$IntentPath) {
  if ($ProcessId -le 0 -or $StartedUtcTicks -le 0) { throw 'Exact launched process/start identity required for caption focus.' }
  Write-Record $IntentPath @{owner='OpenNavX.StockWelcome.Focus.1';status='focus-intent';utc=[datetime]::UtcNow.ToString('o');processId=$ProcessId;processStartedUtcTicks=$StartedUtcTicks;action='Fixed warning caption only';acknowledgementSent=$false}
  return [OpenNavX.StockWelcomeNative]::FocusCaption($ProcessId,$StartedUtcTicks)
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
    review='Pinned English/Swedish GPL/no-warranty/navigation caution only. HTML body is verified by human review of the captured pixels; no text-accessibility claim.'}
  if ($Job.reviewAction -ceq 'FocusWelcome') {
    $null=Read-StockReview $Job;$Process.Refresh();Assert-StockProcess $Process $Job $Review.launch $Sid $Session
    $ticks=([datetime]::Parse($Review.launch.processStartedUtc).ToUniversalTime()).Ticks
    $info=Invoke-StockWelcomeFocus $Process.Id $ticks (Join-Path $directory 'focus-intent.json')
    $null=Read-StockReview $Job;$Process.Refresh();Assert-StockProcess $Process $Job $Review.launch $Sid $Session
    $image=Join-Path $directory 'focused-warning.png'
    $result.imageSha256=Save-StockWelcomeCapture $Process.Id $info $image
    $result.image=$image;$result.nativeWindow=$info;$result.focusVerified=$true;$result.acknowledgementSent=$false
    $result.review='Fixed warning caption focused; warning remains present. This is not an InspectWelcome record and cannot authorize acknowledgement.'
    return $result
  }
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
  $info=$inspection.nativeWindow
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

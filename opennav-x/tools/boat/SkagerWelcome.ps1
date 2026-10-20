# Fixed installed SKAGER notice only. Guard and image-settling mechanics copied
# from StockWelcome.ps1 at db8a087b; stock operations remain unchanged. Called
# only after InstalledWelcome completes its full live runtime/source proof.
function Initialize-SkagerWelcomeNative {
  if (-not ('OpenNavX.SkagerWelcomeNative' -as [type])) { Add-Type -Path (Join-Path $PSScriptRoot 'SkagerWelcomeNative.cs') }
}
# The frozen qualified product and reviewed corresponding-source tuple. These
# source hashes are the same checked by installer-welcome.py in native CI:
# StartupNavigationNotice.cpp (CRLF): 0750845170b2fe1b2f042ba52041abaeebc61d8cde0aae06b775c39c216c9a7a
# startup-presentation.patch: dfa5863b67701dcf4e9e25e59619e96ad2216ed30065e6183762225d41bbb502
# Source archive: 6b5dbaa7228bd7ad074af7f166b5739176b62bc49c2a6b76a4c96bfc98bd2e89
# Actual retained native qualification: 39331651669, product bf489683. This
# selector is not a version range or an observed-title fallback.
function Get-InstalledWelcomePresentation($Launch,$Job) {
  if($Launch.mode -cnotin @('--xnav','--legacy','--safe-mode')) { throw 'Reviewed installed mode required.' }
  if($Launch.mode -cne '--xnav') { return 'opencpn' }
  if($Job.buildCommit -ceq 'bf4896833fda8389cfb03924cde7d6c89de5e5e6') {
    if($Job.executableSha256 -cne 'cb1627969e5564af9ff598e0327b0a6028ea7c774f5ec95d90fa9282fa824ced') { throw 'Reviewed SKAGER executable/source tuple differs.' }
    return 'skager'
  }
  return 'opencpn'
}
function Assert-SkagerWelcomeWindow($Info,[int]$ProcessId) {
  Initialize-SkagerWelcomeNative
  $field=[OpenNavX.SkagerWelcomeNative].GetField('NavigationNotice',[Reflection.BindingFlags]'NonPublic,Static')
  if($ProcessId -le 0 -or $Info.ProcessId -ne $ProcessId -or $Info.Title -cne 'Welcome to SKAGER' -or
     $Info.FrameTitle -cne 'SKAGER / OpenCPN' -or $Info.ModalClass -cne '#32770' -or
     $Info.AgreeText -cne 'Agree' -or $Info.CancelText -cne 'Cancel' -or $Info.AgreeId -ne 5100 -or $Info.CancelId -ne 5101 -or
     $Info.AgreeClass -cne 'wxWindowNR' -or $Info.CancelClass -cne 'wxWindowNR' -or
     $Info.GripperClass -cne 'ScrollBar' -or $Info.GripperId -ne 0 -or $Info.GripperStyle -ne 0x5400001c -or $Info.GripperExStyle -ne 0 -or
     $Info.BodyClass -cne 'Edit' -or ($Info.BodyStyle -band 0x0804) -ne 0x0804 -or
     $Info.BodyText -cne $field.GetRawConstantValue()) { throw 'Captured notice differs from the exact reviewed SKAGER safety notice.' }
}
function Convert-SkagerWelcomeWindow($Info) {
  # JSON includes C# Rect's read-only Width/Height. Rebuild only writable fields,
  # and prove the serialized derived dimensions instead of discarding them.
  Initialize-SkagerWelcomeNative
  $names=@('Frame','Modal','Agree','Cancel','Body','ProcessId','AgreeId','CancelId','Dpi','Bounds','Title','ModalClass','BodyClass','BodyText','AgreeText','CancelText','BodyStyle','FrameTitle','AgreeClass','CancelClass','AgreeBounds','Gripper','GripperId','GripperStyle','GripperExStyle','GripperClass','GripperBounds')
  if ($null -eq $Info -or ((@($Info.PSObject.Properties.Name | Sort-Object) -join '|') -cne (($names | Sort-Object) -join '|'))) { throw 'Captured warning field schema differs.' }
  function Integer($Value,[long]$Minimum,[long]$Maximum) {
    if (($Value -isnot [int] -and $Value -isnot [long] -and $Value -isnot [uint32]) -or $Value -lt $Minimum -or $Value -gt $Maximum) { throw 'Captured notice integer type or bounds differ.' }
    return [long]$Value
  }
  function Rectangle($bounds) {
    if ($null -eq $bounds -or ((@($bounds.PSObject.Properties.Name | Sort-Object) -join '|') -cne 'Bottom|Height|Left|Right|Top|Width')) { throw 'Captured notice rectangle schema differs.' }
    $rectangle=New-Object OpenNavX.SkagerWelcomeNative+Rect
    foreach($name in @('Left','Top','Right','Bottom')) { $rectangle.$name=[int](Integer $bounds.$name ([int]::MinValue) ([int]::MaxValue)) }
    $width=Integer $bounds.Width 1 ([int]::MaxValue);$height=Integer $bounds.Height 1 ([int]::MaxValue)
    if (([long]$rectangle.Right-$rectangle.Left) -ne $width -or ([long]$rectangle.Bottom-$rectangle.Top) -ne $height) { throw 'Captured notice derived dimensions differ.' }
    return $rectangle
  }
  $result=New-Object OpenNavX.SkagerWelcomeNative+NoticeInfo
  foreach($name in @('Frame','Modal','Agree','Cancel','Body','Gripper')) { $result.$name=Integer $Info.$name 1 ([long]::MaxValue) }
  foreach($name in @('ProcessId','AgreeId','CancelId')) { $result.$name=[int](Integer $Info.$name 1 ([int]::MaxValue)) }
  $result.Dpi=[uint32](Integer $Info.Dpi 72 384);$result.Bounds=Rectangle $Info.Bounds;$result.AgreeBounds=Rectangle $Info.AgreeBounds
  $result.GripperBounds=Rectangle $Info.GripperBounds
  $result.GripperId=[int](Integer $Info.GripperId 0 0)
  $result.GripperExStyle=[int](Integer $Info.GripperExStyle 0 0)
  $result.GripperStyle=[int](Integer $Info.GripperStyle 0 ([int]::MaxValue))
  $result.BodyStyle=[int](Integer $Info.BodyStyle ([int]::MinValue) ([int]::MaxValue))
  foreach($name in @('Title','ModalClass','BodyClass','BodyText','AgreeText','CancelText','FrameTitle','AgreeClass','CancelClass','GripperClass')) {
    if ($Info.$name -isnot [string]) { throw 'Captured warning text type differs.' };$result.$name=$Info.$name
  }
  Assert-SkagerWelcomeWindow $result $result.ProcessId
  return $result
}
function Save-SkagerWelcomeCapture([int]$ProcessId,$Info,[string]$Path) {
  $path=Assert-LocalPath $Path
  if (Test-Path -LiteralPath $path) { throw 'A new private warning image is required.' }
  Add-Type -AssemblyName System.Drawing
  $bitmap=New-Object Drawing.Bitmap($Info.Bounds.Width,$Info.Bounds.Height);$graphics=[Drawing.Graphics]::FromImage($bitmap)
  try {
    [OpenNavX.SkagerWelcomeNative]::AssertUnchanged($ProcessId,$Info)
    $graphics.CopyFromScreen($Info.Bounds.Left,$Info.Bounds.Top,0,0,$bitmap.Size)
    [OpenNavX.SkagerWelcomeNative]::AssertUnchanged($ProcessId,$Info)
    $bitmap.Save($path,[Drawing.Imaging.ImageFormat]::Png)
  } finally { $graphics.Dispose();$bitmap.Dispose() }
  return Get-Digest $path
}
function Restore-SkagerWelcomeAgreementForeground([int]$ProcessId,$Info) {
  # A separately scheduled acknowledgement can leave its own console foreground.
  # Use only the existing ordinary activation/rendezvous; no caption input.
  # Fresh discovery is never substituted for the saved reviewed observation.
  $null=[OpenNavX.SkagerWelcomeNative]::Inspect($ProcessId)
  [OpenNavX.SkagerWelcomeNative]::AssertUnchanged($ProcessId,$Info)
}
function Save-SkagerWelcomeSettledCapture([int]$ProcessId,$Info,[string]$ExpectedImageHash,[string]$BeforeImage) {
  if ($ExpectedImageHash -cnotmatch '^[a-f0-9]{64}$' -or (Test-Path -LiteralPath $BeforeImage)) { throw 'New image path and exact reviewed hash required.' }
  $watch=[Diagnostics.Stopwatch]::StartNew();$consecutive=0
  # DWM can finish painting a newly active title bar after WM_NULL completes.
  # Observe only: no focus/input retries, cropped pixels, or changed authority.
  for($attempt=0;$attempt -lt 20 -and $watch.ElapsedMilliseconds -lt 5000;$attempt++) {
    $candidate=$BeforeImage+'.settle-'+$attempt.ToString('00')+'.png'
    $hash=Save-SkagerWelcomeCapture $ProcessId $Info $candidate
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
function Invoke-SkagerWelcomeAgreement([int]$ProcessId,$Info,[string]$ExpectedImageHash,[string]$BeforeImage,[string]$IntentPath) {
  if ($ExpectedImageHash -cnotmatch '^[a-f0-9]{64}$') { throw 'Reviewed warning image hash required.' }
  $Info=Convert-SkagerWelcomeWindow $Info
  if ($Info.ProcessId -ne $ProcessId) { throw 'Captured warning belongs to another process.' }
  Restore-SkagerWelcomeAgreementForeground $ProcessId $Info
  $hash=Save-SkagerWelcomeSettledCapture $ProcessId $Info $ExpectedImageHash $BeforeImage
  if ($hash -cne $ExpectedImageHash) { throw 'Warning pixels changed since inspection; no acknowledgement sent.' }
  # Durable exclusive one-use intent. Even uncertain delivery cannot be retried
  # using this inspection. Never confuse transmission with modal dismissal.
  Write-Record $IntentPath @{owner='OpenNavX.SkagerWelcome.1';status='agree-intent';utc=[datetime]::UtcNow.ToString('o');processId=$ProcessId;imageSha256=$ExpectedImageHash;nativeWindow=$Info}
  [OpenNavX.SkagerWelcomeNative]::Agree($ProcessId,$Info)
}
function Invoke-SkagerWelcomeFocus([int]$ProcessId,[long]$StartedUtcTicks,[string]$IntentPath) {
  if ($ProcessId -le 0 -or $StartedUtcTicks -le 0) { throw 'Exact launched process/start identity required for caption focus.' }
  Write-Record $IntentPath @{owner='OpenNavX.SkagerWelcome.Focus.1';status='focus-intent';utc=[datetime]::UtcNow.ToString('o');processId=$ProcessId;processStartedUtcTicks=$StartedUtcTicks;action='Fixed warning caption only';acknowledgementSent=$false}
  return [OpenNavX.SkagerWelcomeNative]::FocusCaption($ProcessId,$StartedUtcTicks)
}

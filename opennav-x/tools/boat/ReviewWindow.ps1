# Read-only vessel/UI review. No application launch or equipment commands.
. (Join-Path $PSScriptRoot 'Common.ps1')
function Get-WindowReviewActions {
  return @('Capture','Resize1280x800','Menu','Navigation','Routes','Waypoints','AIS','Instruments','Advice','PilotView','Anchor','Settings','Sources','Route','Energy','Diagnostics','System','Alerts','Escape','CyclePalette','ZoomIn','ZoomOut','Center','PageUp','PageDown','SelectFirstVisibleWaypoint','SelectFirstVisibleAis')
}
function Assert-WindowReviewPolicy($Job,$Installed,$Build,$Launch,$Request,[datetime]$Now) {
  if($Job.action -cne 'ReviewWindow' -or $Job.reviewAction -cnotin (Get-WindowReviewActions)){throw 'Unsupported read-only window action.'}
  if($Job.buildCommit -cnotmatch '^[a-f0-9]{40}$' -or $Job.generation -cnotmatch '^[a-f0-9]{32}$' -or
      $Job.executableSha256 -cnotmatch '^[a-f0-9]{64}$' -or $Job.processId -le 0){throw 'Exact installed process/build identities required.'}
  if($Installed.state.current -cne $Job.generation -or $Installed.ownership.commit -cne $Job.buildCommit -or
      $Installed.ownership.version -cne '0.4.0-beta2' -or $Installed.executable -ine $Job.executable){throw 'Installed generation changed.'}
  if($Build.test_fixtures -isnot [bool] -or $Build.test_fixtures -ne $false -or $Build.build_purpose -cne 'INSTALLED PRODUCT' -or
      $Build.version -cne '0.4.0-beta2' -or $Build.commit -cne $Job.buildCommit -or $Build.executable_sha256 -cne $Job.executableSha256){throw 'Only the exact fixture-free installed product may be reviewed.'}
  if($Launch.status -cne 'passed' -or $Launch.action -cne 'Launch' -or $Launch.mode -cne '--xnav' -or $Launch.pid -ne $Job.processId -or
      $Request.action -cne 'Launch' -or $Request.mode -cne '--xnav' -or $Request.executable -ine $Job.executable -or
      $Request.executableSha256 -cne $Job.executableSha256 -or $Request.workspace -ine $Job.workspace){throw 'Successful audited installed XNav launch is required.'}
  $at=[datetime]::Parse($Launch.utc).ToUniversalTime()
  if($at -gt $Now -or ($Now-$at).TotalHours -gt 4){throw 'Window review requires a recent audited launch.'}
}
function Assert-WindowReviewProcess($Process,$Job,$Launch,[int]$Session) {
  if($Process.Id -ne $Job.processId -or $Process.Path -ine $Job.executable -or $Process.SessionId -ne $Session -or
      -not $Process.MainWindowHandle -or $Process.HasExited){throw 'Reviewed process/session/window identity changed.'}
  $start=$Process.StartTime.ToUniversalTime();$launchAt=[datetime]::Parse($Launch.utc).ToUniversalTime()
  if($start -lt $launchAt -or ($start-$launchAt).TotalSeconds -gt 180){throw 'Process creation does not match audited launch; PID reuse refused.'}
}
function Read-WindowReview($Job) {
  foreach($field in @('launchResultSha256','launchRequestSha256','reviewHelperSha256','nativeHelperSha256')) {
    if($Job.$field -cnotmatch '^[a-f0-9]{64}$'){throw 'Exact launch evidence hashes required.'}
  }
  if((Get-Digest (Join-Path $PSScriptRoot 'ReviewWindow.ps1')) -cne $Job.reviewHelperSha256 -or
      (Get-Digest (Join-Path $PSScriptRoot 'ReviewWindowNative.cs')) -cne $Job.nativeHelperSha256){throw 'Review helper changed after dispatch.'}
  $launchPath=Assert-LocalPath $Job.launchResult
  $requestPath=Join-Path ([IO.Path]::GetDirectoryName($launchPath)) 'request.json'
  if([IO.Path]::GetFileName($launchPath) -cne 'result.json' -or (Get-Digest $launchPath) -cne $Job.launchResultSha256 -or
      (Get-Digest $requestPath) -cne $Job.launchRequestSha256){throw 'Launch evidence changed.'}
  $launch=Read-Record $launchPath;$request=Read-Record $requestPath
  if($request.resultPath -ine $launchPath){throw 'Launch request/result path mismatch.'}
  $installed=Get-Installed
  $buildPath=Join-Path $installed.generation 'docs\PRODUCT_BUILD.json'
  $record=@($installed.ownership.managedFiles | Where-Object {$_.path -ceq 'docs/PRODUCT_BUILD.json'})
  if($record.Count -ne 1 -or (Get-Digest $buildPath) -cne $record[0].sha256){throw 'Installed build report is not the exact owned file.'}
  $build=Read-Record $buildPath
  Assert-WindowReviewPolicy $Job $installed $build $launch $request ([datetime]::UtcNow)
  if((Get-Digest $installed.executable) -cne $Job.executableSha256){throw 'Reviewed executable changed.'}
  return [pscustomobject]@{installed=$installed;launch=$launch;request=$request}
}
function Initialize-WindowReviewNative {
  if(-not ('OpenNavX.ReviewWindowNative' -as [type])){Add-Type -Path (Join-Path $PSScriptRoot 'ReviewWindowNative.cs')}
}
function Save-WindowReviewImage([IntPtr]$Frame,[int]$ProcessId,[string]$Path) {
  $path=Assert-LocalPath $Path
  if(Test-Path -LiteralPath $path){throw 'A fresh screenshot path is required.'}
  $info=[OpenNavX.ReviewWindowNative]::AssertFrame($Frame,$ProcessId)
  Add-Type -AssemblyName System.Drawing
  $bitmap=New-Object Drawing.Bitmap($info.Bounds.Width,$info.Bounds.Height)
  $graphics=[Drawing.Graphics]::FromImage($bitmap)
  try {
    [OpenNavX.ReviewWindowNative]::AssertCapture($Frame,$ProcessId,$info)
    $graphics.CopyFromScreen($info.Bounds.Left,$info.Bounds.Top,0,0,$bitmap.Size)
    # Pure checks here: never reacquire foreground after copying desktop pixels.
    [OpenNavX.ReviewWindowNative]::AssertCapture($Frame,$ProcessId,$info)
    $bitmap.Save($path,[Drawing.Imaging.ImageFormat]::Png)
  } finally {$graphics.Dispose();$bitmap.Dispose()}
  return @{path=$path;sha256=(Get-Digest $path);window=$info}
}
function Invoke-WindowReview($Job) {
  $review=Read-WindowReview $Job
  $directory=Assert-LocalPath $Job.evidenceDirectory
  if([IO.Path]::GetDirectoryName($directory) -ine (Join-Path (Assert-LocalPath $Job.workspace) 'runs') -or
      [IO.Path]::GetFileName($directory) -cnotmatch '^\d{8}-\d{6}-window-review-[a-f0-9]{8}$'){throw 'Expected an owned private window-review evidence directory.'}
  if(-not [IO.Directory]::Exists($directory) -or @(Get-ChildItem -LiteralPath $directory -Force).Count){throw 'A new empty private evidence directory is required.'}
  $acl=Get-Acl -LiteralPath $directory
  if(-not $acl.AreAccessRulesProtected){throw 'Review screenshots must not inherit public access.'}
  $allowed=@([Security.Principal.WindowsIdentity]::GetCurrent().User.Value,'S-1-5-18','S-1-5-32-544')
  foreach($rule in $acl.GetAccessRules($true,$true,[Security.Principal.SecurityIdentifier])) {
    if($rule.IdentityReference.Value -cnotin $allowed){throw 'Unexpected principal on private screenshot directory.'}
  }
  $process=Get-Process -Id $Job.processId -ErrorAction Stop
  try {
    Assert-WindowReviewProcess $process $Job $review.launch ([Diagnostics.Process]::GetCurrentProcess().SessionId)
    Initialize-WindowReviewNative
    $oldDpi=[OpenNavX.ReviewWindowNative]::SetThreadDpiAwarenessContext([IntPtr](-4))
    if($oldDpi -eq [IntPtr]::Zero){throw 'Physical pixel DPI context unavailable.'}
    try {
      $frame=$process.MainWindowHandle
      [OpenNavX.ReviewWindowNative]::Foreground($frame,$process.Id)
      $before=Save-WindowReviewImage $frame $process.Id (Join-Path $directory 'before.png')
      # Repeat identity and evidence binding immediately before a single action.
      $null=Read-WindowReview $Job;$process.Refresh()
      Assert-WindowReviewProcess $process $Job $review.launch ([Diagnostics.Process]::GetCurrentProcess().SessionId)
      if($process.MainWindowHandle -ne $frame){throw 'Reviewed main window changed.'}
      $selection=$null
      switch($Job.reviewAction) {
        'Capture' {}
        'Resize1280x800' {[OpenNavX.ReviewWindowNative]::Resize1280x800($frame,$process.Id)}
        'Escape' {[OpenNavX.ReviewWindowNative]::Escape($frame,$process.Id)}
        {$_ -cin @('SelectFirstVisibleWaypoint','SelectFirstVisibleAis')} {$selection=[OpenNavX.ReviewWindowNative]::SelectRow($frame,$process.Id,$Job.reviewAction)}
        default {[OpenNavX.ReviewWindowNative]::Click($frame,$process.Id,$Job.reviewAction)}
      }
      $after=Save-WindowReviewImage $frame $process.Id (Join-Path $directory 'after.png')
      return @{status='passed';action='ReviewWindow';reviewAction=$Job.reviewAction;utc=[datetime]::UtcNow.ToString('o');processId=$process.Id;
        buildCommit=$Job.buildCommit;generation=$Job.generation;executableSha256=$Job.executableSha256;launchResultSha256=$Job.launchResultSha256;
        reviewHelperSha256=$Job.reviewHelperSha256;nativeHelperSha256=$Job.nativeHelperSha256;
        before=$before;after=$after;nativeWindow=$after.window;selection=$selection;pages=@([OpenNavX.ReviewWindowNative]::VisiblePageLabels($frame));
        inputMethod='Targeted reviewed HWND mouse messages or focused-HWND Escape; no global keyboard/mouse input';readOnly=$true;actionsSent=$(if($Job.reviewAction -ceq 'Capture'){0}else{1});review='Private native pixels require human per-step visual review; no feature acceptance inferred.'}
    } finally {$null=[OpenNavX.ReviewWindowNative]::SetThreadDpiAwarenessContext($oldDpi)}
  } finally {$process.Dispose()}
}

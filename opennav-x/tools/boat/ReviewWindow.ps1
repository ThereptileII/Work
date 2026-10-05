# Read-only vessel/UI review. No application launch or equipment commands.
. (Join-Path $PSScriptRoot 'Common.ps1')
function Get-WindowReviewActions {
  return @('Capture','Resize1280x800','Menu','Navigation','Routes','Waypoints','AIS','Instruments','Advice','PilotView','Anchor','Settings','Sources','Display','ToggleFullscreen','ToggleOrientation','Route','Energy','Diagnostics','System','Alerts','Escape','CyclePalette','ZoomIn','ZoomOut','Center','PanRight','PageUp','PageDown','SelectFirstVisibleWaypoint','SelectFirstVisibleAis','Layers','ChartPalettePreferences','RevealChartPalettePreference')
}
function Convert-WindowReviewChart($Data,[string]$Commit,[datetime]$Written,[datetime]$Now) {
  if($Commit -cnotmatch '^[a-f0-9]{40}$' -or $Written -gt $Now -or ($Now-$Written).TotalSeconds -gt 5 -or $Data.build_commit -cne $Commit -or
     $Data.build_purpose -cne 'INSTALLED PRODUCT' -or $Data.data_mode -cne 'OPENCPN selected navigation' -or
     $Data.ui_page -cne 'Navigation' -or $Data.runtime.display.route_creation_active -isnot [bool] -or
     $Data.runtime.display.route_creation_active -ne $false){throw 'Fresh installed Navigation observation without route creation required for pan.'}
  $c=$Data.runtime.display.chart_region
  foreach($name in @('x','y','width','height')) {
    if($c.$name -isnot [int] -and $c.$name -isnot [long]){throw 'Chart geometry must use exact integer pixels.'}
    if($c.$name -lt 0 -or $c.$name -gt 32768){throw 'Unexpected chart pixel bounds.'}
  }
  if($c.width -lt 100 -or $c.height -lt 100){throw 'Chart is not visible.'}
  Initialize-WindowReviewNative
  $rect=New-Object OpenNavX.ReviewWindowNative+Rect
  $rect.Left=$c.x;$rect.Top=$c.y;$rect.Right=$c.x+$c.width;$rect.Bottom=$c.y+$c.height
  return $rect
}
function Invoke-WindowReviewPan([IntPtr]$Frame,[int]$ProcessId,[string]$Workspace,[string]$Commit) {
  $config=Get-Target $Workspace
  # Installed Attach/Configure writes beneath the shared profile's opennav-logs
  # directory. The profile-root path belongs to explicit --configdir tests and
  # must not be used as a fallback for an installed boat review.
  $path=Assert-LocalPath (Join-Path (Join-Path $config.profileDirectory 'opennav-logs') 'opennav-diagnostics.json')
  $written=(Get-Item -LiteralPath $path).LastWriteTimeUtc;$data=Read-Record $path
  $chart=Convert-WindowReviewChart $data $Commit $written ([datetime]::UtcNow)
  [OpenNavX.ReviewWindowNative]::PanRight($Frame,$ProcessId,$chart)
}
function Convert-WindowReviewAis($Data,[string]$Commit,[datetime]$Written,[datetime]$Now,[string]$Page) {
  if($Page -cnotin @('AIS targets','AIS target') -or $Commit -cnotmatch '^[a-f0-9]{40}$' -or
     $Written -gt $Now -or ($Now-$Written).TotalSeconds -gt 5 -or $Data.build_commit -cne $Commit -or
     $Data.build_purpose -cne 'INSTALLED PRODUCT' -or $Data.data_mode -cne 'OPENCPN selected navigation' -or
     $Data.ui_page -cne $Page -or $Data.runtime.display.route_creation_active -isnot [bool] -or
     $Data.runtime.display.route_creation_active -ne $false){throw 'Fresh exact installed AIS page observation required.'}
  $tick=[string]$Data.runtime.ui_update.ticks
  [ulong]$parsedTick=0
  if($tick -cnotmatch '^[0-9]{1,20}$' -or -not [ulong]::TryParse($tick,[ref]$parsedTick)){throw 'Exact AIS observation tick required.'}
  $mmsi=$Data.runtime.ais_selected_mmsi
  if(($mmsi -isnot [int] -and $mmsi -isnot [long]) -or
     ($Page -ceq 'AIS targets' -and $mmsi -ne 0) -or
     ($Page -ceq 'AIS target' -and ($mmsi -lt 1 -or $mmsi -gt 999999999))){throw 'Current selected AIS identity does not match the observed page.'}
  return [pscustomobject]@{tick=$parsedTick;selectedMmsi=[int]$mmsi}
}
function Invoke-WindowReviewSelection([IntPtr]$Frame,[int]$ProcessId,[string]$Action,[string]$Workspace,[string]$Commit) {
  $identity=[OpenNavX.ReviewWindowNative]::AssertFrame($Frame,$ProcessId)
  if($Action -cne 'SelectFirstVisibleAis' -or $identity.Shell -cne 'prototype') {
    return [OpenNavX.ReviewWindowNative]::SelectRow($Frame,$ProcessId,$Action)
  }
  $config=Get-Target $Workspace
  $path=Assert-LocalPath (Join-Path (Join-Path $config.profileDirectory 'opennav-logs') 'opennav-diagnostics.json')
  $before=Convert-WindowReviewAis (Read-Record $path) $Commit (Get-Item -LiteralPath $path).LastWriteTimeUtc ([datetime]::UtcNow) 'AIS targets'
  $selected=[OpenNavX.ReviewWindowNative]::SelectRow($Frame,$ProcessId,$Action)
  # Wait for observation only. Never repeat the sole selection input, and never
  # infer a target identity from the pixels or manufacture HWNDs for painted rows.
  $deadline=[datetime]::UtcNow.AddSeconds(3)
  do {
    [OpenNavX.ReviewWindowNative]::AssertPrototypeAisDetail($Frame,$ProcessId)
    $current=[OpenNavX.ReviewWindowNative]::AssertFrame($Frame,$ProcessId)
    if(@($current.Surfaces|Where-Object {$_.Title -ceq 'SKAGER vessel traffic' -and $_.Handle -eq $selected.SurfaceHandle}).Count -ne 1){throw 'Traffic drawer changed while awaiting its selected identity.'}
    try {
      $after=Convert-WindowReviewAis (Read-Record $path) $Commit (Get-Item -LiteralPath $path).LastWriteTimeUtc ([datetime]::UtcNow) 'AIS target'
      if($after.tick -gt $before.tick) {$selected.SelectedMmsi=$after.selectedMmsi;return $selected}
    } catch {if([datetime]::UtcNow -ge $deadline){throw}}
    Start-Sleep -Milliseconds 100
  } while([datetime]::UtcNow -lt $deadline)
  throw 'Selection has no fresh current AIS identity; no input retry.'
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
      $Request.executableSha256 -cne $Job.executableSha256 -or $Request.workspace -ine $Job.workspace){throw 'Successful audited installed SKAGER launch is required.'}
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
      $before=$null;$resize=$null
      if($Job.reviewAction -cne 'Resize1280x800') {
        [OpenNavX.ReviewWindowNative]::Foreground($frame,$process.Id)
        $before=Save-WindowReviewImage $frame $process.Id (Join-Path $directory 'before.png')
      }
      # Fixed resize may recover the normal rectangle left partly offscreen by
      # Windows restore. Record native before/restored/after geometry instead of
      # publishing a misleading clipped before screenshot. Other actions keep
      # the strict complete-frame capture guard.
      # Repeat identity and evidence binding immediately before a single action.
      $null=Read-WindowReview $Job;$process.Refresh()
      Assert-WindowReviewProcess $process $Job $review.launch ([Diagnostics.Process]::GetCurrentProcess().SessionId)
      if($process.MainWindowHandle -ne $frame){throw 'Reviewed main window changed.'}
      $selection=$null
      switch($Job.reviewAction) {
        'Capture' {}
        'Resize1280x800' {$resize=[OpenNavX.ReviewWindowNative]::Resize1280x800($frame,$process.Id)}
        'Escape' {[OpenNavX.ReviewWindowNative]::Escape($frame,$process.Id)}
        'PanRight' {Invoke-WindowReviewPan $frame $process.Id $Job.workspace $Job.buildCommit}
        {$_ -cin @('SelectFirstVisibleWaypoint','SelectFirstVisibleAis')} {$selection=Invoke-WindowReviewSelection $frame $process.Id $Job.reviewAction $Job.workspace $Job.buildCommit}
        default {[OpenNavX.ReviewWindowNative]::Click($frame,$process.Id,$Job.reviewAction)}
      }
      $after=Save-WindowReviewImage $frame $process.Id (Join-Path $directory 'after.png')
      return @{status='passed';action='ReviewWindow';reviewAction=$Job.reviewAction;utc=[datetime]::UtcNow.ToString('o');processId=$process.Id;
        buildCommit=$Job.buildCommit;generation=$Job.generation;executableSha256=$Job.executableSha256;launchResultSha256=$Job.launchResultSha256;
        reviewHelperSha256=$Job.reviewHelperSha256;nativeHelperSha256=$Job.nativeHelperSha256;
        before=$before;after=$after;resize=$resize;nativeWindow=$after.window;selection=$selection;pages=@([OpenNavX.ReviewWindowNative]::VisiblePageLabels($frame));
        inputMethod='Targeted reviewed HWND mouse messages, focused-HWND Escape or exact-canvas Right arrow; no global input';readOnly=$true;actionsSent=$(if($Job.reviewAction -ceq 'Capture'){0}else{1});review='Private native pixels require human per-step visual review; no feature acceptance inferred.'}
    } finally {$null=[OpenNavX.ReviewWindowNative]::SetThreadDpiAwarenessContext($oldDpi)}
  } finally {$process.Dispose()}
}

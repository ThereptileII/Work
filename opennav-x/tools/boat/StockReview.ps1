# Official stock OpenCPN coexistence review after uninstall. No XNav identity,
# synthetic data, arbitrary arguments, navigation actions or equipment commands.
. (Join-Path $PSScriptRoot 'Commissioning.ps1')
. (Join-Path $PSScriptRoot 'StockWelcome.ps1')
function Get-StockTarget([string]$Workspace) {
  $config=Get-Target $Workspace
  $local=Assert-LocalPath ([Environment]::GetFolderPath('LocalApplicationData'))
  if (-not $env:LOCALAPPDATA -or (Assert-LocalPath $env:LOCALAPPDATA) -ine $local) { throw 'Stock plugin account is ambiguous.' }
  foreach ($name in @('state.json','transaction.json')) {
    if (Test-Path -LiteralPath (Join-Path (Join-Path $local 'OpenNavXAlpha1') $name)) { throw 'Remove/resolve the installed OpenNav integration before stock coexistence review.' }
  }
  foreach ($view in @([Microsoft.Win32.RegistryView]::Registry32,[Microsoft.Win32.RegistryView]::Registry64)) {
    $base=[Microsoft.Win32.RegistryKey]::OpenBaseKey([Microsoft.Win32.RegistryHive]::CurrentUser,$view)
    try {
      $key=$base.OpenSubKey('Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenNavXAlpha1')
      if ($key) { $key.Dispose();throw 'OpenNav uninstall registration remains; no stock coexistence claim.' }
    } finally { $base.Dispose() }
  }
  $profile=Assert-LocalPath (Join-Path ([Environment]::GetFolderPath('CommonApplicationData')) 'opencpn')
  if ((Assert-LocalPath $config.profileDirectory) -ine $profile) { throw 'Stock review requires the actual shared profile.' }
  Assert-StockAuditIdentity $null (Get-Digest $config.stockExecutable) $config.stockReadOnlyAudit
  return $config
}
function Assert-StockLaunchAudit($Config,[string]$Workspace) {
  $audit=$Config.stockReadOnlyAudit
  foreach ($field in @('connectionsOutputDisabled','pluginOutputsReviewed','noActiveRouteOutput')) { Assert-TrueBoolean $audit.$field $field }
  # Full source review, original/input byte proof, active applied transaction,
  # quarantine locations, every stock/managed helper byte and live input-only INI.
  & (Join-Path $PSScriptRoot 'verify-commissioning-launch.ps1') -Workspace $Workspace -Audit $audit -Stock
}
function Assert-StockRequest($Job) {
  if ($Job.action -cne 'LaunchStock' -or $Job.mode -cne 'StockLegacy' -or
      $Job.arguments -isnot [string] -or $Job.arguments.Length -ne 0 -or
      $Job.executableSha256 -cne '7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c') { throw 'Stock launch requires the exact official binary with empty arguments.' }
  foreach ($field in @('restartReview','restartBinding','restartSessionRecord','restartSessionSha256')) {
    if ($Job.PSObject.Properties[$field]) { throw 'Stock launch cannot arm an OpenNav restart broker.' }
  }
}
function Assert-StockReviewPolicy($Job,$Launch,$Request,[datetime]$Now) {
  Assert-StockRequest $Request
  if ($Job.action -cne 'ReviewStock' -or $Job.reviewAction -cnotin @('Capture','Resize1280x800','ZoomOut','Close','InspectWelcome','FocusWelcome','AcknowledgeWelcome') -or
      $Launch.status -cne 'passed' -or $Launch.action -cne 'LaunchStock' -or $Launch.mode -cne 'StockLegacy' -or
      $Job.processId -le 0 -or $Launch.pid -ne $Job.processId -or $Job.executable -ine $Request.executable -or
      $Job.executableSha256 -cne $Request.executableSha256 -or $Job.workspace -ine $Request.workspace -or
      $Launch.executableSha256 -cne $Request.executableSha256 -or $Launch.targetSha256 -cne $Request.targetSha256 -or
      $Launch.sid -cnotmatch '^S-1-5-[0-9-]+$' -or $Launch.sessionId -le 0) { throw 'Successful exact stock launch required for this fixed review action.' }
  $at=[datetime]::Parse($Launch.utc).ToUniversalTime();$start=[datetime]::Parse($Launch.processStartedUtc).ToUniversalTime()
  if ($at -gt $Now -or ($Now-$at).TotalHours -gt 4 -or $start -lt $at -or $start -gt $Now) { throw 'Stock review launch evidence expired or inconsistent.' }
}
function Assert-StockProcess($Process,$Job,$Launch,[string]$Sid,[int]$Session) {
  if ($Process.Id -ne $Job.processId -or $Process.Path -ine $Job.executable -or $Process.SessionId -ne $Session -or
      $Launch.sessionId -ne $Session -or $Launch.sid -cne $Sid -or $Process.HasExited -or -not $Process.MainWindowHandle -or
      $Process.StartTime.ToUniversalTime().Ticks -ne ([datetime]::Parse($Launch.processStartedUtc).ToUniversalTime()).Ticks) { throw 'Exact stock PID, start time, user session and window required.' }
}
function Read-StockReview($Job) {
  $config=Get-StockTarget $Job.workspace
  foreach ($field in @('launchResultSha256','launchRequestSha256','reviewHelperSha256','nativeHelperSha256')) {
    if ($Job.$field -cnotmatch '^[a-f0-9]{64}$') { throw 'Exact stock review evidence hashes required.' }
  }
  if ((Get-Digest (Join-Path $PSScriptRoot 'StockReview.ps1')) -cne $Job.reviewHelperSha256 -or
      (Get-Digest (Join-Path $PSScriptRoot 'StockReviewNative.cs')) -cne $Job.nativeHelperSha256) { throw 'Stock review helper changed.' }
  $resultPath=Assert-LocalPath $Job.launchResult;$directory=[IO.Path]::GetDirectoryName($resultPath)
  if ([IO.Path]::GetDirectoryName($directory) -ine (Join-Path (Assert-LocalPath $Job.workspace) 'runs') -or
      [IO.Path]::GetFileName($directory) -cnotmatch '^\d{8}-\d{6}-launchstock-[a-f0-9]{8}$' -or
      [IO.Path]::GetFileName($resultPath) -cne 'result.json') { throw 'Expected owned stock launch evidence.' }
  $requestPath=Join-Path $directory 'request.json'
  if ((Get-Digest $resultPath) -cne $Job.launchResultSha256 -or (Get-Digest $requestPath) -cne $Job.launchRequestSha256) { throw 'Stock launch evidence changed.' }
  $launch=Read-Record $resultPath;$request=Read-Record $requestPath
  Assert-StockReviewPolicy $Job $launch $request ([datetime]::UtcNow)
  if ($request.resultPath -ine $resultPath -or $config.stockExecutable -ine $Job.executable -or
      (Get-Digest (Join-Path $Job.workspace 'boat-target.json')) -cne $request.targetSha256) { throw 'Stock target/audit changed after launch.' }
  $binding=$config.stockReadOnlyAudit.commissioning
  $prepared=Read-Record $binding.record
  if ((Get-Digest $binding.record) -cne $binding.recordSha256 -or $prepared.owner -cne $script:CommissioningOwner -or
      $prepared.status -cne 'prepared' -or $null -ne $prepared.context.installation) { throw 'Stock prepared transaction changed or covers an installed generation.' }
  $active=Read-Record (Join-Path $Job.workspace 'commissioning-active.json')
  if ($active.owner -cne $script:CommissioningOwner -or $active.record -ine $binding.record -or $active.recordSha256 -cne $binding.recordSha256) { throw 'Stock commissioning transaction is no longer active.' }
  $cold=[IO.Path]::GetDirectoryName($binding.record)
  $null=Get-PreparedCommissioningBaseline $prepared $cold $Job.workspace
  if (@(Get-ChildItem -LiteralPath $cold -Filter 'restore*.json' -Force).Count -or
      (Get-Digest (Join-Path $cold 'applied.json')) -cne $binding.appliedSha256) { throw 'Stock transaction restored, incomplete or changed.' }
  $roots=@((Join-Path ([Environment]::GetFolderPath('LocalApplicationData')) 'opencpn\plugins'),(Join-Path ([IO.Path]::GetDirectoryName($Job.executable)) 'plugins')) | Sort-Object -Unique
  if (((@($prepared.context.pluginRoots) | Sort-Object) -join '|') -ine ($roots -join '|')) { throw 'Stock loader-root set changed.' }
  $inventoryPath=Join-Path $cold 'inventory.json'
  if ((Get-Digest $inventoryPath) -cne $prepared.inventorySha256) { throw 'Stock plugin inventory changed.' }
  $inventory=Read-Record $inventoryPath
  Assert-CommissioningInventory $inventory $prepared.context
  Assert-CommissioningTrees $inventory.trees $prepared.quarantine -AllowMoved
  foreach ($move in @($prepared.quarantine)) {
    if (Test-Path -LiteralPath $move.path) { throw 'Quarantined plugin returned during stock review.' }
    if ((Get-Digest $move.backup) -cne $move.sha256) { throw 'Stock plugin recovery copy changed.' }
  }
  if ((Get-Digest (Join-Path $cold 'review-plan.json')) -cne $prepared.planSha256) { throw 'Stock source plan changed.' }
  $plan=Read-Record (Join-Path $cold 'review-plan.json')
  foreach ($time in @($config.stockReadOnlyAudit.reviewedUtc,$plan.reviewedUtc)) {
    $at=[datetime]::Parse($time).ToUniversalTime()
    if ($at -gt [datetime]::UtcNow -or ([datetime]::UtcNow-$at).TotalHours -gt 24) { throw 'Stock source/profile review expired.' }
  }
  foreach ($item in @($prepared.evidence)) { if ((Get-Digest $item.path) -cne $item.sha256) { throw 'Stock source evidence changed.' } }
  Assert-InputOnlyProfile (Read-ProfileForAudit (Join-Path $config.profileDirectory 'opencpn.ini'))
  Assert-CommissioningRestoreIni (Join-Path $cold 'input-only.ini') (Join-Path $config.profileDirectory 'opencpn.ini')
  return [pscustomobject]@{launch=$launch;request=$request}
}
function Initialize-StockReviewNative {
  if (-not ('OpenNavX.StockReviewNative' -as [type])) { Add-Type -Path (Join-Path $PSScriptRoot 'StockReviewNative.cs') }
}
function Assert-StockImageDirectory($Job,[string]$Sid) {
  $directory=Assert-LocalPath $Job.evidenceDirectory
  if ([IO.Path]::GetDirectoryName($directory) -ine (Join-Path (Assert-LocalPath $Job.workspace) 'runs') -or
      [IO.Path]::GetFileName($directory) -cnotmatch '^\d{8}-\d{6}-stock-review-[a-f0-9]{8}$' -or @(Get-ChildItem -LiteralPath $directory -Force).Count) { throw 'New private stock review directory required.' }
  $acl=Get-Acl -LiteralPath $directory
  if (-not $acl.AreAccessRulesProtected) { throw 'Stock images must not inherit public access.' }
  foreach ($rule in $acl.GetAccessRules($true,$true,[Security.Principal.SecurityIdentifier])) {
    if ($rule.IdentityReference.Value -cnotin @($Sid,'S-1-5-18','S-1-5-32-544')) { throw 'Unexpected image-directory principal.' }
  }
  return $directory
}
function Invoke-StockReview($Job) {
  $review=Read-StockReview $Job
  $process=Get-Process -Id $Job.processId -ErrorAction Stop
  try {
    $sid=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value;$session=[Diagnostics.Process]::GetCurrentProcess().SessionId
    Assert-StockProcess $process $Job $review.launch $sid $session
    Initialize-StockReviewNative
    $oldDpi=[OpenNavX.StockReviewNative]::SetThreadDpiAwarenessContext([IntPtr](-4))
    if ($oldDpi -eq [IntPtr]::Zero) { throw 'Physical DPI context unavailable.' }
    try {
      if ($Job.reviewAction -cin @('InspectWelcome','FocusWelcome','AcknowledgeWelcome')) { return Invoke-StockWelcomeReview $Job $review $process $sid $session }
      $frame=$process.MainWindowHandle
      # Fixed resize validates/focuses its exact frame internally and can recover
      # a partially offscreen restored rectangle. Other operations stay strict.
      if ($Job.reviewAction -cne 'Resize1280x800') { [OpenNavX.StockReviewNative]::Foreground($frame,$process.Id) }
      $null=Read-StockReview $Job;$process.Refresh();Assert-StockProcess $process $Job $review.launch $sid $session
      if ($process.MainWindowHandle -ne $frame) { throw 'Stock main window changed during verification.' }
      $result=@{status='passed';action='ReviewStock';reviewAction=$Job.reviewAction;utc=[datetime]::UtcNow.ToString('o');processId=$process.Id;
        executableSha256=$Job.executableSha256;launchResultSha256=$Job.launchResultSha256;mode='StockLegacy';actuatorCommandsIssuedByTool=$false;physicalBusSilenceNotClaimed=$true}
      if ($Job.reviewAction -ceq 'Close') {
        $result.close=Invoke-ReviewedNormalClose $process $Job.processId ([datetime]::Parse($review.launch.processStartedUtc).ToUniversalTime().Ticks)
        $result.exitCode=$result.close.exitCode
      } else {
        $directory=Assert-StockImageDirectory $Job $sid
        if ($Job.reviewAction -ceq 'Resize1280x800') {
          $result.resize=[OpenNavX.StockReviewNative]::Resize1280x800($frame,$process.Id)
          # Preserve numeric geometry even if a later toast/overlay correctly
          # refuses the separate strict screenshot. This is not capture proof.
          Write-Record (Join-Path $directory 'resize.json') @{owner='OpenNavX.StockResize.1';processId=$process.Id;utc=[datetime]::UtcNow.ToString('o');
            launchResultSha256=$Job.launchResultSha256;nativeHelperSha256=$Job.nativeHelperSha256;geometry=$result.resize;captureVerified=$false}
        }
        if($Job.reviewAction -ceq 'ZoomOut') {
          Write-Record (Join-Path $directory 'zoom-intent.json') @{owner='OpenNavX.StockChartReview.1';action='ZoomOut';commandId=2001;processId=$process.Id;utc=[datetime]::UtcNow.ToString('o');nativeHelperSha256=$Job.nativeHelperSha256;noRetry=$true}
          $result.zoom=[OpenNavX.StockReviewNative]::ZoomOut($frame,$process.Id)
          $null=Read-StockReview $Job;$process.Refresh();Assert-StockProcess $process $Job $review.launch $sid $session
        }
        $info=[OpenNavX.StockReviewNative]::AssertFrame($frame,$process.Id)
        Add-Type -AssemblyName System.Drawing
        $bitmap=New-Object Drawing.Bitmap($info.Bounds.Width,$info.Bounds.Height);$graphics=[Drawing.Graphics]::FromImage($bitmap)
        try {
          [OpenNavX.StockReviewNative]::AssertCapture($frame,$process.Id,$info)
          $graphics.CopyFromScreen($info.Bounds.Left,$info.Bounds.Top,0,0,$bitmap.Size)
          [OpenNavX.StockReviewNative]::AssertCapture($frame,$process.Id,$info)
          $image=Join-Path $directory 'stock.png';$bitmap.Save($image,[Drawing.Imaging.ImageFormat]::Png)
        } finally { $graphics.Dispose();$bitmap.Dispose() }
        $result.image=$image;$result.imageSha256=Get-Digest $image;$result.nativeWindow=$info
      }
      $result.review='Official stock process only; chart pixels, normal close and post-close profile changes require human review. Stock Safe Mode is not exercised.'
      return $result
    } finally { $null=[OpenNavX.StockReviewNative]::SetThreadDpiAwarenessContext($oldDpi) }
  } finally { $process.Dispose() }
}
function Invoke-StockLaunch($Job) {
  Assert-StockRequest $Job
  $config=Get-StockTarget $Job.workspace
  if ($config.stockExecutable -ine $Job.executable -or
      (Get-Digest (Join-Path $Job.workspace 'boat-target.json')) -cne $Job.targetSha256) { throw 'Stock target/audit changed between dispatch and execution.' }
  $environment=Assert-StockLaunchAudit $config $Job.workspace
  if ((Assert-LocalPath $environment.workingDirectory) -ine [IO.Path]::GetDirectoryName($Job.executable)) { throw 'Stock working directory differs from reviewed context.' }
  if (@(Get-Process -Name opencpn -ErrorAction SilentlyContinue).Count) { throw 'Close existing OpenCPN normally before stock launch.' }
  foreach ($key in @('OPENNAV_COMMISSIONING_RESTART_SESSION','OPENNAV_COMMISSIONING_RESTART_RECORD_SHA256')) {
    if ([Environment]::GetEnvironmentVariables().Contains($key)) { throw 'Stock launch cannot inherit an OpenNav restart session.' }
  }
  $null=Get-StockTarget $Job.workspace
  if ((Get-Digest (Join-Path $Job.workspace 'boat-target.json')) -cne $Job.targetSha256) { throw 'Stock target changed during full commissioning verification.' }
  $start=New-Object Diagnostics.ProcessStartInfo
  $start.FileName=$Job.executable;$start.Arguments='';$start.WorkingDirectory=$environment.workingDirectory;$start.UseShellExecute=$false
  $start.EnvironmentVariables['PATH']=$environment.path
  $at=[datetime]::UtcNow.ToString('o')
  $process=[Diagnostics.Process]::Start($start)
  try {
    $started=$process.StartTime.ToUniversalTime().ToString('o')
    $deadline=[datetime]::UtcNow.AddSeconds(45)
    do { Start-Sleep -Milliseconds 250;$process.Refresh() } while (-not $process.HasExited -and -not $process.MainWindowHandle -and [datetime]::UtcNow -lt $deadline)
    if ($process.HasExited -or -not $process.MainWindowHandle) { throw 'Official stock did not expose its normal window; no force termination.' }
    if ($process.Path -ine $Job.executable -or $process.SessionId -ne [Diagnostics.Process]::GetCurrentProcess().SessionId) { throw 'Stock child identity/session mismatch.' }
    return @{status='passed';action='LaunchStock';mode='StockLegacy';arguments='';utc=$at;pid=$process.Id;processStartedUtc=$started;
      sid=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value;sessionId=$process.SessionId;executableSha256=$Job.executableSha256;targetSha256=$Job.targetSha256;
      commissioning=$config.stockReadOnlyAudit.commissioning;profile='Shared normal OpenCPN profile';actuatorCommandsIssuedByTool=$false;physicalBusSilenceNotClaimed=$true;restartBrokerArmed=$false}
  } finally { $process.Dispose() }
}

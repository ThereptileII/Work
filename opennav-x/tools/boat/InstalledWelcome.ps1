# Separate installed-product identity for the shared, source-specific upstream
# warning. This never accepts a stock, portable or restarted-child launch receipt.
. (Join-Path $PSScriptRoot 'Commissioning.ps1')
. (Join-Path $PSScriptRoot 'StockWelcome.ps1')
$script:InstalledWelcomeFiles=@('InstalledWelcome.ps1','StockWelcome.ps1','StockWelcomeNative.cs','RestartWindowNative.cs')
function Assert-InstalledWelcomePolicy($Job,$Launch,$Request,$Installed,$Build,[datetime]$Now) {
  if ($Job.action -cne 'ReviewInstalledWelcome' -or $Job.reviewAction -cnotin @('InspectWelcome','FocusWelcome','AcknowledgeWelcome') -or
      $Job.processId -le 0 -or $Job.buildCommit -cnotmatch '^[a-f0-9]{40}$' -or $Job.generation -cnotmatch '^[a-f0-9]{32}$' -or
      $Job.executableSha256 -cnotmatch '^[a-f0-9]{64}$') { throw 'Exact installed warning review required.' }
  if ($Launch.status -cne 'passed' -or $Launch.action -cne 'Launch' -or $Request.action -cne 'Launch' -or
      $Launch.mode -cnotin @('--xnav','--legacy','--safe-mode') -or $Request.mode -cne $Launch.mode -or
      $Launch.pid -ne $Job.processId -or $Request.executable -ine $Job.executable -or $Request.executableSha256 -cne $Job.executableSha256 -or
      $Request.workspace -ine $Job.workspace -or $Launch.executableSha256 -cne $Job.executableSha256 -or
      $Launch.generation -cne $Job.generation -or $Launch.buildCommit -cne $Job.buildCommit -or
      $Launch.targetSha256 -cnotmatch '^[a-f0-9]{64}$' -or $Launch.sid -cnotmatch '^S-1-5-[0-9-]+$' -or $Launch.sessionId -le 0) { throw 'Recent exact normal installed launch receipt required; older or other launch kinds cannot substitute.' }
  if ($Installed.state.current -cne $Job.generation -or $Installed.ownership.commit -cne $Job.buildCommit -or
      $Installed.ownership.version -cne '0.4.0-beta2' -or $Installed.executable -ine $Job.executable -or
      $Build.test_fixtures -isnot [bool] -or $Build.test_fixtures -ne $false -or $Build.build_purpose -cne 'INSTALLED PRODUCT' -or
      $Build.version -cne '0.4.0-beta2' -or $Build.commit -cne $Job.buildCommit -or $Build.executable_sha256 -cne $Job.executableSha256) { throw 'Exact owned fixture-free Beta 2 product build required.' }
  $at=[datetime]::Parse($Launch.utc).ToUniversalTime();$started=[datetime]::Parse($Launch.processStartedUtc).ToUniversalTime()
  if ($at -gt $Now -or ($Now-$at).TotalHours -gt 4 -or $started -lt $at -or $started -gt $Now) { throw 'Installed startup receipt expired or has inconsistent process creation.' }
}
function Assert-InstalledWelcomeProcess($Process,$Job,$Launch,[string]$Sid,[int]$Session) {
  if ($Process.Id -ne $Job.processId -or $Process.Path -ine $Job.executable -or $Process.HasExited -or -not $Process.MainWindowHandle -or
      $Process.SessionId -ne $Session -or $Launch.sessionId -ne $Session -or $Launch.sid -cne $Sid -or
      $Process.StartTime.ToUniversalTime().Ticks -ne ([datetime]::Parse($Launch.processStartedUtc).ToUniversalTime()).Ticks) { throw 'Exact installed PID, start tick and user session required.' }
}
function Get-InstalledWelcomeEnvironment($Config,$Installed) {
  $local=Assert-LocalPath ([Environment]::GetFolderPath('LocalApplicationData'))
  if (-not $env:LOCALAPPDATA -or (Assert-LocalPath $env:LOCALAPPDATA) -ine $local) { throw 'Interactive plugin account is ambiguous.' }
  return [pscustomobject]@{local=$local;profile=(Assert-LocalPath (Join-Path ([Environment]::GetFolderPath('CommonApplicationData')) 'opencpn'));
    roots=@((Join-Path $local 'opencpn\plugins'),(Join-Path ([IO.Path]::GetDirectoryName($Config.stockExecutable)) 'plugins'),(Join-Path ([IO.Path]::GetDirectoryName($Installed.executable)) 'plugins'))}
}
function Get-InstalledCommissioningBasemap($Installed) {
  # InstalledResources.cpp reads this managed marker and only supplies missing
  # defaults from the original supported application. Never accept a generation
  # resource path, another installation or a caller-provided replacement path.
  $stock=Assert-LocalPath $Installed.state.stock.path
  if ((Get-Digest $stock) -cne '7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c') { throw 'Installed resource default requires the exact supported stock executable.' }
  $marker=Assert-LocalPath (Join-Path $Installed.generation 'app/OPENNAV_INSTALLED_STOCK')
  $owned=@($Installed.ownership.managedFiles | Where-Object {$_.path -ceq 'app/OPENNAV_INSTALLED_STOCK'})
  if ($owned.Count -ne 1 -or $owned[0].sha256 -cnotmatch '^[a-f0-9]{64}$' -or
      (Get-Item -LiteralPath $marker).Length -gt 4096 -or (Get-Digest $marker) -cne $owned[0].sha256) { throw 'Exact owned stock resource locator required.' }
  $encoding=New-Object Text.UTF8Encoding($false,$true)
  if ($encoding.GetString([IO.File]::ReadAllBytes($marker)) -cne $stock) { throw 'Stock resource locator differs from the installation binding.' }
  $parent=[IO.Path]::GetDirectoryName($stock)
  foreach ($name in @('tcdata/harmonics-dwf-20210110-free.tcd','tcdata/HARMONICS_NO_US.IDX','tcdata/HARMONICS_NO_US','gshhs/poly-c-1.dat','basemap_shp/basemap_low.shp','sounds/2bells.wav')) {
    $path=Assert-LocalPath (Join-Path $parent $name)
    if (-not [IO.File]::Exists($path) -or (Get-Item -LiteralPath $path).Length -le 0) { throw 'Pinned installed resource selector prerequisites are missing.' }
  }
  # wxFileConfig escapes each Windows backslash on disk. Preserve exact bytes;
  # no relaxed slash, case, traversal, quote or alternate-path comparison.
  return (Join-Path $parent 'basemap_shp').Replace('\','\\')
}
function Assert-InstalledWelcomeRuntime($Config,$Installed,$Launch,[string]$Workspace) {
  # The accepted cold launch already checked the full source-plan semantics and
  # one-byte transform. Recheck all immutable proof bytes and live trees now;
  # do not invoke or weaken the cold verifier's closed-process precondition.
  $audit=$Config.readOnlyAudit;$binding=$audit.commissioning
  foreach($field in @('record','recordSha256','appliedSha256')) { if ($binding.$field -cne $Launch.commissioning.$field) { throw 'Commissioning binding changed since launch.' } }
  foreach($field in @('connectionsOutputDisabled','pluginOutputsReviewed','noActiveRouteOutput')) { Assert-TrueBoolean $audit.$field $field }
  if ($audit.buildCommit -cne $Installed.ownership.commit) { throw 'Source/profile review belongs to another product.' }
  $record=Assert-LocalPath $binding.record;$cold=[IO.Path]::GetDirectoryName($record)
  if ([IO.Path]::GetFileName($record) -cne 'prepared.json' -or [IO.Path]::GetDirectoryName($cold) -ine (Join-Path $Workspace 'runs') -or
      (Get-Digest $record) -cne $binding.recordSha256) { throw 'Expected immutable installed preparation.' }
  $prepared=Read-Record $record;$expected=$prepared.context.installation
  if ($prepared.owner -cne $script:CommissioningOwner -or $prepared.status -cne 'prepared' -or -not $expected -or
      $expected.root -ine $Installed.root -or $expected.generation -ine $Installed.generation -or $expected.executable -ine $Installed.executable -or
      $expected.commit -cne $Installed.ownership.commit -or $expected.executableSha256 -cne (Get-Digest $Installed.executable) -or
      $expected.stateSha256 -cne (Get-Digest (Join-Path $Installed.root 'state.json')) -or
      $expected.ownershipSha256 -cne (Get-Digest (Join-Path $Installed.generation 'ownership.json'))) { throw 'Installed generation differs from prepared commissioning.' }
  $environment=Get-InstalledWelcomeEnvironment $Config $Installed;$local=$environment.local;$profile=$environment.profile
  if ((Assert-LocalPath $Config.profileDirectory) -ine $profile -or
      $prepared.context.profile -ine $profile -or $prepared.context.localAppData -ine $local -or
      $prepared.context.sid -cne $Launch.sid -or $prepared.context.session -ne $Launch.sessionId -or
      $Config.stockExecutable -ine $Installed.state.stock.path) { throw 'Real account/profile/stock installation differs from accepted launch.' }
  $roots=@($environment.roots) | Sort-Object -Unique
  if (((@($prepared.context.pluginRoots) | Sort-Object) -join '|') -ine ($roots -join '|')) { throw 'Installed loader-root set changed.' }
  $active=Read-Record (Join-Path $Workspace 'commissioning-active.json')
  if ($active.owner -cne $script:CommissioningOwner -or $active.record -ine $record -or $active.recordSha256 -cne $binding.recordSha256 -or
      @(Get-ChildItem -LiteralPath $cold -Filter 'restore*.json' -Force).Count -or
      (Get-Digest (Join-Path $cold 'applied.json')) -cne $binding.appliedSha256) { throw 'Installed commissioning is no longer the unchanged active/applied transaction.' }
  $null=Get-PreparedCommissioningBaseline $prepared $cold $Workspace
  if ((Get-Digest (Join-Path $cold 'baseline.ini')) -cne $prepared.baselineSha256 -or
      (Get-Digest (Join-Path $cold 'input-only.ini')) -cne $prepared.inputSha256 -or
      (Get-Digest (Join-Path $cold 'review-plan.json')) -cne $prepared.planSha256 -or
      (Get-Digest (Join-Path $cold 'inventory.json')) -cne $prepared.inventorySha256) { throw 'Prepared profile or full source inventory changed.' }
  $inventory=Read-Record (Join-Path $cold 'inventory.json');Assert-CommissioningInventory $inventory $prepared.context
  Assert-CommissioningTrees $inventory.trees $prepared.quarantine -AllowMoved
  foreach($move in @($prepared.quarantine)) {
    if ((Test-Path -LiteralPath $move.path) -or (Get-Digest $move.destination) -cne $move.sha256 -or (Get-Digest $move.backup) -cne $move.sha256) { throw 'Quarantined plugin returned or its preservation changed.' }
  }
  foreach($entry in @($prepared.evidence)) { if ((Get-Digest $entry.path) -cne $entry.sha256) { throw 'Source-review evidence changed.' } }
  $plan=Read-Record (Join-Path $cold 'review-plan.json')
  foreach($time in @($audit.reviewedUtc,$plan.reviewedUtc)) {
    $at=[datetime]::Parse($time).ToUniversalTime()
    if ($at -gt [datetime]::UtcNow -or ([datetime]::UtcNow-$at).TotalHours -gt 24) { throw 'Read-only source/profile review expired.' }
  }
  Assert-InputOnlyProfile (Read-ProfileForAudit (Join-Path $profile 'opencpn.ini'))
  $before=Read-ProfileForAudit (Join-Path $cold 'input-only.ini')
  $after=Read-ProfileForAudit (Join-Path $profile 'opencpn.ini')
  $default=''
  if ($before['Directories/BaseShapefileDir'] -cne $after['Directories/BaseShapefileDir']) {
    $default=Get-InstalledCommissioningBasemap $Installed
  }
  Assert-CommissioningProtectedValues $before $after $default
}
function Read-InstalledWelcome($Job) {
  if (@($Job.helperFiles).Count -ne $script:InstalledWelcomeFiles.Count) { throw 'Exact warning helper inventory required.' }
  foreach($name in $script:InstalledWelcomeFiles) {
    $entry=@($Job.helperFiles | Where-Object {$_.name -ceq $name})
    if ($entry.Count -ne 1 -or $entry[0].sha256 -cnotmatch '^[a-f0-9]{64}$' -or (Get-Digest (Join-Path $PSScriptRoot $name)) -cne $entry[0].sha256) { throw 'Installed warning helper changed.' }
  }
  $path=Assert-LocalPath $Job.launchResult;$directory=[IO.Path]::GetDirectoryName($path)
  if ([IO.Path]::GetFileName($path) -cne 'result.json' -or [IO.Path]::GetDirectoryName($directory) -ine (Join-Path (Assert-LocalPath $Job.workspace) 'runs') -or
      [IO.Path]::GetFileName($directory) -cnotmatch '^\d{8}-\d{6}-launch-[a-f0-9]{8}$' -or
      $Job.launchResultSha256 -cnotmatch '^[a-f0-9]{64}$' -or $Job.launchRequestSha256 -cnotmatch '^[a-f0-9]{64}$' -or
      (Get-Digest $path) -cne $Job.launchResultSha256 -or (Get-Digest (Join-Path $directory 'request.json')) -cne $Job.launchRequestSha256) { throw 'Expected owned installed cold-launch request/result.' }
  $launch=Read-Record $path;$request=Read-Record (Join-Path $directory 'request.json')
  $installed=Get-Installed;$buildPath=Join-Path $installed.generation 'docs\PRODUCT_BUILD.json'
  $entry=@($installed.ownership.managedFiles | Where-Object {$_.path -ceq 'docs/PRODUCT_BUILD.json'})
  if ($entry.Count -ne 1 -or (Get-Digest $buildPath) -cne $entry[0].sha256 -or $request.resultPath -ine $path) { throw 'Owned fixture-free product report or launch path changed.' }
  Assert-InstalledWelcomePolicy $Job $launch $request $installed (Read-Record $buildPath) ([datetime]::UtcNow)
  if ((Get-Digest $installed.executable) -cne $Job.executableSha256 -or (Get-Digest (Join-Path $Job.workspace 'boat-target.json')) -cne $launch.targetSha256) { throw 'Executable or read-only target changed since cold launch.' }
  Assert-InstalledWelcomeRuntime (Get-Target $Job.workspace) $installed $launch $Job.workspace
  return [pscustomobject]@{launch=$launch;installed=$installed}
}
function Assert-InstalledWelcomeInspection($Inspection,$Job,$Launch,[datetime]$Now) {
  if ($Inspection.status -cne 'passed' -or $Inspection.action -cne 'ReviewInstalledWelcome' -or $Inspection.reviewAction -cne 'InspectWelcome' -or
      $Inspection.mode -cne $Launch.mode -or $Inspection.processId -ne $Job.processId -or $Inspection.buildCommit -cne $Job.buildCommit -or
      $Inspection.generation -cne $Job.generation -or $Inspection.executableSha256 -cne $Job.executableSha256 -or
      $Inspection.launchResultSha256 -cne $Job.launchResultSha256 -or $Inspection.launchRequestSha256 -cne $Job.launchRequestSha256 -or
      ($Inspection.helperFiles | ConvertTo-Json -Depth 4 -Compress) -cne ($Job.helperFiles | ConvertTo-Json -Depth 4 -Compress) -or
      $Inspection.imageSha256 -cnotmatch '^[a-f0-9]{64}$') { throw 'Review the warning from this exact installed launch first.' }
  $at=[datetime]::Parse($Inspection.utc).ToUniversalTime()
  if ($at -gt $Now -or ($Now-$at).TotalMinutes -gt 30) { throw 'Installed warning inspection expired.' }
  Assert-StockWelcomeWindow $Inspection.nativeWindow $Job.processId
}
function Invoke-InstalledWelcomeFocus($Job,$Proof,$Process,[string]$Sid,[int]$Session,[string]$Directory) {
  if ($Job.reviewAction -cne 'FocusWelcome') { throw 'This fixed operation only focuses the installed startup warning.' }
  # Same complete generation/profile/source/quarantine proof as inspection,
  # rechecked around the separately qualified fixed native caption primitive.
  $null=Read-InstalledWelcome $Job;$Process.Refresh();Assert-InstalledWelcomeProcess $Process $Job $Proof.launch $Sid $Session
  $ticks=([datetime]::Parse($Proof.launch.processStartedUtc).ToUniversalTime()).Ticks
  $info=Invoke-StockWelcomeFocus $Process.Id $ticks (Join-Path $Directory 'focus-intent.json')
  $null=Read-InstalledWelcome $Job;$Process.Refresh();Assert-InstalledWelcomeProcess $Process $Job $Proof.launch $Sid $Session
  $image=Join-Path $Directory 'focused-warning.png'
  $hash=Save-StockWelcomeCapture $Process.Id $info $image
  return @{image=$image;imageSha256=$hash;nativeWindow=$info;focusVerified=$true;acknowledgementSent=$false}
}
function Invoke-InstalledWelcome($Job) {
  $proof=Read-InstalledWelcome $Job
  $sid=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value;$session=[Diagnostics.Process]::GetCurrentProcess().SessionId
  $directory=Assert-LocalPath $Job.evidenceDirectory
  if ([IO.Path]::GetDirectoryName($directory) -ine (Join-Path $Job.workspace 'runs') -or
      [IO.Path]::GetFileName($directory) -cnotmatch '^\d{8}-\d{6}-installed-welcome-[a-f0-9]{8}$' -or @(Get-ChildItem -LiteralPath $directory -Force).Count) { throw 'New private installed warning directory required.' }
  $acl=Get-Acl -LiteralPath $directory
  if (-not $acl.AreAccessRulesProtected) { throw 'Warning captures must not inherit public access.' }
  foreach($rule in $acl.GetAccessRules($true,$true,[Security.Principal.SecurityIdentifier])) { if ($rule.IdentityReference.Value -cnotin @($sid,'S-1-5-18','S-1-5-32-544')) { throw 'Unexpected warning-directory principal.' } }
  $process=Get-Process -Id $Job.processId -ErrorAction Stop
  try {
    Assert-InstalledWelcomeProcess $process $Job $proof.launch $sid $session
    Initialize-StockWelcomeNative
    if (-not ('OpenNavX.RestartWindowNative' -as [type])) { Add-Type -Path (Join-Path $PSScriptRoot 'RestartWindowNative.cs') }
    $oldDpi=[OpenNavX.RestartWindowNative]::SetThreadDpiAwarenessContext([IntPtr](-4));if ($oldDpi -eq [IntPtr]::Zero) { throw 'Physical DPI context unavailable.' }
    try {
      $result=@{status='passed';action='ReviewInstalledWelcome';reviewAction=$Job.reviewAction;utc=[datetime]::UtcNow.ToString('o');processId=$process.Id;
        mode=$proof.launch.mode;buildCommit=$Job.buildCommit;generation=$Job.generation;executableSha256=$Job.executableSha256;
        launchResultSha256=$Job.launchResultSha256;launchRequestSha256=$Job.launchRequestSha256;helperFiles=$Job.helperFiles;
        bodyTextAccessible=$false;actuatorCommandsIssuedByTool=$false;physicalBusSilenceNotClaimed=$true;
        review='Pinned upstream navigation caution before OpenNav Attach; HTML body requires human review of captured pixels. No other dialog or runtime command is authorized.'}
      if ($Job.reviewAction -ceq 'FocusWelcome') {
        $focus=Invoke-InstalledWelcomeFocus $Job $proof $process $sid $session $directory
        foreach($key in $focus.Keys) { $result[$key]=$focus[$key] }
        $result.review='Fixed installed startup-warning caption focused. Warning remains present; this result cannot authorize acknowledgement. Inspect separately.'
        return $result
      }
      if ($Job.reviewAction -ceq 'InspectWelcome') {
        $info=[OpenNavX.StockWelcomeNative]::Inspect($process.Id)
        $null=Read-InstalledWelcome $Job;$process.Refresh();Assert-InstalledWelcomeProcess $process $Job $proof.launch $sid $session
        $image=Join-Path $directory 'welcome.png';$result.imageSha256=Save-StockWelcomeCapture $process.Id $info $image
        $result.image=$image;$result.nativeWindow=$info;$result.acknowledgementSent=$false;return $result
      }
      $inspectionPath=Assert-LocalPath $Job.welcomeInspection;$inspectionDir=[IO.Path]::GetDirectoryName($inspectionPath)
      if ($Job.welcomeInspectionSha256 -cnotmatch '^[a-f0-9]{64}$' -or [IO.Path]::GetFileName($inspectionPath) -cne 'review.json' -or
          [IO.Path]::GetDirectoryName($inspectionDir) -ine (Join-Path $Job.workspace 'runs') -or $inspectionDir -ieq $directory -or
          [IO.Path]::GetFileName($inspectionDir) -cnotmatch '^\d{8}-\d{6}-installed-welcome-[a-f0-9]{8}$' -or
          (Get-Digest $inspectionPath) -cne $Job.welcomeInspectionSha256) { throw 'Exact separately reviewed installed warning record required.' }
      $inspection=Read-Record $inspectionPath;Assert-InstalledWelcomeInspection $inspection $Job $proof.launch ([datetime]::UtcNow)
      if ($inspection.image -ine (Join-Path $inspectionDir 'welcome.png') -or (Get-Digest $inspection.image) -cne $inspection.imageSha256) { throw 'Inspected warning pixels changed.' }
      $info=$inspection.nativeWindow;$intent=Join-Path $inspectionDir 'agree-intent.json'
      if (Test-Path -LiteralPath $intent) { throw 'Warning acknowledgement already consumed; no retry.' }
      $null=Read-InstalledWelcome $Job;$process.Refresh();Assert-InstalledWelcomeProcess $process $Job $proof.launch $sid $session
      $before=Join-Path $directory 'welcome-before-agree.png';Invoke-StockWelcomeAgreement $process.Id $info $inspection.imageSha256 $before $intent
      $result.acknowledgementSent=$true;$result.inspectionSha256=$Job.welcomeInspectionSha256;$result.before=$before;$result.beforeSha256=Get-Digest $before
      $result.nativeWindow=$info;$result.modalDismissalConfirmed=$false
      $deadline=[datetime]::UtcNow.AddSeconds(20)
      do {
        Start-Sleep -Milliseconds 250;$process.Refresh();if ($process.HasExited) { break }
        try {
          Assert-InstalledWelcomeProcess $process $Job $proof.launch $sid $session;$frame=$process.MainWindowHandle
          [OpenNavX.RestartWindowNative]::Foreground($frame,$process.Id,$proof.launch.mode)
          $window=[OpenNavX.RestartWindowNative]::AssertFrame($frame,$process.Id,$proof.launch.mode)
          $result.modalDismissalConfirmed=$true
          $bitmap=New-Object Drawing.Bitmap($window.Bounds.Width,$window.Bounds.Height);$graphics=[Drawing.Graphics]::FromImage($bitmap)
          try {
            [OpenNavX.RestartWindowNative]::AssertCapture($frame,$process.Id,$window)
            $graphics.CopyFromScreen($window.Bounds.Left,$window.Bounds.Top,0,0,$bitmap.Size)
            [OpenNavX.RestartWindowNative]::AssertCapture($frame,$process.Id,$window)
            $after=Join-Path $directory 'after-agree.png';$bitmap.Save($after,[Drawing.Imaging.ImageFormat]::Png)
          } finally { $graphics.Dispose();$bitmap.Dispose() }
          $result.after=$after;$result.afterSha256=Get-Digest $after;$result.afterNativeWindow=$window;break
        } catch { $result.attention=$_.Exception.Message }
      } while ([datetime]::UtcNow -lt $deadline)
      if (-not $result.modalDismissalConfirmed -or -not $result.ContainsKey('after')) { $result.status='attention' }
      return $result
    } finally { $null=[OpenNavX.RestartWindowNative]::SetThreadDpiAwarenessContext($oldDpi) }
  } finally { $process.Dispose() }
}

# Pure policy, native interop compilation and isolated commissioning trees.
# No real application, profile, registry, desktop or boat access.
[CmdletBinding()]
param([switch]$PortableContracts,[switch]$IsolatedLocal)
. (Join-Path $PSScriptRoot 'StockReview.ps1')
$checks=New-Object 'Collections.Generic.List[string]'
function Pass([string]$Name,[scriptblock]$Body) { & $Body;$checks.Add($Name) }
function Refuse([string]$Name,[scriptblock]$Body) { $bad=$false;try {$null=& $Body} catch {$bad=$true};if (-not $bad) {throw ('Accepted unsafe stock request: '+$Name)};$checks.Add($Name) }
function CopyValue($Value) { return ($Value | ConvertTo-Json -Depth 8 | ConvertFrom-Json) }
$hash='7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c'
$audit=[pscustomobject]@{launchKind='StockLegacy';executableSha256=$hash;upstreamCommit='37fd0cddb7334fe489e9f18aa163977a9c5c84f7'}
Pass 'Exact official stock without Installed state uses distinct provenance' {Assert-StockAuditIdentity $null $hash $audit}
foreach($installed in @(@{},[pscustomobject]@{executable='C:\app\opencpn.exe'})) {Refuse 'Any installed state prevents stock qualification' {Assert-StockAuditIdentity $installed $hash $audit}}
Refuse 'A version label cannot extend stock hash allowlist' {Assert-StockAuditIdentity $null ('a'*64) $audit}
foreach($field in @('launchKind','executableSha256','upstreamCommit')) {Refuse "Changed stock provenance $field" {$v=CopyValue $audit;$v.$field='wrong';Assert-StockAuditIdentity $null $hash $v}}
$now=[datetime]::UtcNow
$request=[pscustomobject]@{action='LaunchStock';mode='StockLegacy';arguments='';executable='C:\Program Files (x86)\OpenCPN\opencpn.exe';executableSha256=$hash;workspace='C:\XNav';targetSha256=('b'*64)}
$launch=[pscustomobject]@{status='passed';action='LaunchStock';mode='StockLegacy';pid=42;utc=$now.AddMinutes(-1).ToString('o');processStartedUtc=$now.AddSeconds(-58).ToString('o');executableSha256=$hash;targetSha256=('b'*64);sid='S-1-5-21-1';sessionId=1}
$job=[pscustomobject]@{action='ReviewStock';reviewAction='Capture';processId=42;executable=$request.executable;executableSha256=$hash;workspace=$request.workspace}
foreach($action in @('Capture','Resize1280x800','Close','InspectWelcome','FocusWelcome','AcknowledgeWelcome')) {Pass "One fixed stock action: $action" {$v=CopyValue $job;$v.reviewAction=$action;Assert-StockReviewPolicy $v $launch $request $now}}
foreach($launchArguments in @('--legacy','--safe-mode','--safe_mode','--xnav','--portable',' ', $null,@())) {Refuse 'Any nonempty or nonstring argument refused' {$v=CopyValue $request;$v.arguments=$launchArguments;Assert-StockRequest $v}}
foreach($field in @('restartReview','restartBinding','restartSessionRecord','restartSessionSha256')) {Refuse 'Stock cannot arm an OpenNav restart broker' {$v=CopyValue $request;$v | Add-Member -NotePropertyName $field -NotePropertyValue @{};Assert-StockRequest $v}}
foreach($action in @('AUTO','STBY','TRACK','WIND','ZoomIn','Center','Menu','Key','Click','Launch','LaunchPortableReview','Restart','Capture;Close','capture','')) {Refuse "Not a stock display action: $action" {$v=CopyValue $job;$v.reviewAction=$action;Assert-StockReviewPolicy $v $launch $request $now}}
foreach($field in @('action','mode','executable','executableSha256','workspace','targetSha256')) {Refuse "Mismatched request $field" {$v=CopyValue $request;$v.$field='wrong';Assert-StockReviewPolicy $job $launch $v $now}}
foreach($field in @('status','action','mode','pid','executableSha256','targetSha256','sid','sessionId')) {Refuse "Mismatched launch $field" {$v=CopyValue $launch;$v.$field=$(if($field -cin @('pid','sessionId')){-1}else{'wrong'});Assert-StockReviewPolicy $job $v $request $now}}
foreach($at in @($now.AddHours(-5),$now.AddMinutes(1))) {Refuse 'Expired/future launch rejected' {$v=CopyValue $launch;$v.utc=$at.ToString('o');Assert-StockReviewPolicy $job $v $request $now}}
$process=[pscustomobject]@{Id=42;Path=$request.executable;SessionId=1;MainWindowHandle=123;HasExited=$false;StartTime=[datetime]::Parse($launch.processStartedUtc)}
Pass 'Exact launch PID/start tick/session match without touching a process' {Assert-StockProcess $process $job $launch 'S-1-5-21-1' 1}
foreach($field in @('Id','Path','SessionId','MainWindowHandle','HasExited','StartTime')) {Refuse "Changed/reused process $field" {$v=CopyValue $process;$v.$field=switch($field){'Id'{43};'Path'{'C:\Other\opencpn.exe'};'SessionId'{2};'MainWindowHandle'{0};'HasExited'{$true};'StartTime'{$process.StartTime.AddTicks(1)}};Assert-StockProcess $v $job $launch 'S-1-5-21-1' 1}}
Refuse 'Different user SID refused' {Assert-StockProcess $process $job $launch 'S-1-5-21-2' 1}
foreach($fileName in @('StockReview.ps1','run-stock.ps1','review-stock.ps1','verify-commissioning-launch.ps1','InteractiveJob.ps1')) {Pass "Parses $fileName" {$tokens=$null;$errors=$null;$null=[Management.Automation.Language.Parser]::ParseFile((Join-Path $PSScriptRoot $fileName),[ref]$tokens,[ref]$errors);if($errors.Count){throw ($errors|Out-String)}}}
Pass 'Stock native helper compiles without invoking Windows APIs' {Initialize-StockReviewNative}
Pass 'Stock native helper exports no key/mouse/menu/command entry point' {
  $allowed=@('Foreground','AssertFrame','AssertCapture','Resize1280x800','SetThreadDpiAwarenessContext')
  foreach($method in [OpenNavX.StockReviewNative].GetMethods([Reflection.BindingFlags]'Public,Static,DeclaredOnly')) {if($method.Name -cnotin $allowed){throw 'Unexpected native action'}}
  $source=[IO.File]::ReadAllText((Join-Path $PSScriptRoot 'StockReviewNative.cs'))
  if($source -match 'extern[^;]*(SendInput|keybd_event|mouse_event|SetCursorPos)'){throw 'Global input injector present'}
}
$fixtureArguments=@{StockFixture=$true};if($PortableContracts){$fixtureArguments.PortableContracts=$true};if($IsolatedLocal){$fixtureArguments.IsolatedLocal=$true}
$transaction=& (Join-Path $PSScriptRoot 'test-commissioning-launch.ps1') @fixtureArguments | ConvertFrom-Json
if($transaction.status -cne 'passed' -or $transaction.identity -cne 'stock-only'){throw 'Stock full-tree verification fixture failed'}
[pscustomobject]@{status='passed';count=$checks.Count+$transaction.count;policyCount=$checks.Count;transactionCount=$transaction.count;checks=@($checks);transactionChecks=$transaction.checks;nativeInteropCompiled=$true;applicationLaunched=$false;boatAccess=$false;hardwareCommands=$false;stockOrProductAcceptance=$false} | ConvertTo-Json -Depth 6

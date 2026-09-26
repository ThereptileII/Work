# Pure policy tests; no registry, installer, desktop input, network or boat access.
[CmdletBinding()]
param()
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'OfficialUpgradePolicy.ps1')
$checks=New-Object 'Collections.Generic.List[string]'
function Pass([string]$Name,[scriptblock]$Test) { & $Test;$checks.Add($Name) }
function Refuse([string]$Name,[scriptblock]$Test) {
  $rejected=$false;try {& $Test | Out-Null} catch {$rejected=$true}
  if(-not $rejected){throw "Policy accepted unsafe input: $Name"};$checks.Add($Name)
}
function Controls([string[]]$Labels) { @($Labels | ForEach-Object {[pscustomobject]@{Text=$_;Class='Static';Enabled=$true}}) }
$version='OpenCPN Version 5.12.4-0+37fd0cd'
$pages=@{
 Language=@('Please select a language:','English','OK'); Welcome=@("Welcome to $version Setup");
 License=@('License Agreement'); Upgrade=@('Already Installed','Upgrade OpenCPN using previous settings (recommended)','Parallel Installation (advanced users)');
 Components=@('Installation Settings'); StartMenu=@('Choose Start Menu Folder'); Ready=@('Ready to Install');
 Installing=@('Installing',"Please wait while $version is being installed.");
 Finish=@("Completing $version Setup","$version has been installed on your computer.","&Run $version",'Show Install Log file','&Finish')
}
foreach($page in $pages.Keys){
 Pass "Recognizes complete exact $page page" {if((Get-OfficialUpgradePage (Controls $pages[$page])) -cne $page){throw 'Page mismatch'}}
}
Pass 'Incomplete Finish waits without clicking' {if((Get-OfficialUpgradePage (Controls $pages.Finish[0..1])) -cne 'Unknown'){throw 'Half-built Finish accepted'}}
Pass 'Unknown and other release pages have no action' {foreach($label in @('Surprise','Welcome to OpenCPN Version 5.12.2 Setup','Version Already Installed','Previous Installations')){if((Get-OfficialUpgradePage (Controls @($label))) -cne 'Unknown'){throw 'Unknown release/page accepted'}}}
foreach($page in @('Choose Install Location','Default configuration settings')) {Refuse "Refuses unexpected $page" {Get-OfficialUpgradePage (Controls @($page))}}
$sequence=@('Start','Language','Welcome','License','Upgrade','Components','StartMenu','Ready','Installing','Finish')
Pass 'Only complete visible Upgrade sequence accepted' {for($i=1;$i -lt $sequence.Count;$i++){Assert-OfficialUpgradeTransition $sequence[$i-1] $sequence[$i]}}
Pass 'Optional language/startmenu and very fast Install transitions accepted' {Assert-OfficialUpgradeTransition Start Welcome;Assert-OfficialUpgradeTransition Components Ready;Assert-OfficialUpgradeTransition Ready Finish}
foreach($pair in @(@('License','Components'),@('Welcome','Upgrade'),@('Upgrade','Ready'),@('Components','Finish'),@('Start','Finish'),@('Ready','Upgrade'),@('Finish','Welcome'),@('Start','Unknown'))){
 Refuse ('Refuses skipped/repeated/reversed '+($pair -join ' -> ')) {Assert-OfficialUpgradeTransition $pair[0] $pair[1]}
}
$destination='C:\Program Files (x86)\OpenCPN'
$summary="Setup type:`r`n`tUpgrade`r`n`r`nDestination location:`r`n`t$destination`r`n`r`nDelete (existing) Config Subdirectories/ Files:`r`n`tnone`r`n`r`nCreate shortcuts:`r`n`tStart Menu: OpenCPN`r`n`tDesktop: C:\Users\Test\Desktop"
Pass 'Exact Upgrade final summary preserves profile' {Assert-OfficialUpgradeSummary $summary $destination}
Pass 'Escaped upstream summary separators validated identically' {Assert-OfficialUpgradeSummary ($summary.Replace("`r`n",'\r\n').Replace("`t",'\t')) $destination}
foreach($type in @('Fresh installation','Downgrade','Reinstall','Parallel installation')) {Refuse "Refuses final $type" {Assert-OfficialUpgradeSummary ($summary.Replace('Upgrade',$type)) $destination}}
Refuse 'Refuses different destination' {Assert-OfficialUpgradeSummary ($summary.Replace($destination,'C:\Other')) $destination}
Refuse 'Refuses selected deletion in final summary' {Assert-OfficialUpgradeSummary ($summary.Replace('none','Delete config file (opencpn.ini)')) $destination}
Refuse 'Refuses extra configuration writer section' {Assert-OfficialUpgradeSummary ($summary+"`r`n`r`nOpenCPN Configuration Settings:`r`n`tLanguage") $destination}
Refuse 'Refuses missing deletion summary' {Assert-OfficialUpgradeSummary ($summary.Replace("Delete (existing) Config Subdirectories/ Files:`r`n`tnone`r`n`r`n",'')) $destination}
function SafeItems { @((Get-OfficialResetNames) | ForEach-Object {[pscustomobject]@{Text=$_;State=1}}) }
Pass 'Every known reset/delete item unchecked' {Assert-OfficialResetItems (SafeItems)}
Pass 'Readonly unchecked component remains disabled' {$items=SafeItems;$items[1].State=4;Assert-OfficialResetItems $items}
foreach($state in @(0,2,3,5,6,7)) {Refuse "Refuses unknown/checked/mixed component state $state" {$items=SafeItems;$items[1].State=$state;Assert-OfficialResetItems $items}}
Refuse 'Refuses selected reset parent' {$items=SafeItems;$items[0].State=2;Assert-OfficialResetItems $items}
Refuse 'Refuses unknown destructive component' {Assert-OfficialResetItems (@(SafeItems)+@([pscustomobject]@{Text='Delete everything';State=1}))}
Refuse 'Refuses selected configuration writer' {Assert-OfficialResetItems (@(SafeItems)+@([pscustomobject]@{Text='OpenCPN Configuration Settings';State=2}))}
Refuse 'Refuses missing reset parent' {$items=SafeItems;Assert-OfficialResetItems $items[1..6]}
Refuse 'Refuses duplicated component identity' {$items=SafeItems;Assert-OfficialResetItems (@($items)+@($items[1]))}
function Registration { [pscustomobject]@{key='OpenCPN 5.12.2';location=$destination;version='5.12.2-0';uninstaller='"'+$destination+'\uninstall.exe"';values=[pscustomobject]@{InstallLocation=[pscustomobject]@{kind='String';value=$destination};Example=[pscustomobject]@{kind='DWord';value=7}}} }
$original=Registration
Pass 'Exact prepared registry values and types retained' {Assert-OfficialRegistration @(Registration) @($original)}
Refuse 'Refuses extra OpenCPN registration' {Assert-OfficialRegistration @((Registration),(Registration)) @($original)}
Refuse 'Refuses changed registration path' {$changed=Registration;$changed.location='C:\Other';Assert-OfficialRegistration @($changed) @($original)}
Refuse 'Refuses changed registry value' {$changed=Registration;$changed.values.Example.value=8;Assert-OfficialRegistration @($changed) @($original)}
Refuse 'Refuses changed registry type' {$changed=Registration;$changed.values.Example.kind='String';Assert-OfficialRegistration @($changed) @($original)}
Refuse 'Refuses added registry value' {$changed=Registration;$changed.values | Add-Member -MemberType NoteProperty -Name Unexpected -Value ([pscustomobject]@{kind='DWord';value=1});Assert-OfficialRegistration @($changed) @($original)}
function CoreRegistration {
 $entry=Registration;$entry.key='OpenCPN 5.12.2-0+b69f44c';$entry.version='5.12.2-0+b69f44c'
 $entry.values | Add-Member -MemberType NoteProperty -Name CompareVersion -Value ([pscustomobject]@{kind='String';value='2025.19.0101.00'})
 return $entry
}
function PluginRegistration {
 [pscustomobject]@{key='OpenCPN';location=$null;version='1.3.1';uninstaller=$destination+'\Uninstall rtlsdr_pi.exe';values=[pscustomobject]@{
   DisplayName=[pscustomobject]@{kind='String';value='OpenCPN rtlsdr_pi'};Publisher=[pscustomobject]@{kind='String';value='opencpn.org'};
   NoModify=[pscustomobject]@{kind='String';value='1'}}}
}
Pass 'Core remains the only installation with exact observed bare RTL-SDR present' {
 $split=Split-OfficialRegistration @((CoreRegistration),(PluginRegistration)) $destination
 if($split.core.Count -ne 1 -or $split.plugins.Count -ne 1 -or $split.core[0].key -ceq 'OpenCPN'){throw 'Plugin mistaken for stock installation'}
}
Pass 'Core-only preparation is valid without inventing plugin entries' {$split=Split-OfficialRegistration @((CoreRegistration)) $destination;if($split.plugins.Count -ne 0){throw 'Invented plugin'}}
Pass 'Registry enumeration hashtables and deserialized JSON compare identically' {
 $entry=PluginRegistration;$map=@{};foreach($property in $entry.values.PSObject.Properties){$map[$property.Name]=$property.Value};$entry.values=$map
 Assert-OfficialRegistration @($entry) @(PluginRegistration)
 $split=Split-OfficialRegistration @((CoreRegistration),$entry) $destination;if($split.plugins.Count -ne 1){throw 'Hashtable plugin not recognized'}
}
foreach($field in @('version','location','uninstaller','key')) {
 Refuse "Refuses misleading plugin $field" {$entry=PluginRegistration;$entry.$field='unexpected';Split-OfficialRegistration @((CoreRegistration),$entry) $destination}
}
Refuse 'Refuses changed plugin display identity' {$entry=PluginRegistration;$entry.values.DisplayName.value='OpenCPN';Split-OfficialRegistration @((CoreRegistration),$entry) $destination}
Refuse 'Refuses changed plugin publisher' {$entry=PluginRegistration;$entry.values.Publisher.value='other';Split-OfficialRegistration @((CoreRegistration),$entry) $destination}
Refuse 'Refuses plugin claiming core CompareVersion' {$entry=PluginRegistration;$entry.values | Add-Member NoteProperty CompareVersion ([pscustomobject]@{kind='String';value='2025.19.0101.00'});Split-OfficialRegistration @((CoreRegistration),$entry) $destination}
Refuse 'Refuses core without upstream detector CompareVersion' {$entry=CoreRegistration;$entry.values.PSObject.Properties.Remove('CompareVersion');Split-OfficialRegistration @($entry,(PluginRegistration)) $destination}
Refuse 'Refuses plugin-only environment' {Split-OfficialRegistration @((PluginRegistration)) $destination}
Refuse 'Refuses duplicate core with plugin present' {Split-OfficialRegistration @((CoreRegistration),(CoreRegistration),(PluginRegistration)) $destination}
Refuse 'Refuses duplicate observed plugin' {Split-OfficialRegistration @((CoreRegistration),(PluginRegistration),(PluginRegistration)) $destination}
Pass 'Exact plugin registry inventory is retained' {Assert-OfficialPluginRegistration @((PluginRegistration)) @((PluginRegistration));Assert-OfficialPluginRegistration @() @()}
Refuse 'Refuses disappearance of prepared plugin registration' {Assert-OfficialPluginRegistration @() @((PluginRegistration))}
Refuse 'Refuses new plugin registration after preparation' {Assert-OfficialPluginRegistration @((PluginRegistration)) @()}
Refuse 'Refuses changed plugin value/type despite same key' {$entry=PluginRegistration;$entry.values.NoModify.kind='DWord';Assert-OfficialPluginRegistration @($entry) @((PluginRegistration))}
# Load only the task identity predicate, never the real scheduler entry point.
$dispatchPath=Join-Path $PSScriptRoot 'start-official-upgrade.ps1'
$tokens=$null;$parseErrors=$null
$dispatchAst=[Management.Automation.Language.Parser]::ParseFile($dispatchPath,[ref]$tokens,[ref]$parseErrors)
Pass 'Dispatcher parses without performing scheduler or registry operations' {if($parseErrors.Count){throw 'Dispatcher parse failed'}}
$taskPredicate=$dispatchAst.Find({param($node) $node -is [Management.Automation.Language.FunctionDefinitionAst] -and $node.Name -ceq 'Assert-DispatchTask'},$true)
if(-not $taskPredicate){throw 'Dispatcher task predicate unavailable'}
. ([scriptblock]::Create($taskPredicate.Extent.Text))
$dispatchRequest=[pscustomobject]@{taskName='OpenNavX-OfficialUpgrade-aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa';userSid='S-1-5-21-123';execute='C:\Windows\System32\WindowsPowerShell\v1.0\powershell.exe';arguments='exact reviewed driver arguments'}
function TaskFixture {
 [pscustomobject]@{TaskPath='\';TaskName=$dispatchRequest.taskName;
 Principal=[pscustomobject]@{UserId=$dispatchRequest.userSid;LogonType='Interactive';RunLevel='Highest'};
 Actions=@([pscustomobject]@{Execute=$dispatchRequest.execute;Arguments=$dispatchRequest.arguments});
 Settings=[pscustomobject]@{ExecutionTimeLimit='PT0S';StopIfGoingOnBatteries=$false;DisallowStartIfOnBatteries=$false;AllowHardTerminate=$false}}
}
Pass 'Exact interactive elevated task has no forced timeout/termination' {Assert-DispatchTask (TaskFixture) $dispatchRequest}
if ([Environment]::OSVersion.Platform -eq 'Win32NT') {
  Pass 'Native scheduler account spelling resolves to the same exact user SID' {
    $savedSid=$dispatchRequest.userSid
    try {
      $identity=[Security.Principal.WindowsIdentity]::GetCurrent()
      $dispatchRequest.userSid=$identity.User.Value
      $task=TaskFixture;$task.Principal.UserId=$identity.Name
      Assert-DispatchTask $task $dispatchRequest
    } finally {$dispatchRequest.userSid=$savedSid}
  }
}
foreach($field in @('TaskName','TaskPath')) {Refuse "Refuses changed scheduler $field" {$task=TaskFixture;$task.$field='other';Assert-DispatchTask $task $dispatchRequest}}
foreach($field in @('UserId','LogonType','RunLevel')) {Refuse "Refuses changed scheduler principal $field" {$task=TaskFixture;$task.Principal.$field='other';Assert-DispatchTask $task $dispatchRequest}}
foreach($field in @('Execute','Arguments')) {Refuse "Refuses changed scheduler action $field" {$task=TaskFixture;$task.Actions[0].$field='other';Assert-DispatchTask $task $dispatchRequest}}
Refuse 'Refuses additional scheduled action' {$task=TaskFixture;$task.Actions+=@($task.Actions[0]);Assert-DispatchTask $task $dispatchRequest}
Refuse 'Refuses scheduler-enforced timeout' {$task=TaskFixture;$task.Settings.ExecutionTimeLimit='PT1M';Assert-DispatchTask $task $dispatchRequest}
foreach($field in @('StopIfGoingOnBatteries','DisallowStartIfOnBatteries','AllowHardTerminate')) {Refuse "Refuses unsafe scheduler $field" {$task=TaskFixture;$task.Settings.$field=$true;Assert-DispatchTask $task $dispatchRequest}}
# Compiles the exact native helper without invoking any Windows or desktop API.
Pass 'Native helper compiles without invoking desktop/process APIs' {Add-Type -Path (Join-Path $PSScriptRoot 'OfficialWizardNative.cs')}
[pscustomobject]@{status='passed';checks=$checks.Count;tests=@($checks);nativeWizardExecuted=$false} | ConvertTo-Json -Depth 4

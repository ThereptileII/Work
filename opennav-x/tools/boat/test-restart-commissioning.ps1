# Pure policy/parser/framing and disposable journal tests. Never starts OpenCPN,
# changes the real profile, connects to a marine device, or opens a named pipe.
[CmdletBinding()]
param([switch]$IsolatedLocal)
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'
if([Environment]::OSVersion.Platform -eq 'Win32NT' -and $env:GITHUB_ACTIONS -cne 'true' -and -not $IsolatedLocal){throw 'Use CI or explicit -IsolatedLocal for temporary-only tests.'}
if($IsolatedLocal -and @(Get-Process -Name opencpn -ErrorAction SilentlyContinue).Count){throw 'Close OpenCPN before local disposable tests.'}
. (Join-Path $PSScriptRoot 'RestartCommissioning.ps1')
Initialize-RestartNative
$script:checks=0
function Check([bool]$Okay,[string]$Name){if(-not $Okay){throw ('FAILED: '+$Name)};$script:checks++}
function Refuse([scriptblock]$Action,[string]$Name){$failed=$false;try{& $Action}catch{$failed=$true};Check $failed $Name}
function Message($Object){return ,[OpenNavX.RestartCommissioningNative]::Message([Text.Encoding]::UTF8.GetBytes(($Object|ConvertTo-Json -Depth 5 -Compress)))}
function Clone($Object){return Message $Object}
Check (Test-RestartImagePath 'C:\WINDOWS\System32\WindowsPowerShell\v1.0\powershell.exe' 'C:\WINDOWS\system32\WindowsPowerShell\v1.0\powershell.exe') 'native Windows process path letter case is identity-equivalent'
Check (-not (Test-RestartImagePath 'C:\other\powershell.exe' 'C:\Windows\System32\WindowsPowerShell\v1.0\powershell.exe')) 'different image directory remains refused'
Check (-not (Test-RestartImagePath 'C:\Windows\SysWOW64\WindowsPowerShell\v1.0\powershell.exe' 'C:\Windows\System32\WindowsPowerShell\v1.0\powershell.exe')) 'redirected 32-bit PowerShell image remains refused'
Check (-not (Test-RestartImagePath '' '')) 'missing image identity remains refused'
$session=[pscustomobject]@{session=('a'*64);recordSha256=('b'*64);windowsSessionId='1';executable='C:\XNav\owned\app\opencpn.exe';executableSha256=('c'*64);helper='C:\XNav\owned\app\opennav-restart.exe';helperSha256=('d'*64);workingDirectory='C:\XNav\owned\app';path='C:\XNav\owned\app;C:\Windows\System32;C:\Windows'}
$parent=[pscustomobject]@{pid='42';createdFiletime='133000000000000000'}
$request=Message @{protocol=1;kind='request';session=$session.session;recordSha256=$session.recordSha256;nonce=('e'*64);parentPid=$parent.pid;parentCreatedFiletime=$parent.createdFiletime;parentExitCode='0';helperPid='43';helperCreatedFiletime='133000000001000000';windowsSessionId='1';executable=$session.executable;executableSha256=$session.executableSha256;helper=$session.helper;helperSha256=$session.helperSha256;workingDirectory=$session.workingDirectory;path=$session.path;arguments=@('--legacy')}
Assert-RestartRequest $request $session $parent '--legacy' '43';Check $true 'valid exact request'
foreach($key in @($request.Keys)) {
 $bad=Clone $request;$null=$bad.Remove($key)
 Refuse {Assert-RestartRequest $bad $session $parent '--legacy' '43'} ('missing '+$key)
}
$bad=Clone $request;$bad.Add('execute','anything');Refuse {Assert-RestartRequest $bad $session $parent '--legacy' '43'} 'unknown field'
foreach($key in @('session','recordSha256','executableSha256','helperSha256')) {
 $bad=Clone $request;$bad[$key]='f'*64;Refuse {Assert-RestartRequest $bad $session $parent '--legacy' '43'} ('changed '+$key)
}
foreach($key in @('parentPid','parentCreatedFiletime','windowsSessionId','helperPid','executable','helper','workingDirectory','path','parentExitCode')) {
 $bad=Clone $request;$bad[$key]='unexpected';Refuse {Assert-RestartRequest $bad $session $parent '--legacy' '43'} ('wrong '+$key)
}
foreach($number in @('0','01','-1','1.0','1e3',' 1','18446744073709551616')) {
 $bad=Clone $request;$bad['helperCreatedFiletime']=$number;Refuse {Assert-RestartRequest $bad $session $parent '--legacy' '43'} ('invalid numeric '+$number)
}
foreach($arguments in @(@('--legacy','--demo'),@('--xnav'),@('--legacy --demo'),@('--safe-mode'),@())) {
 $bad=Clone $request;$bad['arguments']=[string[]]$arguments;Refuse {Assert-RestartRequest $bad $session $parent '--legacy' '43'} 'nonexact arguments'
}
$bad=Clone $request;$bad['path']='C:\unreviewed;'+$session.path;Refuse {Assert-RestartRequest $bad $session $parent '--legacy' '43'} 'expanded runtime PATH'
foreach($text in @('{"protocol":1,"protocol":1}','{"a":"x","a":"y"}','{"a":null}','{"a":true}','{"a":1.0}','{"a":10}','{"a":{}}','{"a":["a","b","c","d","e"]}','{} trailing','{"a":"\ud800"}',([string][char]0xfeff+'{}'),'{"a":"x"}{}')) {
 Refuse {[OpenNavX.RestartCommissioningNative]::Message([Text.Encoding]::UTF8.GetBytes($text))} 'strict malformed JSON'
}
Refuse {[OpenNavX.RestartCommissioningNative]::Message([byte[]]@(0xff,0xfe,0x7b,0x7d))} 'invalid UTF8'
$case=[OpenNavX.RestartCommissioningNative]::Message([Text.Encoding]::UTF8.GetBytes('{"protocol":1,"Protocol":"1"}'));Check ($case.Count -eq 2) 'parser preserves key case rather than normalizing'
Refuse {Assert-RestartRequest $case $session $parent '--legacy' '43'} 'case variant schema refused'
$receipt=Message @{protocol=1;kind='receipt';session=$session.session;recordSha256=$session.recordSha256;nonce=$request['nonce'];requestSha256=('f'*64);permitId=('1'*64);status='started';childPid='44';childCreatedFiletime='133000000002000000';win32Error='0'}
Assert-RestartReceipt $receipt $request ('f'*64) ('1'*64);Check $true 'actual native receipt schema'
foreach($key in @($receipt.Keys)){$bad=Clone $receipt;$null=$bad.Remove($key);Refuse {Assert-RestartReceipt $bad $request ('f'*64) ('1'*64)} ('receipt missing '+$key)}
foreach($key in @('session','nonce','recordSha256','requestSha256','permitId','status','childPid','childCreatedFiletime','win32Error')){$bad=Clone $receipt;$bad[$key]='x';Refuse {Assert-RestartReceipt $bad $request ('f'*64) ('1'*64)} ('receipt wrong '+$key)}
$failed=Clone $receipt;$failed['status']='failed';$failed['childPid']='0';$failed['childCreatedFiletime']='0';$failed['win32Error']='5';Assert-RestartReceipt $failed $request ('f'*64) ('1'*64);Check $true 'explicit failed receipt'
$failed['childPid']='44';Refuse {Assert-RestartReceipt $failed $request ('f'*64) ('1'*64)} 'failed receipt cannot claim child'
$before=@{'Settings/Foo'='unchanged';'OpenNav/InterfaceMode'='xnav';'Settings/NMEADataSource/DataConnections'='exact bytes';'Plugins/pilot/bEnabled'='0';'Directories/SENCFileLocation'='original';'ChartDirectories/ChartDir1'='original'}
$after=$before.Clone();$after['OpenNav/InterfaceMode']='legacy'
$delta=@(Assert-RestartIniDelta $before $after '--legacy');Check ($delta.Count -eq 1) 'only requested interface persists'
$values=@{chartindex='42';size='1280';position='-1920';boolean='1';color='3';latlon='   57.1234,   16.4567';scale='0.005';rotation='359'}
foreach($key in (Get-RestartDisplayKeys).Keys) {
 $change=$after.Clone();$change[$key]=$values[(Get-RestartDisplayKeys)[$key]]
 $delta=@(Assert-RestartIniDelta $before $change '--legacy');Check ($delta.Count -eq 2) ('reviewed typed key '+$key)
 $change[$key]='unexpected';Refuse {Assert-RestartIniDelta $before $change '--legacy'} ('invalid typed key '+$key)
}
foreach($key in @('Settings/Foo','Settings/NMEADataSource/DataConnections','Plugins/pilot/bEnabled','Directories/SENCFileLocation','ChartDirectories/ChartDir1','Settings/SoundCmd','OpenNav/AutopilotControlEnabled','Settings/PersistActiveRoute','Settings/Unknown')) {
 $change=$after.Clone();$change[$key]='changed';Refuse {Assert-RestartIniDelta $before $change '--legacy'} ('unreviewed mutation '+$key)
}
$change=$after.Clone();$change.Remove('Settings/Foo');Refuse {Assert-RestartIniDelta $before $change '--legacy'} 'removed key'
$change=[Collections.Generic.Dictionary[string,string]]::new([StringComparer]::Ordinal);foreach($key in $after.Keys){$change.Add($key,$after[$key])};$change.Remove('Settings/Foo')|Out-Null;$change.Add('settings/Foo','unchanged');Refuse {Assert-RestartIniDelta $before $change '--legacy'} 'key case replacement'
Refuse {Assert-RestartIniDelta $before $after '--safe-mode'} 'Safe does not change persisted interface'
$delta=@(Assert-RestartIniDelta $before $before '--safe-mode');Check ($delta.Count -eq 0) 'Safe unchanged mode accepted'
Refuse {Assert-RestartIniDelta $before $before '--legacy'} 'target mode mismatch'
$invalid=@{chartindex=@('-2','1000001','NaN','1.5');size=@('0','32769','-1','01','1.1');position=@('-32769','32769');boolean=@('true','false','2');color=@('0','4');latlon=@('91,0','0,181','NaN,0','Infinity,0','1,2,3');scale=@('0','-1','NaN','Infinity','1e999','10001');rotation=@('-360','360')}
foreach($kind in $invalid.Keys){foreach($value in $invalid[$kind]){Refuse {Assert-RestartScalar $kind $value} ('scalar bound '+$kind)}}
$aui='layout2|name=ChartCanvas;caption=;state=768;dir=5;layer=0;row=0;pos=0;prop=100000;bestw=5;besth=5;minw=256;minh=800;maxw=-1;maxh=-1;floatx=-1;floaty=-1;floatw=-1;floath=-1|dock_size(5,0,0)=1280|'
$withLayout=$before.Clone();$withLayout['AUI/AUIPerspective']=$aui
$afterLayout=$after.Clone();$afterLayout['AUI/AUIPerspective']=$aui.Replace('minw=256','minw=320')
$delta=@(Assert-RestartIniDelta $withLayout $afterLayout '--legacy');Check ($delta.Count -eq 2) 'exact parsed AUI branch integrated'
Refuse {Assert-RestartIniDelta $before $afterLayout '--legacy'} 'missing AUI baseline cannot grant new panes'
$afterLayout['AUI/AUIPerspective']=$aui.Replace('name=ChartCanvas','name=other');Refuse {Assert-RestartIniDelta $withLayout $afterLayout '--legacy'} 'AUI identity change rejected by actual delta policy'
$withCounter=$before.Clone();$withCounter['PlugIns/Dashboard/SumLogNM']='100.5'
$afterCounter=$after.Clone();$afterCounter['PlugIns/Dashboard/SumLogNM']='101.0'
$delta=@(Assert-RestartIniDelta $withCounter $afterCounter '--legacy');Check ($delta.Count -eq 2) 'exact monotonic Dashboard counter branch integrated'
$afterCounter['PlugIns/Dashboard/SumLogNM']='1.0';Refuse {Assert-RestartIniDelta $withCounter $afterCounter '--legacy'} 'counter reset rejected by actual delta policy'
Refuse {Assert-RestartIniDelta $before $afterCounter '--legacy'} 'new unreviewed Dashboard counter refused'
# Actual C# framing against MemoryStream: bounds/truncation/expired deadline.
$deadline=[DateTime]::UtcNow.AddSeconds(30);$memory=New-Object IO.MemoryStream
[OpenNavX.RestartCommissioningNative]::WriteFrame($memory,[Text.Encoding]::UTF8.GetBytes('valid'),$deadline);$memory.Position=0
$roundtrip=[OpenNavX.RestartCommissioningNative]::ReadFrame($memory,$deadline);Check ([Text.Encoding]::UTF8.GetString($roundtrip) -ceq 'valid') 'length frame round trip';$memory.Dispose()
foreach($bytes in @([byte[]]@(0,0,0,0),[byte[]]@(1,0,1,0),[byte[]]@(2,0,0,0,1),[byte[]]@(1,2))) {
 $stream=New-Object IO.MemoryStream(,$bytes);try{Refuse {[OpenNavX.RestartCommissioningNative]::ReadFrame($stream,$deadline)} 'invalid bounded frame'}finally{$stream.Dispose()}
}
$memory=New-Object IO.MemoryStream;try{Refuse {[OpenNavX.RestartCommissioningNative]::WriteFrame($memory,[byte[]]@(1),[DateTime]::UtcNow.AddSeconds(-1))} 'expired absolute deadline';Check ($memory.Length -eq 0) 'expired deadline writes no bytes'}finally{$memory.Dispose()}
$memory=New-Object IO.MemoryStream(,[byte[]]@(1,0,0,0,42));try{Refuse {[OpenNavX.RestartCommissioningNative]::ReadFrame($memory,[DateTime]::UtcNow.AddSeconds(-1))} 'expired read deadline';Check ($memory.Position -eq 0) 'expired deadline consumes no bytes'}finally{$memory.Dispose()}
$reply=[OpenNavX.RestartCommissioningNative]::Reply([string[]]@($script:RestartMagic,'DENY'));Check ([BitConverter]::ToUInt32($reply,0) -eq 2) 'fixed denial vector'
Refuse {[OpenNavX.RestartCommissioningNative]::Reply([string[]]@('arbitrary','ALLOW','extra'))} 'variable reply schema'
$build=[pscustomobject]@{executable_sha256=('c'*64);restart_helper_sha256=('d'*64);commissioning_restart_protocol=1;test_fixtures=$false;build_purpose='INSTALLED PRODUCT';commit=('9'*40)}
Assert-RestartBuild $build ('9'*40) ('c'*64) ('d'*64);Check $true 'guard-capable fixture-free product'
$roundtripBuild=($build|ConvertTo-Json|ConvertFrom-Json);Assert-RestartBuild $roundtripBuild ('9'*40) ('c'*64) ('d'*64);Check $true 'capability integer survives native JSON number representation'
foreach($badVersion in @('1',2,1.0,$true)){$build.commissioning_restart_protocol=$badVersion;Refuse {Assert-RestartBuild $build ('9'*40) ('c'*64) ('d'*64)} 'wrong capability type/version'}
$build.commissioning_restart_protocol=1;$build.test_fixtures=$true;Refuse {Assert-RestartBuild $build ('9'*40) ('c'*64) ('d'*64)} 'fixture binary cannot arm'
$build.test_fixtures=$false;Refuse {Assert-RestartBuild $build ('8'*40) ('c'*64) ('d'*64)} 'wrong build cannot arm'
Refuse {Assert-RestartBuild $build ('9'*40) ('f'*64) ('d'*64)} 'capability wrong actual application hash'
Refuse {Assert-RestartBuild $build ('9'*40) ('c'*64) ('f'*64)} 'capability wrong actual helper hash'
$build.PSObject.Properties.Remove('commissioning_restart_protocol');Refuse {Assert-RestartBuild $build ('9'*40) ('c'*64) ('d'*64)} 'old product without guard cannot arm'
$arm=[pscustomobject]@{execute='C:\Windows\System32\WindowsPowerShell\v1.0\powershell.exe';arguments='fixed verified arguments'}
$task=[pscustomobject]@{State='Ready';Actions=@([pscustomobject]@{Execute=$arm.execute;Arguments=$arm.arguments;WorkingDirectory=$null});Principal=[pscustomobject]@{UserId='S-1-5-21-100';RunLevel='Limited';LogonType='Interactive'};Triggers=@()}
Assert-RestartTaskIdentity $task $arm 'S-1-5-21-100';Check $true 'only completed owned limited task collectable'
$task.Triggers=$null;Assert-RestartTaskIdentity $task $arm 'S-1-5-21-100';Check $true 'native null trigger property represents no trigger'
$task.Triggers=@($null);Refuse {Assert-RestartTaskIdentity $task $arm 'S-1-5-21-100'} 'array containing unknown null trigger is not an empty provider property'
$task.Triggers=@()
Refuse {Resolve-RestartTaskSid ''} 'missing task principal refused'
Refuse {Assert-RestartTaskIdentity $task $arm 'S-1-5-21-101'} 'different exact SID refused'
if([Environment]::OSVersion.Platform -eq 'Win32NT') {
 $current=[Security.Principal.WindowsIdentity]::GetCurrent()
 try {
  $task.Principal.UserId=$current.Name;$task.Triggers=$null
  Assert-RestartTaskIdentity $task $arm $current.User.Value;Check $true 'actual Windows account name resolves to exact current SID'
  $task.Principal.UserId=[Environment]::UserName
  Assert-RestartTaskIdentity $task $arm $current.User.Value;Check $true 'native provider unqualified account name resolves to exact current SID'
  Refuse {Assert-RestartTaskIdentity $task $arm 'S-1-5-18'} 'resolved current account cannot substitute SYSTEM SID'
  $task.Principal.UserId='OpenNav-NoSuchAccount-'+[guid]::NewGuid().ToString('N')
  Refuse {Assert-RestartTaskIdentity $task $arm $current.User.Value} 'unresolvable native account refused'
 } finally {$current.Dispose();$task.Principal.UserId='S-1-5-21-100';$task.Triggers=@()}
}
$task.State='Running';Refuse {Assert-RestartTaskIdentity $task $arm 'S-1-5-21-100'} 'running broker cannot be collected';$task.State='Ready'
Refuse {Assert-RestartTaskIdentity $task $arm 'S-1-5-21-100' 'Running'} 'completed task cannot be presented as a listening broker'
$task.State='Running';Assert-RestartTaskIdentity $task $arm 'S-1-5-21-100' 'Running';Check $true 'explicit listening check requires the exact running owned task'
Refuse {Assert-RestartTaskIdentity $task $arm 'S-1-5-21-101' 'Running'} 'listening check still rejects different principal'
$task.Actions[0].Arguments='changed';Refuse {Assert-RestartTaskIdentity $task $arm 'S-1-5-21-100' 'Running'} 'listening check still rejects different action arguments';$task.Actions[0].Arguments=$arm.arguments
$task.State='Ready';Refuse {Assert-RestartTaskIdentity $task $arm 'S-1-5-21-100' 'Disabled'} 'unsupported expected task state refused'
foreach($field in @('Execute','Arguments','WorkingDirectory')) {$saved=$task.Actions[0].$field;$task.Actions[0].$field='changed';Refuse {Assert-RestartTaskIdentity $task $arm 'S-1-5-21-100'} ('changed task '+$field);$task.Actions[0].$field=$saved}
foreach($field in @('UserId','RunLevel','LogonType')) {$saved=$task.Principal.$field;$task.Principal.$field='changed';Refuse {Assert-RestartTaskIdentity $task $arm 'S-1-5-21-100'} ('changed task principal '+$field);$task.Principal.$field=$saved}
$task.Triggers=@([pscustomobject]@{Enabled=$true});Refuse {Assert-RestartTaskIdentity $task $arm 'S-1-5-21-100'} 'unexpected repeat trigger refused'
$retained=@([pscustomobject]@{path='dashboard_pi.dll';sourceRevision=('1'*40)})
$shutdown=[pscustomobject]@{schema=1;physicalCommands=0;reviewedUtc=[DateTime]::UtcNow.ToString('o');plugins=@([pscustomobject]@{plugin='dashboard_pi';revision=('1'*40);sourceSha256=('2'*64);shutdownBoundary='Source-reviewed local settings only'})}
Assert-RestartShutdownReview $shutdown $retained;Check $true 'retained shutdown source binding'
$shutdown.plugins[0].revision='3'*40;Refuse {Assert-RestartShutdownReview $shutdown $retained} 'wrong retained shutdown source revision'
$shutdown.plugins[0].revision='1'*40;$shutdown.plugins[0].plugin='unknown_pi';Refuse {Assert-RestartShutdownReview $shutdown $retained} 'unknown retained plugin refused'
$shutdown.plugins[0].plugin='dashboard_pi';$shutdown.reviewedUtc=[DateTime]::UtcNow.AddHours(-25).ToString('o');Refuse {Assert-RestartShutdownReview $shutdown $retained} 'expired shutdown review'
# Journal tests are confined to a fresh random temp directory. Test-only path
# override cannot reach the real Windows profile, registry, workspace or app.
$script:temporary=Join-Path ([IO.Path]::GetTempPath()) ('OpenNav-Restart-Policy-'+[guid]::NewGuid().ToString('N'))
$null=New-Item -ItemType Directory -Path $script:temporary
function Assert-LocalPath([string]$Path){$full=[IO.Path]::GetFullPath($Path);if($full -cne $script:temporary -and -not $full.StartsWith($script:temporary+[IO.Path]::DirectorySeparatorChar,[StringComparison]::Ordinal)){throw 'Test escaped disposable directory.'};return $full}
try {
 $raw=Join-Path $script:temporary 'raw.ini'
 [IO.File]::WriteAllText($raw,"[Settings]`nControl=untouched  `n[OpenNav]`nInterfaceMode=xnav`n")
 $rawBefore=Read-RestartIni $raw;Check ($rawBefore['Settings/Control'] -ceq 'untouched  ') 'strict parser preserves protected whitespace'
 [IO.File]::WriteAllText($raw,"[Settings]`nControl=untouched `n[OpenNav]`nInterfaceMode=xnav`n")
 $rawAfter=Read-RestartIni $raw;Refuse {Assert-RestartIniDelta $rawBefore $rawAfter '--safe-mode'} 'protected whitespace mutation refused'
 foreach($invalidIni in @("[Settings]`nFoo=1`nfoo=1","[Settings]`nFoo=1`n[settings]`nBar=2","[Settings]`nnot an assignment")) {
   [IO.File]::WriteAllText($raw,$invalidIni);Refuse {Read-RestartIni $raw} 'ambiguous strict INI rejected'
 }
 $coldDir=Join-Path $script:temporary 'cold';$null=New-Item -ItemType Directory -Path $coldDir
 $binding=[pscustomobject]@{session=$session;recordSha256=$session.recordSha256;directory=$coldDir}
 $start=New-Object Diagnostics.ProcessStartInfo
 $start.FileName=$session.executable;$start.WorkingDirectory=$session.workingDirectory;$start.Arguments='--xnav';$start.UseShellExecute=$false
 $start.EnvironmentVariables['PATH']=$session.path
 $start.EnvironmentVariables['UNRELATED_TEST_SENTINEL']='preserved'
 Set-RestartLaunchBinding $start $binding
 Check ($start.EnvironmentVariables['OPENNAV_COMMISSIONING_RESTART_SESSION'] -ceq $session.session -and $start.EnvironmentVariables['OPENNAV_COMMISSIONING_RESTART_RECORD_SHA256'] -ceq $session.recordSha256) 'only exact two cold bindings supplied'
 Check ($start.EnvironmentVariables['UNRELATED_TEST_SENTINEL'] -ceq 'preserved' -and $start.EnvironmentVariables['PATH'] -ceq $session.path) 'cold binding preserves reviewed environment'
 Refuse {Set-RestartLaunchBinding $start $binding} 'one cold launch only'
 $start.Arguments='--demo';Refuse {Set-RestartLaunchBinding $start $binding} 'fixture argument cannot arm'
 $ordinary=[pscustomobject]@{action='Launch'}
 Check ($null -eq (Get-OptionalRestartBinding $ordinary $null $null $null)) 'unarmed launch unchanged'
 foreach($badJob in @([pscustomobject]@{action='Launch';restartSessionRecord='x'},[pscustomobject]@{action='Launch';restartSessionRecord='';restartSessionSha256=''},[pscustomobject]@{action='LaunchPortableReview';restartSessionRecord='x';restartSessionSha256=('a'*64)},[pscustomobject]@{action='Launch';RestartSessionRecord='x';restartSessionSha256=('a'*64)})) {
   Refuse {Get-OptionalRestartBinding $badJob $null $null $null} 'malformed/inapplicable explicit binding refused'
 }
 $journal=Join-Path $script:temporary 'journal';$null=New-Item -ItemType Directory -Path $journal
 $permitPath=Publish-RestartPermit $journal @{permitId=('1'*64);status='consumed-before-allow'};$hash=Get-Digest $permitPath
 Refuse {Publish-RestartPermit $journal @{permitId=('2'*64)}} 'consumed permit cannot be replaced'
 Check ((Get-Digest $permitPath) -ceq $hash) 'original consumption journal unchanged'
 $chain=Join-Path $script:temporary 'chain';$null=New-Item -ItemType Directory -Path $chain
 $first=Join-Path $chain 'before.ini';[IO.File]::WriteAllText($first,"[Settings]`nFoo=1`n[OpenNav]`nInterfaceMode=xnav`n")
 $session|Add-Member beforeIni $first;$session|Add-Member beforeIniSha256 (Get-Digest $first)
 Write-Record (Join-Path $chain 'cold-launch-consumed.json') @{owner=$script:RestartOwner;session=$session.session;recordSha256=$session.recordSha256;status='consumed-before-start';mode='--xnav'}
 Write-Record (Join-Path $chain 'cold-child.json') @{owner=$script:RestartOwner;session=$session.session;recordSha256=$session.recordSha256;pid=$parent.pid;createdFiletime=$parent.createdFiletime;launchSha256=(Get-Digest (Join-Path $chain 'cold-launch-consumed.json'))}
 $baseline=Get-RestartBaseline $session (Join-Path $chain 'session.json') $parent
 Check ($baseline.sha256 -ceq $session.beforeIniSha256) 'initial baseline is cold copied bytes'
 $transition=$baseline.nextDirectory;$null=New-Item -ItemType Directory -Path $transition
 Refuse {Get-RestartBaseline $session (Join-Path $chain 'session.json') $parent} 'interrupted arm cannot silently retry'
 $post=Join-Path $transition 'post-close.ini';[IO.File]::WriteAllText($post,"[Settings]`nFoo=1`n[OpenNav]`nInterfaceMode=legacy`n")
 $wire=[Text.Encoding]::UTF8.GetBytes(($request|ConvertTo-Json -Compress));Save-RestartWire (Join-Path $transition 'request.json') $wire;$requestHash=Get-RestartBytesHash $wire
 $receipt['requestSha256']=$requestHash;$wire=[Text.Encoding]::UTF8.GetBytes(($receipt|ConvertTo-Json -Compress));Save-RestartWire (Join-Path $transition 'receipt.json') $wire
 $permitPath=Publish-RestartPermit $transition @{owner=$script:RestartOwner;session=$session.session;recordSha256=$session.recordSha256;status='consumed-before-allow';beforeSha256=$session.beforeIniSha256;profileSha256=(Get-Digest $post);requestSha256=$requestHash;mode='--legacy';permitId=('1'*64)}
 Write-Record (Join-Path $transition 'completion.json') @{owner=$script:RestartOwner;status='child-identity-verified';session=$session.session;recordSha256=$session.recordSha256;permitSha256=(Get-Digest $permitPath);receiptSha256=(Get-Digest (Join-Path $transition 'receipt.json'));requestSha256=$requestHash}
 $child=[pscustomobject]@{pid='44';createdFiletime='133000000002000000'}
 $next=Get-RestartBaseline $session (Join-Path $chain 'session.json') $child
 Check ($next.path -ceq $post) 'verified next child uses last reviewed post-close bytes'
 $delta=@(Assert-RestartIniDelta (Read-RestartIni $next.path) (Read-RestartIni $next.path) '--safe-mode');Check ($delta.Count -eq 0) 'Legacy to Safe keeps previous accepted mode baseline'
 Refuse {Get-RestartBaseline $session (Join-Path $chain 'session.json') $parent} 'old parent cannot reuse completed chain'
 [IO.File]::AppendAllText($post,"[Plugins/pilot]`nbEnabled=1`n")
 Refuse {Get-RestartBaseline $session (Join-Path $chain 'session.json') $child} 'post-close copy tampering rejected'
}finally{Remove-Item -LiteralPath $script:temporary -Recurse -Force}
[pscustomobject]@{status='passed';checks=$script:checks;environment=$(if($env:GITHUB_ACTIONS -ceq 'true'){'CI'}elseif([Environment]::OSVersion.Platform -eq 'Win32NT'){'isolated local Windows'}else{'portable Linux'});nativePipeExecuted=$false;applicationLaunched=$false;physicalOutput=$false}|ConvertTo-Json

# Pure identity/protocol checks; Windows additionally uses disposable fake pipes.
# No vendor executable, OpenCPN, profile, chart, equipment or boat access.
[CmdletBinding()]
param([switch]$IsolatedLocal)
. (Join-Path $PSScriptRoot 'ChartHelperShutdown.ps1')
$native=[Environment]::OSVersion.Platform -eq 'Win32NT'
if($native -and -not $IsolatedLocal -and $env:GITHUB_ACTIONS -cne 'true'){throw 'Native temporary tests require CI or explicit IsolatedLocal.'}
Add-Type -Path (Join-Path $PSScriptRoot 'ChartHelperShutdownNative.cs')
$checks=New-Object 'Collections.Generic.List[string]'
function Check([bool]$Value,[string]$Name){if(-not $Value){throw $Name};$checks.Add($Name)}
function Refuse([string]$Name,[scriptblock]$Action){$failed=$false;try{$null=&$Action}catch{$failed=$true};Check $failed $Name}
function Clone($Value){return ($Value|ConvertTo-Json -Depth 8|ConvertFrom-Json)}
$at=[datetime]::UtcNow.AddMinutes(-2)
$launch=[pscustomobject]@{pid=24640;sessionId=1;sid='S-1-5-21-123';processStartedUtc=$at.AddMinutes(-1).ToString('o')}
$helper=[pscustomobject]@{pid=27228;parentPid=24640;sessionId=1;sid=$launch.sid;startedUtc=$at.ToString('o');path='C:\fixture\oexserverd.exe';commandLine='"C:\fixture\oexserverd.exe" -p OCPN4640'}
Assert-ChartHelperIdentity $helper $launch 27228 $at.Ticks $helper.path $script:ChartHelperHash
Check $true 'Exact helper creation/parent/session/SID/path/hash and derived pipe arguments accepted'
foreach($field in @('pid','parentPid','sessionId','sid','startedUtc','path','commandLine')) {
 Refuse ('Changed helper identity '+$field) {$bad=Clone $helper;switch($field){
  'pid' {$bad.pid++};'parentPid' {$bad.parentPid++};'sessionId' {$bad.sessionId++};'sid' {$bad.sid+='1'}
  'startedUtc' {$bad.startedUtc=$at.AddTicks(1).ToString('o')};'path' {$bad.path='C:\other\oexserverd.exe'};'commandLine' {$bad.commandLine+=' -d'}
 };Assert-ChartHelperIdentity $bad $launch 27228 $at.Ticks $helper.path $script:ChartHelperHash}
}
foreach($badCommand in @('oexserverd.exe -p OCPN4640','"C:\fixture\oexserverd.exe" -p OCPN4641','"C:\fixture\oexserverd.exe" -p mynamedpipe','"C:\fixture\oexserverd.exe" -p OCPN4640 -s')){
 Refuse 'Different or extra source command arguments refused' {$bad=Clone $helper;$bad.commandLine=$badCommand;Assert-ChartHelperIdentity $bad $launch 27228 $at.Ticks $helper.path $script:ChartHelperHash}
}
Refuse 'Unknown helper bytes refused' {Assert-ChartHelperIdentity $helper $launch 27228 $at.Ticks $helper.path ('f'*64)}
Refuse 'Helper predating reviewed parent refused' {$bad=Clone $helper;$bad.startedUtc=$at.AddHours(-1).ToString('o');Assert-ChartHelperIdentity $bad $launch 27228 ([datetime]::Parse($bad.startedUtc).Ticks) $helper.path $script:ChartHelperHash}
$flags=[Reflection.BindingFlags]'NonPublic,Static';$type=[OpenNavX.ChartHelperShutdownNative]
$packet=$type.GetMethod('ExitPacket',$flags).Invoke($null,@())
Check ($packet.Length -eq 1025 -and $packet[0] -eq 2 -and @($packet|Select-Object -Skip 1|Where-Object {$_ -ne 0}).Count -eq 0) 'Exact all-char native protocol packet with zeroed empty/reserved fields'
Check ($type::HelperSha256 -ceq $script:ChartHelperHash) 'Both fixed helper binary boundaries agree'
$public=@($type.GetMethods([Reflection.BindingFlags]'Public,Static')|Where-Object {$_.DeclaringType -eq $type})
Check ($public.Count -eq 1 -and $public[0].Name -ceq 'Shutdown' -and $public[0].GetParameters().Count -eq 5) 'Only one fixed operation is public; no pipe/payload/hash/deadline override'
foreach($file in @('ChartHelperShutdown.ps1','stop-chart-helper.ps1','test-chart-helper-shutdown.ps1','../../tests/chart-helper/pipe-fixture.ps1')){
 $tokens=$null;$errors=$null;$null=[Management.Automation.Language.Parser]::ParseFile((Join-Path $PSScriptRoot $file),[ref]$tokens,[ref]$errors)
 Check ($errors.Count -eq 0) ('Parses '+$file)
}
$nativeCases=New-Object 'Collections.Generic.List[object]'
if($native) {
 $root=Join-Path ([IO.Path]::GetTempPath()) ('OpenNav chart pipe '+[guid]::NewGuid().ToString('N'));$null=New-Item -ItemType Directory $root
 $method=$type.GetMethod('StopExact',$flags);$parentProcess=[Diagnostics.Process]::GetCurrentProcess();$parentPid=$parentProcess.Id
 $shell=$parentProcess.MainModule.FileName;$fixture=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../../tests/chart-helper/pipe-fixture.ps1'))
 try {
  foreach($case in @('normal','short','no-reply','nonzero','timeout','arbitrary-reply','wrong-start','wrong-hash','wrong-path','wrong-peer')) {
   $directory=Join-Path $root $case;$null=New-Item -ItemType Directory $directory
   $behaviour=if($case.StartsWith('wrong-')){'normal'}else{$case}
   $start=New-Object Diagnostics.ProcessStartInfo;$start.FileName=$shell;$start.UseShellExecute=$false;$start.CreateNoWindow=$true
   $start.Arguments='-NoProfile -ExecutionPolicy Bypass -File "'+$fixture+'" -Directory "'+$directory+'" -ParentProcessId '+$parentPid+' -Behaviour '+$behaviour
   $child=[Diagnostics.Process]::Start($start)
   try {
    $null=$child.Handle;$deadline=[datetime]::UtcNow.AddSeconds(20)
    while(-not (Test-Path -LiteralPath (Join-Path $directory 'ready')) -and -not $child.HasExited -and [datetime]::UtcNow -lt $deadline){Start-Sleep -Milliseconds 50}
    if(-not (Test-Path -LiteralPath (Join-Path $directory 'ready'))){throw ('Native fake pipe did not become ready: '+$case)}
    $target=$child;if($case -ceq 'wrong-peer'){$target=$parentProcess}
    $processPath=$target.MainModule.FileName;$ticks=$target.StartTime.ToUniversalTime().Ticks;$digest=Get-Digest $processPath
    if($case -ceq 'wrong-start'){$ticks++};if($case -ceq 'wrong-hash'){$digest='f'*64};if($case -ceq 'wrong-path'){$processPath+='-changed'}
    # Wrong-peer must reach the peer check, so use a different derived parent
    # whose modulo10000 pipe suffix still equals this fixture's parent.
    $boundParent=$parentPid;if($case -ceq 'wrong-peer'){$boundParent=$parentPid+10000}
    $result=$method.Invoke($null,[object[]]@([int]$target.Id,[long]$ticks,[int]$boundParent,[int]$target.SessionId,$processPath,$digest,[int]1500))
    if($case -cin @('normal','arbitrary-reply')) {
      Check ($result.Succeeded -and $result.WriteCompleted -and $result.ReplyBytes -eq 3 -and -not $result.ReplyMeaningValidated -and $result.ExitCodeKnown -and $result.ExitCode -eq 0) ('Exact native fake-pipe exit observed: '+$case)
    } else {
      Check (-not $result.Succeeded) ('Native failure stays refused: '+$case)
      if($case.StartsWith('wrong-')){Check (-not $result.WriteAttempted) ('Wrong identity never writes: '+$case)}
      if($case -ceq 'wrong-peer'){Check ($result.Stage -ceq 'peer') 'A modulo-colliding pipe name cannot bypass actual server PID'}
      if($case -ceq 'nonzero'){Check ($result.ExitCodeKnown -and $result.ExitCode -eq 17) 'Measured nonzero helper exit remains explicit'}
      if($case -ceq 'timeout'){Check ($result.WriteAttempted -and $result.WriteCompleted -and -not $result.ExitObserved -and $result.IoCancellationRequested -and $result.PendingIoDrained) 'Reply timeout cancels/drains own pipe I/O and retains delivery uncertainty without retry'}
    }
    $nativeCases.Add([pscustomobject]@{case=$case;result=$result})
   } finally {
    [IO.File]::WriteAllText((Join-Path $directory 'release'),'normal fixture cleanup')
    if(-not $child.WaitForExit(15000)){throw ('Owned pipe fixture did not close normally; no force termination: '+$case)}
    $child.Dispose()
   }
   $received=Join-Path $directory 'received'
   if(Test-Path -LiteralPath $received){$packetProof=[IO.File]::ReadAllText($received);Check ($packetProof -cin @('1025:exact','0:not-exact')) 'Fixture received only exact CMD_EXIT or zero bytes'}
  }
 } finally {$parentProcess.Dispose();Remove-Item -LiteralPath $root -Recurse -Force}
}
[pscustomobject]@{status='passed';count=$checks.Count;checks=$checks.ToArray();nativeCases=$nativeCases.ToArray();nativePipeExecuted=$native;vendorHelperExecuted=$false;boatAccess=$false;physicalOutput=$false}|ConvertTo-Json -Depth 7

# Pure synthetic bytes and immutable timezones. No application or device access.
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'RendererLog.ps1')
$checks=0
function Check([bool]$Value,[string]$Label){if(-not $Value){throw ('FAILED: '+$Label)};$script:checks++}
function Bytes([string]$Text){return ,([Text.Encoding]::GetEncoding(28591).GetBytes($Text))}
function Line([string]$Time,[string]$Body,[string]$Source='glChartCanvas.cpp'){return ($Time+' MESSAGE '+$Source+':123 '+$Body+"`r`n")}
function Banner([string]$Date='2026-09-27',[string]$Time='03:58:00.100'){return (Line $Time ('------- OpenCPN version 5.12.4-0+37fd0cd restarted at '+$Date+' -------') 'logger.cpp')}
$zone=[TimeZoneInfo]::CreateCustomTimeZone('Fixture plus two',[TimeSpan]::FromHours(2),'Fixture plus two','Fixture plus two')
$start=[DateTimeOffset]::Parse('2026-09-27T01:58:00Z').UtcDateTime;$observed=$start.AddMinutes(2)
$before=Bytes ('unrelated older marine/private content'+"`n")
$session=(Banner)+(Line '03:58:00.200' 'OpenGL-> Renderer String: Intel(R) Example Graphics')+
 (Line '03:58:00.300' 'OpenGL-> Version reported:  4.6.0 - Build 31.0')+
 (Line '03:58:00.400' 'OpenGL-> GLSL Version reported:  4.60')+
 (Line '03:58:00.500' 'OpenGL-> Minimum symbol line width:  1.0')+
 (Line '03:58:00.600' 'OnInitTimer...Finalize Canvases' 'ocpn_frame.cpp')
function Observe([byte[]]$Current,[byte[]]$Old=$before,[byte[]]$Rotated=$null,[datetime]$Started=$start,[datetime]$At=$observed,[TimeZoneInfo]$Zone=$zone,[string]$Hash=''){
 if(-not $Hash){$Hash=Get-RendererBytesHash $Old}
 return Get-RendererLogObservation -Baseline $Old -Current $Current -Rotated $Rotated -BaselineSha256 $Hash -ProcessStartedUtc $Started -ObservedUtc $At -TimeZone $Zone
}
function Appended([string]$Text){return ,([byte[]](@($before)+(Bytes $Text)))}
$result=Observe (Appended $session)
Check ($result.status -ceq 'observed' -and $result.provenance -ceq 'EXACT_APPENDED_PREFIX' -and $result.startupFinalized) 'Exact append links current launch with completed startup'
Check ($result.openGL -ceq 'CANVAS_CONTEXT_INITIALIZED_DURING_LAUNCH' -and $result.contexts.Count -eq 1 -and $result.contexts[0].renderer -ceq 'Intel(R) Example Graphics') 'Actual canvas marker sequence retained'
Check (-not $result.hardwareAccelerationVerified -and -not $result.currentRenderingBackendVerified -and -not $result.chartRenderingVerified) 'Initialization never claims hardware, current backend or chart acceptance'
Check (($result|ConvertTo-Json -Depth 6) -notlike '*unrelated older*') 'Raw historical/private text absent'
Check ((Observe $before).reason -ceq 'NO_NEW_LOG_BYTES') 'Old complete data is not fresh'
Check ((Observe (Appended (Line '03:58:01.000' 'OnInitTimer...Finalize Canvases' 'ocpn_frame.cpp'))).reason -ceq 'MARKER_BEFORE_CURRENT_STARTUP') 'Finalization alone cannot identify startup'
Check ((Observe (Appended $session) -Hash ('f'*64)).reason -ceq 'BASELINE_HASH_MISMATCH') 'Cold copy hash is mandatory'
Check ((Observe (Bytes $session)).reason -ceq 'UNPROVEN_LOG_REPLACEMENT') 'Replaced log cannot inherit old proof'
$large=Bytes ('x'*1000001)
Check ((Observe (Bytes $session) -Old $large -Rotated $large).provenance -ceq 'EXACT_ROTATED_BASELINE') 'Pinned above-threshold rotation proves the exact former log'
Check ((Observe (Bytes $session) -Old $large -Rotated (Bytes ('x'*1000000+'y'))).reason -ceq 'UNPROVEN_LOG_REPLACEMENT') 'One changed rotated byte refuses'
Check ((Observe (Bytes $session) -Old $before -Rotated $before).reason -ceq 'UNPROVEN_LOG_REPLACEMENT') 'Unexplained below-threshold replacement refuses'
Check ((Observe (Bytes $session) -Old (Bytes '')).status -ceq 'observed') 'Explicit empty cold log is valid with exact fresh banner'
Check ((Observe (Appended ($session+(Banner)))).reason -ceq 'DIFFERENT_OR_MULTIPLE_STARTUPS') 'Multiple new startup banners are ambiguous even with same version'
Check ((Observe (Appended ($session.Replace('5.12.4-0+37fd0cd','5.12.4+37fd0cd')))).reason -ceq 'DIFFERENT_OR_MULTIPLE_STARTUPS') 'Installed binary banner cannot pass as official stock'
Check ((Observe (Appended $session) -Zone ([TimeZoneInfo]::Utc)).reason -ceq 'STARTUP_DOES_NOT_MATCH_LAUNCH') 'Local logger clock is not assumed UTC'
Check ((Observe (Appended $session) -Started ($start.AddMinutes(-3))).reason -ceq 'STARTUP_DOES_NOT_MATCH_LAUNCH') 'Old launch does not authorize another session'
Check ((Observe (Appended $session) -At ($start.AddHours(5))).reason -ceq 'INVALID_OBSERVATION_WINDOW') 'Receipt lifetime bounded'
Check ((Observe (Appended $session) -At ($start.AddMilliseconds(350))).reason -ceq 'MARKER_TIME_OUTSIDE_SESSION') 'Future marker refuses rather than mixing observation time'
Check ((Observe (Appended $session.Replace('03:58:00.500','03:57:55.000'))).reason -ceq 'MARKER_TIME_OUTSIDE_SESSION') 'Backward time beyond tolerance refuses'
$midnight=$session.Replace('2026-09-27','2026-09-28').Replace('03:58:00.100','00:00:00.100').Replace('03:58:00.','00:00:00.')
$midStart=[DateTimeOffset]::Parse('2026-09-27T22:00:00Z').UtcDateTime
Check ((Observe (Appended $midnight) -Started $midStart -At $midStart.AddMinutes(1)).status -ceq 'observed') 'Local date and UTC date differ correctly'
$cross=(Banner '2026-09-27' '23:59:59.100')+(Line '00:00:00.100' 'OnInitTimer...Finalize Canvases' 'ocpn_frame.cpp')
$crossStart=[DateTimeOffset]::Parse('2026-09-27T21:59:59Z').UtcDateTime
Check ((Observe (Appended $cross) -Started $crossStart -At $crossStart.AddSeconds(3)).startupFinalized) 'Marker rollover across local midnight stays same session'
foreach($value in @("Intel`0Driver",'C:\Users\Private\driver','bad@example',('unbounded'+('x'*128)),'../sensitive')){
 $bad=Observe (Appended $session.Replace('Intel(R) Example Graphics',$value))
 Check ($bad.status -ceq 'unknown' -and $bad.contexts.Count -eq 0 -and ($bad|ConvertTo-Json -Depth 6) -notlike ('*'+$value+'*')) 'Malformed/unnecessary renderer text never emitted'
}
$capability=(Banner)+(Line '03:58:00.200' 'OpenGL determined CAPABLE.' 'OCPNPlatform.cpp')
$result=Observe (Appended $capability)
Check ($result.capability -ceq 'PROBE_CAPABLE' -and $result.openGL -ceq 'UNKNOWN' -and $result.contexts.Count -eq 0) 'Capability is not canvas initialization'
$result=Observe (Appended ((Banner)+(Line '03:58:00.200' 'Failed to initialize OpenGL')))
Check ($result.openGL -ceq 'INITIALIZATION_FAILED_DURING_LAUNCH') 'Canvas failure remains explicit'
Check ((Observe (Appended ($session.Replace('glChartCanvas.cpp','plugin.cpp')))).reason -ceq 'UNEXPECTED_MARKER_SOURCE') 'Different producer cannot impersonate canvas marker'
Check ((Observe (Appended ((Banner)+(Line '03:58:00.200' 'OpenGL-> Version reported: 4.6')))).reason -ceq 'RENDERER_MARKERS_OUT_OF_ORDER') 'Partial out-of-order context refuses'
Check ((Observe (Appended ((Banner)+(Line '03:58:00.200' 'OpenGL-> Renderer String: GPU')))).openGL -ceq 'CANVAS_CONTEXT_SETUP_INCOMPLETE') 'Renderer alone does not prove late setup'
Check ((Observe (Appended ((Banner)+(Line '03:58:00.200' 'OpenGL-> Renderer String: GPU').TrimEnd("`r","`n")))).contexts.Count -eq 0) 'Incomplete final write excluded'
Check ((Observe (Appended ($session.Replace('1.0','1,0')))).status -ceq 'observed') 'Swedish decimal formatting in late marker supported'
# Fixed custom DST rules avoid platform timezone database IDs.
$dstStart=[TimeZoneInfo+TransitionTime]::CreateFixedDateRule([datetime]'0001-01-01T02:00:00',3,29)
$dstEnd=[TimeZoneInfo+TransitionTime]::CreateFixedDateRule([datetime]'0001-01-01T03:00:00',10,25)
$rule=[TimeZoneInfo+AdjustmentRule]::CreateAdjustmentRule([datetime]'2026-01-01',[datetime]'2026-12-31',[TimeSpan]::FromHours(1),$dstStart,$dstEnd)
$dst=[TimeZoneInfo]::CreateCustomTimeZone('Fixture DST',[TimeSpan]::FromHours(1),'Fixture DST','standard','daylight',[TimeZoneInfo+AdjustmentRule[]]@($rule))
foreach($date in @('2026-03-29','2026-10-25')){
 $stamp=[DateTimeOffset]::Parse($date+'T01:30:00Z').UtcDateTime
 $result=Observe (Appended (Banner $date '02:30:00.000')) -Started $stamp -At $stamp.AddMinutes(1) -Zone $dst
 Check ($result.reason -ceq 'AMBIGUOUS_LOCAL_TIME') 'DST missing/ambiguous local hour cannot fabricate UTC provenance'
}
function Refuse([scriptblock]$Action,[string]$Label){$failed=$false;try{$null=& $Action}catch{$failed=$true};Check $failed $Label}
$stockHash='7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c'
$launch=[pscustomobject]@{status='passed';action='LaunchStock';mode='StockLegacy';arguments='';pid=42;sessionId=1;sid='S-1-5-21-1';executableSha256=$stockHash;targetSha256=('a'*64);utc=$start.AddSeconds(-1).ToString('o');processStartedUtc=$start.ToString('o')}
$request=[pscustomobject]@{action='LaunchStock';mode='StockLegacy';arguments='';executableSha256=$stockHash;targetSha256=('a'*64);workspace='C:\XNav';resultPath='C:\XNav\runs\launch\result.json';executable='C:\OpenCPN\opencpn.exe'}
$config=[pscustomobject]@{stockExecutable=$request.executable;profileDirectory='C:\ProgramData\opencpn'}
function Receipt($L=$launch,$R=$request){return Assert-RendererReceipt $L $R $config $request.workspace $request.resultPath $config.profileDirectory 'S-1-5-21-1' ('a'*64) $observed}
function Clone($Value){return ($Value|ConvertTo-Json|ConvertFrom-Json)}
Check ((Receipt).Ticks -eq $start.Ticks) 'Exact successful official receipt binds process start'
foreach($field in @('status','action','mode','arguments','executableSha256','targetSha256','sid')){
 Refuse {$v=Clone $launch;$v.$field='changed';Receipt $v} ('Altered launch '+$field+' refuses')
}
foreach($field in @('action','mode','arguments','executableSha256','targetSha256','workspace','resultPath','executable')){
 Refuse {$v=Clone $request;$v.$field='changed';Receipt $launch $v} ('Altered request '+$field+' refuses')
}
Refuse {$v=Clone $launch;$v.utc=$start.AddHours(-5).ToString('o');Receipt $v} 'Expired launch receipt refuses'
Refuse {$v=Clone $launch;$v.processStartedUtc=$start.AddSeconds(-5).ToString('o');Receipt $v} 'Start cannot precede launch'
Refuse {Get-RendererUtc '2026-09-27T01:58:00.0000000'} 'Receipt local time needs an explicit offset'
Refuse {Get-RendererUtc ([datetime]::SpecifyKind($start,[DateTimeKind]::Unspecified))} 'Unspecified decoded datetime refuses'
Check ((Get-RendererUtc $start).Ticks -eq $start.Ticks -and (Get-RendererUtc $start.ToString('o')).Ticks -eq $start.Ticks) 'PS5 string and PS7 decoded datetime preserve exact UTC ticks'
$process=[pscustomobject]@{Id=42;Path=$request.executable;SessionId=1;HasExited=$false;StartTime=$start}
Assert-RendererProcess $process $launch $request.executable
Check $true 'Live exact process identity is observed independently of log markers'
foreach($field in @('Id','Path','SessionId','HasExited','StartTime')){
 Refuse {$v=Clone $process;$v.$field=switch($field){'Id'{43};'Path'{'C:\Other\opencpn.exe'};'SessionId'{2};'HasExited'{$true};'StartTime'{$start.AddTicks(1)}};Assert-RendererProcess $v $launch $request.executable} ('Reused/changed process '+$field+' refuses')
}
Check ((Observe (Appended ('x'*16385+"`n"+$session))).reason -ceq 'LOG_RECORD_LIMIT') 'Single huge log record refused without emitting it'
Check ((Observe (New-Object byte[] 4194305)).reason -ceq 'LOG_SIZE_LIMIT') 'Parser preserves shared bounded-read limit'
foreach($file in @('RendererLog.ps1','observe-stock-renderer.ps1')){
 $tokens=$null;$errors=$null;$null=[Management.Automation.Language.Parser]::ParseFile((Join-Path $PSScriptRoot $file),[ref]$tokens,[ref]$errors)
 Check ($errors.Count -eq 0) ('Parses '+$file)
}
[pscustomobject]@{status='passed';count=$checks;nativeInput=$false;rawMarineLogUsed=$false;applicationLaunched=$false;boatAccess=$false;hardwareAccelerationAcceptance=$false}|ConvertTo-Json

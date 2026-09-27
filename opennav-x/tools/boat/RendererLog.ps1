# Bounded read-only evidence parser. Never returns raw log lines or source paths.
. (Join-Path $PSScriptRoot 'StartupLog.ps1')
function Get-RendererBytesHash([byte[]]$Bytes) {
  $sha=[Security.Cryptography.SHA256]::Create()
  try { return ([BitConverter]::ToString($sha.ComputeHash($Bytes))).Replace('-','').ToLowerInvariant() }
  finally { $sha.Dispose() }
}
function Get-RendererUtc([object]$Value) {
  if ($Value -is [datetime]) {
    if ($Value.Kind -eq [DateTimeKind]::Unspecified) { throw 'AMBIGUOUS_RECEIPT_TIME' }
    return $Value.ToUniversalTime()
  }
  if ($Value -isnot [string] -or $Value -cnotmatch '^[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}\.[0-9]{7}(Z|[+-][0-9]{2}:[0-9]{2})$') { throw 'AMBIGUOUS_RECEIPT_TIME' }
  return [DateTimeOffset]::ParseExact($Value,'o',[Globalization.CultureInfo]::InvariantCulture,[Globalization.DateTimeStyles]::None).UtcDateTime
}
function Assert-RendererReceipt($Launch,$Request,$Config,[string]$Workspace,[string]$ReceiptPath,[string]$Profile,[string]$Sid,[string]$TargetHash,[datetime]$Now) {
  if ($Launch.status -cne 'passed' -or $Launch.action -cne 'LaunchStock' -or $Launch.mode -cne 'StockLegacy' -or
      $Launch.arguments -isnot [string] -or $Launch.arguments.Length -ne 0 -or $Launch.pid -le 0 -or $Launch.sessionId -le 0 -or $Launch.sid -cne $Sid -or
      $Launch.executableSha256 -cne '7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c' -or
      $Request.action -cne 'LaunchStock' -or $Request.mode -cne 'StockLegacy' -or $Request.arguments -isnot [string] -or $Request.arguments.Length -ne 0 -or
      $Request.executableSha256 -cne $Launch.executableSha256 -or $Request.targetSha256 -cne $Launch.targetSha256 -or
      $TargetHash -cnotmatch '^[a-f0-9]{64}$' -or $TargetHash -cne $Launch.targetSha256 -or
      $Request.resultPath -ine $ReceiptPath -or $Request.workspace -ine $Workspace -or $Request.executable -ine $Config.stockExecutable -or $Config.profileDirectory -ine $Profile) { throw 'STOCK_LAUNCH_PROOF_MISMATCH' }
  $at=Get-RendererUtc $Launch.utc;$start=Get-RendererUtc $Launch.processStartedUtc
  if ($Now.Kind -ne [DateTimeKind]::Utc -or $at -gt $Now -or $start -lt $at -or $start -gt $Now -or ($Now-$at).TotalHours -gt 4) { throw 'LAUNCH_RECEIPT_EXPIRED' }
  return $start
}
function Assert-RendererProcess($Process,$Launch,[string]$Executable) {
  if ($Process.HasExited -or $Process.Id -ne $Launch.pid -or $Process.Path -ine $Executable -or
      $Process.SessionId -ne $Launch.sessionId -or $Process.StartTime.ToUniversalTime().Ticks -ne (Get-RendererUtc $Launch.processStartedUtc).Ticks) { throw 'PROCESS_IDENTITY_CHANGED' }
}
function New-RendererObservation {
  return [ordered]@{schema=1;status='unknown';reason='UNOBSERVED';provenance='UNKNOWN';startupFinalized=$false;
    openGL='UNKNOWN';capability='NOT_OBSERVED';contexts=@();hardwareAccelerationVerified=$false;currentRenderingBackendVerified=$false;
    chartRenderingVerified=$false;rawLogIncluded=$false;unrelatedPathsIncluded=$false}
}
function Convert-RendererLogTime([datetime]$Local,[TimeZoneInfo]$Zone) {
  $localTime=[datetime]::SpecifyKind($Local,[DateTimeKind]::Unspecified)
  if ($Zone.IsInvalidTime($localTime) -or $Zone.IsAmbiguousTime($localTime)) { throw 'AMBIGUOUS_LOCAL_TIME' }
  return [TimeZoneInfo]::ConvertTimeToUtc($localTime,$Zone)
}
function Get-RendererLogObservation {
  param([byte[]]$Baseline,[byte[]]$Current,[byte[]]$Rotated,[string]$BaselineSha256,
    [datetime]$ProcessStartedUtc,[datetime]$ObservedUtc,[TimeZoneInfo]$TimeZone)
  $result=New-RendererObservation
  try {
    foreach($bytes in @($Baseline,$Current,$Rotated)) { if ($null -ne $bytes -and $bytes.Length -gt 4194304) { throw 'LOG_SIZE_LIMIT' } }
    if ($null -eq $Baseline -or $null -eq $Current -or $null -eq $TimeZone -or $BaselineSha256 -cnotmatch '^[a-f0-9]{64}$') { throw 'MISSING_PROVENANCE' }
    if ((Get-RendererBytesHash $Baseline) -cne $BaselineSha256) { throw 'BASELINE_HASH_MISMATCH' }
    if ($ProcessStartedUtc.Kind -ne [DateTimeKind]::Utc -or $ObservedUtc.Kind -ne [DateTimeKind]::Utc -or
        $ProcessStartedUtc -gt $ObservedUtc -or ($ObservedUtc-$ProcessStartedUtc).TotalHours -gt 4) { throw 'INVALID_OBSERVATION_WINDOW' }
    $result.baselineLogSha256=$BaselineSha256;$result.baselineBytes=$Baseline.Length
    $result.currentLogSha256=Get-RendererBytesHash $Current;$result.currentBytes=$Current.Length
    $result.processStartedUtc=$ProcessStartedUtc.ToString('o');$result.observedUtc=$ObservedUtc.ToString('o');$result.timeZoneId=$TimeZone.Id
    $encoding=[Text.Encoding]::GetEncoding(28591)
    $old=$encoding.GetString($Baseline);$now=$encoding.GetString($Current)
    if ($now.StartsWith($old,[StringComparison]::Ordinal)) {
      $fresh=$now.Substring($old.Length);$provenance='EXACT_APPENDED_PREFIX'
    } elseif ($Baseline.Length -gt 1000000 -and $null -ne $Rotated -and (Get-RendererBytesHash $Rotated) -ceq $BaselineSha256) {
      $fresh=$now;$provenance='EXACT_ROTATED_BASELINE';$result.rotatedLogSha256=$BaselineSha256
    } else { throw 'UNPROVEN_LOG_REPLACEMENT' }
    if ($fresh.Length -eq 0) { throw 'NO_NEW_LOG_BYTES' }
    # Only complete logger records are considered. Partial final writes are not
    # promoted to evidence. Latin1 preserves exact byte offsets and ASCII markers.
    $end=$fresh.LastIndexOf("`n",[StringComparison]::Ordinal)
    if ($end -lt 0) { throw 'NO_COMPLETE_LOG_RECORD' }
    $reader=New-Object IO.StringReader($fresh.Substring(0,$end+1))
    $records=New-Object 'Collections.Generic.List[object]';$banners=0;$lines=0
    try {
      while ($null -ne ($line=$reader.ReadLine())) {
        if (++$lines -gt 65536 -or $line.Length -gt 16384) { throw 'LOG_RECORD_LIMIT' }
        if ($line -notmatch '^(?<time>[0-9]{2}:[0-9]{2}:[0-9]{2}\.[0-9]{3}) +(?<level>[A-Z]+) +(?<source>[A-Za-z0-9_.-]+\.cpp):[0-9]{1,7} (?<body>.*)$') {
          if ($line.Contains('------- OpenCPN version ')) { throw 'UNRECOGNIZED_STARTUP_RECORD' }
          continue
        }
        $stamp=$Matches.time;$source=$Matches.source;$body=$Matches.body
        if ($body.StartsWith('------- OpenCPN version ',[StringComparison]::Ordinal)) {
          $banners++
          if ($banners -ne 1 -or $source -cne 'logger.cpp' -or $body -cnotmatch '^------- OpenCPN version 5\.12\.4-0\+37fd0cd restarted at (?<date>[0-9]{4}-[0-9]{2}-[0-9]{2}) -------$') { throw 'DIFFERENT_OR_MULTIPLE_STARTUPS' }
          $dateText=$Matches.date
          $local=[datetime]::ParseExact(($dateText+' '+$stamp),'yyyy-MM-dd HH:mm:ss.fff',[Globalization.CultureInfo]::InvariantCulture,[Globalization.DateTimeStyles]::None)
          $start=Convert-RendererLogTime $local $TimeZone
          if (($start-$ProcessStartedUtc).TotalSeconds -lt -1 -or ($start-$ProcessStartedUtc).TotalSeconds -gt 120 -or $start -gt $ObservedUtc) { throw 'STARTUP_DOES_NOT_MATCH_LAUNCH' }
          $result.startupUtc=$start.ToString('o');$result.startupLocal=$local.ToString('yyyy-MM-ddTHH:mm:ss.fff',[Globalization.CultureInfo]::InvariantCulture)
          $day=$local.Date;$previous=$start;$previousClock=$local.TimeOfDay
          continue
        }
        $recognized=$body -match '^(OpenGL-> (Renderer String:|Version reported:|GLSL Version reported:|Minimum symbol line width:)|OnInitTimer\.\.\.Finalize Canvases$|Failed to initialize OpenGL$|OpenGL determined CAPABLE\.$|BuildGLCaps fails\.$)'
        if (-not $recognized) { continue }
        if ($banners -ne 1) { throw 'MARKER_BEFORE_CURRENT_STARTUP' }
        $clock=[datetime]::ParseExact($stamp,'HH:mm:ss.fff',[Globalization.CultureInfo]::InvariantCulture,[Globalization.DateTimeStyles]::None).TimeOfDay
        if (($previousClock-$clock).TotalHours -gt 12) { $day=$day.AddDays(1) }
        $at=Convert-RendererLogTime ($day+$clock) $TimeZone
        if ($at -lt $start -or ($previous-$at).TotalSeconds -gt 1 -or $at -gt $ObservedUtc) { throw 'MARKER_TIME_OUTSIDE_SESSION' }
        $previous=$at;$previousClock=$clock
        if ($records.Count -ge 128) { throw 'RENDERER_MARKER_LIMIT' }
        $records.Add([pscustomobject]@{body=$body;source=$source})
      }
    } finally { $reader.Dispose() }
    if ($banners -ne 1) { throw 'NO_CURRENT_STARTUP' }
    $contexts=New-Object 'Collections.Generic.List[object]';$context=$null;$failed=$false
    foreach($record in $records) {
      $body=$record.body
      if ($body -ceq 'OnInitTimer...Finalize Canvases') {
        if ($record.source -cne 'ocpn_frame.cpp') { throw 'UNEXPECTED_MARKER_SOURCE' };$result.startupFinalized=$true;continue
      }
      if ($body -cin @('OpenGL determined CAPABLE.','BuildGLCaps fails.')) {
        if ($record.source -cne 'OCPNPlatform.cpp') { throw 'UNEXPECTED_MARKER_SOURCE' }
        $result.capability=if($body -ceq 'OpenGL determined CAPABLE.'){'PROBE_CAPABLE'}else{'PROBE_FAILED'};continue
      }
      if ($record.source -cne 'glChartCanvas.cpp') { throw 'UNEXPECTED_MARKER_SOURCE' }
      if ($body -ceq 'Failed to initialize OpenGL') { $failed=$true;continue }
      if ($body -cmatch '^OpenGL-> (?<name>Renderer String|Version reported|GLSL Version reported): +(?<value>.*)$') {
        $name=$Matches.name;$value=$Matches.value.Trim()
        if ($value -cnotmatch '^[A-Za-z0-9 ()/.,+_-]{1,128}$' -or $value.Contains('..')) { throw 'UNSAFE_OR_MALFORMED_RENDERER_VALUE' }
        if ($name -ceq 'Renderer String') {
          if ($contexts.Count -ge 8) { throw 'RENDERER_CONTEXT_LIMIT' }
          $context=[ordered]@{renderer=$value;version=$null;glslVersion=$null;lateSetupObserved=$false};$contexts.Add($context)
        } else {
          if ($null -eq $context) { throw 'RENDERER_MARKERS_OUT_OF_ORDER' }
          $key=if($name -ceq 'Version reported'){'version'}else{'glslVersion'}
          if ($null -ne $context[$key]) { throw 'DUPLICATE_RENDERER_MARKER' };$context[$key]=$value
        }
      } elseif ($body -cmatch '^OpenGL-> Minimum symbol line width: +[0-9]{1,3}[.,][0-9]$') {
        if ($null -eq $context -or -not $context.version -or -not $context.glslVersion) { throw 'RENDERER_MARKERS_OUT_OF_ORDER' };$context.lateSetupObserved=$true
      } else { throw 'MALFORMED_RENDERER_MARKER' }
    }
    $result.provenance=$provenance;$result.contexts=$contexts.ToArray();$result.status='observed';$result.reason='CURRENT_LAUNCH_BOUND'
    if ($failed) { $result.openGL='INITIALIZATION_FAILED_DURING_LAUNCH' }
    elseif ($contexts.Count -gt 0) {
      $complete=@($contexts | Where-Object {$_.lateSetupObserved})
      $result.openGL=if($complete.Count -eq $contexts.Count){'CANVAS_CONTEXT_INITIALIZED_DURING_LAUNCH'}else{'CANVAS_CONTEXT_SETUP_INCOMPLETE'}
    }
  } catch {
    $code=$_.Exception.Message
    $result.reason=if($code -cmatch '^[A-Z][A-Z0-9_]{2,64}$'){$code}else{'MALFORMED_OR_UNSUPPORTED_LOG'}
    $result.status='unknown';$result.provenance='UNKNOWN';$result.openGL='UNKNOWN';$result.contexts=@();$result.startupFinalized=$false;$result.capability='NOT_OBSERVED'
    $result.Remove('startupUtc');$result.Remove('startupLocal')
  }
  return [pscustomobject]$result
}

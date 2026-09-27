# Closed policy: only source-reviewed display persistence may differ at restart.
# No process, file, application, transport or registry operation in this module.
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'RestartAuiPersistence.ps1')
. (Join-Path $PSScriptRoot 'RestartDashboardPersistence.ps1')
$script:RestartMagic='OpenNavX.CommissioningRestart.1'
function Get-RestartDisplayKeys {
  $keys=[Collections.Generic.Dictionary[string,string]]::new([StringComparer]::Ordinal)
  foreach($key in @('FrameWinX','FrameWinY','ClientSzX','ClientSzY')){$keys.Add('Settings/GlobalState/'+$key,'size')}
  foreach($key in @('FrameWinPosX','FrameWinPosY','ClientPosX','ClientPosY')){$keys.Add('Settings/GlobalState/'+$key,'position')}
  $keys.Add('Settings/GlobalState/FrameMax','boolean')
  $keys.Add('Settings/GlobalState/nColorScheme','color')
  $keys.Add('Settings/GlobalState/OwnShipLatLon','latlon')
  foreach($key in @('Fullscreen','ShowStatusBar','ShowMenuBar','ShowCompassWindow')){$keys.Add('Settings/'+$key,'boolean')}
  foreach($canvas in @('Canvas/CanvasConfig1/','Canvas/CanvasConfig2/')) {
    $keys.Add($canvas+'canvasInitialdBIndex','chartindex')
    $keys.Add($canvas+'canvasVPLatLon','latlon');$keys.Add($canvas+'canvasVPScale','scale')
    $keys.Add($canvas+'canvasVPRotation','rotation');$keys.Add($canvas+'canvasSizeX','size');$keys.Add($canvas+'canvasSizeY','size')
    foreach($key in @('canvasbFollow','canvasCourseUp','canvasHeadUp','canvasLookahead')){$keys.Add($canvas+$key,'boolean')}
  }
  return ,$keys
}
function Assert-RestartScalar([string]$Kind,[string]$Value) {
  if($Value.Length -gt 128){throw 'Display value exceeds bound.'}
  switch -CaseSensitive ($Kind) {
    'boolean' {if($Value -cnotmatch '^[01]$'){throw 'Expected exact persisted boolean.'};return}
    'latlon' {
      if($Value -cnotmatch '^\s*-?[0-9]+(?:\.[0-9]+)?\s*,\s*-?[0-9]+(?:\.[0-9]+)?\s*$'){throw 'Invalid viewport coordinates.'}
      $parts=$Value.Split(',');$lat=[double]::Parse($parts[0],[Globalization.CultureInfo]::InvariantCulture);$lon=[double]::Parse($parts[1],[Globalization.CultureInfo]::InvariantCulture)
      if([double]::IsNaN($lat) -or [double]::IsInfinity($lat) -or [double]::IsNaN($lon) -or [double]::IsInfinity($lon) -or $lat -lt -90 -or $lat -gt 90 -or $lon -lt -180 -or $lon -gt 180){throw 'Viewport outside geographic bounds.'};return
    }
    'scale' {
      if($Value -cnotmatch '^[0-9]+(?:\.[0-9]+)?(?:[eE][+-]?[0-9]+)?$'){throw 'Invalid viewport scale.'}
      $valueNumber=[double]::Parse($Value,[Globalization.CultureInfo]::InvariantCulture)
      if([double]::IsNaN($valueNumber) -or [double]::IsInfinity($valueNumber) -or $valueNumber -le 0 -or $valueNumber -gt 10000){throw 'Viewport scale outside bounds.'};return
    }
    default {
      if($Value -cnotmatch '^-?(?:0|[1-9][0-9]*)$'){throw 'Expected canonical display integer.'}
      $number=[long]::Parse($Value,[Globalization.CultureInfo]::InvariantCulture)
      switch -CaseSensitive ($Kind) {
        'size' {if($number -lt 1 -or $number -gt 32768){throw 'Display size outside bounds.'}}
        'position' {if($number -lt -32768 -or $number -gt 32768){throw 'Display position outside bounds.'}}
        'rotation' {if($number -lt -359 -or $number -gt 359){throw 'Viewport rotation outside bounds.'}}
        'chartindex' {if($number -lt -1 -or $number -gt 1000000){throw 'Chart reference outside bounds.'}}
        'color' {if($number -lt 1 -or $number -gt 3){throw 'Expected Day/Dusk/Night color scheme.'}}
        default {throw 'Unknown display persistence type.'}
      }
    }
  }
}
function Assert-RestartIniDelta($Before,$After,[string]$Mode) {
  if($Mode -cnotin @('--xnav','--legacy','--safe-mode')){throw 'Invalid target mode.'}
  $original=$Before;$Before=[Collections.Generic.Dictionary[string,string]]::new([StringComparer]::Ordinal)
  foreach($key in $original.Keys){$Before.Add($key,$original[$key])}
  $original=$After;$After=[Collections.Generic.Dictionary[string,string]]::new([StringComparer]::Ordinal)
  foreach($key in $original.Keys){$After.Add($key,$original[$key])}
  $policy=Get-RestartDisplayKeys;$changes=New-Object 'Collections.Generic.List[object]'
  $all=New-Object 'Collections.Generic.HashSet[string]' ([StringComparer]::Ordinal)
  foreach($key in @($Before.Keys)+@($After.Keys)){$null=$all.Add($key)}
  foreach($key in $all) {
    $was=$Before.ContainsKey($key);$is=$After.ContainsKey($key)
    if($was -and $is -and $Before[$key] -ceq $After[$key]){continue}
    if(-not $is){throw ('Removed profile key requires review: '+$key)}
    if($key -ceq 'OpenNav/InterfaceMode') {
      if($Mode -ceq '--safe-mode' -or $After[$key] -cne $Mode.Substring(2)){throw 'Persisted interface does not match explicit restart.'}
      if($was -and $Before[$key] -cnotin @('xnav','legacy')){throw 'Original persisted interface invalid.'}
    } elseif($key -ceq 'AUI/AUIPerspective') {
      if(-not $was){throw 'AUI needs a separately reviewed existing baseline.'}
      Assert-RestartAuiDelta $Before[$key] $After[$key]
    } elseif($key -ceq 'PlugIns/Dashboard/SumLogNM' -or $key -cmatch '^PlugIns/Dashboard/Dashboard(?:[1-9]|1[0-9]|20)/PersistSize[XY]$') {
      if(-not $was){throw 'Dashboard persistence needs an existing reviewed baseline.'}
      Assert-RestartDashboardDelta $key $Before[$key] $After[$key]
    } else {
      if(-not $policy.ContainsKey($key)){throw ('Unreviewed profile delta: '+$key)}
      Assert-RestartScalar $policy[$key] $After[$key]
      if($was){Assert-RestartScalar $policy[$key] $Before[$key]}
    }
    $changes.Add([pscustomobject]@{key=$key;before=$(if($was){$Before[$key]}else{$null});after=$After[$key]})
  }
  if($Mode -cne '--safe-mode' -and $After['OpenNav/InterfaceMode'] -cne $Mode.Substring(2)){throw 'Target mode was not persisted by normal close.'}
  return $changes.ToArray()
}
function Assert-RestartDecimal($Value,[string]$Label,[bool]$AllowZero=$false) {
  if($Value -isnot [string] -or $Value -cnotmatch '^(?:0|[1-9][0-9]{0,19})$'){throw ('Invalid canonical identity: '+$Label)}
  $number=[UInt64]::Parse($Value,[Globalization.CultureInfo]::InvariantCulture)
  if(-not $AllowZero -and $number -eq 0){throw ('Zero identity: '+$Label)}
}
function Assert-RestartRequest($Request,$Session,$Parent,[string]$Mode,[string]$PeerPid) {
  $required=@('protocol','kind','session','recordSha256','nonce','parentPid','parentCreatedFiletime','parentExitCode','helperPid','helperCreatedFiletime','windowsSessionId','executable','executableSha256','helper','helperSha256','workingDirectory','path','arguments')
  if($Request.Count -ne $required.Count -or @($required | Where-Object {-not $Request.ContainsKey($_)}).Count){throw 'Request schema differs.'}
  if($Request['protocol'] -isnot [int] -or $Request['protocol'] -ne 1 -or $Request['kind'] -cne 'request'){throw 'Unknown restart request.'}
  foreach($key in @('session','recordSha256','nonce','executableSha256','helperSha256')){if($Request[$key] -isnot [string] -or $Request[$key] -cnotmatch '^[a-f0-9]{64}$'){throw 'Invalid exact hash/session identity.'}}
  foreach($key in @('parentPid','parentCreatedFiletime','helperPid','helperCreatedFiletime','windowsSessionId')){Assert-RestartDecimal $Request[$key] $key}
  if($Request['parentExitCode'] -cne '0' -or $Request['parentPid'] -cne $Parent.pid -or $Request['parentCreatedFiletime'] -cne $Parent.createdFiletime -or
    $Request['helperPid'] -cne $PeerPid -or $Request['windowsSessionId'] -cne $Session.windowsSessionId){throw 'Parent/peer/session identity differs.'}
  foreach($key in @('session','recordSha256','executable','executableSha256','helper','helperSha256','workingDirectory')) {
    if($Request[$key] -isnot [string] -or $Request[$key] -cne $Session.$key){throw ('Request differs from cold identity: '+$key)}
  }
  # Native startup captures the clean environment before plugins can extend it.
  if($Request['path'] -isnot [string] -or $Request['path'] -cne $Session.path){throw 'Inherited search path differs from the immutable cold environment.'}
  if($Request['arguments'] -isnot [string[]] -or $Request['arguments'].Length -ne 1 -or $Request['arguments'][0] -cne $Mode -or $Mode -cnotin @('--xnav','--legacy','--safe-mode')){throw 'Only the one explicitly armed mode is permitted.'}
}
function Assert-RestartTaskIdentity($Task,$Arm,[string]$Sid) {
  if($Task.State.ToString() -cne 'Ready' -or @($Task.Actions).Count -ne 1 -or
     $Task.Actions[0].Execute -cne $Arm.execute -or $Task.Actions[0].Arguments -cne $Arm.arguments -or $Task.Actions[0].WorkingDirectory -or
     $Task.Principal.UserId -cne $Sid -or $Task.Principal.RunLevel.ToString() -cnotin @('Limited','0') -or
     $Task.Principal.LogonType.ToString() -cnotin @('Interactive','3') -or @($Task.Triggers).Count -ne 0){
    $diagnostic=@{state=$Task.State.ToString();actionCount=@($Task.Actions).Count;
      actions=@($Task.Actions|Select-Object Execute,Arguments,WorkingDirectory);
      principal=@{userId=$Task.Principal.UserId;runLevel=$Task.Principal.RunLevel.ToString();logonType=$Task.Principal.LogonType.ToString()};
      triggerCount=@($Task.Triggers).Count;triggersNull=($null -eq $Task.Triggers);
      expected=@{execute=$Arm.execute;arguments=$Arm.arguments;sid=$Sid}}
    throw ('Broker task running or differs from the exact owned limited interactive action: '+($diagnostic|ConvertTo-Json -Depth 5 -Compress))
  }
}

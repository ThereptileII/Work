# Pure, closed wxWidgets 3.2.8 SavePerspective persistence policy. No I/O.
# Raw INI values are decoded through the specific wxFileConfig layer first.
Set-StrictMode -Version Latest
function ConvertFrom-RestartAuiIni([string]$Raw) {
  if($Raw.Length -lt 9 -or $Raw.Length -gt 65536 -or $Raw -match '[\x00-\x1f\x7f]' -or -not $Raw.StartsWith('layout2|')){throw 'Unsupported AUI persistence.'}
  $value=[Text.StringBuilder]::new()
  for($i=0;$i -lt $Raw.Length;$i++) {
    if($Raw[$i] -eq '\') {
      $i++
      # SavePerspective starts with layout2, so wxFileConfig never quotes it.
      # Its only admissible escape here is a doubled literal backslash.
      if($i -ge $Raw.Length -or $Raw[$i] -ne '\'){throw 'Unsupported AUI INI escape.'}
    }
    $null=$value.Append($Raw[$i])
  }
  return $value.ToString()
}
function Get-RestartAuiInteger([string]$Text,[long]$Minimum,[long]$Maximum) {
  if($Text -cnotmatch '^(?:0|[1-9][0-9]*|-[1-9][0-9]*)$' -or $Text.Length -gt 11){throw 'Noncanonical AUI number.'}
  $number=[long]::Parse($Text,[Globalization.CultureInfo]::InvariantCulture)
  if($number -lt $Minimum -or $number -gt $Maximum){throw 'AUI number outside bounds.'}
  return $number
}
function Read-RestartAui([string]$Raw) {
  $value=ConvertFrom-RestartAuiIni $Raw
  if(-not $value.EndsWith('|')){throw 'Incomplete AUI perspective.'}
  # Match wxAuiManager's delimiter escaping, not PowerShell regex splitting.
  $value=$value.Replace('\|',[string][char]7).Replace('\;',[string][char]8)
  $records=$value.Split('|')
  if($records.Count -gt 258 -or $records[0] -cne 'layout2' -or $records[-1] -cne ''){throw 'AUI record bounds differ.'}
  $panes=[Collections.Generic.Dictionary[string,object]]::new([StringComparer]::Ordinal)
  $docks=[Collections.Generic.Dictionary[string,long]]::new([StringComparer]::Ordinal)
  $fields=@('name','caption','state','dir','layer','row','pos','prop','bestw','besth','minw','minh','maxw','maxh','floatx','floaty','floatw','floath')
  $knownState=[uint64]0
  foreach($bit in @(0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,21,22,23,24,26,27,28,30,31)){$knownState=$knownState -bor ([uint64]1 -shl $bit)}
  $haveDocks=$false
  for($i=1;$i -lt $records.Count-1;$i++) {
    $record=$records[$i]
    if($record.StartsWith('dock_size(')) {
      $haveDocks=$true
      if($record -cnotmatch '^dock_size\(([0-9]+),([0-9]+),([0-9]+)\)=([0-9]+)$'){throw 'Malformed AUI dock.'}
      $dir=Get-RestartAuiInteger $Matches[1] 1 5
      $layer=Get-RestartAuiInteger $Matches[2] 0 128
      $row=Get-RestartAuiInteger $Matches[3] 0 128
      $size=Get-RestartAuiInteger $Matches[4] 1 32768
      $key="$dir,$layer,$row"
      if($docks.ContainsKey($key)){throw 'Duplicate AUI dock.'};$docks.Add($key,$size)
      continue
    }
    if($haveDocks -or -not $record.StartsWith('name=')){throw 'Unknown AUI record.'}
    $parts=$record.Split(';');if($parts.Count -ne $fields.Count){throw 'AUI pane fields differ.'}
    $pane=[Collections.Generic.Dictionary[string,object]]::new([StringComparer]::Ordinal)
    for($f=0;$f -lt $fields.Count;$f++) {
      $prefix=$fields[$f]+'='
      if(-not $parts[$f].StartsWith($prefix)){throw 'Duplicate, reordered or unknown AUI field.'}
      $text=$parts[$f].Substring($prefix.Length)
      if($f -lt 2) {
        $text=$text.Replace([string][char]7,'|').Replace([string][char]8,';')
        if($text.Length -gt 1024 -or $text -cne $text.Trim() -or ($f -eq 0 -and -not $text)){throw 'Ambiguous AUI pane identity.'}
        $pane.Add($fields[$f],$text)
      } else {
        $min=0L;$max=32768L
        switch -CaseSensitive ($fields[$f]) {
          'state' {$max=4294967295L}
          'dir' {$max=5L}
          'layer' {$max=128L}
          'row' {$max=128L}
          'prop' {$max=1000000L}
          {$_ -in @('floatx','floaty')} {$min=-32768L}
          {$_ -in @('bestw','besth','minw','minh','maxw','maxh','floatw','floath')} {$min=-1L}
        }
        $pane.Add($fields[$f],(Get-RestartAuiInteger $text $min $max))
      }
    }
    if(([uint64]$pane['state'] -band (-bnot $knownState)) -ne 0){throw 'Unknown AUI state flag.'}
    if($panes.ContainsKey($pane['name'])){throw 'Duplicate AUI pane identity.'}
    $panes.Add($pane['name'],$pane)
  }
  if(-not $panes.Count -or $panes.Count -gt 128){throw 'AUI pane count outside bounds.'}
  # wxAUI only persists nonempty docks belonging to shown, docked panes.
  foreach($key in $docks.Keys) {
    $found=$false
    foreach($pane in $panes.Values) {
      if(($pane['state'] -band 3) -eq 0 -and $key -ceq ($pane['dir'].ToString()+','+$pane['layer']+','+$pane['row'])){$found=$true;break}
    }
    if(-not $found){throw 'AUI dock has no matching visible pane.'}
  }
  return [pscustomobject]@{panes=$panes;docks=$docks}
}
function Assert-RestartAuiDelta([string]$Before,[string]$After) {
  $old=Read-RestartAui $Before;$new=Read-RestartAui $After
  if($old.panes.Count -ne $new.panes.Count){throw 'AUI pane membership changed.'}
  # Floating, hidden, active, maximized and saved hidden status are UI state.
  # Capabilities, destroy-on-close, buttons and actionPane remain unchanged.
  $mutable=[uint64]0
  foreach($bit in @(0,1,14,16,30)){$mutable=$mutable -bor ([uint64]1 -shl $bit)}
  foreach($name in $old.panes.Keys) {
    if(-not $new.panes.ContainsKey($name)){throw 'AUI pane identity changed.'}
    $a=$old.panes[$name];$b=$new.panes[$name]
    if($a['caption'] -cne $b['caption']){throw 'AUI pane caption changed.'}
    if((([uint64]$a['state'] -bxor [uint64]$b['state']) -band (-bnot $mutable)) -ne 0){throw 'AUI pane capabilities changed.'}
  }
}

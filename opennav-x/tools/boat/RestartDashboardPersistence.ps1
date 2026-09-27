# Exact read-only Dashboard persistence. No plugin-prefix grants or I/O.
Set-StrictMode -Version Latest
function Assert-RestartDashboardDelta([string]$Key,[string]$Before,[string]$After) {
  if($Key -ceq 'PlugIns/Dashboard/SumLogNM') {
    $numbers=@()
    foreach($text in @($Before,$After)) {
      if($text.Length -gt 64 -or $text -cnotmatch '^(?:0|[1-9][0-9]*)(?:\.[0-9]+)?(?:[eE][+-]?[0-9]+)?$'){throw 'Invalid Dashboard distance counter.'}
      $number=[double]::Parse($text,[Globalization.CultureInfo]::InvariantCulture)
      if([double]::IsNaN($number) -or [double]::IsInfinity($number) -or $number -lt 0 -or $number -gt 100000000){throw 'Dashboard distance counter outside bounds.'}
      $numbers+=,$number
    }
    # Four-hour commissioning session, conservatively bounded to 1000 NM.
    # Reset/decrease is an explicit user edit and is never silently accepted.
    if($numbers[1] -lt $numbers[0] -or $numbers[1]-$numbers[0] -gt 1000){throw 'Dashboard distance counter reset or excessive advance.'}
    return
  }
  if($Key -cnotmatch '^PlugIns/Dashboard/Dashboard(?:[1-9]|1[0-9]|20)/PersistSize[XY]$'){throw 'Unreviewed Dashboard key.'}
  foreach($text in @($Before,$After)) {
    if($text -cnotmatch '^(?:0|[1-9][0-9]{0,4})$' -or [long]$text -gt 32768){throw 'Dashboard pane size outside bounds.'}
  }
}

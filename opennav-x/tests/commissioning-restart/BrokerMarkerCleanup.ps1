# Test-only marker protocol. Never terminates a process or touches a real app.
function Assert-BrokerMarkerChildRecord([string[]]$Lines,[string]$Session,[string]$RecordSha256) {
 if($Lines.Count -ne 10 -or $Lines[0] -cnotmatch '^[1-9][0-9]{0,9}$' -or
    $Lines[1] -cnotmatch '^[1-9][0-9]{0,19}$' -or $Lines[2] -cne '1' -or
    $Lines[3] -cne $Session -or $Lines[4] -cne $RecordSha256){throw 'Marker child identity/session record differs.'}
 $number=[uint32]0;$created=[uint64]0
 if(-not[uint32]::TryParse($Lines[0],[ref]$number) -or -not[uint64]::TryParse($Lines[1],[ref]$created)){throw 'Marker child numeric identity overflows.'}
}
function Wait-BrokerMarkerChildren([string]$Application,[string]$Executable,[string]$ExecutableSha256,[string]$Session,[string]$RecordSha256) {
 $markers=@(Get-ChildItem -LiteralPath $Application -Filter 'child--*.txt')
 if($markers.Count -gt 1){throw 'Unexpected extra marker children; no cleanup release.'}
 $children=New-Object 'Collections.Generic.List[object]'
 try {
  foreach($marker in $markers) {
   if($marker.Name -cnotin @('child--xnav.txt','child--legacy.txt','child--safe-mode.txt') -or $marker.Attributes -band [IO.FileAttributes]::ReparsePoint){throw 'Unexpected marker child record path.'}
   $lines=[IO.File]::ReadAllLines($marker.FullName);Assert-BrokerMarkerChildRecord $lines $Session $RecordSha256
   $child=Get-Process -Id ([int]$lines[0]);$children.Add($child);$null=$child.Handle
   if($child.HasExited -or -not[StringComparer]::OrdinalIgnoreCase.Equals($child.Path,$Executable) -or
      $child.StartTime.ToUniversalTime().ToFileTimeUtc().ToString() -cne $lines[1] -or
      $child.SessionId -ne [Diagnostics.Process]::GetCurrentProcess().SessionId -or
      (Get-Digest $child.Path) -cne $ExecutableSha256){throw 'Held marker process identity differs; no release.'}
  }
  # Capture genuine child handles BEFORE release. The marker deliberately stays
  # alive another 1200 ms after release; waiting only for its helper races delete.
  [IO.File]::WriteAllText((Join-Path $Application 'child-release.txt'),'fixture cleanup release')
  foreach($child in $children) {
   if(-not $child.WaitForExit(15000) -or $child.ExitCode -ne 0){throw 'Exact marker child did not exit normally; no forced termination.'}
  }
  return $children.Count
 } finally {foreach($child in $children){$child.Dispose()}}
}

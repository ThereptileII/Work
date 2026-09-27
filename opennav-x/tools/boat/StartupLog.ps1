# Read-only readiness observation. Never return or persist raw marine log text.
function Read-StartupLogBytes([string]$Path) {
  if (-not [IO.File]::Exists($Path)) { return ,([byte[]]@()) }
  $limit=4*1024*1024
  $share=[IO.FileShare]::ReadWrite -bor [IO.FileShare]::Delete
  $stream=New-Object IO.FileStream($Path,[IO.FileMode]::Open,[IO.FileAccess]::Read,$share)
  $copy=New-Object IO.MemoryStream
  try {
    if ($stream.Length -gt $limit) { throw 'Startup log exceeds the bounded observation size.' }
    $buffer=New-Object byte[] 16384
    while (($count=$stream.Read($buffer,0,$buffer.Length)) -gt 0) {
      if ($copy.Length+$count -gt $limit) { throw 'Startup log grew beyond the bounded observation size.' }
      $copy.Write($buffer,0,$count)
    }
    return ,($copy.ToArray())
  } finally { $copy.Dispose();$stream.Dispose() }
}

function Test-StartupInitializedSince([byte[]]$Before,[byte[]]$Current) {
  # ISO-8859-1 is a lossless byte-to-character mapping. It avoids interpreting
  # plugin/locale text and permits exact ordinal prefix/ASCII marker matching.
  $encoding=[Text.Encoding]::GetEncoding(28591)
  $old=$encoding.GetString($Before);$now=$encoding.GetString($Current)
  $fresh=if ($now.StartsWith($old,[StringComparison]::Ordinal)) { $now.Substring($old.Length) } else { $now }
  $started=$fresh.LastIndexOf('------- OpenCPN version ',[StringComparison]::Ordinal)
  return $started -ge 0 -and $fresh.IndexOf('OnInitTimer...Finalize Canvases',$started,[StringComparison]::Ordinal) -ge 0
}

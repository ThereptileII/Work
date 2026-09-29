# Test-only closed-marker reader. Not part of the application/restart protocol.
function Read-FixtureMarkerLines([string]$Path) {
 $stream=$null;$reader=$null;$stage='open'
 try {
  # Rename can expose the final name while a DELETE-only handle is still open.
  # Permit that handle, but never permit an unclosed content writer. One open:
  # sharing/permission/path errors remain failures, not reasons to retry.
  $stream=[IO.FileStream]::new($Path,[IO.FileMode]::Open,[IO.FileAccess]::Read,
    ([IO.FileShare]::Read -bor [IO.FileShare]::Delete))
  $stage='size'
  if($stream.Length -gt 1048576){throw [IO.InvalidDataException]::new('Fixture marker exceeds one MiB.')}
  $stage='read'
  $reader=[IO.StreamReader]::new($stream,[Text.UTF8Encoding]::new($false,$true),$false,4096,$false)
  $lines=[Collections.Generic.List[string]]::new()
  while($null -ne ($line=$reader.ReadLine())){$lines.Add($line)}
  return ,$lines.ToArray()
 } catch {
  $cause=$_.Exception.GetBaseException()
  throw [IO.IOException]::new(('Fixture marker read failed at {0}: {1}; HRESULT=0x{2:x8}; {3}' -f $stage,$Path,$cause.HResult,$cause.Message),$cause)
 } finally {
  if($reader){$reader.Dispose()}elseif($stream){$stream.Dispose()}
 }
}

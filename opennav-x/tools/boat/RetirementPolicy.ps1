# Closed filename/hash policy; no filesystem, process or application operations.
function Get-DownloadRetirementKind([string]$Name,[string]$ExpectedSha256,[bool]$Beta1Setup) {
  if ($ExpectedSha256 -cnotmatch '^[a-f0-9]{64}$') { throw 'An exact accepted release hash is required.' }
  if ($Beta1Setup) {
    if ($Name -cne 'OpenNavX-Beta1-Setup.exe' -or
        $ExpectedSha256 -cne '8e1b3432a5a44499ffb41b125f62df07e846b2cfe1ca0936409ed021d413e128') {
      throw 'Only the exact accepted Beta 1 setup filename and release hash may be retired as an executable.'
    }
    return 'beta1-setup'
  }
  if ($Name -cnotmatch '^OpenNavX-[A-Za-z0-9_-]+( \([0-9]+\))?\.zip$') {
    throw 'Only an explicitly identified OpenNav ZIP download can be archived by default.'
  }
  return 'zip'
}

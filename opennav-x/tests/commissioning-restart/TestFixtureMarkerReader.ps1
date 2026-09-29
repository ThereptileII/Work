# Deterministic test-only sharing checks. No application, profile or hardware.
. (Join-Path $PSScriptRoot 'ReadFixtureMarker.ps1')
function Invoke-FixtureMarkerReaderChecks {
 $checks=[Collections.Generic.List[string]]::new()
 $directory=Join-Path ([IO.Path]::GetTempPath()) ('OpenNav marker reader & '+[guid]::NewGuid().ToString('N'))
 $null=[IO.Directory]::CreateDirectory($directory)
 $path=Join-Path $directory 'child--xnav.txt'
 $encoding=[Text.UTF8Encoding]::new($false,$true)
 try {
  $expected=@('42','123456789','1','session','record','path','local','roaming','','unchanged')
  [IO.File]::WriteAllText($path,([string]::Join("`n",$expected)+"`n"),$encoding)
  $actual=Read-FixtureMarkerLines $path
  if($actual.Count -ne 10 -or [string]::Join('|',$actual) -cne [string]::Join('|',$expected)){throw 'Immediate marker read changed payload/empty lines.'}
  $checks.Add('One immediate marker read preserves complete ten-line payload and empty fields')
  [IO.File]::WriteAllText($path,"unicode: "+[char]0x00e5+"`r`n`r`n",$encoding)
  $actual=Read-FixtureMarkerLines $path
  if($actual.Count -ne 2 -or $actual[0] -cne ('unicode: '+[char]0x00e5) -or $actual[1] -cne ''){throw 'UTF-8/CRLF marker data differs.'}
  $checks.Add('Strict UTF-8 and CRLF preserve the final empty field')
  foreach($case in @('missing','oversized','invalid-utf8')) {
   if($case -ceq 'missing'){[IO.File]::Delete($path)}
   elseif($case -ceq 'oversized'){[IO.File]::WriteAllBytes($path,[byte[]]::new(1048577))}
   else{[IO.File]::WriteAllBytes($path,[byte[]]@(0xff))}
   $failure=$null;try{$null=Read-FixtureMarkerLines $path}catch{$failure=$_}
   if(-not $failure -or $failure.Exception.Message -notmatch 'Fixture marker read failed at' -or
      -not $failure.Exception.Message.Contains($path) -or $failure.Exception.Message -notmatch 'HRESULT=0x'){throw ('Missing contextual immediate read failure: '+$case)}
   $checks.Add('Immediate reader refuses '+$case+' and preserves path/stage/error evidence')
  }
  [IO.File]::WriteAllText($path,([string]::Join("`n",$expected)+"`n"),$encoding)
  if([Environment]::OSVersion.Platform -eq 'Win32NT') {
   if(-not ('OpenNavMarkerSharingTest' -as [type])) {
    Add-Type -TypeDefinition @'
using System;
using System.ComponentModel;
using System.Runtime.InteropServices;
using Microsoft.Win32.SafeHandles;
public static class OpenNavMarkerSharingTest {
 [DllImport("kernel32.dll",CharSet=CharSet.Unicode,SetLastError=true)]
 static extern SafeFileHandle CreateFileW(string path,uint access,uint share,IntPtr security,uint disposition,uint flags,IntPtr template);
 public static SafeFileHandle HoldDelete(string path) {
  var handle=CreateFileW(path,0x00010000,7,IntPtr.Zero,3,0x80,IntPtr.Zero);
  if(handle.IsInvalid){int error=Marshal.GetLastWin32Error();handle.Dispose();throw new Win32Exception(error);}
  return handle;
 }
}
'@
   }
   # Hold the same DELETE-only access left by a rename, without scheduler timing.
   $handle=[OpenNavMarkerSharingTest]::HoldDelete($path)
   try {
    $failure=$null;try{$null=[IO.File]::ReadAllLines($path)}catch{$failure=$_}
    if(-not $failure -or ($failure.Exception.GetBaseException().HResult -band 0xffff) -ne 32){throw 'Legacy reader did not reproduce native rename-sharing refusal.'}
    $checks.Add('Native held DELETE handle deterministically reproduces old ReadAllLines sharing error 32')
    $actual=Read-FixtureMarkerLines $path
    if($actual.Count -ne 10 -or [string]::Join('|',$actual) -cne [string]::Join('|',$expected)){throw 'Delete-sharing reader changed the complete marker.'}
    $checks.Add('One immediate read succeeds with DELETE handle held and every marker byte closed')
   } finally {$handle.Dispose()}
   # Sharing DELETE does not permit WRITE: a prematurely published marker fails.
   $writer=[IO.FileStream]::new($path,[IO.FileMode]::Open,[IO.FileAccess]::Write,
     ([IO.FileShare]::ReadWrite -bor [IO.FileShare]::Delete))
   try {
    $failure=$null;try{$null=Read-FixtureMarkerLines $path}catch{$failure=$_}
    if(-not $failure -or ($failure.Exception.GetBaseException().HResult -band 0xffff) -ne 32){throw 'Marker reader permitted an unclosed writer.'}
    $checks.Add('An unclosed WRITE handle still fails immediately with native sharing error 32')
   } finally {$writer.Dispose()}
  }
  return $checks.ToArray()
 } finally {[IO.Directory]::Delete($directory,$true)}
}

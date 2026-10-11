# Focused owner/record policy and harmless native marker fixtures. No real app,
# installer, preserved profile, network, operator state or physical output.
[CmdletBinding()]
param([switch]$PolicyOnly)
$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest
. (Join-Path $PSScriptRoot '../installer/windows/UpdateSupervisor.ps1')
function Check([bool]$Value,[string]$Message) { if(-not $Value){throw $Message};Write-Host ('PASS: '+$Message) }
function Reject([scriptblock]$Operation,[string]$Message) { $failed=$false;try { & $Operation | Out-Null } catch { $failed=$true };Check $failed $Message }
function Identity([char]$Digit) { [pscustomobject]@{generation=([string]$Digit)*32;commit=([string]$Digit)*40;packageSha256=([string]$Digit)*64;executableSha256=([string]$Digit)*64} }
function HashFile([string]$File) { (Get-FileHash -LiteralPath $File -Algorithm SHA256).Hash.ToLowerInvariant() }
$module=Join-Path $PSScriptRoot '../installer/windows/UpdateTransaction.ps1'
$tokens=$null;$errors=$null
$ast=[Management.Automation.Language.Parser]::ParseFile($module,[ref]$tokens,[ref]$errors)
Check ($errors.Count -eq 0) 'Production transaction source parses'
$source=@($ast.FindAll({param($node) $node -is [Management.Automation.Language.StringConstantExpressionAst] -and $node.Value.StartsWith('using System;')},$true))
Check ($source.Count -eq 1) 'One exact embedded native source selected'
if(-not ('Skager.UpdateStartupPipe' -as [type])) { Add-Type -TypeDefinition $source[0].Value }
Check ($null -ne ('Skager.UpdateImageOwner' -as [type])) 'Exact embedded owner compiles without invoking Win32'
# Exercise the exact pure name parser without calling a native API on Linux.
$parseApplication=[Skager.UpdateImageOwner].GetMethod('ParseApplication',[Reflection.BindingFlags]'NonPublic,Static')
function ParseApplication([string]$FinalName,[string]$NativeName) { $parseApplication.Invoke($null,@($FinalName,$NativeName)) }
$nativeFixture='\Device\HarddiskVolume12\held tree\app\opencpn.exe'
Check ((ParseApplication '\\?\C:\held tree\app\opencpn.exe' $nativeFixture) -ceq 'C:\held tree\app\opencpn.exe') 'Held DOS parser strips exactly the extended prefix and preserves the physical suffix'
Check ((ParseApplication '\\?\z:\held tree\app\opencpn.exe' $nativeFixture) -ceq 'z:\held tree\app\opencpn.exe') 'Held DOS parser accepts a conventional local drive spelling'
foreach($name in @(
 'C:\held tree\app\opencpn.exe',
 '\\?\UNC\server\held tree\app\opencpn.exe',
 '\\?\Volume{12345678-1234-1234-1234-123456789abc}\held tree\app\opencpn.exe',
 '\\?\GLOBALROOT\Device\HarddiskVolume12\held tree\app\opencpn.exe',
 '\\?\C:held tree\app\opencpn.exe',
 '\\?\C:\other tree\app\opencpn.exe',
 '\\?\C:\Held tree\app\opencpn.exe'
)) { Reject { ParseApplication $name $nativeFixture } 'Unsupported or physically mismatched application spelling refused' }
foreach($tail in @('app\\opencpn.exe','app\.\opencpn.exe','app\..\opencpn.exe','app.\opencpn.exe','app \opencpn.exe','app\opencpn.exe:stream','app\open?pn.exe','app\open*pn.exe','app\open"pn.exe','app\open<pn.exe','app\open|pn.exe','app/open.exe','CON\opencpn.exe','app\LPT1.exe',('app\open'+[char]0+'pn.exe'))) {
 # Matching suffixes deliberately bypass mismatch refusal to test unsafe components.
 Reject { ParseApplication ('\\?\C:\'+$tail) ('\Device\HarddiskVolume12\'+$tail) } 'Ambiguous DOS application component refused'
}
Reject { ParseApplication '\\?\C:\app\opencpn.exe' '\Device\Mup\app\opencpn.exe' } 'Nonlocal native volume refused'
Reject { ParseApplication '\\?\C:\' '\Device\HarddiskVolume12\' } 'Volume root cannot be selected as an executable'
$r=New-UpdatePendingRecord (Identity 'a') (Identity 'b')
$r.attempts=1;$r.session='c'*32;$r.nativeLaunchV1='armed'
Assert-UpdatePendingRecord $r
Reject { Stop-InterruptedUpdateCandidate 'unused-no-filesystem-access' $r } 'Durable armed launch refuses automatic recovery'
$legacy=$r | ConvertTo-Json -Depth 8 | ConvertFrom-Json
$legacy.PSObject.Properties.Remove('nativeLaunchV1');$legacy.PSObject.Properties.Remove('processCreationFileTimeV1')
Assert-UpdatePendingRecord $legacy
Reject { Stop-InterruptedUpdateCandidate 'unused-no-filesystem-access' $legacy } 'Historical attempted record remains readable but cannot authorize native cleanup'
$r.nativeLaunchV1='running';$r.processId=123;$r.processCreationFileTimeV1='133000000000000000'
Assert-UpdatePendingRecord $r
$missing=$r | ConvertTo-Json -Depth 8 | ConvertFrom-Json;$missing.PSObject.Properties.Remove('processId')
Reject { Assert-UpdatePendingRecord $missing } 'Running record without PID explicitly refused'
foreach($invalid in @('','-1','0','9223372036854775808','utc-ticks')) {
 $r.processCreationFileTimeV1=$invalid
 Reject { Assert-UpdatePendingRecord $r } 'Malformed native creation identity refused'
}
if($PolicyOnly -or [Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT) {
 Write-Host 'SKIP: native creation/alias/health/cleanup require Windows; portable compilation is not native qualification.'
 exit 0
}
if($env:GITHUB_ACTIONS -ne 'true' -or $PSVersionTable.PSVersion.Major -ne 5) { throw 'Native fixtures require disposable Windows CI and Windows PowerShell 5.1.' }
Assert-NoUpdateApplication
Add-Type -TypeDefinition @'
using System;
using System.Runtime.InteropServices;
public static class ImageFixtureAlias {
 [DllImport("kernel32.dll",CharSet=CharSet.Unicode,SetLastError=true)] public static extern bool CreateHardLink(string link,string existing,IntPtr security);
 [DllImport("kernel32.dll",CharSet=CharSet.Unicode,SetLastError=true)] public static extern bool DefineDosDevice(uint flags,string name,string target);
 [DllImport("kernel32.dll",CharSet=CharSet.Unicode,SetLastError=true)] public static extern uint QueryDosDevice(string name,System.Text.StringBuilder target,int count);
}
'@
$fixture=Join-Path ([IO.Path]::GetTempPath()) ('SkagerImageOwner-'+[guid]::NewGuid().ToString('N'))
$owners=New-Object 'Collections.Generic.List[IDisposable]'
$children=New-Object 'Collections.Generic.List[Diagnostics.Process]'
$aliases=New-Object 'Collections.Generic.List[string]'
$alias='';$oldRoot=$env:SKAGER_IMAGE_FIXTURE_ROOT;$oldCommit=$env:SKAGER_IMAGE_FIXTURE_COMMIT
function MapAlias([string]$Target) {
 $native='\??\'+$Target
 if(-not [ImageFixtureAlias]::DefineDosDevice(9,$alias,$native)){throw 'Disposable local alias creation failed.'}
 $aliases.Add($native)
}
function WaitMarker([string]$Name) {
 $deadline=[DateTime]::UtcNow.AddSeconds(10)
 while(-not [IO.File]::Exists((Join-Path $fixture $Name))) {
  if([DateTime]::UtcNow -ge $deadline){throw 'Inert marker did not start.'};Start-Sleep -Milliseconds 25
 }
}
function StartInfo([string]$Executable) {
 $start=New-Object Diagnostics.ProcessStartInfo
 $start.FileName=$Executable;$start.WorkingDirectory=[IO.Path]::GetDirectoryName($Executable);$start.UseShellExecute=$false
 foreach($name in @('SKAGER_UPDATE_PIPE','SKAGER_UPDATE_GENERATION','SKAGER_UPDATE_CHALLENGE')){$start.EnvironmentVariables.Remove($name)}
 $start.EnvironmentVariables['SKAGER_IMAGE_FIXTURE_ROOT']=$fixture
 return $start
}
try {
 $null=New-Item -ItemType Directory -Path $fixture
 # Use the actual private production opens on empty directories: no held
 # executable below them can accidentally supply rename exclusion.
 $directoryOpen=[Skager.UpdateArmedBarrier].GetMethod('OpenDirectory',[Reflection.BindingFlags]'NonPublic,Static')
 $nativeName=[Skager.UpdateImageOwner].GetMethod('Native',[Reflection.BindingFlags]'NonPublic,Static')
 $ownerOpen=[Skager.UpdateImageOwner].GetMethod('Open',[Reflection.BindingFlags]'NonPublic,Instance')
 foreach($kind in @('armed-barrier','image-owner')) {
  [string]$empty=Join-Path $fixture ('empty-pin-'+$kind);[string]$renamed=$empty+'-renamed'
  [object[]]$directoryArguments=@([string]$empty)
  $null=[IO.Directory]::CreateDirectory($empty)
  [IO.Directory]::Move($empty,$renamed);[IO.Directory]::Move($renamed,$empty)
  [Microsoft.Win32.SafeHandles.SafeFileHandle]$probe=$directoryOpen.Invoke($null,$directoryArguments)
  try {
   [object[]]$nativeArguments=@([Microsoft.Win32.SafeHandles.SafeFileHandle]$probe)
   [string]$native=$nativeName.Invoke($null,$nativeArguments)
  } finally { $probe.Dispose() }
  $held=$null;$directoryOwner=$null
  try {
   if($kind -ceq 'armed-barrier') { $held=$directoryOpen.Invoke($null,$directoryArguments) }
   else {
    $directoryOwner=[Activator]::CreateInstance([Skager.UpdateImageOwner],$true);$owners.Add($directoryOwner)
    [object[]]$ownerArguments=@([string]$native,[bool]$true)
    $null=$ownerOpen.Invoke($directoryOwner,$ownerArguments)
   }
   $sharingDenied=$false
   try { [IO.Directory]::Move($empty,$renamed) }
   catch { $sharingDenied=($_.Exception.GetBaseException().HResult -band 0xffff) -eq 32 }
   Check $sharingDenied ($kind+' empty directory rename reports sharing violation while pinned')
  } finally {
   if($null -ne $held){$held.Dispose()}
   if($null -ne $directoryOwner){$directoryOwner.Dispose()}
  }
  [IO.Directory]::Move($empty,$renamed)
  Check ([IO.Directory]::Exists($renamed) -and -not [IO.Directory]::Exists($empty)) ($kind+' empty directory rename succeeds after pin release')
 }
 $a=Join-Path $fixture 'A';$b=Join-Path $fixture 'B'
 foreach($root in @($a,$b)){ $null=New-Item -ItemType Directory -Path (Join-Path $root 'app') -Force }
 $aExe=Join-Path $a 'app/opencpn.exe';$bExe=Join-Path $b 'app/opencpn.exe'
 $markerSource=[IO.File]::ReadAllText((Join-Path $PSScriptRoot 'fixtures/update-image-marker.cs'))
 # Two different real x86 managed images, hardcoded A/B markers. No app build.
 $compiler=Join-Path ([Runtime.InteropServices.RuntimeEnvironment]::GetRuntimeDirectory()) 'csc.exe'
 $aSource=Join-Path $fixture 'marker-A.cs';$bSource=Join-Path $fixture 'marker-B.cs'
 [IO.File]::WriteAllText($aSource,$markerSource)
 [IO.File]::WriteAllText($bSource,$markerSource.Replace('FixtureImage="A"','FixtureImage="B"'))
 foreach($pair in @(@($aSource,$aExe),@($bSource,$bExe))) {
  & $compiler /nologo /target:winexe /platform:x86 /reference:System.Windows.Forms.dll /reference:System.Drawing.dll ('/out:'+$pair[1]) $pair[0]
  if($LASTEXITCODE -ne 0 -or -not [IO.File]::Exists($pair[1])){throw 'Inert marker compiler failed.'}
 }
 $aHash=HashFile $aExe;$bHash=HashFile $bExe
 $linked=Join-Path $fixture 'linked.exe';$otherLink=Join-Path $fixture 'linked-alias.exe'
 Copy-Item -LiteralPath $aExe -Destination $linked
 Check ([ImageFixtureAlias]::CreateHardLink($otherLink,$linked,[IntPtr]::Zero)) 'Disposable multi-link image created'
 Reject { [Skager.UpdateImageOwner]::Acquire($linked,$fixture,$aHash) } 'Actual acquired multi-link image refused'
 $released=[IO.File]::OpenWrite($linked);$released.Dispose()
 Remove-Item -LiteralPath $linked,$otherLink
 Check ($aHash -cne $bHash) 'Replacement marker has genuinely different bytes'
 foreach($letter in @('Z:','Y:','X:','W:','V:')) {
  $buffer=New-Object Text.StringBuilder 32768
  if([ImageFixtureAlias]::QueryDosDevice($letter,$buffer,$buffer.Capacity) -eq 0 -and [Runtime.InteropServices.Marshal]::GetLastWin32Error() -eq 2){$alias=$letter;break}
 }
 if(-not $alias){throw 'No disposable unused DOS drive letter.'}
 MapAlias $a
 $aliased=$alias+'\app\opencpn.exe'
 $owner=[Skager.UpdateImageOwner]::Acquire($aliased,($alias+'\app'),$aHash);$owners.Add($owner)
 Check ([string]::Equals($owner.ApplicationImage,$aExe,[StringComparison]::OrdinalIgnoreCase)) 'Application spelling derives from the held physical image rather than the original SUBST path'
 # Simulate an unavailable or redirected selected name, without remapping a host
 # volume drive. Validate must refuse before creation; selection is never retried.
 $applicationProperty=[Skager.UpdateImageOwner].GetProperty('ApplicationImage')
 foreach($selected in @($aliased,$bExe,(Join-Path $a 'app/missing.exe'))) {
  $refusalOwner=[Skager.UpdateImageOwner]::Acquire($aExe,(Split-Path $aExe -Parent),$aHash);$owners.Add($refusalOwner)
  $applicationProperty.SetValue($refusalOwner,$selected,$null)
  Reject { $refusalOwner.Validate() } 'Selected application root/image/unavailable resolution refuses against actual held pins'
  Reject { $refusalOwner.Start((StartInfo $aExe)) } 'Selected application refusal prevents native creation'
  Check (-not $refusalOwner.CreationAttempted) 'Selected application refusal does not arm an unattempted creation'
  Check ($refusalOwner.ApplicationImage -ceq $selected) 'Selected application refusal never substitutes or rederives a name'
  $refusalOwner.Dispose()
 }
 Reject { $stream=[IO.File]::OpenWrite($aExe);$stream.Dispose() } 'Retained image denies concurrent write access'
 Reject { [IO.Directory]::Move($a,($a+'-moved')) } 'Physical ancestor above SUBST root cannot be renamed while held'
 MapAlias $b
 $child=$owner.Start((StartInfo $aliased));$children.Add($child)
 WaitMarker 'marker-A'
 Check (-not [IO.File]::Exists((Join-Path $fixture 'marker-B'))) 'Alias remap before creation never executes replacement bytes'
 $child.Refresh()
 Check ($child.MainWindowHandle -ne [IntPtr]::Zero) 'Inert marker exposes its main window for the actual normal-close test'
 Check ($owner.Matches($child,$aliased,$aHash)) 'Created child remains bound to retained physical image after DOS remap'
 $owner.Close($child,10000)
 Check $child.HasExited 'Native owner normally closes its original child after alias remap'
 $owner.Dispose()
 $released=[IO.File]::OpenWrite($aExe);$released.Dispose()
 [IO.Directory]::Move($a,($a+'-moved'));[IO.Directory]::Move(($a+'-moved'),$a)
 Check ([IO.Directory]::Exists($a)) 'Image and all directory pins release after owned child closure'
 # Restore the A alias and exercise actual supervisor -> exact health -> close.
 MapAlias $a
 Remove-Item -LiteralPath (Join-Path $fixture 'marker-A')
 $env:SKAGER_IMAGE_FIXTURE_ROOT=$fixture;$env:SKAGER_IMAGE_FIXTURE_COMMIT='a'*40
 $identity=Identity 'a';$identity.executableSha256=$aHash
 $generation=[pscustomobject]@{directory=($alias+'\');executable=$aliased;identity=$identity}
 $session=New-UpdateHealthSession $identity;$owners.Add($session.server)
 $child=Start-SupervisedGeneration $generation $session $fixture;$children.Add($child)
 MapAlias $b
 Check (Wait-UpdateGenerationStartupSuccess $identity $session $child $aliased 10000) 'Actual supervisor health retains PID/challenge/frame checks across DOS remap'
 Check ([string]::Equals($session.server.VerifiedImage,$aliased,[StringComparison]::OrdinalIgnoreCase)) 'Receipt audit spelling remains original validated DOS path'
 Check ($session.server.VerifiedNativeImage.StartsWith('\Device\HarddiskVolume')) 'Health records the separate held native image identity'
 Stop-SupervisedProcess $child $aliased '' $session
 Check $child.HasExited 'Actual supervisor normal cleanup closes owned child across DOS remap'
 $session.server.Dispose()
 # Acquired-image hash/argument rejection is positively before native creation.
 $server=New-UpdateHealthSession $identity;$owners.Add($server.server)
 Reject { $server.server.StartOwned((StartInfo $aExe),('0'*64)) } 'Changed expected hash refused before creation'
 Check (-not $server.server.NativeCreationAttempted) 'Hash refusal does not become uncertain native creation'
 $server.server.Dispose()
 $server=New-UpdateHealthSession $identity;$owners.Add($server.server)
 $unsupported=StartInfo $aExe;$unsupported.RedirectStandardOutput=$true
 Reject { $server.server.StartOwned($unsupported,$aHash) } 'Unsupported inherited stream request refused before creation'
 Check (-not $server.server.NativeCreationAttempted) 'Argument refusal does not become uncertain native creation'
 $server.server.Dispose()
 # A real native CreateProcess refusal stays armed even when no marker ran.
 $invalid=Join-Path $fixture 'invalid-image.exe';[IO.File]::WriteAllText($invalid,'not an executable')
 $server=New-UpdateHealthSession $identity;$owners.Add($server.server)
 Reject { $server.server.StartOwned((StartInfo $invalid),(HashFile $invalid)) } 'Native invalid-image creation refused'
 Check ($server.server.NativeCreationAttempted -and $server.server.UncertainCreation) 'Actual CreateProcess refusal denies fallback'
 $server.server.Dispose()
 # Exact-source fault injection only in this test assembly: call the real
 # ResumeThread, then force its uncertain-result branch. No production hook.
 $faultSource=$source[0].Value.Replace('namespace Skager {','namespace SkagerResumeFault {')
 $needle='if(ResumeThread(thread)!=1)'
 Check ($faultSource.Split(@($needle),[StringSplitOptions]::None).Count -eq 2) 'Resume fault binds to one exact production branch'
 $faultSource=$faultSource.Replace($needle,'if(ResumeThread(thread)!=1 || Environment.OSVersion.Platform==PlatformID.Win32NT)')
 Add-Type -TypeDefinition $faultSource
 Remove-Item -LiteralPath (Join-Path $fixture 'marker-A')
 $faultOwner=[SkagerResumeFault.UpdateImageOwner]::Acquire($aExe,(Split-Path $aExe -Parent),$aHash);$owners.Add($faultOwner)
 Reject { $faultOwner.Start((StartInfo $aExe)) } 'Uncertain resume result refuses successful start'
 $child=[Diagnostics.Process]::GetProcessById($faultOwner.ProcessId);$null=$child.Handle;$children.Add($child)
 WaitMarker 'marker-A'
 Check ($faultOwner.CreationAttempted -and -not $faultOwner.StartSucceeded -and -not $child.HasExited) 'Resume uncertainty preserves the possibly running owned child'
 $null=$child.CloseMainWindow();Check ($child.WaitForExit(10000)) 'Test alone normally closes its inert uncertain child'
 $faultOwner.Dispose()
 # Simulate lost creator lifetime, then use fresh refusal-only native cleanup.
 Remove-Item -LiteralPath (Join-Path $fixture 'marker-A')
 $owner=[Skager.UpdateImageOwner]::Acquire($aExe,(Split-Path $aExe -Parent),$aHash);$owners.Add($owner)
 $child=$owner.Start((StartInfo $aExe));$children.Add($child);WaitMarker 'marker-A'
 $creation=$owner.CreationFileTime;$id=$child.Id;$owner.Dispose()
 Check (-not [Skager.UpdateImageOwner]::CloseInterrupted($child,$aExe,$aHash,$id,($creation+1),1000)) 'Wrong creation identity never signals interrupted child'
 Check (-not $child.HasExited) 'Wrong creation identity preserves live inert child'
 Check ([Skager.UpdateImageOwner]::CloseInterrupted($child,$aExe,$aHash,$id,$creation,10000)) 'Fresh native cleanup normally closes only the recorded matching child'
 Check (-not [IO.File]::Exists((Join-Path $fixture 'marker-B'))) 'Replacement image never executed in any native fixture'
 Write-Host 'PASS: focused suspended image/alias/health/normal/interrupted cleanup fixtures only; no full custody or product acceptance.'
 } finally {
 $cleanupFailed=$false
 foreach($child in $children) {
  try { if(-not $child.HasExited){$child.Kill();if(-not $child.WaitForExit(5000)){throw 'Inert child remains live.'}} }
  catch {$cleanupFailed=$true} finally { $child.Dispose() }
 }
 foreach($owner in $owners){try{$owner.Dispose()}catch{$cleanupFailed=$true}}
 for($i=$aliases.Count-1;$i -ge 0;$i--){if(-not [ImageFixtureAlias]::DefineDosDevice(15,$alias,$aliases[$i])){$cleanupFailed=$true}}
 $env:SKAGER_IMAGE_FIXTURE_ROOT=$oldRoot;$env:SKAGER_IMAGE_FIXTURE_COMMIT=$oldCommit
 if(-not $cleanupFailed -and [IO.Directory]::Exists($fixture)){Remove-Item -LiteralPath $fixture -Recurse -Force}
 if($cleanupFailed){throw 'Inert fixture cleanup uncertain; disposable fixture retained.'}
}

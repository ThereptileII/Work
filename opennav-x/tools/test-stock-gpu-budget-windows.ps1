# Evidence-only actual official executable fixture. No commissioning policy edit,
# real profile, marine connection, plugin helper, forced exit or boat operation.
[CmdletBinding()]
param([Parameter(Mandatory=$true)][string]$Evidence)
Set-StrictMode -Version Latest;$ErrorActionPreference='Stop';$ProgressPreference='SilentlyContinue'
if([Environment]::OSVersion.Platform -ne 'Win32NT' -or $env:GITHUB_ACTIONS -cne 'true'){throw 'Native disposable CI only.'}
. (Join-Path $PSScriptRoot 'boat\Common.ps1')
. (Join-Path $PSScriptRoot 'boat\StartupLog.ps1')
if(@(Get-Process -Name opencpn -ErrorAction SilentlyContinue).Count){throw 'Every navigation application must be closed.'}
$normal=Join-Path ([Environment]::GetFolderPath('CommonApplicationData')) 'opencpn'
$managed=Join-Path ([Environment]::GetFolderPath('LocalApplicationData')) 'opencpn\plugins'
if(Test-Path -LiteralPath $normal){throw 'A fresh disposable runner without a shared OpenCPN profile is required.'}
if((Test-Path -LiteralPath $managed) -and @(Get-ChildItem -LiteralPath $managed -File -Recurse -Filter '*.dll').Count){throw 'No pre-existing managed plugin binaries may be loaded.'}
$evidence=Assert-LocalPath ([IO.Path]::GetFullPath($Evidence))
if((Test-Path -LiteralPath $evidence) -and @(Get-ChildItem -LiteralPath $evidence -Force).Count){throw 'A new evidence directory is required.'}
$null=New-Item -ItemType Directory -Path $evidence -Force
$root=Join-Path (Assert-LocalPath $env:RUNNER_TEMP) ('OpenNav stock GPU '+[guid]::NewGuid().ToString('N'))
$null=New-Item -ItemType Directory -Path $root
$setup=Join-Path $root 'official-setup.exe'
$setupHash='e949f55de57611afe2fc0dad5a8ac33795c46ba488cb40ca07b65f639a07b8aa'
$exeHash='7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c'
$version='Version 5.12.4-0+37fd0cd Build 2025-09-12'
$pluginNames=@('chartdldr_pi.dll','dashboard_pi.dll','grib_pi.dll','wmm_pi.dll')
$checks=New-Object 'Collections.Generic.List[string]';$cases=New-Object 'Collections.Generic.List[object]'
$report=@{status='running';officialSetupSha256=$setupHash;officialExecutableSha256=$exeHash;sourceRevision='37fd0cddb7334fe489e9f18aa163977a9c5c84f7';
 policyChanged=$false;boatAccess=$false;marineConnections=$false;rendererBackendAcceptance=$false;cases=@()}
function Check([bool]$Value,[string]$Message){if(-not $Value){throw ('FAILED: '+$Message)};$checks.Add($Message);Write-Host ('PASS: '+$Message)}
Add-Type -TypeDefinition @'
using System;
using System.Runtime.InteropServices;
public static class StockGpuFixtureWindow {
 [DllImport("user32.dll")] static extern bool IsWindowEnabled(IntPtr window);
 [DllImport("user32.dll")] static extern bool IsWindowVisible(IntPtr window);
 [DllImport("user32.dll")] static extern uint GetWindowThreadProcessId(IntPtr window,out uint pid);
 public static bool Ordinary(IntPtr window,int pid){uint owner;GetWindowThreadProcessId(window,out owner);return window!=IntPtr.Zero&&owner==(uint)pid&&IsWindowEnabled(window)&&IsWindowVisible(window);}
}
'@
try {
 Invoke-WebRequest -UseBasicParsing -Uri 'https://github.com/OpenCPN/OpenCPN/releases/download/Release_5.12.4/opencpn_5.12.4-0%2B3720.37fd0cd_setup.exe' -OutFile $setup -TimeoutSec 120
 Check ((Get-Digest $setup) -ceq $setupHash) 'Exact official setup verified before extraction'
 $seven=Join-Path ${env:ProgramFiles} '7-Zip\7z.exe';if(-not [IO.File]::Exists($seven)){throw 'Reviewed runner 7-Zip is required.'}
 foreach($case in @('expert-absent','expert-false','expert-true-control')) {
  $caseEvidence=Join-Path $evidence $case;$null=New-Item -ItemType Directory -Path $caseEvidence
  $app=Join-Path $root $case;$exe=Join-Path $app 'opencpn.exe';$ini=Join-Path $app 'opencpn.ini';$log=Join-Path $app 'opencpn.log'
  $item=@{name=$case;status='running';sameVersion=$version;beforeBudget=64;expectedBudget=$(if($case -ceq 'expert-true-control'){64}else{128});configuredOpenGL=1}
  $process=$null;$output=$null;$errorOutput=$null;$closeAttempted=$false
  try {
   & $seven x -y ('-o'+$app) $setup | Out-File -LiteralPath (Join-Path $caseEvidence 'extraction.log') -Encoding utf8
   if($LASTEXITCODE -ne 0){throw 'Official payload extraction failed.'}
   Check ((Get-Digest $exe) -ceq $exeHash) ($case+': exact official executable')
   $plugins=@(Get-ChildItem -LiteralPath (Join-Path $app 'plugins') -Filter '*_pi.dll' -File -Recurse|ForEach-Object {$_.Name}|Sort-Object)
   Check (($plugins -join '|') -ceq ($pluginNames -join '|')) ($case+': only reviewed bundled plugins exist')
   if((Test-Path -LiteralPath $ini) -or (Test-Path -LiteralPath $log)){throw 'Archive unexpectedly contained a profile or startup log.'}
   $text="[Settings]`r`nConfigVersionString=$version`r`nNavMessageShown=1`r`nLocale=en_US`r`nOpenGL=1`r`nGPUTextureMemSize=64`r`nGPUTextureDimension=512`r`nGPUTextureCompression=0`r`nGPUTextureCompressionCaching=0`r`n"
   if($case -ceq 'expert-false'){$text+="OpenGLExpert=0`r`n"}
   if($case -ceq 'expert-true-control'){$text+="OpenGLExpert=1`r`n"}
   $text+="[Settings/GlobalState]`r`nFrameWinX=900`r`nFrameWinY=640`r`nFrameWinPosX=20`r`nFrameWinPosY=20`r`nFrameMax=0`r`n[Settings/NMEADataSource]`r`nDataConnections=`r`n"
   foreach($name in $plugins){$text+="[PlugIns/$name]`r`nbEnabled=0`r`n"}
   [IO.File]::WriteAllText($ini,$text,(New-Object Text.UTF8Encoding($false)))
   Copy-Item -LiteralPath $ini -Destination (Join-Path $caseEvidence 'before.ini');$item.beforeIniSha256=Get-Digest $ini
   $before=Read-ProfileForAudit $ini
   $private=Join-Path $root ($case+' environment');$null=New-Item -ItemType Directory -Path $private
   foreach($folder in @('Local','Roaming')){$null=New-Item -ItemType Directory -Path (Join-Path $private $folder)}
   $start=New-Object Diagnostics.ProcessStartInfo;$start.FileName=$exe;$start.Arguments='--portable';$start.WorkingDirectory=$app;$start.UseShellExecute=$false
   $start.RedirectStandardOutput=$true;$start.RedirectStandardError=$true
   $start.EnvironmentVariables['LOCALAPPDATA']=Join-Path $private 'Local';$start.EnvironmentVariables['APPDATA']=Join-Path $private 'Roaming'
   $start.EnvironmentVariables['PATH']=$app+';'+[Environment]::GetFolderPath('System')+';'+$env:WINDIR
   $process=[Diagnostics.Process]::Start($start);$null=$process.Handle
   $output=$process.StandardOutput.ReadToEndAsync();$errorOutput=$process.StandardError.ReadToEndAsync()
   $item.processId=$process.Id;$item.processStartedUtc=$process.StartTime.ToUniversalTime().ToString('o');$item.arguments=$start.Arguments
   Check ($process.Path -ieq $exe -and (Get-Digest $process.Path) -ceq $exeHash) ($case+': launched only exact isolated official executable')
   $deadline=[datetime]::UtcNow.AddSeconds(90);$initialized=$false
   do {
    Start-Sleep -Milliseconds 250;$process.Refresh()
    if($process.HasExited){throw 'Actual official application exited before final initialization.'}
    $initialized=Test-StartupInitializedSince ([byte[]]@()) (Read-StartupLogBytes $log)
   } while(-not $initialized -and [datetime]::UtcNow -lt $deadline)
   Check $initialized ($case+': fresh actual startup finalized without first-run/version-upgrade acknowledgement')
   Check ($process.MainWindowTitle.StartsWith('OpenCPN',[StringComparison]::Ordinal) -and [StockGpuFixtureWindow]::Ordinary($process.MainWindowHandle,$process.Id)) ($case+': exact ordinary frame is visible and enabled; no modal bypass')
   $observer=Get-Process -Id $process.Id
   try {$closeAttempted=$true;$item.close=Invoke-ReviewedNormalClose $observer $process.Id $process.StartTime.ToUniversalTime().Ticks}finally{$observer.Dispose()}
   Check ($item.close.exitCodeKnown -and $item.close.exitCode -eq 0) ($case+': measured normal zero exit')
   Copy-Item -LiteralPath $ini -Destination (Join-Path $caseEvidence 'after.ini');$item.afterIniSha256=Get-Digest $ini
   $after=Read-ProfileForAudit $ini
   $item.afterBudget=$after['Settings/GPUTextureMemSize'];$item.afterOpenGL=$after['Settings/OpenGL'];$item.afterVersion=$after['Settings/ConfigVersionString']
   $item.expertBefore=$(if($before.ContainsKey('Settings/OpenGLExpert')){$before['Settings/OpenGLExpert']}else{$null})
   $item.expertAfter=$(if($after.ContainsKey('Settings/OpenGLExpert')){$after['Settings/OpenGLExpert']}else{$null})
   Check ($after['Settings/ConfigVersionString'] -ceq $version -and $after['Settings/NavMessageShown'] -ceq '1') ($case+': exact version and existing notice state unchanged')
   Check ($after['Settings/OpenGL'] -ceq '1') ($case+': configured OpenGL remains1; no inference about hardware acceleration')
   Check ($item.expertBefore -ceq $item.expertAfter) ($case+': absent/explicit expert preference preserved')
   Check ($after['Settings/GPUTextureMemSize'] -ceq [string]$item.expectedBudget) ($case+': actual same-version texture budget matches source-derived result')
   Check ($after['Settings/NMEADataSource/DataConnections'] -ceq '') ($case+': marine connections remain empty')
   foreach($name in $plugins){Check ($after['PlugIns/'+$name+'/bEnabled'] -ceq '0') ($case+': bundled plugin remains disabled: '+$name)}
   Check (-not (Test-Path -LiteralPath $normal)) ($case+': shared normal profile untouched')
   $item.status='passed'
  } catch {$item.status='failed';$item.error=$_.Exception.Message;$item.stack=$_.ScriptStackTrace;throw}
  finally {
   if($process) {
    if(-not $process.HasExited -and -not $closeAttempted) {
     try {
      $process.Refresh()
      if($process.Path -ine $exe -or (Get-Digest $exe) -cne $exeHash -or -not $process.MainWindowTitle.StartsWith('OpenCPN',[StringComparison]::Ordinal) -or -not [StockGpuFixtureWindow]::Ordinary($process.MainWindowHandle,$process.Id)){throw 'Unknown or blocked fixture window; no close interaction.'}
      $closeAttempted=$true;$item.cleanup=Invoke-ReviewedNormalClose $process $process.Id $process.StartTime.ToUniversalTime().Ticks
     } catch {$item.cleanupFailure=$_.Exception.Message}
    }
    if(-not $process.HasExited){$item.leftForDisposableRunnerTeardown=$true}
    foreach($stream in @(@('stdout',$output),@('stderr',$errorOutput))){if($stream[1] -and $stream[1].IsCompleted){try{[IO.File]::WriteAllText((Join-Path $caseEvidence ($stream[0]+'.log')),$stream[1].GetAwaiter().GetResult())}catch{}}}
    $process.Dispose()
   }
   if([IO.File]::Exists($log)){[IO.File]::WriteAllBytes((Join-Path $caseEvidence 'opencpn.log'),(Read-StartupLogBytes $log))}
   $item.normalCloseAttempted=$closeAttempted;$cases.Add([pscustomobject]$item);Write-Record (Join-Path $caseEvidence 'result.json') $item
  }
 }
 $report.status='passed'
} catch {$report.status='failed';$report.error=$_.Exception.Message;$report.stack=$_.ScriptStackTrace;throw}
finally {$report.cases=$cases.ToArray();$report.checks=$checks.ToArray();$report.count=$checks.Count;Write-Record (Join-Path $evidence 'stock-gpu-budget-results.json') $report}

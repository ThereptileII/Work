$ErrorActionPreference='Stop'

$root='C:\XNav\ComponentChecks\installer-pe-87fe6c5d'
if((Get-FileHash -LiteralPath ($root+'\fixtures.zip') -Algorithm SHA256).Hash.ToLowerInvariant() -cne '89144f7fa569bf002683dd7a3cb35d0e375535eee5d09f2ea542efca93207b03'){throw 'archive hash differs'}
Expand-Archive -LiteralPath ($root+'\fixtures.zip') -DestinationPath $root
if((Get-FileHash -LiteralPath 'C:\XNav\ComponentChecks\installer-pe-87fe6c5d\opennav-x\installer\windows\Lifecycle.ps1' -Algorithm SHA256).Hash.ToLowerInvariant() -cne 'b4284a432d8b34842e87c31f66478a587974333573a5af93e60b99fca470c6fb'){throw 'source hash differs'}; if((Get-FileHash -LiteralPath 'C:\XNav\ComponentChecks\installer-pe-87fe6c5d\opennav-x\tools\test-installer-pe.ps1' -Algorithm SHA256).Hash.ToLowerInvariant() -cne '170507753ecba6db6362879100cebba930f801bd89566841b2ab118554383373'){throw 'source hash differs'}
$sources=@(($root+'\opennav-x\installer\windows\Lifecycle.ps1'),($root+'\opennav-x\tools\test-installer-pe.ps1'))
foreach($source in $sources){$tokens=$null;$parseErrors=$null;$null=[Management.Automation.Language.Parser]::ParseFile($source,[ref]$tokens,[ref]$parseErrors);if(@($parseErrors).Count){throw 'source parse failed'}}
$temp=$root+'\temporary-files';$null=New-Item -ItemType Directory -Path $temp
$hostImage=$env:WINDIR+'\SysWOW64\WindowsPowerShell\v1.0\powershell.exe'
if(-not [IO.File]::Exists($hostImage)){throw 'native x86 PowerShell required'}
$stdout=$root+'\tests-stdout.log';$stderr=$root+'\tests-stderr.log'
$savedTemp=$env:TEMP;$savedTmp=$env:TMP
$p=$null;$timedOut=$false;$code=$null
try{
 $env:TEMP=$temp;$env:TMP=$temp
 $p=Start-Process -FilePath $hostImage -ArgumentList @('-NoProfile','-NonInteractive','-ExecutionPolicy','Bypass','-File',($root+'\opennav-x\tools\test-installer-pe.ps1')) -RedirectStandardOutput $stdout -RedirectStandardError $stderr -PassThru
 $null=$p.get_Handle()
 $started=$p.StartTime.ToUniversalTime().ToString('o')
 if(-not $p.WaitForExit(90000)){$timedOut=$true;$p.Kill();$p.WaitForExit()}
 $code=$p.get_ExitCode()
}finally{$env:TEMP=$savedTemp;$env:TMP=$savedTmp}
$text=[IO.File]::ReadAllText($stdout)
$passed=(-not $timedOut -and $code -eq 0 -and $text.Contains('Installer extensionless and export forwarder policy fixtures passed.') -and $text.Contains('Installer staged PE import closure fixtures passed.'))
$result=[ordered]@{schema=1;sourceCommit='87fe6c5dad6ba0875bb13f48bfbb01a6a0e70001';status=$(if($passed){'passed'}else{'failed'});host='Native Windows PowerShell x86';os=[Environment]::OSVersion.VersionString;processId=$p.Id;startedUtc=$started;exitCode=$code;timedOut=$timedOut;sourceHashesVerified=$true;nativeParsePassed=$true;scope='Actual installer PE helpers imported by AST; inert temporary PE fixtures only; no install/profile/service/application operation';navigation=@(Get-Process opencpn -ErrorAction SilentlyContinue|Select-Object Id,Responding)}
$result|ConvertTo-Json -Depth 5|Set-Content -LiteralPath ($root+'\receipt.json') -Encoding UTF8
$p.Dispose()
if(-not $passed){throw 'native PE fixtures failed; preserve evidence'}

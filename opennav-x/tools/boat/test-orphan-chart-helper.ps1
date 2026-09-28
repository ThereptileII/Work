# Pure orphan identity and process-guard tests. No vendor/boat process is started.
. (Join-Path $PSScriptRoot 'OrphanChartHelper.ps1')
$checks=New-Object 'Collections.Generic.List[string]'
function Check([bool]$Value,[string]$Name){if(-not $Value){throw $Name};$checks.Add($Name)}
function Refuse([string]$Name,[scriptblock]$Action){$failed=$false;try{$null=&$Action}catch{$failed=$true};Check $failed $Name}
function Clone($Value){return ($Value|ConvertTo-Json -Depth 8|ConvertFrom-Json)}
$now=[datetime]::UtcNow;$at=$now.AddMinutes(-2);$prepared=$at.AddHours(-1)
$context=[pscustomobject]@{sid='S-1-5-21-123';session=1;managed='C:\fixture'}
$helper=[pscustomobject]@{pid=27228;parentPid=1040;sessionId=1;sid=$context.sid;startedUtc=$at.ToString('o');path='C:\fixture\oexserverd.exe';commandLine='"C:\fixture\oexserverd.exe" -p OCPN1040'}
# Join-Path requires an actual provider drive on Linux; no files are created.
$drive=$false
if(-not (Get-PSDrive C -ErrorAction SilentlyContinue)){$null=New-PSDrive C FileSystem '/';$drive=$true}
try {
 Assert-OrphanChartHelper $helper $context 27228 1040 $at.Ticks $script:ChartHelperHash $prepared $now
 Check $true 'Exact cold-record-bound orphan identity accepted without inventing a launch receipt'
 foreach($field in @('pid','parentPid','sessionId','sid','startedUtc','path','commandLine')) {
  Refuse ('Changed orphan identity '+$field) {$bad=Clone $helper;switch($field){
   'pid' {$bad.pid++};'parentPid' {$bad.parentPid++};'sessionId' {$bad.sessionId++};'sid' {$bad.sid+='1'}
   'startedUtc' {$bad.startedUtc=$at.AddTicks(1).ToString('o')};'path' {$bad.path='C:\other\oexserverd.exe'};'commandLine' {$bad.commandLine+=' -d'}
  };Assert-OrphanChartHelper $bad $context 27228 1040 $at.Ticks $script:ChartHelperHash $prepared $now}
 }
 Refuse 'Unknown decoder bytes refused' {Assert-OrphanChartHelper $helper $context 27228 1040 $at.Ticks ('f'*64) $prepared $now}
 Refuse 'Helper predating transaction refused' {Assert-OrphanChartHelper $helper $context 27228 1040 $at.Ticks $script:ChartHelperHash $now $now}
 Refuse 'Future helper time refused' {Assert-OrphanChartHelper $helper $context 27228 1040 $at.Ticks $script:ChartHelperHash $prepared $prepared}
 Refuse 'Helper cannot be its own parent' {Assert-OrphanChartHelper $helper $context 27228 27228 $at.Ticks $script:ChartHelperHash $prepared $now}
 Refuse 'Session zero refused' {$bad=Clone $context;$bad.session=0;Assert-OrphanChartHelper $helper $bad 27228 1040 $at.Ticks $script:ChartHelperHash $prepared $now}
 foreach($processName in @('opencpn.exe','oexserverd.exe','rtl_ais.exe','ais-catcher.exe')) {
  Refuse ('Every other process still blocks: '+$processName) {Assert-PreparationProcesses @([pscustomobject]@{Name=$processName;ExecutablePath='C:\elsewhere\app.exe'}) @('C:\fixture')}
 }
 foreach($file in @('OrphanChartHelper.ps1','stop-orphan-chart-helper.ps1','Preparation.ps1','Commissioning.ps1')){
  $tokens=$null;$errors=$null;$null=[Management.Automation.Language.Parser]::ParseFile((Join-Path $PSScriptRoot $file),[ref]$tokens,[ref]$errors)
  Check ($errors.Count -eq 0) ('Parses '+$file)
 }
} finally {if($drive){Remove-PSDrive C}}
[pscustomobject]@{status='passed';count=$checks.Count;checks=$checks.ToArray();boatAccess=$false;physicalOutput=$false}|ConvertTo-Json -Depth 5

[CmdletBinding()]
param([Parameter(Mandatory=$true)][string]$Directory)
Set-StrictMode -Version Latest;$ErrorActionPreference='Stop'
$config=Get-Content -LiteralPath (Join-Path $Directory 'fixture.json') -Raw|ConvertFrom-Json
if($config.owner -cne 'OpenNavX.StockClose.Fixture.1' -or $config.exitCode -notin @(0,17)){throw 'Unknown close marker fixture.'}
Add-Type -AssemblyName System.Windows.Forms,System.Drawing
$form=New-Object Windows.Forms.Form;$form.Text='OpenNav disposable close marker';$form.Width=500;$form.Height=250
$form.Add_Shown({
 try {
 $process=[Diagnostics.Process]::GetCurrentProcess()
 $value=@{pid=$PID;startedTicks=$process.StartTime.ToUniversalTime().Ticks;handle=$form.Handle.ToInt64()}
 [IO.File]::WriteAllText((Join-Path $Directory 'ready.partial'),($value|ConvertTo-Json -Compress));[IO.File]::Move((Join-Path $Directory 'ready.partial'),(Join-Path $Directory 'ready.json'))
 } catch {[Console]::Error.WriteLine('Close marker readiness failed: '+$_.Exception.Message);$form.Close()}
})
$started=[datetime]::UtcNow;$timer=New-Object Windows.Forms.Timer;$timer.Interval=100
$timer.Add_Tick({if((Test-Path -LiteralPath (Join-Path $Directory 'release')) -or ([datetime]::UtcNow-$started).TotalSeconds -ge 45){$form.Close()}})
try{$timer.Start();[Windows.Forms.Application]::Run($form)}finally{$timer.Stop();$timer.Dispose();$form.Dispose()}
exit ([int]$config.exitCode)

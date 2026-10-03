[CmdletBinding()]
param(
 [Parameter(Mandatory=$true)][string]$SessionRecord,
 [Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{64}$')][string]$ExpectedSessionSha256,
 [Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{40}$')][string]$ExpectedCommit,
 [Parameter(Mandatory=$true)][ValidateSet('RequestMode','RequestChartPalette','ReviewChild')][string]$Action,
 [string]$Mode,[uint32]$ParentProcessId,[string]$ParentCreatedFiletime,
 [string]$ExpectedArmSha256,[string]$ExpectedReadySha256,
 [string]$CompletionFile,[string]$ExpectedCompletionSha256,
 [string]$ReviewAction='Capture',
 [string]$ChartPalette=''
)
. (Join-Path $PSScriptRoot 'RestartWindowReview.ps1')
Initialize-RestartNative
$record=Assert-LocalPath $SessionRecord;$session=Read-RestartSession $record $ExpectedSessionSha256
$job=[pscustomobject]@{action='';reviewAction='';sessionRecord=$record;sessionRecordSha256=$ExpectedSessionSha256;
 workspace=$session.workspace;executable=$session.executable;executableSha256=$session.executableSha256;generation=$session.generation;
 buildCommit=$ExpectedCommit;helperFiles=@($script:RestartWindowFiles|ForEach-Object {@{name=$_;sha256=(Get-Digest (Join-Path $PSScriptRoot $_))}})}
Assert-RestartChartPalette $Mode $ChartPalette
if(($Action -ceq 'RequestChartPalette') -ne [bool]$ChartPalette){throw 'Only RequestChartPalette accepts/requires explicit XNav/Standard.'}
if($Action -cin @('RequestMode','RequestChartPalette')) {
 if($Mode -cnotin @('--xnav','--legacy','--safe-mode') -or -not $ParentProcessId -or -not $ParentCreatedFiletime -or
    $CompletionFile -or $ExpectedCompletionSha256 -or $ReviewAction -cne 'Capture'){throw 'RequestMode needs exact parent, mode and Arm/readiness hashes only.'}
 $job.action='RequestGuardedMode';$job.reviewAction=$Action
 $job|Add-Member -NotePropertyMembers @{processId=$ParentProcessId;processCreatedFiletime=$ParentCreatedFiletime;mode=$Mode;chartPalette=$ChartPalette;
   armFile=(Join-Path ([IO.Path]::GetDirectoryName($record)) ('arm-'+$ParentProcessId+'-'+$ParentCreatedFiletime+'.json'));
   armSha256=$ExpectedArmSha256;readySha256=$ExpectedReadySha256}
} else {
 if($Mode -or $ParentProcessId -or $ParentCreatedFiletime -or $ExpectedArmSha256 -or $ExpectedReadySha256 -or
    -not $CompletionFile -or $ExpectedCompletionSha256 -cnotmatch '^[a-f0-9]{64}$'){throw 'ReviewChild needs exact completed transition evidence only.'}
 $completionPath=Assert-LocalPath $CompletionFile
 if([IO.Path]::GetFileName($completionPath) -cne 'completion.json' -or (Get-Digest $completionPath) -cne $ExpectedCompletionSha256){throw 'Exact completion record required.'}
 $completion=Read-Record $completionPath
 $permit=Read-Record (Join-Path ([IO.Path]::GetDirectoryName($completionPath)) 'permit-consumed.json')
 $job.action='ReviewRestartChild';$job.reviewAction=$ReviewAction
 $job|Add-Member -NotePropertyMembers @{processId=[uint32]$completion.child.pid;processCreatedFiletime=$completion.child.createdFiletime;
   mode=$permit.mode;completionFile=$completionPath;completionSha256=$ExpectedCompletionSha256}
}
$null=Read-RestartWindowReview $job
$directory=New-PreparationDirectory ([pscustomobject]@{workspace=$session.workspace;sid=$session.sid}) 'restart-window'
$job|Add-Member -NotePropertyName evidenceDirectory -NotePropertyValue $directory
$job.PSObject.Properties.Remove('workspace')
$result=Invoke-InteractiveJob $session.workspace $job
Write-Record (Join-Path $directory 'review.json') $result
$result|ConvertTo-Json -Depth 12

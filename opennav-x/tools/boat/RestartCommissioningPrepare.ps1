# Prepare only. Does not launch, change a profile, or alter the existing audit.
[CmdletBinding()]
param([string]$Workspace='C:\XNav',[Parameter(Mandatory=$true)][string]$ShutdownReview,[Parameter(Mandatory=$true)][string]$ShutdownReviewSha256)
. (Join-Path $PSScriptRoot 'RestartCommissioning.ps1')
$config=Get-Target $Workspace;$installed=Get-Installed
$environment=Assert-ReadOnlyAudit $config $installed $Workspace
$buildFile=Join-Path $installed.generation 'docs\PRODUCT_BUILD.json'
$buildRecord=@($installed.ownership.managedFiles | Where-Object {$_.path -ceq 'docs/PRODUCT_BUILD.json'})
$helper=Join-Path ([IO.Path]::GetDirectoryName($installed.executable)) 'opennav-restart.exe'
$helperRecord=@($installed.ownership.managedFiles | Where-Object {$_.path -ceq 'app/opennav-restart.exe'})
if($buildRecord.Count -ne 1 -or (Get-Digest $buildFile) -cne $buildRecord[0].sha256 -or $helperRecord.Count -ne 1 -or (Get-Digest $helper) -cne $helperRecord[0].sha256){throw 'Owned fixture-free build and restart helper required.'}
$build=Read-Record $buildFile
Assert-RestartBuild $build $installed.ownership.commit (Get-Digest $installed.executable) (Get-Digest $helper)
. (Join-Path $PSScriptRoot 'Commissioning.ps1')
$context=Get-CommissioningContext $Workspace
$directory=New-PreparationDirectory $context 'restart-session'
$profile=Join-Path $context.profile 'opencpn.ini';$before=Join-Path $directory 'before.ini';$hash=Get-Digest $profile
if($hash -cne $config.readOnlyAudit.profileIniSha256){throw 'Cold audit changed before session preparation.'}
Copy-PreparationFile $profile $before $hash (Get-Item -LiteralPath $profile).Length
$plan=Read-Record (Join-Path ([IO.Path]::GetDirectoryName($config.readOnlyAudit.commissioning.record)) 'review-plan.json')
if($ShutdownReviewSha256 -cnotmatch '^[a-f0-9]{64}$' -or (Get-Digest $ShutdownReview) -cne $ShutdownReviewSha256){throw 'Exact independently inspected shutdown review required.'}
Assert-RestartShutdownReview (Read-Record $ShutdownReview) @($plan.plugins | Where-Object {$_.decision -ceq 'retain'})
$shutdownCopy=Join-Path $directory 'shutdown-review.json'
Copy-PreparationFile $ShutdownReview $shutdownCopy $ShutdownReviewSha256 (Get-Item -LiteralPath $ShutdownReview).Length
$scripts=@($script:RestartDependencies | ForEach-Object {@{name=$_;sha256=(Get-Digest (Join-Path $PSScriptRoot $_))}})
$record=Join-Path $directory 'session.json';$now=[DateTime]::UtcNow
$session=@{schema=1;owner=$script:RestartOwner;session=(New-RestartToken);createdUtc=$now.ToString('o');expiresUtc=$now.AddHours(4).ToString('o');
 sid=$context.sid;windowsSessionId=$context.session.ToString();workspace=$context.workspace;generation=$installed.state.current;buildCommit=$installed.ownership.commit;
 executable=$installed.executable;executableSha256=(Get-Digest $installed.executable);helper=$helper;helperSha256=(Get-Digest $helper);
 productBuild=$buildFile;productBuildSha256=(Get-Digest $buildFile);profile=$profile;beforeIni=$before;beforeIniSha256=$hash;audit=$config.readOnlyAudit;targetSha256=(Get-Digest (Join-Path $Workspace 'boat-target.json'));
 shutdownReview=$shutdownCopy;shutdownReviewSha256=$ShutdownReviewSha256;workingDirectory=$environment.workingDirectory;path=$environment.path;toolDirectory=$PSScriptRoot;scripts=$scripts}
Write-Record $record $session
$recordHash=Get-Digest $record;$null=Read-RestartSession $record $recordHash
[pscustomobject]@{status='prepared-only';record=$record;recordSha256=$recordHash;session=$session.session;
 environment=@{OPENNAV_COMMISSIONING_RESTART_SESSION=$session.session;OPENNAV_COMMISSIONING_RESTART_RECORD_SHA256=$recordHash};
 applicationLaunched=$false;profileChanged=$false} | ConvertTo-Json -Depth 6

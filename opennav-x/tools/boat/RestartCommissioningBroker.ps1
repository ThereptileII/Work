# Deliberately arm ONE restart before its human mode-switch action. This broker
# never sends UI input, starts a child, changes an INI, kills, retries or restores.
[CmdletBinding()]
param(
 [Parameter(Mandatory=$true)][string]$SessionRecord,
 [Parameter(Mandatory=$true)][string]$ExpectedSha256,
 [Parameter(Mandatory=$true)][uint32]$ParentProcessId,
 [Parameter(Mandatory=$true)][string]$ParentCreatedFiletime,
 [Parameter(Mandatory=$true)][ValidateSet('--xnav','--legacy','--safe-mode')][string]$Mode
)
. (Join-Path $PSScriptRoot 'RestartCommissioning.ps1')
Initialize-RestartNative
$session=Read-RestartSession $SessionRecord $ExpectedSha256
$nativePowerShell=Join-Path ([Environment]::GetFolderPath('System')) 'WindowsPowerShell\v1.0\powershell.exe'
$broker=Get-RestartProcess ([uint32]$PID) $session $nativePowerShell
$parent=Get-RestartProcess $ParentProcessId $session $session.executable
Assert-RestartDecimal $ParentCreatedFiletime 'parent creation'
if($parent.createdFiletime -cne $ParentCreatedFiletime -or [DateTime]::FromFileTimeUtc([long]$ParentCreatedFiletime) -lt [DateTime]::Parse($session.createdUtc).ToUniversalTime()){throw 'Parent is not the exact post-preparation process.'}
# Keep a genuine process handle open through exit and audit; PID reuse cannot
# satisfy this handle or the separately checked peer/request creation times.
$parentHandle=Get-Process -Id $ParentProcessId
$pipe=$null;$directory=$null;$ownsDirectory=$false;$allowIssued=$false
try {
 $null=$parentHandle.Handle
 if($parentHandle.HasExited -or $parentHandle.StartTime.ToUniversalTime().ToFileTimeUtc().ToString() -cne $ParentCreatedFiletime){throw 'Parent exited/reused while arming.'}
 $baseline=Get-RestartBaseline $session $SessionRecord $parent
 $pipe=[OpenNavX.RestartCommissioningNative]::NewPipe($session.session,$session.sid)
 $directory=Assert-LocalPath $baseline.nextDirectory
 if(Test-Path -LiteralPath $directory){throw 'This parent transition has already been armed; do not retry.'}
 $null=New-Item -ItemType Directory -Path $directory
 $ownsDirectory=$true
 Write-Record (Join-Path $directory 'ready.json') @{owner=$script:RestartOwner;session=$session.session;recordSha256=$ExpectedSha256;parent=$parent;broker=$broker;mode=$Mode;beforeSha256=$baseline.sha256;createdUtc=[DateTime]::UtcNow.ToString('o')}
 [pscustomobject]@{status='listening-for-one-explicit-restart';parentPid=$parent.pid;mode=$Mode;directory=$directory} | ConvertTo-Json -Compress
 [OpenNavX.RestartCommissioningNative]::Connect($pipe,[DateTime]::UtcNow.AddSeconds(120))
 $deadline=[DateTime]::UtcNow.AddSeconds(120)
 $peer=[OpenNavX.RestartCommissioningNative]::ClientPid($pipe)
 $helper=Get-RestartProcess $peer $session $session.helper
 $payload=[OpenNavX.RestartCommissioningNative]::ReadFrame($pipe,$deadline)
 $request=[OpenNavX.RestartCommissioningNative]::Message($payload)
 Assert-RestartRequest $request $session $parent $Mode $peer.ToString()
 if($helper.createdFiletime -cne $request['helperCreatedFiletime'] -or [long]$helper.createdFiletime -lt [long]$parent.createdFiletime){throw 'Helper creation identity differs.'}
 if(-not $parentHandle.WaitForExit(1000) -or $parentHandle.ExitCode -ne 0){throw 'The inspected parent has not exited normally.'}
 $requestHash=Get-RestartBytesHash $payload
 Save-RestartWire (Join-Path $directory 'request.json') $payload
 # All plugin helpers must now be closed. The sole restart helper lives outside
 # the stock/managed/plugin roots and is separately pinned above, not exempted
 # from an otherwise weakened closed-process rule.
 $session=Read-RestartSession $SessionRecord $ExpectedSha256
 $config=Get-Target $session.workspace;$installed=Get-Installed
 $postHash=Get-Digest $session.profile
 $postCopy=Join-Path $directory 'post-close.ini'
 Copy-PreparationFile $session.profile $postCopy $postHash (Get-Item -LiteralPath $session.profile).Length
 # Parse the exact retained bytes whose digest was proven during the copy.
 # Never prove a live A hash while accidentally parsing a transient B version.
 $beforeValues=Read-RestartIni $baseline.path;$afterValues=Read-RestartIni $postCopy
 $changes=@(Assert-RestartIniDelta $beforeValues $afterValues $Mode)
 Assert-InputOnlyProfile $afterValues
 Write-Record (Join-Path $directory 'reviewed-delta.json') @{owner=$script:RestartOwner;beforeSha256=$baseline.sha256;afterSha256=$postHash;mode=$Mode;changes=$changes}
 # Only an in-memory audit copy gets the newly PROVEN hash. The independent
 # boat-target audit remains immutable, and the complete cold verifier still
 # checks quarantine, every plugin/helper tree, stock and active transaction.
 $config.readOnlyAudit.profileIniSha256=$postHash
 $environment=Assert-ReadOnlyAudit $config $installed $session.workspace
 if($environment.workingDirectory -cne $session.workingDirectory -or $environment.path -cne $session.path -or
    (Get-Digest $session.profile) -cne $postHash -or (Get-Digest $postCopy) -cne $postHash -or (Get-Digest $baseline.path) -cne $baseline.sha256){throw 'Reviewed profile/environment changed during audit.'}
 $helperAgain=Get-RestartProcess $peer $session $session.helper
 if($helperAgain.createdFiletime -cne $helper.createdFiletime -or $parentHandle.ExitCode -ne 0){throw 'Peer/parent lifetime changed before permit.'}
 if([DateTime]::UtcNow -ge $deadline){throw 'Restart audit exceeded the absolute deadline.'}
 # Expiry and immutable tool/source evidence are checked again after the
 # expensive full-tree audit, not merely when the request first arrived.
 $session=Read-RestartSession $SessionRecord $ExpectedSha256
 if((Get-Digest $session.profile) -cne $postHash -or (Get-Digest $postCopy) -cne $postHash -or [DateTime]::UtcNow -ge $deadline){throw 'Profile/deadline changed before permit consumption.'}
 $permitId=New-RestartToken;$issued=[DateTime]::UtcNow;$expires=$issued.AddSeconds(10)
 $permit=@{owner=$script:RestartOwner;status='consumed-before-allow';session=$session.session;recordSha256=$ExpectedSha256;permitId=$permitId;requestSha256=$requestHash;
 mode=$Mode;parent=$parent;helper=$helper;beforeSha256=$baseline.sha256;profileSha256=$postHash;issuedFiletime=$issued.ToFileTimeUtc().ToString();expiresFiletime=$expires.ToFileTimeUtc().ToString()}
 $permitPath=Publish-RestartPermit $directory $permit
 $fields=[string[]]@($script:RestartMagic,'ALLOW',$session.session,$ExpectedSha256,$request['nonce'],$requestHash,$permit.issuedFiletime,$permit.expiresFiletime,
 $session.executable,$session.executableSha256,$session.helper,$session.helperSha256,$session.profile,$postHash,$environment.workingDirectory,$environment.path,$permitId)
 # From this point even a failed/partial pipe write is an uncertain consumed
 # permit. Never send DENY/retry or infer that a child did not start.
 $allowIssued=$true
 [OpenNavX.RestartCommissioningNative]::WriteFrame($pipe,[OpenNavX.RestartCommissioningNative]::Reply($fields),$deadline)
 $receiptPayload=[OpenNavX.RestartCommissioningNative]::ReadFrame($pipe,$deadline)
 $receipt=[OpenNavX.RestartCommissioningNative]::Message($receiptPayload)
 Assert-RestartReceipt $receipt $request $requestHash $permitId
 Save-RestartWire (Join-Path $directory 'receipt.json') $receiptPayload
 if($receipt['status'] -cne 'started'){throw 'Native child creation failed; permit stays consumed.'}
 $child=Get-RestartProcess ([uint32]$receipt['childPid']) $session $session.executable
 if($child.createdFiletime -cne $receipt['childCreatedFiletime'] -or [long]$child.createdFiletime -lt $issued.ToFileTimeUtc()){throw 'Child receipt does not match the actual new process.'}
 Write-Record (Join-Path $directory 'completion.json') @{owner=$script:RestartOwner;session=$session.session;recordSha256=$ExpectedSha256;requestSha256=$requestHash;permitSha256=(Get-Digest $permitPath);receiptSha256=(Get-Digest (Join-Path $directory 'receipt.json'));child=$child;status='child-identity-verified';completedUtc=[DateTime]::UtcNow.ToString('o')}
 [pscustomobject]@{status='child-identity-verified';directory=$directory;childPid=$child.pid;mode=$Mode;profileWritten=$false} | ConvertTo-Json -Compress
} catch {
 if($pipe -and $pipe.IsConnected -and -not $allowIssued){try{[OpenNavX.RestartCommissioningNative]::WriteFrame($pipe,[OpenNavX.RestartCommissioningNative]::Reply([string[]]@($script:RestartMagic,'DENY')),[DateTime]::UtcNow.AddSeconds(2))}catch{}}
 if($ownsDirectory -and (Test-Path -LiteralPath $directory)){Write-Record (Join-Path $directory 'failure.json') @{owner=$script:RestartOwner;status=$(if($allowIssued){'uncertain-or-failed-consumed-permit'}else{'denied'});message=$_.Exception.Message;at=[DateTime]::UtcNow.ToString('o')}}
 throw
} finally {if($pipe){$pipe.Dispose()};$parentHandle.Dispose()}

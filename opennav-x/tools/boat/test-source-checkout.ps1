# Local temporary Git repositories only: no network, profile, application or boat.
[CmdletBinding()]
param([switch]$PortableContracts,[switch]$IsolatedLocal)
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'
$native=[Environment]::OSVersion.Platform -eq 'Win32NT'
if (-not $native -and -not $PortableContracts) { throw 'Choose portable contracts explicitly outside Windows.' }
if ($native -and -not $IsolatedLocal -and $env:GITHUB_ACTIONS -ne 'true') { throw 'Choose isolated local testing explicitly outside CI.' }
. (Join-Path $PSScriptRoot 'SourceCheckout.ps1')
$testRoot=Join-Path ([IO.Path]::GetTempPath()) ('OpenNav source & fixture '+[guid]::NewGuid().ToString('N'))
$null=New-Item -ItemType Directory -Path $testRoot
if (-not $native) {
  function Assert-LocalPath([string]$Path) {
    $full=[IO.Path]::GetFullPath($Path)
    if ($full -ne $testRoot -and -not $full.StartsWith($testRoot+'/',[StringComparison]::Ordinal)) { throw 'Test path escaped its disposable root.' }
    $walk=$full
    while ($walk -ne $testRoot) {
      if ((Test-Path -LiteralPath $walk) -and ((Get-Item -LiteralPath $walk -Force).Attributes -band [IO.FileAttributes]::ReparsePoint)) { throw 'Redirected source path refused.' }
      $walk=[IO.Path]::GetDirectoryName($walk)
    }
    return $full
  }
}
$checks=New-Object 'Collections.Generic.List[string]'
function Reject([scriptblock]$Action,[string]$Reason) { $rejected=$false;try {$null=& $Action} catch {$rejected=$true};if (-not $rejected) {throw ('Unsafe source operation accepted: '+$Reason)} }
try {
  foreach ($name in @('SourceCheckout.ps1','update-source.ps1')) {
    $tokens=$null;$errors=$null;$null=[Management.Automation.Language.Parser]::ParseFile((Join-Path $PSScriptRoot $name),[ref]$tokens,[ref]$errors)
    if ($errors.Count) { throw ($errors | Out-String) }
  }
  $checks.Add('Entry point and helpers parse without running boat preflight')
  $seed=Join-Path $testRoot 'seed & origin';$null=Invoke-SourceGit '' @('init','--quiet',$seed)
  $null=Invoke-SourceGit $seed @('config','user.name','Source test')
  $null=Invoke-SourceGit $seed @('config','user.email','source-test@example.invalid')
  [IO.File]::WriteAllText((Join-Path $seed 'file.txt'),"first`n")
  [IO.File]::WriteAllText((Join-Path $seed '.gitignore'),"ignored/`n")
  $null=Invoke-SourceGit $seed @('add','.')
  $null=Invoke-SourceGit $seed @('-c','commit.gpgsign=false','commit','--quiet','-m','First fixture')
  $first=@(Invoke-SourceGit $seed @('rev-parse','HEAD'))[0]
  [IO.File]::WriteAllText((Join-Path $seed 'file.txt'),"second`n")
  $null=Invoke-SourceGit $seed @('add','.')
  $null=Invoke-SourceGit $seed @('-c','commit.gpgsign=false','commit','--quiet','-m','Second fixture')
  $second=@(Invoke-SourceGit $seed @('rev-parse','HEAD'))[0]
  $legacy=Join-Path $testRoot 'old no checkout'
  $null=Invoke-SourceGit '' @('clone','--quiet','--no-checkout',$seed,$legacy)
  if (-not @(Invoke-SourceGit $legacy @('status','--porcelain')).Count -or @(Invoke-SourceGit $legacy @('ls-files')).Count) { throw 'No-checkout regression fixture did not reproduce the empty-index/deletion state.' }
  $checks.Add('Reproduces clone --no-checkout reporting tracked deletions with an empty index')
  $workspace=Join-Path $testRoot 'managed & workspace';$null=New-Item -ItemType Directory -Path $workspace
  $environment=@{};foreach($name in @('GIT_TERMINAL_PROMPT','GIT_ASKPASS','SSH_ASKPASS','GCM_INTERACTIVE')) {$environment[$name]=[Environment]::GetEnvironmentVariable($name,'Process')}
  $template=Join-Path $testRoot 'unrelated template';$null=New-Item -ItemType Directory -Path $template
  [IO.File]::WriteAllText((Join-Path $template 'unreviewed-template-marker'),'not an owned Git template')
  $oldTemplate=[Environment]::GetEnvironmentVariable('GIT_TEMPLATE_DIR','Process')
  try {
    [Environment]::SetEnvironmentVariable('GIT_TEMPLATE_DIR',$template,'Process')
    $created=Update-SourceCheckout $workspace $first $seed
  } finally {
    if($null -eq $oldTemplate){[Environment]::SetEnvironmentVariable('GIT_TEMPLATE_DIR',[NullString]::Value,'Process')}
    else {[Environment]::SetEnvironmentVariable('GIT_TEMPLATE_DIR',$oldTemplate,'Process')}
  }
  if (Test-Path -LiteralPath (Join-Path $created.source '.git/unreviewed-template-marker')) { throw 'Fresh checkout inherited an unrelated Git template.' }
  $checks.Add('Fresh initialization uses its explicit empty owned template, ignoring ambient templates')
  if (-not $created.fresh -or $created.commit -cne $first -or [IO.File]::ReadAllText((Join-Path $created.source 'file.txt')).TrimEnd("`r`n") -cne 'first') { throw 'Fresh exact source publication failed.' }
  foreach($name in $environment.Keys) {if([Environment]::GetEnvironmentVariable($name,'Process') -cne $environment[$name]){throw ('Source update altered caller credential environment: '+$name)}}
  $checks.Add('Fresh staged fetch publishes exact detached source with spaces/ampersands and restores credential environment')
  $updated=Update-SourceCheckout $workspace $second $seed
  if ($updated.fresh -or $updated.commit -cne $second -or [IO.File]::ReadAllText((Join-Path $updated.source 'file.txt')).TrimEnd("`r`n") -cne 'second') { throw 'Owned update failed.' }
  $checks.Add('Existing owned clean detached checkout updates to the exact new commit')
  $source=$updated.source;$tracked=Join-Path $source 'file.txt'
  # Save Git's actual checkout bytes. Recreating LF text after a CRLF checkout
  # is a fixture edit of its own, even when Git initially normalizes status.
  $trackedBaseline=[IO.File]::ReadAllBytes($tracked);$baselineDigest=Get-Digest $tracked
  [IO.File]::AppendAllText($tracked,'user edit')
  $changed=Get-Digest $tracked
  Reject {Update-SourceCheckout $workspace $first $seed} 'tracked user edit'
  if ((Get-Digest $tracked) -cne $changed) { throw 'Dirty tracked file was overwritten.' }
  [IO.File]::WriteAllBytes($tracked,$trackedBaseline)
  if ((Get-Digest $tracked) -cne $baselineDigest) { throw 'Dirty-file fixture did not restore exact native checkout bytes.' }
  Assert-SourceClean $source
  $untracked=Join-Path $source 'private.txt';[IO.File]::WriteAllText($untracked,'user data')
  Reject {Update-SourceCheckout $workspace $first $seed} 'untracked user file'
  Remove-Item -LiteralPath $untracked
  $ignored=Join-Path $source 'ignored';$null=New-Item -ItemType Directory -Path $ignored
  [IO.File]::WriteAllText((Join-Path $ignored 'build.txt'),'user build')
  Reject {Update-SourceCheckout $workspace $first $seed} 'ignored user artifact'
  Remove-Item -LiteralPath $ignored -Recurse
  $checks.Add('Tracked edits, untracked files and ignored build artifacts are refused and preserved')
  $null=Invoke-SourceGit $source @('update-index','--assume-unchanged','file.txt')
  Reject {Update-SourceCheckout $workspace $first $seed} 'hidden index edit flag'
  $null=Invoke-SourceGit $source @('update-index','--no-assume-unchanged','file.txt')
  $null=Invoke-SourceGit $source @('update-index','--skip-worktree','file.txt')
  Reject {Update-SourceCheckout $workspace $first $seed} 'sparse/skip-worktree flag'
  $null=Invoke-SourceGit $source @('update-index','--no-skip-worktree','file.txt')
  $checks.Add('Assume-unchanged and skip-worktree index flags cannot hide local changes')
  $null=Invoke-SourceGit $source @('remote','set-url','origin',(Join-Path $testRoot 'other-origin'))
  Reject {Update-SourceCheckout $workspace $first $seed} 'different remote'
  $null=Invoke-SourceGit $source @('remote','set-url','origin',$seed)
  $checks.Add('Unknown origin is refused before fetch or checkout')
  $null=Invoke-SourceGit $source @('checkout','--quiet','--detach',$first,'--')
  Reject {Update-SourceCheckout $workspace $second $seed} 'unmanaged detached HEAD'
  $null=Invoke-SourceGit $source @('checkout','--quiet','--detach',$second,'--')
  $null=Invoke-SourceGit $source @('checkout','--quiet','-b','user-branch')
  Reject {Update-SourceCheckout $workspace $first $seed} 'user branch'
  $null=Invoke-SourceGit $source @('checkout','--quiet','--detach',$second,'--')
  $checks.Add('User commits/checkouts and named branches remain untouched')
  $hook=Join-Path $source '.git/hooks/post-checkout'
  $null=New-Item -ItemType Directory -Path ([IO.Path]::GetDirectoryName($hook)) -Force
  [IO.File]::WriteAllText($hook,"#!/bin/sh`nprintf executed > hook-ran.txt`n",(New-Object Text.UTF8Encoding($false)))
  if (-not $native) { & chmod '+x' $hook;if($LASTEXITCODE -ne 0){throw 'Fixture hook could not be made executable.'} }
  $null=Update-SourceCheckout $workspace $first $seed
  if (Test-Path -LiteralPath (Join-Path $source 'hook-ran.txt')) { throw 'Local checkout hook executed.' }
  $checks.Add('Checkout disables local hooks instead of executing unrelated source automation')
  foreach ($name in @('GIT_DIR','GIT_COMMON_DIR','GIT_WORK_TREE','GIT_INDEX_FILE','GIT_OBJECT_DIRECTORY','GIT_ALTERNATE_OBJECT_DIRECTORIES','GIT_NAMESPACE')) {
    $original=[Environment]::GetEnvironmentVariable($name,'Process')
    try {
      [Environment]::SetEnvironmentVariable($name,(Join-Path $testRoot 'unowned-redirection'),'Process')
      Reject {Update-SourceCheckout $workspace $second $seed} 'inherited Git filesystem redirection'
    } finally {
      if($null -eq $original){[Environment]::SetEnvironmentVariable($name,[NullString]::Value,'Process')}
      else {[Environment]::SetEnvironmentVariable($name,$original,'Process')}
    }
  }
  if(Test-Path -LiteralPath (Join-Path $testRoot 'unowned-redirection')){throw 'Git redirection touched an unowned path.'}
  $checks.Add('Inherited Git directory/index/object redirection cannot reach an unowned path')
  # Windows junctions require no symlink privilege. Linux also exercises the
  # confirmed file-level config redirect; both must fail before Git is invoked.
  if (-not $native) {
    $config=Join-Path $source '.git/config';$external=Join-Path $testRoot 'external config'
    [IO.File]::Move($config,$external);$before=Get-Digest $external
    try {
      $null=New-Item -ItemType SymbolicLink -Path $config -Target $external
      Reject {Update-SourceCheckout $workspace $second $seed} 'linked Git config'
      if ((Get-Digest $external) -cne $before) { throw 'Git wrote through linked config into an unrelated file.' }
    } finally { if (Test-Path -LiteralPath $config) { [IO.File]::Delete($config) };[IO.File]::Move($external,$config) }
  }
  $refs=Join-Path $source '.git/refs';$external=Join-Path $testRoot 'external refs'
  [IO.Directory]::Move($refs,$external)
  $marker=Join-Path $external 'preserve-marker';[IO.File]::WriteAllText($marker,'unowned refs contents');$before=Get-Digest $marker
  try {
    $kind=if($native){'Junction'}else{'SymbolicLink'}
    $null=New-Item -ItemType $kind -Path $refs -Target $external
    Reject {Update-SourceCheckout $workspace $second $seed} 'redirected Git refs directory'
    if ((Get-Digest $marker) -cne $before -or @(Get-ChildItem -LiteralPath $external -Force).Count -ne 3) { throw 'Redirected metadata tree was changed.' }
  } finally { if (Test-Path -LiteralPath $refs) { [IO.Directory]::Delete($refs,$false) };Remove-Item -LiteralPath $marker;[IO.Directory]::Move($external,$refs) }
  $checks.Add('Git metadata symlinks/junctions are refused before writes and preserve unrelated bytes')
  foreach ($relative in @('.git/commondir','.git/objects/info/alternates','.git/objects/info/http-alternates')) {
    $redirect=Join-Path $source $relative;$null=New-Item -ItemType Directory -Path ([IO.Path]::GetDirectoryName($redirect)) -Force
    [IO.File]::WriteAllText($redirect,(Join-Path $testRoot 'external git storage'))
    try { Reject {Update-SourceCheckout $workspace $second $seed} 'external Git metadata/object storage' }
    finally { Remove-Item -LiteralPath $redirect }
  }
  $checks.Add('Plain-text common-directory and local/HTTP object alternates are refused')
  $unknownWorkspace=Join-Path $testRoot 'unknown-workspace';$null=New-Item -ItemType Directory -Path $unknownWorkspace
  $unknown=Join-Path $unknownWorkspace 'source';$null=Invoke-SourceGit '' @('clone','--quiet',$seed,$unknown)
  Reject {Update-SourceCheckout $unknownWorkspace $first $seed} 'same-origin but unowned checkout'
  if (-not [IO.File]::Exists((Join-Path $unknown 'file.txt'))) {throw 'Unowned checkout was damaged.'}
  $checks.Add('Matching origin alone never grants ownership of an existing checkout')
  $failed=Join-Path $testRoot 'failed-workspace';$null=New-Item -ItemType Directory -Path $failed
  Reject {Update-SourceCheckout $failed ('0'*40) $seed} 'missing exact commit'
  if (Test-Path -LiteralPath (Join-Path $failed 'source')) {throw 'Failed fetch published source.'}
  if (@(Get-ChildItem -LiteralPath (Join-Path $failed 'runs') -Directory).Count -ne 1) {throw 'Failed source stage/journal was not retained.'}
  Write-Record (Join-Path $failed 'source-owner.json') @{owner='interrupted-fixture'}
  Reject {Update-SourceCheckout $failed $first $seed} 'ownership without published checkout'
  $checks.Add('Failed fetch retains its stage/journal without publication; interrupted ownership refuses replacement')
  $owner=Join-Path $workspace 'source-owner.json';[IO.File]::AppendAllText($owner,' ')
  Reject {Update-SourceCheckout $workspace $second $seed} 'changed ownership record'
  $checks.Add('Copied ownership records must remain byte-identical')
  [pscustomobject]@{status='passed';environment=$(if($native){'native-windows-disposable-git'}else{'linux-portable-git-contracts'});count=$checks.Count;checks=@($checks);networkAccess=$false;boatAccess=$false;applicationChanged=$false} | ConvertTo-Json -Depth 5
} finally { Remove-Item -LiteralPath $testRoot -Recurse -Force }

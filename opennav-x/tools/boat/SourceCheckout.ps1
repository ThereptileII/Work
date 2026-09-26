# Git-only source maintenance. Never builds, installs, launches or changes a profile.
. (Join-Path $PSScriptRoot 'Common.ps1')
function Invoke-SourceGit([string]$Directory,[string[]]$Arguments) {
  foreach ($name in @('GIT_DIR','GIT_COMMON_DIR','GIT_WORK_TREE','GIT_INDEX_FILE','GIT_OBJECT_DIRECTORY','GIT_ALTERNATE_OBJECT_DIRECTORIES','GIT_NAMESPACE')) {
    if ([Environment]::GetEnvironmentVariable($name,'Process')) { throw 'Git directory/index/object redirection requires separate review.' }
  }
  $prefix=@('--no-replace-objects','-c','credential.helper=','-c','credential.interactive=false','-c','core.askPass=','-c','core.hooksPath=','-c','core.fsmonitor=false','-c','submodule.recurse=false','-c','gc.auto=0','-c','maintenance.auto=false')
  if ($Directory) { $prefix+=@('-C',$Directory) }
  $output=@(& git @prefix @Arguments)
  if ($LASTEXITCODE -ne 0) { throw ('Git source operation failed: '+$Arguments[0]) }
  return $output
}
function Assert-SourceTree([string]$Directory) {
  # Check each entry before descending. Get-ChildItem -Recurse could follow a
  # junction before it is inspected; Git itself may write through config links.
  $directory=Assert-LocalPath $Directory
  $pending=New-Object 'Collections.Generic.Queue[string]';$pending.Enqueue($directory);$count=0
  while ($pending.Count) {
    $current=$pending.Dequeue()
    foreach ($entry in @(Get-ChildItem -LiteralPath $current -Force)) {
      $count++;if ($count -gt 200000) { throw 'Source inventory exceeds the reviewed bound.' }
      if ($entry.Attributes -band [IO.FileAttributes]::ReparsePoint) { throw 'Source or Git metadata contains a redirected entry; preserve it.' }
      if ($entry.PSIsContainer) { $pending.Enqueue($entry.FullName) }
    }
  }
  foreach ($relative in @('.git/commondir','.git/objects/info/alternates','.git/objects/info/http-alternates')) {
    if (Test-Path -LiteralPath (Join-Path $directory $relative)) { throw 'External Git metadata/object storage requires separate review.' }
  }
}
function Assert-SourceOrigin([string]$Directory,[string]$Origin) {
  $value=@(Invoke-SourceGit $Directory @('remote','get-url','origin'))
  if ($value.Count -ne 1 -or $value[0] -cne $Origin) { throw 'Existing source has another/effectively rewritten origin; preserve it.' }
}
function Assert-SourceClean([string]$Directory) {
  Assert-SourceTree $Directory
  if (@(Invoke-SourceGit $Directory @('status','--porcelain=v1','--untracked-files=all','--ignored=matching','--ignore-submodules=none')).Count) { throw 'Source has tracked, untracked or ignored local changes; preserve them.' }
  foreach ($line in @(Invoke-SourceGit $Directory @('ls-files','-v'))) {
    if (-not $line.StartsWith('H ',[StringComparison]::Ordinal)) { throw 'Modified index visibility flags require manual review; preserve source.' }
  }
}
function Assert-OwnedSource([string]$Directory,[string]$Ownership,[string]$Origin) {
  $directory=Assert-LocalPath $Directory;$ownership=Assert-LocalPath $Ownership
  if (-not [IO.Directory]::Exists($directory) -or -not [IO.File]::Exists($ownership)) { throw 'Existing source is not an owned checkout; preserve it.' }
  $gitDirectory=Assert-LocalPath (Join-Path $directory '.git')
  if (-not [IO.Directory]::Exists($gitDirectory)) { throw 'Git worktrees, redirects and unowned git directories are not supported.' }
  Assert-SourceTree $directory
  $inside=Join-Path $gitDirectory 'opennav-source-owner.json'
  if ((Get-Digest $inside) -cne (Get-Digest $ownership)) { throw 'Source ownership records disagree.' }
  $record=Read-Record $ownership
  if ($record.schema -ne 1 -or $record.owner -cne 'OpenNavX.SourceCheckout.1' -or $record.origin -cne $Origin -or
      $record.directory -ine $directory -or $record.identity -cnotmatch '^[a-f0-9]{32}$') { throw 'Unknown source ownership; preserve it.' }
  Assert-SourceOrigin $directory $Origin
  $top=@(Invoke-SourceGit $directory @('rev-parse','--show-toplevel'))
  if ($top.Count -ne 1 -or [IO.Path]::GetFullPath($top[0]) -ine $directory) { throw 'Source is not this exact checkout root.' }
  $head=@(Invoke-SourceGit $directory @('rev-parse','--verify','HEAD'))
  $managed=@(Invoke-SourceGit $directory @('config','--local','--get','opennav.lastManagedCommit'))
  $branch=@(Invoke-SourceGit $directory @('rev-parse','--abbrev-ref','HEAD'))
  if ($head.Count -ne 1 -or $managed.Count -ne 1 -or $head[0] -cnotmatch '^[a-f0-9]{40}$' -or $head[0] -cne $managed[0] -or $branch[0] -cne 'HEAD') { throw 'Source HEAD was changed outside this tool; preserve local commits/branches.' }
  Assert-SourceClean $directory
  return $head[0]
}
function Update-SourceCheckout([string]$Workspace,[string]$Commit,[string]$Origin) {
  if ($Commit -cnotmatch '^[a-f0-9]{40}$') { throw 'A complete immutable commit SHA is required.' }
  $workspace=Assert-LocalPath $Workspace
  $directory=Assert-LocalPath (Join-Path $workspace 'source')
  $ownership=Join-Path $workspace 'source-owner.json'
  $fresh=-not (Test-Path -LiteralPath $directory)
  if ($fresh -and (Test-Path -LiteralPath $ownership)) { throw 'Source ownership exists without its checkout; inspect interrupted work instead of replacing it.' }
  # Suppress all credential/prompt helpers only in this PowerShell process, then
  # restore its environment. No user/machine Git configuration is written.
  $environment=@{}
  foreach ($name in @('GIT_TERMINAL_PROMPT','GIT_ASKPASS','SSH_ASKPASS','GCM_INTERACTIVE')) { $environment[$name]=[Environment]::GetEnvironmentVariable($name,'Process') }
  [Environment]::SetEnvironmentVariable('GIT_TERMINAL_PROMPT','0','Process')
  [Environment]::SetEnvironmentVariable('GCM_INTERACTIVE','Never','Process')
  [Environment]::SetEnvironmentVariable('GIT_ASKPASS',[NullString]::Value,'Process')
  [Environment]::SetEnvironmentVariable('SSH_ASKPASS',[NullString]::Value,'Process')
  try {
    $previous=if (-not $fresh) { Assert-OwnedSource $directory $ownership $Origin } else { $null }
    $run=New-RunDirectory $workspace 'source-checkout'
    $stage=Join-Path $run 'checkout'
    Write-Record (Join-Path $run 'intent.json') @{schema=1;owner='OpenNavX.SourceCheckout.1';origin=$Origin;commit=$Commit;directory=$directory;stage=$(if($fresh){$stage}else{$null});previousCommit=$previous;fresh=$fresh}
    $work=if ($fresh) {$stage} else {$directory}
    if ($fresh) {
      # A new staging repository has no default-branch/index mismatch. Unlike
      # clone --no-checkout, its empty index is not interpreted as user deletions.
      $template=Join-Path $run 'empty-template'
      $null=New-Item -ItemType Directory -Path $template
      $null=Invoke-SourceGit '' @('init','--quiet',('--template='+$template),$stage)
      Assert-SourceTree $stage
      $null=Invoke-SourceGit $stage @('remote','add','origin',$Origin)
      Assert-SourceOrigin $stage $Origin
      if (@(Get-ChildItem -LiteralPath $stage -Force | Where-Object {$_.Name -cne '.git'}).Count) { throw 'Fresh staging checkout contains unexpected files; preserve it.' }
    }
    Assert-SourceClean $work
    $null=Invoke-SourceGit $work @('fetch','--quiet','--no-tags','origin',$Commit)
    # Recheck after network activity; no force/reset/clean and no submodule hook.
    if (-not $fresh -and (Assert-OwnedSource $directory $ownership $Origin) -cne $previous) { throw 'Source HEAD changed during fetch.' }
    Assert-SourceClean $work
    $type=@(Invoke-SourceGit $work @('cat-file','-t',$Commit))
    if ($type.Count -ne 1 -or $type[0] -cne 'commit') { throw 'Requested object is not a commit.' }
    $null=Invoke-SourceGit $work @('checkout','--quiet','--detach',$Commit,'--')
    $head=@(Invoke-SourceGit $work @('rev-parse','--verify','HEAD'))
    if ($head.Count -ne 1 -or $head[0] -cne $Commit) { throw 'Exact fetched commit was not checked out.' }
    Assert-SourceClean $work
    $null=Invoke-SourceGit $work @('config','--local','opennav.lastManagedCommit',$Commit)
    if ($fresh) {
      $owner=@{schema=1;owner='OpenNavX.SourceCheckout.1';identity=[guid]::NewGuid().ToString('N');origin=$Origin;directory=$directory}
      $inside=Join-Path $stage '.git/opennav-source-owner.json'
      Write-Record $inside $owner
      # Ownership is durable before publication. An interruption preserves stage
      # and records; a later call never guesses that an unknown tree is its own.
      Write-Record $ownership $owner
      if ((Get-Digest $inside) -cne (Get-Digest $ownership)) { throw 'New source ownership serialization mismatch.' }
      if (Test-Path -LiteralPath $directory) { throw 'Source destination appeared while fetching; no overwrite attempted.' }
      [IO.Directory]::Move($stage,$directory)
    }
    if ((Assert-OwnedSource $directory $ownership $Origin) -cne $Commit) { throw 'Published source verification failed.' }
    $result=Join-Path $run 'passed.json'
    Write-Record $result @{schema=1;owner='OpenNavX.SourceCheckout.1';status='passed';commit=$Commit;source=$directory;fresh=$fresh;applicationChanged=$false}
    return [pscustomobject]@{status='passed';commit=$Commit;source=$directory;fresh=$fresh;evidence=$result;applicationChanged=$false}
  } finally {
    foreach ($name in $environment.Keys) {
      if ($null -eq $environment[$name]) { [Environment]::SetEnvironmentVariable($name,[NullString]::Value,'Process') }
      else { [Environment]::SetEnvironmentVariable($name,$environment[$name],'Process') }
    }
  }
}

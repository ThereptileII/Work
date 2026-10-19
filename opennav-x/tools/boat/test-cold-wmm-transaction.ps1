# Sourced only by test-cold-baseline.ps1 inside its disposable test root.
# Inert source-pin substitution exercises real ownership/file/capture hashes.
& {
  $originalPins=${function:Get-CommissioningWmmResourcePins}
  try {
    $wmmFixture=New-ColdFixture 'wmm-transaction'
    $installRoot=Join-Path $wmmFixture.root 'installed'
    $oldGeneration=Join-Path (Join-Path $installRoot 'generations') ('a'*32)
    $newGeneration=Join-Path (Join-Path $installRoot 'generations') ('b'*32)
    $pins=[ordered]@{}
    foreach($name in @('WMM.COF','wmm_live.svg','wmm_pi.svg')){
      $pins[$name]=Get-CommissioningHash ($encoding.GetBytes('Inert cold WMM fixture '+$name))
    }
    function Get-CommissioningWmmResourcePins {return $pins}
    foreach($generation in @($oldGeneration,$newGeneration)){
      $resourceDir=Join-Path $generation 'app/plugins/wmm_pi/data'
      [void](New-Item -ItemType Directory -Path $resourceDir -Force)
      $files=@()
      foreach($name in $pins.Keys){
        [IO.File]::WriteAllBytes((Join-Path $resourceDir $name),$encoding.GetBytes('Inert cold WMM fixture '+$name))
        $files+=[pscustomobject]@{path=('app/plugins/wmm_pi/data/'+$name);sha256=$pins[$name]}
      }
      Write-Record (Join-Path $generation 'ownership.json') @{owner='OpenNavX.Alpha1.SideBySide.1';commit=('c'*40);managedFiles=$files}
    }
    $wmmFixture.context.installation=[pscustomobject]@{root=$installRoot;generation=$newGeneration;commit=('c'*40);
      ownershipSha256=(Get-Digest (Join-Path $newGeneration 'ownership.json'));stateSha256=('d'*64);executableSha256=('e'*64)}
    $wmmFixture.context.pluginRoots+=,(Join-Path $newGeneration 'app/plugins')
    $oldLocation=Get-CommissioningWmmLocation (Join-Path $oldGeneration 'app')
    $newLocation=Get-CommissioningWmmLocation (Join-Path $newGeneration 'app')
    $prior=New-AdoptedPredecessor $wmmFixture $oldLocation
    $liveIni=Initialize-ColdLiveProfile $wmmFixture $prior
    [IO.File]::WriteAllText($liveIni,([IO.File]::ReadAllText($liveIni,$encoding).Replace($oldLocation,$newLocation)),$encoding)
    $liveHash=Get-Digest $liveIni
    $captured=Invoke-ColdCapture $wmmFixture
    $capture=Read-Record $captured.captureRecord
    Pass 'Actual cold Capture freezes both owned WMM generation proofs without changing user bytes' {
      if(-not $capture.wmmResourceProof -or (Get-Digest $liveIni) -cne $liveHash){throw 'WMM capture failed to freeze proof or changed live bytes.'}
      Assert-ColdWmmResourceProof (Join-Path ([IO.Path]::GetDirectoryName($captured.captureRecord)) 'predecessor.ini') $liveIni $capture.context $capture.wmmResourceProof -Live
    }
    $reviewPath=Join-Path $wmmFixture.workspace 'independent-wmm-review.json'
    $predecessor=Join-Path ([IO.Path]::GetDirectoryName($prior.record)) 'baseline.ini'
    $review=New-ColdReviewForCapture $predecessor $liveIni $captured.captureSha256 $prior.recordSha256
    $review|Add-Member -NotePropertyName wmmResourceProofSha256 -NotePropertyValue (
      Get-CommissioningHash ($encoding.GetBytes(($capture.wmmResourceProof|ConvertTo-Json -Depth 12 -Compress))))
    Write-Record $reviewPath $review
    $resource=Join-Path $oldGeneration 'app/plugins/wmm_pi/data/WMM.COF'
    $resourceBytes=[IO.File]::ReadAllBytes($resource)
    [IO.File]::AppendAllText($resource,'changed after capture')
    Refuse 'Actual cold Complete refuses changed historical WMM source bytes before publishing completion' {
      $null=Invoke-ColdCapture $wmmFixture 'Complete' $captured.captureRecord $captured.captureSha256 $reviewPath (Get-Digest $reviewPath)
    }
    [IO.File]::WriteAllBytes($resource,$resourceBytes)
    $completed=Invoke-ColdCapture $wmmFixture 'Complete' $captured.captureRecord $captured.captureSha256 $reviewPath (Get-Digest $reviewPath)
    Pass 'Actual cold Complete validates WMM proof and keeps current user bytes with no launch grant' {
      $resolved=& $script:actualBaselineReader $wmmFixture.workspace $completed.baselineRecord $completed.baselineRecordSha256
      if($resolved.sha256 -cne $liveHash -or (Get-Digest $liveIni) -cne $liveHash -or
         $completed.launchPermission -ne $false -or $completed.profileChanged -ne $false -or $completed.applicationLaunched -ne $false){
        throw 'Completed cold WMM lineage altered user bytes or claimed authority.'
      }
    }
    Remove-Item -LiteralPath $oldGeneration -Recurse -Force
    Pass 'Historical cold reader keeps frozen WMM proof usable after old generation retirement' {
      $resolved=& $script:actualBaselineReader $wmmFixture.workspace $completed.baselineRecord $completed.baselineRecordSha256
      if($resolved.sha256 -cne $liveHash){throw 'Retired WMM generation changed the preserved baseline.'}
    }
  } finally {${function:Get-CommissioningWmmResourcePins}=$originalPins}
}

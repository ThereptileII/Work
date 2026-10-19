# Sourced only by test-cold-baseline.ps1 after its disposable-root guards and
# fixture helpers are initialized. All files below are synthetic; no boat data.
& {
  $keyRoot=Join-Path $testRoot 'exact-preservation-keys'
  [void](New-Item -ItemType Directory -Path $keyRoot)
  $keyBefore=Join-Path $keyRoot 'before.ini';$keyAfter=Join-Path $keyRoot 'after.ini'
  $connection='0;0;;0;1;COM8;115200;0;1;0;;0;;0;0;1;0;1;Gateway;0;;0'
  $fontBefore='Meny:1;10;-20;0;0;0;400;0;0;0;1;0;0;2;32;Segoe UI:rgb(0, 0, 0)'
  $fontAfter='Meny:1;12;-24;0;0;0;400;0;0;0;1;0;0;2;32;Segoe UI:rgb(255, 128, 1)'
  $alphaStable='OpenNavXSettings 1\n"pilot.permission" "manual"\n"pilot.interface" "COM8"\n"display.rail" "sog,cog"\n"source.heading" "fixture"\n'
  $keyBase="[Settings]`r`nLocale=sv`r`nLocaleOverride=sv_SE`r`nPersistActiveRoute=0`r`n[Settings/NMEADataSource]`r`nDataConnections=$connection`r`n[Settings/CommPriority]`r`nSource=fixture-source`r`n[OpenNav]`r`nAlphaSettings=$alphaStable`r`nDisplayPreferencesV1=v1|100|balanced`r`nChartPresentationV1=Standard`r`n[OpenNav/OnlineAIS/v1]`r`nRadiusNm=1`r`nEnabled=0`r`n[PlugIns/Dashboard/Dashboard1]`r`nBestSizeX=100`r`nBestSizeY=101`r`nPersistSizeX=102`r`nPersistSizeY=103`r`n[Settings/MSWFonts]`r`nsv-6f52a406=$fontBefore`r`n"
  $keyChanged=$keyBase.Replace('DisplayPreferencesV1=v1|100|balanced','DisplayPreferencesV1=v1|150|instruments')
  $keyChanged=$keyChanged.Replace('ChartPresentationV1=Standard','ChartPresentationV1=XNav').Replace('RadiusNm=1','RadiusNm=200')
  $keyChanged=$keyChanged.Replace('BestSizeX=100','BestSizeX=1').Replace('BestSizeY=101','BestSizeY=32768')
  $keyChanged=$keyChanged.Replace('PersistSizeX=102','PersistSizeX=320').Replace('PersistSizeY=103','PersistSizeY=240').Replace($fontBefore,$fontAfter)
  function Write-ColdKeyPair([string]$BeforeText,[string]$AfterText) {
    [IO.File]::WriteAllText($keyBefore,$BeforeText,$encoding)
    [IO.File]::WriteAllText($keyAfter,$AfterText,$encoding)
  }
  function Assert-ColdKeyPair([string]$BeforeText,[string]$AfterText,[int]$Count=-1) {
    Write-ColdKeyPair $BeforeText $AfterText
    $review=MakeReview $keyBefore $keyAfter
    $diff=@(Assert-ColdBaselineDelta $keyBefore $keyAfter $review)
    if($Count -ge 0 -and $diff.Count -ne $Count){throw 'Unexpected number of independently reviewed cold keys.'}
  }
  Pass 'Eight exact display, AIS radius, Dashboard1 and Swedish font keys preserve valid existing bytes' {
    Assert-ColdKeyPair $keyBase $keyChanged 8
  }
  Pass 'Cold display preferences preserve every existing v1 size and layout choice' {
    foreach($scale in @('100','125','150')) {
      foreach($layout in @('balanced','chart','instruments')) {
        $text=$keyChanged.Replace('v1|150|instruments',('v1|'+$scale+'|'+$layout))
        $count=if($scale -ceq '100' -and $layout -ceq 'balanced'){7}else{8}
        Assert-ColdKeyPair $keyBase $text $count
      }
    }
  }
  Pass 'Exact chart palette and AIS radius boundaries work in both directions' {
    Assert-ColdKeyPair $keyChanged $keyBase 8
  }
  Pass 'Exact optional display and AIS preferences can be added to an existing profile' {
    $missing=$keyBase.Replace("DisplayPreferencesV1=v1|100|balanced`r`n",'').Replace("ChartPresentationV1=Standard`r`n",'').Replace("RadiusNm=1`r`n",'')
    Assert-ColdKeyPair $missing $keyChanged 8
  }
  Refuse 'The sv-6f52a406 policy requires an existing font descriptor' {
    Assert-ColdKeyPair ($keyBase.Replace(('sv-6f52a406='+$fontBefore+"`r`n"),'')) $keyChanged
  }
  foreach($case in @(
    @{name='display version';old='DisplayPreferencesV1=v1|100|balanced';new='DisplayPreferencesV1=v1|150|instruments';bad='DisplayPreferencesV1=v2|100|balanced'},
    @{name='display scale';old='DisplayPreferencesV1=v1|100|balanced';new='DisplayPreferencesV1=v1|150|instruments';bad='DisplayPreferencesV1=v1|175|balanced'},
    @{name='display layout';old='DisplayPreferencesV1=v1|100|balanced';new='DisplayPreferencesV1=v1|150|instruments';bad='DisplayPreferencesV1=v1|100|pilot'},
    @{name='chart palette';old='ChartPresentationV1=Standard';new='ChartPresentationV1=XNav';bad='ChartPresentationV1=xnav'},
    @{name='zero AIS radius';old='RadiusNm=1';new='RadiusNm=200';bad='RadiusNm=0'},
    @{name='large AIS radius';old='RadiusNm=1';new='RadiusNm=200';bad='RadiusNm=201'},
    @{name='noncanonical AIS radius';old='RadiusNm=1';new='RadiusNm=200';bad='RadiusNm=01'},
    @{name='fractional AIS radius';old='RadiusNm=1';new='RadiusNm=200';bad='RadiusNm=1.5'},
    @{name='font color';old=('sv-6f52a406='+$fontBefore);new=('sv-6f52a406='+$fontAfter);bad=('sv-6f52a406='+$fontAfter.Replace('255, 128, 1','256, 128, 1'))},
    @{name='font encoding';old=('sv-6f52a406='+$fontBefore);new=('sv-6f52a406='+$fontAfter);bad='sv-6f52a406=Meny:unparsed'}
  )) {
    Refuse ('Invalid old '+$case.name+' is not legitimized by preserving a valid current value') {
      Assert-ColdKeyPair ($keyBase.Replace($case.old,$case.bad)) $keyChanged
    }
    Refuse ('Invalid current '+$case.name+' is rejected despite an exact review') {
      Assert-ColdKeyPair $keyBase ($keyChanged.Replace($case.new,$case.bad))
    }
  }
  foreach($sizeKey in @('BestSizeX','BestSizeY','PersistSizeX','PersistSizeY')) {
    foreach($badSize in @('0','-1','32769','01','1.5')) {
      foreach($side in @('before','after')) {
        Refuse ('Dashboard1 '+$sizeKey+' rejects '+$badSize+' in '+$side+' bytes') {
          $old=$keyBase;$current=$keyChanged;$pattern='(?m)^'+$sizeKey+'=[^\r\n]*'
          if($side -ceq 'before'){$old=[regex]::Replace($old,$pattern,($sizeKey+'='+$badSize))}
          else{$current=[regex]::Replace($current,$pattern,($sizeKey+'='+$badSize))}
          Assert-ColdKeyPair $old $current
        }
      }
    }
  }
  foreach($side in @('before','after','both')) {
    Refuse ('Swedish font proof requires sv and sv_SE in '+$side+' profile') {
      $old=$keyBase;$current=$keyChanged
      if($side -cne 'after'){$old=$old.Replace('Locale=sv','Locale=en').Replace('LocaleOverride=sv_SE','LocaleOverride=en_US')}
      if($side -cne 'before'){$current=$current.Replace('Locale=sv','Locale=en').Replace('LocaleOverride=sv_SE','LocaleOverride=en_US')}
      Assert-ColdKeyPair $old $current
    }
  }
  foreach($case in @(
    @{name='Alpha pilot permission';old='"pilot.permission" "manual"';new='"pilot.permission" "display-only"'},
    @{name='selected source';old='Source=fixture-source';new='Source=other-fixture-source'},
    @{name='connection interface';old='COM8;115200';new='COM9;115200'},
    @{name='OnlineAIS enable permission';old='Enabled=0';new='Enabled=1'},
    @{name='unreviewed dashboard';old='[PlugIns/Dashboard/Dashboard1]';new='[PlugIns/Dashboard/Dashboard4]'},
    @{name='different font key';old='sv-6f52a406=';new='sv-6f52a407='}
  )) {
    Refuse ('Exact key additions do not authorize changed '+$case.name) {
      Assert-ColdKeyPair $keyBase ($keyChanged.Replace($case.old,$case.new))
    }
  }
  foreach($line in @('DisplayPreferencesV1=v1|150|instruments','ChartPresentationV1=XNav','RadiusNm=200','BestSizeX=1','BestSizeY=32768','PersistSizeX=320','PersistSizeY=240',('sv-6f52a406='+$fontAfter))) {
    Refuse ('A reviewed key cannot be deleted: '+$line.Split('=')[0]) {
      Assert-ColdKeyPair $keyBase ($keyChanged.Replace($line+"`r`n",''))
    }
  }

  $railAfter=$keyChanged.Replace('"display.rail" "sog,cog"','"display.rail" "heading,depth,aws,awa,soc,voltage"')
  Pass 'Cold data-rail selection changes at most six known items while preserving pilot and source bytes' {
    Assert-ColdKeyPair $keyBase $railAfter 9
    Assert-ColdKeyPair $railAfter $keyBase 9
    Assert-ColdKeyPair $keyBase ($keyChanged.Replace('"display.rail" "sog,cog"','"display.rail" "depth"')) 9
  }
  foreach($selection in @('','sog,sog','sog,cog,heading,depth,aws,awa,soc','sog,unknown','SOG,cog')) {
    foreach($side in @('before','after')) {
      Refuse ('Cold rail rejects invalid '+$side+' selection: '+$selection) {
        $old=$keyBase;$current=$railAfter
        if($side -ceq 'before'){$old=$old.Replace('"display.rail" "sog,cog"',('"display.rail" "'+$selection+'"'))}
        else{$current=$current.Replace('"display.rail" "heading,depth,aws,awa,soc,voltage"',('"display.rail" "'+$selection+'"'))}
        Assert-ColdKeyPair $old $current
      }
    }
  }
  foreach($case in @(
    @{name='pilot permission';old='"pilot.permission" "manual"';new='"pilot.permission" "display-only"'},
    @{name='pilot interface';old='"pilot.interface" "COM8"';new='"pilot.interface" "COM9"'},
    @{name='source mapping';old='"source.heading" "fixture"';new='"source.heading" "other"'},
    @{name='record version';old='OpenNavXSettings 1\n';new='OpenNavXSettings 2\n'},
    @{name='duplicate rail';old='"display.rail" "heading,depth,aws,awa,soc,voltage"\n';new='"display.rail" "heading,depth,aws,awa,soc,voltage"\n"display.rail" "sog"\n'},
    @{name='missing rail';old='"display.rail" "heading,depth,aws,awa,soc,voltage"\n';new=''},
    @{name='unknown future field';old='"source.heading" "fixture"\n';new='"source.heading" "fixture"\n"future.setting" "changed"\n'}
  )) {
    Refuse ('A valid cold rail delta does not permit changed '+$case.name) {
      Assert-ColdKeyPair $keyBase ($railAfter.Replace($case.old,$case.new))
    }
  }
  Refuse 'Cold rail preservation cannot introduce a new whole AlphaSettings record' {
    Assert-ColdKeyPair ($keyBase.Replace(('AlphaSettings='+$alphaStable+"`r`n"),'')) $railAfter
  }
  $routeGuid='01234567-89ab-cdef-0123-456789abcdef'
  $routeBefore=$keyBase.Replace("PersistActiveRoute=0`r`n",("PersistActiveRoute=0`r`nActiveRoute="+$routeGuid+"`r`n"))
  $routeAfter=$keyChanged.Replace("PersistActiveRoute=0`r`n","PersistActiveRoute=0`r`nActiveRoute=`r`n")
  Pass 'An inert stored route GUID can become empty and back while persistence remains disabled' {
    Assert-ColdKeyPair $routeBefore $routeAfter 9
    Assert-ColdKeyPair $routeAfter $routeBefore 9
  }
  foreach($side in @('before','after','both')) {
    Refuse ('Stored route retention refuses enabled route persistence in '+$side+' profile') {
      $old=$routeBefore;$current=$routeAfter
      if($side -cne 'after'){$old=$old.Replace('PersistActiveRoute=0','PersistActiveRoute=1')}
      if($side -cne 'before'){$current=$current.Replace('PersistActiveRoute=0','PersistActiveRoute=1')}
      Assert-ColdKeyPair $old $current
    }
  }
  foreach($invalid in @('active','01234567-89ab-cdef-0123-456789abcde','{01234567-89ab-cdef-0123-456789abcdef}')) {
    Refuse 'Malformed prior stored route cannot be normalized by cold retention' {
      Assert-ColdKeyPair ($routeBefore.Replace($routeGuid,$invalid)) $routeAfter
    }
    Refuse 'Malformed current stored route is refused despite an exact review' {
      Assert-ColdKeyPair $routeBefore ($routeAfter.Replace("ActiveRoute=`r`n",('ActiveRoute='+$invalid+"`r`n")))
    }
  }
  Refuse 'Cold route retention cannot delete the stored key' {
    Assert-ColdKeyPair $routeBefore ($routeAfter.Replace("ActiveRoute=`r`n",''))
  }

  # Only the pin oracle is substituted. Ownership bytes, inventory bounds,
  # canonical generation paths and all SHA-256 checks use real disposable files.
  $savedColdWmmPins=${function:Get-CommissioningWmmResourcePins}
  function Get-CommissioningWmmResourcePins {
    $pins=[ordered]@{}
    foreach($name in @('WMM.COF','wmm_live.svg','wmm_pi.svg')) {
      $pins[$name]=Get-CommissioningHash ($encoding.GetBytes('Inert cold WMM fixture '+$name))
    }
    return $pins
  }
  try {
    $wmmRoot=Join-Path $keyRoot 'installed';$generations=Join-Path $wmmRoot 'generations'
    $oldGeneration=Join-Path $generations ('a'*32);$currentGeneration=Join-Path $generations ('b'*32)
    foreach($generation in @($oldGeneration,$currentGeneration)) {
      $resourceDirectory=Join-Path $generation 'app/plugins/wmm_pi/data'
      [void](New-Item -ItemType Directory -Path $resourceDirectory -Force)
      $managedFiles=@()
      foreach($name in (Get-CommissioningWmmResourcePins).Keys) {
        $resourcePath=Join-Path $resourceDirectory $name
        [IO.File]::WriteAllText($resourcePath,('Inert cold WMM fixture '+$name),$encoding)
        $managedFiles+=[pscustomobject]@{path=('app/plugins/wmm_pi/data/'+$name);sha256=(Get-Digest $resourcePath)}
      }
      $commit=if($generation -ceq $oldGeneration){'c'*40}else{'d'*40}
      Write-Record (Join-Path $generation 'ownership.json') @{owner='OpenNavX.Alpha1.SideBySide.1';commit=$commit;managedFiles=$managedFiles}
    }
    $wmmContext=[pscustomobject]@{installation=[pscustomobject]@{root=$wmmRoot;generation=$currentGeneration;commit=('d'*40);
      ownershipSha256=(Get-Digest (Join-Path $currentGeneration 'ownership.json'));stateSha256=('e'*64)}}
    $oldLocation=Get-CommissioningWmmLocation (Join-Path $oldGeneration 'app')
    $currentLocation=Get-CommissioningWmmLocation (Join-Path $currentGeneration 'app')
    $wmmBefore=$keyBase+"[Directories]`r`nWMMDataLocation=$oldLocation`r`n"
    $wmmAfter=$keyChanged+"[Directories]`r`nWMMDataLocation=$currentLocation`r`n"
    Write-ColdKeyPair $wmmBefore $wmmAfter
    $wmmProof=Get-ColdWmmResourceProof $keyBefore $keyAfter $wmmContext
    function New-ColdWmmReview($Proof) {
      $r=MakeReview $keyBefore $keyAfter
      $hash=Get-CommissioningHash ($encoding.GetBytes(($Proof|ConvertTo-Json -Depth 12 -Compress)))
      $r|Add-Member -NotePropertyName wmmResourceProofSha256 -NotePropertyValue $hash
      return $r
    }
    Pass 'Cold WMM proof validates owned old/current generation bytes without a prepared transaction' {
      if($wmmProof.owner -cne 'OpenNavX.ColdWmmResourceReview.1' -or $wmmProof.schema -ne 1 -or
          $wmmProof.original.generation -cne $oldGeneration -or $wmmProof.current.generation -cne $currentGeneration){throw 'Cold WMM proof has the wrong independent identities.'}
      $null=Assert-ColdWmmResourceProof $keyBefore $keyAfter $wmmContext $wmmProof -Live
      $diff=@(Assert-ColdBaselineDelta $keyBefore $keyAfter (New-ColdWmmReview $wmmProof) ([datetime]::UtcNow) $wmmProof $wmmContext)
      if($diff.Count -ne 9){throw 'WMM path was not the sole ninth reviewed key.'}
    }
    Refuse 'Exact text review cannot preserve WMM location without resource proof' {
      $null=Assert-ColdBaselineDelta $keyBefore $keyAfter (MakeReview $keyBefore $keyAfter)
    }
    Refuse 'WMM resource proof needs an explicit hash in the independent review' {
      $null=Assert-ColdBaselineDelta $keyBefore $keyAfter (MakeReview $keyBefore $keyAfter) ([datetime]::UtcNow) $wmmProof $wmmContext
    }
    Refuse 'WMM resource proof cannot be replaced after its review hash is fixed' {
      $r=New-ColdWmmReview $wmmProof;$r.wmmResourceProofSha256='f'*64
      $null=Assert-ColdBaselineDelta $keyBefore $keyAfter $r ([datetime]::UtcNow) $wmmProof $wmmContext
    }
    foreach($case in @(
      @{name='owner';change={param($p);$p.owner='foreign'}},
      @{name='schema';change={param($p);$p.schema=2}},
      @{name='extra field';change={param($p);$p|Add-Member -NotePropertyName unexpected -NotePropertyValue $true}},
      @{name='missing original proof';change={param($p);$p.PSObject.Properties.Remove('original')}},
      @{name='old location';change={param($p);$p.original.location=$currentLocation}},
      @{name='old generation';change={param($p);$p.original.generation=$currentGeneration}},
      @{name='old commit';change={param($p);$p.original.commit='0'*40}},
      @{name='old manifest digest';change={param($p);$p.original.ownershipSha256='0'*64}},
      @{name='old manifest bytes';change={param($p);$p.original.ownershipBase64=[Convert]::ToBase64String($encoding.GetBytes('{}'))}},
      @{name='duplicate ownership entry despite a consistent manifest hash';change={param($p)
        $owner=$encoding.GetString([Convert]::FromBase64String($p.original.ownershipBase64))|ConvertFrom-Json
        $owner.managedFiles=@($owner.managedFiles)+@($owner.managedFiles[0])
        $raw=$encoding.GetBytes(($owner|ConvertTo-Json -Depth 12 -Compress))
        $p.original.ownershipBase64=[Convert]::ToBase64String($raw);$p.original.ownershipSha256=Get-CommissioningHash $raw
      }},
      @{name='current generation';change={param($p);$p.current.generation=$oldGeneration}},
      @{name='current manifest digest';change={param($p);$p.current.ownershipSha256='0'*64}},
      @{name='missing old resource';change={param($p);$p.original.resources=@($p.original.resources|Select-Object -Skip 1)}},
      @{name='duplicate old resource';change={param($p);$p.original.resources=@($p.original.resources[0],$p.original.resources[0],$p.original.resources[2])}},
      @{name='changed old resource hash';change={param($p);$p.original.resources[0].sha256='0'*64}},
      @{name='old/current resource length mismatch';change={param($p);$p.original.resources[0].bytes++}}
    )) {
      Refuse ('A fresh review hash cannot legitimize changed WMM '+$case.name) {
        $bad=Clone $wmmProof;& $case.change $bad
        $null=Assert-ColdBaselineDelta $keyBefore $keyAfter (New-ColdWmmReview $bad) ([datetime]::UtcNow) $bad $wmmContext
      }
    }
    foreach($location in @($oldLocation.TrimEnd('\','/'),($oldLocation+'../'),($oldLocation.Replace(('a'*32),('A'*32))))) {
      Refuse 'Old WMM path must select one distinct canonical owned generation' {
        Write-ColdKeyPair ($wmmBefore.Replace($oldLocation,$location)) $wmmAfter
        $null=Get-ColdWmmResourceProof $keyBefore $keyAfter $wmmContext
      }
    }
    Write-ColdKeyPair $wmmBefore ($wmmAfter.Replace($currentLocation,$oldLocation))
    Refuse 'WMM resource proof is forbidden when there is no WMM delta' {
      $null=Assert-ColdWmmResourceProof $keyBefore $keyAfter $wmmContext $wmmProof
    }
    Refuse 'WMM review hash is forbidden for a display-only change' {
      $null=Assert-ColdBaselineDelta $keyBefore $keyAfter (New-ColdWmmReview $wmmProof)
    }
    Refuse 'The current WMM path cannot name an unrelated generation' {
      Write-ColdKeyPair $wmmBefore ($wmmAfter.Replace($currentLocation,$currentLocation.Replace(('b'*32),('f'*32))))
      $null=Get-ColdWmmResourceProof $keyBefore $keyAfter $wmmContext
    }
    Refuse 'WMM preservation cannot borrow an old generation from another installation root' {
      Write-ColdKeyPair $wmmBefore $wmmAfter
      $wrongRoot=Clone $wmmContext;$wrongRoot.installation.root=Join-Path $keyRoot 'other-installed'
      $null=Get-ColdWmmResourceProof $keyBefore $keyAfter $wrongRoot
    }
    Write-ColdKeyPair $wmmBefore $wmmAfter
    $oldResource=Join-Path $oldGeneration 'app/plugins/wmm_pi/data/WMM.COF'
    $resourceBytes=[IO.File]::ReadAllBytes($oldResource)
    [IO.File]::AppendAllText($oldResource,'changed',$encoding)
    try {
      Refuse 'Live proof rejects old generation resource drift outside current plugin roots' {
        $null=Assert-ColdWmmResourceProof $keyBefore $keyAfter $wmmContext $wmmProof -Live
      }
      Pass 'Frozen proof remains historically verifiable despite later external resource drift' {
        $null=Assert-ColdWmmResourceProof $keyBefore $keyAfter $wmmContext $wmmProof
      }
    } finally {[IO.File]::WriteAllBytes($oldResource,$resourceBytes)}
    $extra=Join-Path $currentGeneration 'app/plugins/wmm_pi/data/unowned.dat'
    [IO.File]::WriteAllText($extra,'unowned',$encoding)
    try {
      Refuse 'Live proof refuses additional unowned current WMM directory contents' {
        $null=Assert-ColdWmmResourceProof $keyBefore $keyAfter $wmmContext $wmmProof -Live
      }
    } finally {[IO.File]::Delete($extra)}
    [IO.Directory]::Move($oldGeneration,($oldGeneration+'-retired'))
    [IO.Directory]::Move($currentGeneration,($currentGeneration+'-retired'))
    Pass 'Historical cold WMM proof survives retirement of both generations' {
      $null=Assert-ColdWmmResourceProof $keyBefore $keyAfter $wmmContext $wmmProof
      $diff=@(Assert-ColdBaselineDelta $keyBefore $keyAfter (New-ColdWmmReview $wmmProof) ([datetime]::UtcNow) $wmmProof $wmmContext)
      if($diff.Count -ne 9){throw 'Historical proof lost its exact reviewed delta.'}
    }
    Refuse 'Retired generation evidence cannot authorize a live completion' {
      $null=Assert-ColdWmmResourceProof $keyBefore $keyAfter $wmmContext $wmmProof -Live
    }
  } finally {Set-Item Function:Get-CommissioningWmmResourcePins $savedColdWmmPins}
}

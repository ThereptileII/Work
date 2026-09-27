# TEST ONLY. Appended to copied dependencies inside a fresh CI TEMP fixture.
# Never packaged or imported by installed/cold-launch product scripts.
if($env:GITHUB_ACTIONS -cne 'true'){throw 'Broker fixture identity is CI-only.'}
$fixtureFile=Join-Path $PSScriptRoot 'fixture.identity.json'
if($env:OPENNAV_BROKER_FIXTURE_IDENTITY_SHA256 -cnotmatch '^[a-f0-9]{64}$' -or (Get-Digest $fixtureFile) -cne $env:OPENNAV_BROKER_FIXTURE_IDENTITY_SHA256){throw 'Explicit immutable test fixture identity required.'}
$script:BrokerFixture=Read-Record $fixtureFile
$fixtureRoot=Assert-LocalPath $script:BrokerFixture.root
$temp=[IO.Path]::GetFullPath([IO.Path]::GetTempPath()).TrimEnd('\')
if(-not $fixtureRoot.StartsWith($temp+'\OpenNav broker fixture ',[StringComparison]::OrdinalIgnoreCase) -or
   $script:BrokerFixture.owner -cne 'OpenNavX.TestOnly.BrokerFixture.1' -or $script:BrokerFixture.noMarineCode -isnot [bool] -or -not $script:BrokerFixture.noMarineCode -or
   $PSScriptRoot -ine (Join-Path $fixtureRoot 'scripts')){throw 'Fixture identity escaped marked disposable directory.'}
foreach($path in @($script:BrokerFixture.workspace,$script:BrokerFixture.commonData,$script:BrokerFixture.localData,$script:BrokerFixture.installedRoot,$script:BrokerFixture.stockExecutable,
 $script:BrokerFixture.context.workspace,$script:BrokerFixture.context.profile,$script:BrokerFixture.context.application,$script:BrokerFixture.context.managed,
 $script:BrokerFixture.context.installation.executable,$script:BrokerFixture.context.launchEnvironment.workingDirectory)+@($script:BrokerFixture.context.pluginRoots)) {
 if(-not (Assert-LocalPath $path).StartsWith($fixtureRoot+'\',[StringComparison]::OrdinalIgnoreCase)){throw 'Fixture path escaped TEMP ownership.'}
}
function Get-Target([string]$Workspace) {
 if($Workspace -cne $script:BrokerFixture.workspace){throw 'Fixture workspace mismatch.'}
 if((Get-Digest $script:BrokerFixture.stockExecutable) -cne $script:BrokerFixture.stockSha256){throw 'Fixture stock identity changed.'}
 return Read-Record (Join-Path $Workspace 'boat-target.json')
}
function Get-Installed([string]$Purpose='Launch') {
 if($Purpose -cne 'Launch'){throw 'No fixture maintenance action permitted.'}
 return Read-InstalledIdentity $script:BrokerFixture.installedRoot 'Launch'
}
function Get-CommissioningContext([string]$Workspace) {
 if($Workspace -cne $script:BrokerFixture.workspace){throw 'Fixture context escaped.'}
 $context=$script:BrokerFixture.context
 # Real OS process audit still runs. Only the identity roots are synthetic.
 Assert-PreparationClosed (@($context.application,$context.managed)+@($context.pluginRoots))
 return $context
}
$script:CommissioningBaseline=$script:BrokerFixture.baselineSha256

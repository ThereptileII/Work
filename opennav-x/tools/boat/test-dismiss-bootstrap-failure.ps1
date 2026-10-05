# Portable incident-policy/parser checks only. No Win32 UI or process operation.
$ErrorActionPreference='Stop'
$path=Join-Path $PSScriptRoot 'dismiss-bootstrap-failure.ps1'
$tokens=$null;$errors=$null
$ast=[Management.Automation.Language.Parser]::ParseFile($path,[ref]$tokens,[ref]$errors)
if($errors.Count){throw ($errors|Out-String)}
foreach($name in @('Assert-BootstrapFailureIdentity','Assert-BootstrapFailureDialog')){
  $functions=@($ast.FindAll({param($node)$node -is [Management.Automation.Language.FunctionDefinitionAst] -and $node.Name -ceq $name},$true))
  if($functions.Count -ne 1){throw 'Exact pure incident policy required.'}
  . ([scriptblock]::Create($functions[0].Extent.Text))
}
$source=[IO.File]::ReadAllText($path)
$native=[regex]::Match($source,"(?s)Add-Type -TypeDefinition @'\r?\n(.*?)\r?\n'@")
if(-not $native.Success){throw 'Incident window boundary missing.'}
Add-Type -TypeDefinition $native.Groups[1].Value # Compile declarations; never invoke P/Invoke.
$count=2
function Refuse([scriptblock]$Body){$bad=$false;try{& $Body}catch{$bad=$true};if(-not $bad){throw 'Invalid incident accepted.'};$script:count++}
function Clone($Value){return $Value|ConvertTo-Json|ConvertFrom-Json}
$image='C:\inert\app\skager-start.exe';$hash='a'*64;$sid='S-1-5-21-123';$session=2
$identity=[pscustomobject]@{pid=1228;startedTicks=([datetime]::Parse('2026-10-05T20:01:22.9834203Z')).ToUniversalTime().Ticks;exited=$false;image=$image;sha256=$hash;sid=$sid;session=$session}
Assert-BootstrapFailureIdentity $identity $image $hash $sid $session;$count++
foreach($entry in @(@('pid',7196),@('pid',1229),@('startedTicks',($identity.startedTicks+1)),@('exited',$true),@('image','C:\inert\app\opencpn.exe'),@('sha256',('b'*64)),@('sid','S-1-5-21-456'),@('session',3))){
  $bad=Clone $identity;$bad.($entry[0])=$entry[1];Refuse {Assert-BootstrapFailureIdentity $bad $image $hash $sid $session}
}
$dialog=[pscustomobject]@{processId=1228;visible=$true;className='#32770';caption='SKAGER startup';text="SKAGER could not complete startup.`n`nAn update may need recovery. Open SKAGER Maintenance diagnostics before trying again.`n`nYou can try the Legacy or Safe Mode shortcut while checking the problem.";buttonCount=1;buttonText='OK';buttonEnabled=$true}
Assert-BootstrapFailureDialog $dialog;$count++
foreach($entry in @(@('processId',7196),@('visible',$false),@('className','wxWindowNR'),@('caption','SKAGER'),@('text','Different warning'),@('buttonCount',0),@('buttonCount',2),@('buttonText','Agree'),@('buttonEnabled',$false))){
  $bad=Clone $dialog;$bad.($entry[0])=$entry[1];Refuse {Assert-BootstrapFailureDialog $bad}
}
[pscustomobject]@{status='passed';count=$count;scope='portable parser, compiled PInvoke declarations and negative incident policies only';nativeDialogAcceptance=$false;boatAccess=$false}|ConvertTo-Json

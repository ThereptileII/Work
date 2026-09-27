# Pure exact menu policy and interop layout; no app, profile, desktop or input.
[CmdletBinding()]
param()
. (Join-Path $PSScriptRoot 'StockReview.ps1')
Initialize-StockReviewNative
$checks=New-Object 'Collections.Generic.List[string]'
function Check([bool]$Okay,[string]$Name){if(-not $Okay){throw $Name};$checks.Add($Name)}
$type=[OpenNavX.StockReviewNative];$flags=[Reflection.BindingFlags]'NonPublic,Static'
$tuple=$type.GetMethod('ZoomTuple',$flags);$state=$type.GetMethod('OrdinaryZoomEntry',$flags)
$en=@('&Navigate',"Zoom In`t+","Zoom Out`t-");$sv=@('&Navigation',"Zooma in`t+","Zooma ut`t-")
foreach($value in @($en,$sv)) {
 Check ($tuple.Invoke($null,[object[]]$value)) 'One exact source-derived full English/Swedish zoom tuple accepted'
 for($index=0;$index -lt 3;$index++) {
  foreach($replacement in @('', 'Zoom Out', 'Go To', ($value[$index]+' '), ($value[$index]+'x'), $value[$index].ToLowerInvariant())) {
   $bad=$value.Clone();$bad[$index]=$replacement
   Check (-not $tuple.Invoke($null,[object[]]$bad)) 'Changed/partial/action-like/case-changed native menu identity refused'
  }
 }
}
for($index=0;$index -lt 3;$index++){$bad=$en.Clone();$bad[$index]=$sv[$index];Check (-not $tuple.Invoke($null,[object[]]$bad)) 'Mixed-locale menu tuple refused'}
Check ($state.Invoke($null,[object[]]@([uint32]0,[IntPtr]::Zero))) 'Enabled ordinary text leaf accepted'
foreach($mask in @([uint32]1,[uint32]2,[uint32]4,[uint32]8,[uint32]16,[uint32]128,[uint32]256,[uint32]2048,[uint32]::MaxValue)) {
 Check (-not $state.Invoke($null,[object[]]@($mask,[IntPtr]::Zero))) 'Disabled/bitmap/checked/submenu/active/owner-drawn/separator menu state refused'
}
Check (-not $state.Invoke($null,[object[]]@([uint32]0,[IntPtr]1))) 'Submenu handle refuses despite plain state'
$gui=$type.GetNestedType('ChartGuiInfo',[Reflection.BindingFlags]'NonPublic')
Check ([Runtime.InteropServices.Marshal]::SizeOf([Activator]::CreateInstance($gui)) -eq $(if([IntPtr]::Size -eq 8){72}else{48})) 'GUITHREADINFO native ABI matches current architecture'
$zoom=$type.GetMethod('ZoomOut',[Reflection.BindingFlags]'Public,Static');$params=$zoom.GetParameters()
Check ($params.Count -eq 2 -and $params[0].ParameterType -eq [IntPtr] -and $params[1].ParameterType -eq [int]) 'Zoom accepts only reviewed window and PID, never caller-supplied command or coordinates'
foreach($file in @('StockReview.ps1','review-stock.ps1','../test-stock-welcome-windows.ps1')) {
 $tokens=$null;$errors=$null;$null=[Management.Automation.Language.Parser]::ParseFile((Join-Path $PSScriptRoot $file),[ref]$tokens,[ref]$errors)
 Check ($errors.Count -eq 0) ('Parses '+$file)
}
$diagnostic=$type.GetMethod('FormatObscurerDiagnostic',$flags)
$bounds=New-Object OpenNavX.StockReviewNative+Rect;$bounds.Left=-10;$bounds.Top=20;$bounds.Right=100;$bounds.Bottom=200
$title='Owned "fixture" '+[char]0x00e4+"`n"+'\title'
$value=$diagnostic.Invoke($null,[object[]]@([long]123,[uint32]456,[long]789,'test-class',$title,$bounds,[int]0,[uint32]1))|ConvertFrom-Json
Check ($value.hwnd -eq 123 -and $value.pid -eq 456 -and $value.owner -eq 789 -and $value.class -ceq 'test-class' -and $value.title -ceq $title) 'Private obscurer JSON preserves exact bounded identity and escapes caption text'
Check ($value.bounds.left -eq -10 -and $value.bounds.top -eq 20 -and $value.bounds.right -eq 100 -and $value.bounds.bottom -eq 200 -and $value.cloakResult -eq 0 -and $value.cloak -eq 1) 'Diagnostic geometry and cloak observation are explicit without granting capture'
$value=$diagnostic.Invoke($null,[object[]]@([long]1,[uint32]2,[long]0,('x'*1024),('y'*1024),$bounds,[int]-1,[uint32]0))|ConvertFrom-Json
Check ($value.class.Length -eq 256 -and $value.title.Length -eq 256 -and $null -eq $value.cloak -and $value.cloakResult -eq -1) 'Caption lengths are bounded and failed cloak query remains unknown'
$beforeCulture=[Globalization.CultureInfo]::CurrentCulture
try {
 [Globalization.CultureInfo]::CurrentCulture=[Globalization.CultureInfo]::GetCultureInfo('sv-SE')
 $value=$diagnostic.Invoke($null,[object[]]@([long]12345,[uint32]2,[long]0,$null,$null,$bounds,[int]0,[uint32]0))|ConvertFrom-Json
 Check ($value.hwnd -eq 12345 -and $value.class -ceq '' -and $value.title -ceq '') 'Diagnostic JSON uses invariant numbers and handles missing observed text'
} finally {[Globalization.CultureInfo]::CurrentCulture=$beforeCulture}
[pscustomobject]@{status='passed';count=$checks.Count;checks=$checks.ToArray();nativeInteropCompiled=$true;nativeApisInvoked=$false;boatAccess=$false;applicationLaunched=$false}|ConvertTo-Json -Depth 5

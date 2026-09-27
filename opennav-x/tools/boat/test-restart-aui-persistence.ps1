# Pure in-memory tests; no window, profile, plugin, network or actuator access.
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'RestartAuiPersistence.ps1')
. (Join-Path $PSScriptRoot 'RestartDashboardPersistence.ps1')
$script:checks=0
function Pass([scriptblock]$Action,[string]$Name){& $Action;$script:checks++}
function Refuse([scriptblock]$Action,[string]$Name){$failed=$false;try{& $Action}catch{$failed=$true};if(-not $failed){throw ('FAILED refusal: '+$Name)};$script:checks++}
$pane='name=ChartCanvas;caption=;state=768;dir=5;layer=0;row=0;pos=0;prop=100000;bestw=5;besth=5;minw=256;minh=800;maxw=-1;maxh=-1;floatx=-1;floaty=-1;floatw=-1;floath=-1'
$plugin=$pane.Replace('name=ChartCanvas;caption=','name=dashboard_1;caption=Instruments').Replace('state=768','state=2098125').Replace('dir=5','dir=4')
$old='layout2|'+$pane+'|'+$plugin+'|dock_size(5,0,0)=1280|'
Pass {Assert-RestartAuiDelta $old $old} 'unchanged parsed source format'
$new=$old.Replace('minw=256','minw=320').Replace('minh=800','minh=640').Replace('=1280|','=1440|')
Pass {Assert-RestartAuiDelta $old $new} 'bounded geometry'
Pass {Assert-RestartAuiDelta $old ($old.Replace('state=2098125','state=2098127'))} 'hidden floating plugin'
Pass {Assert-RestartAuiDelta $old ($old.Replace('state=2098125','state=2098124').Replace('dock_size(5,0,0)=1280|','dock_size(5,0,0)=1280|dock_size(4,0,0)=250|'))} 'dock existing plugin'
Pass {Assert-RestartAuiDelta $old ($old.Replace('state=768','state=17152'))} 'active chart'
Pass {Assert-RestartAuiDelta $old ($old.Replace('state=768','state=66304'))} 'maximized chart'
Pass {Assert-RestartAuiDelta $old ($old.Replace('state=768','state=1073742592'))} 'saved hidden layout flag'
$escaped=$old.Replace('caption=Instruments','caption=Depth\\; Wind\\| Speed')
Pass {Assert-RestartAuiDelta $escaped ($escaped.Replace('minh=800','minh=720'))} 'actual two-layer escaped delimiters'
$unicode=$old.Replace('caption=Instruments',('caption=Vind '+[char]0x00b0))
Pass {Assert-RestartAuiDelta $unicode $unicode} 'unicode caption identity'
$badValues=@('', 'layout1|', ($old+'extra'), ($old+'|'), $old.Replace('layout2','layout3'),
 $old.Replace('name=ChartCanvas','name=dashboard_1'), $old.Replace('caption=Instruments','caption=Changed'),
 $old.Replace('state=768','state=4864'), $old.Replace('state=768','state=262912'),
 $old.Replace('state=768','state=2147484416'), $old.Replace('dir=5','dir=6'),
 $old.Replace('prop=100000','prop=1000001'), $old.Replace('floatx=-1','floatx=-32769'),
 $old.Replace('minw=256','minw=-2'), $old.Replace('minw=256','minw=NaN'),
 $old.Replace('minw=256','minw=2.5'), $old.Replace('minw=256','minw=0256'),
 $old.Replace('minw=256','minw=-0'), $old.Replace('row=0','row=129'),
 $old.Replace('caption=Instruments','command=Instruments'),
 $old.Replace(';floatw=-1',';floatw=-1;output=enabled'),
 $old.Replace('dock_size(5,0,0)=1280','dock_size(5,0,0)=0'),
 $old.Replace('dock_size(5,0,0)=1280','dock_size(5,0,0)=32769'),
 $old.Replace('dock_size(5,0,0)=1280','dock_size(4,0,0)=1280'),
 ($old+'dock_size(5,0,0)=1280|'), $old.Replace('name=dashboard_1','name=other_plugin'),
 ('layout2|'+$pane+'|dock_size(5,0,0)=1280|'),
 $old.Replace('caption=Instruments','caption=Bad\q'),
 $old.Replace('caption=Instruments','caption=Bad\n'),
 $old.Replace('caption=Instruments',('caption=Bad'+[char]7)),
 $old.Replace('caption=Instruments','caption= Instruments'),
 $old.Replace('minw=256','MINW=256'))
for($badIndex=0;$badIndex -lt $badValues.Count;$badIndex++){$bad=$badValues[$badIndex];Refuse {Assert-RestartAuiDelta $old $bad} ('malformed/unknown/changed protected content '+$badIndex)}
Refuse {Assert-RestartAuiDelta '' $old} 'missing reviewed baseline'
Refuse {Assert-RestartAuiDelta ($old.Replace('state=768','state=262912')) $old} 'malformed original not normalized'
Pass {Assert-RestartDashboardDelta 'PlugIns/Dashboard/SumLogNM' '123.5' '124.012345'} 'normal distance observation'
Pass {Assert-RestartDashboardDelta 'PlugIns/Dashboard/SumLogNM' '0' '1e-06'} 'native scientific notation'
Pass {Assert-RestartDashboardDelta 'PlugIns/Dashboard/SumLogNM' '1e6' '1000000'} 'equivalent finite formatting'
foreach($value in @('NaN','Infinity','-1','1e309','100000001','0','1124',' 124','124 ','+124')) {
 Refuse {Assert-RestartDashboardDelta 'PlugIns/Dashboard/SumLogNM' '123' $value} 'invalid/reset/excessive counter'
}
foreach($axis in @('X','Y')) {
 Pass {Assert-RestartDashboardDelta ('PlugIns/Dashboard/Dashboard1/PersistSize'+$axis) '320' '640'} 'actual pane geometry'
 foreach($bad in @('-1','32769','NaN','1.1','01','')){Refuse {Assert-RestartDashboardDelta ('PlugIns/Dashboard/Dashboard1/PersistSize'+$axis) '320' $bad} 'invalid pane geometry'}
}
foreach($key in @('PlugIns/Dashboard/UseInternSumlog','PlugIns/Dashboard/Dashboard0/PersistSizeX','PlugIns/Dashboard/Dashboard21/PersistSizeX','PlugIns/Dashboard/Dashboard1/BestSizeX','plugins/Dashboard/SumLogNM','PlugIns/Dashboard/Dashboard1/Output')) {
 Refuse {Assert-RestartDashboardDelta $key '1' '2'} 'nonallowlisted plugin field'
}
Refuse {Assert-RestartDashboardDelta 'PlugIns/Dashboard/SumLogNM' '' '1'} 'missing counter baseline'
[pscustomobject]@{suite='pure restart AUI and Dashboard persistence';checks=$script:checks;result='passed';applicationLaunched=$false}|ConvertTo-Json

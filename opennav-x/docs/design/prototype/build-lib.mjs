import fs from 'node:fs';
import {loadSymbolLibrary,symbolSpriteCss} from './symbol-library.mjs';
export function assemble(){
 const read=name=>fs.readFileSync('src/'+name,'utf8');
 const css=read('style.css')+'\n'+read('suite.css')+'\n'+symbolSpriteCss()+'\n'+read('symbols.css')+'\n'+read('chart-symbols.css');
 const symbols='const chartSymbolLibrary='+JSON.stringify(loadSymbolLibrary()).replace(/</g,'\\u003c')+';\nconst chartSymbolLicense='+JSON.stringify(fs.readFileSync('vendor/opencpn/COPYING','utf8')).replace(/</g,'\\u003c')+';\nconst seamarkGuide='+read('seamarks.json')+';\n'+read('symbols.js')+'\nconst demoChartSymbols='+read('chart-symbols.json')+';\n'+read('chart-symbols.js');
 const app=read('app.js').replace('/* NAVIGATION */',()=>read('navigation.js')).replace('/* SUITE */',()=>read('backup.js')+'\n'+read('suite.js')+'\n'+read('health.js')+'\n'+symbols+'\n'+read('chart-marker-art.js')+'\n'+read('light-sectors.js')).replace('/* RADAR */',()=>read('radar.js'));
 return read('shell.html').replace('/* STYLES */',()=>css).replace('/* APPLICATION */',()=>read('chart.js')+'\n'+app);
}

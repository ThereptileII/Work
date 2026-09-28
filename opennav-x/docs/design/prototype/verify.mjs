import fs from 'node:fs';
import vm from 'node:vm';
import assert from 'node:assert/strict';
import {assemble} from './build-lib.mjs';
import {loadSymbolLibrary,hpglGeometry} from './symbol-library.mjs';

const html=fs.readFileSync('index.html','utf8');
const script=html.match(/<script>([\s\S]*?)<\/script>/)?.[1];
assert.ok(script,'Embedded application script exists');
new vm.Script(script,{filename:'index.html embedded script'});
assert.ok(!/<script\b[^>]*\bsrc\s*=/i.test(html),'No external script');
assert.ok(!/<link\b[^>]*\bhref\s*=/i.test(html),'No external stylesheet');
assert.ok(!/\b(?:fetch|XMLHttpRequest|WebSocket)\s*\(/.test(script),'No network data dependency');
assert.ok(!html.includes('/* APPLICATION */')&&!html.includes('/* STYLES */'),'Build placeholders replaced');
const shell=fs.readFileSync('src/shell.html','utf8');
const ids=[...shell.matchAll(/\bid="([^"]+)"/g)].map(m=>m[1]);
assert.equal(ids.length,new Set(ids).size,'Shell IDs are unique');
const tokens=JSON.parse(fs.readFileSync('design-tokens.json','utf8'));
assert.deepEqual(Object.keys(tokens.palettes),['day','dusk','night']);
const expected=assemble();
assert.equal(html,expected,'Standalone HTML matches source');
const validate=vm.runInNewContext(fs.readFileSync('src/backup.js','utf8')+'\nvalidateBackup;');
const valid={format:'OpenNavX-design-backup',version:2,mock:true,sections:{vessel:true,settings:false,sensors:false,routes:true},vessel:{name:'Test vessel',draft:1.8,safetyDepth:3.5,capacity:24.8,reserve:25},routes:{name:'Test passage',destination:'Test harbour',waypoints:[],passage:[{name:'Harbour',x:200,y:350}]}};
validate(valid);
assert.throws(()=>validate({...valid,sections:{...valid.sections,sensors:true}}),'Selected section must exist');
assert.throws(()=>validate({...valid,routes:{...valid.routes,passage:[]}}),'An active passage needs a waypoint');
assert.throws(()=>validate({...valid,vessel:{...valid.vessel,safetyDepth:1}}),'Safety depth cannot be shallower than draft');
assert.throws(()=>validate({...valid,routes:{...valid.routes,passage:[{name:'Invalid',x:1200,y:30}]}}),'Coordinates must lie in the demo chart');
assert.throws(()=>validate({...valid,version:999}),'Unsupported versions must be rejected');
assert.ok(!new RegExp(['ST','4000'].join(''),'i').test(html),'Retired hardware name removed');
assert.ok(['drawerBack','chartReturn'].every(id=>ids.includes(id)),'Shared return controls are present');
console.log('PASS: script syntax, embedded assets, no network dependencies, unique shell IDs, three palettes, source/build equality.');
console.log('PASS: backup validation, retired hardware copy removed, shared return controls.');

const library=loadSymbolLibrary(),seamarks=JSON.parse(fs.readFileSync('src/seamarks.json','utf8'));
assert.equal(library.items.length,1107,'All source definitions indexed');
assert.equal(new Set(library.items.map(s=>s.id)).size,1107,'Catalogue identities are unique across types');
assert.deepEqual(['point','line','pattern'].map(k=>library.items.filter(s=>s.kind===k).length),[1018,59,30]);
assert.deepEqual(JSON.parse(fs.readFileSync('src/symbol-catalogue.json','utf8')),library,'Exported catalogue matches the pinned source');
for(const mode of ['day','dusk','night']){
 const png=fs.readFileSync('vendor/opencpn/rastersymbols-'+(mode==='night'?'dark':mode)+'.png');
 assert.equal(png.readUInt32BE(16),library.sheet.width);assert.equal(png.readUInt32BE(20),library.sheet.height);
 for(const item of library.items){
  assert.ok(library.categories.some(c=>c.id===item.category),'Known category '+item.id);
  if(item.vector){assert.ok(item.vector.shapes.length,'Nonempty geometry '+item.id);assert.ok(item.vector.viewBox.every(Number.isFinite));for(const shape of item.vector.shapes)assert.ok(library.palettes[mode][shape.color],'Known vector colour '+item.id+': '+shape.color);}
 }
}
for(const mark of seamarks)for(const key of ['code','codeA','codeB'])if(mark[key])assert.ok(library.items.some(s=>s.id==='point:'+mark[key]),'Guide glyph exists: '+mark[key]);
assert.equal(seamarks.length,12);
assert.equal(seamarks.filter(s=>!s.code&&!s.codeA).length,1,'Emergency wreck is explicitly a physical illustration');
assert.ok(seamarks.find(s=>s.id==='emergency').note.includes('no dedicated'));
assert.equal(hpglGeometry('SPA;PU0,0;PD;PU200,200;PD;','ACHBLK').shapes.length,2,'Pen-down without coordinates renders pattern dots');
assert.throws(()=>hpglGeometry('ZZ123;','ACHBLK'),'Unknown HPGL commands must not silently disappear');
const symbolUi=vm.createContext({suite:{},chartSymbolLibrary:library,seamarkGuide:seamarks,actions:{},document:{addEventListener(){}},esc:s=>String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/"/g,'&quot;'),icon:()=>''});
vm.runInContext(fs.readFileSync('src/symbols.js','utf8'),symbolUi);
for(const mode of ['day','dusk','night']){symbolUi.palette=mode;const output=vm.runInContext('chartSymbolLibrary.items.map(s=>symbolGlyph(s,palette))',symbolUi);assert.equal(output.length,1107);assert.ok(output.every(s=>s.length>0&&!/NaN|undefined|Infinity/.test(s)),'All glyphs render in '+mode);}
vm.runInContext("suite.symbolQuery='bränning';",symbolUi);assert.ok(vm.runInContext('filteredSymbols().length',symbolUi)>0,'Swedish category search');
vm.runInContext("suite.symbolQuery='BOYCAR01';",symbolUi);assert.equal(vm.runInContext('filteredSymbols()[0].code',symbolUi),'BOYCAR01','Exact symbol-code search');
vm.runInContext("suite.symbolQuery='no-such-symbol';",symbolUi);assert.equal(vm.runInContext('filteredSymbols().length',symbolUi),0,'Empty search results');
console.log('PASS: 1,107 unique source definitions, 3 palettes, sprite bounds, all vector colours, guide references, 3,321 glyph renders, Swedish/code search and offline catalogue export.');

assert.ok(html.includes('GNU GENERAL PUBLIC LICENSE'),'Upstream license is available in the standalone preview');

const placements=JSON.parse(fs.readFileSync('src/chart-symbols.json','utf8'));
assert.equal(placements.length,29);
assert.equal(new Set(placements.map(p=>p.id)).size,placements.length,'Chart object identities are unique');
for(const p of placements){
 assert.ok(library.items.some(s=>s.id===p.symbol),'Chart source exists: '+p.symbol);
 assert.ok(Number.isFinite(p.x)&&Number.isFinite(p.y)&&p.x>=0&&p.x<=1000&&p.y>=0&&p.y<=630,'Chart position inside scene: '+p.id);
 if(p.guide)assert.ok(seamarks.some(s=>s.id===p.guide),'Known seamark guide: '+p.guide);
}
Object.assign(symbolUi,{demoChartSymbols:placements,structuredClone,state:{theme:'day',zoom:1,orientation:0,heading:41,panel:null},$:()=>null,$$:()=>[]});
vm.runInContext(fs.readFileSync('src/chart-symbols.js','utf8'),symbolUi);
vm.runInContext(fs.readFileSync('src/chart-marker-art.js','utf8'),symbolUi);
assert.ok(vm.runInContext('demoChartSymbols.every(p=>chartMarkerDesigned(symbolById.get(p.symbol)))',symbolUi),'Every initial chart object has a designed vector');
for(const mode of ['day','dusk','night']){
 symbolUi.state.theme=mode;
 const output=vm.runInContext('chartSymbolLibrary.items.map(s=>chartSymbolGraphic(s))',symbolUi);
 assert.ok(output.every(s=>s.length>0&&!/NaN|undefined|Infinity/.test(s)),'All chart glyphs render in '+mode);
 assert.ok(output.every(s=>!/<image|foreignObject|background-image/.test(s)),'Chart artwork contains no raster assets');
}
assert.equal(vm.runInContext('suite.chartSymbolsData.length',symbolUi),29);
console.log('PASS: 29 unique chart placements, valid source/guide references and positions, all 3,321 chart-glyph palette combinations.');

vm.runInContext(fs.readFileSync('src/light-sectors.js','utf8'),symbolUi);
for(const [bearing,x,y] of [[0,0,-100],[90,100,0],[180,0,100],[270,-100,0]]){
 const p=vm.runInContext(`lightSectorPoint(${bearing},100)`,symbolUi);
 assert.ok(Math.abs(p.x-x)<1e-8&&Math.abs(p.y-y)<1e-8,'Sector bearings follow chart north clockwise');
}
const sectorLight=placements.find(p=>p.sectors);
assert.ok(sectorLight.sectorExtent.expanded>sectorLight.sectorExtent.compact*5,'Extended sectors reach substantially farther');
assert.deepEqual(sectorLight.sectors.map(s=>s.code),['G','W','R']);
for(let i=0;i<sectorLight.sectors.length;i++){
 const sector=sectorLight.sectors[i];assert.ok(sector.to>sector.from);
 if(i)assert.equal(sectorLight.sectors[i-1].to,sector.from,'Adjacent sectors share the same boundary');
}
console.log('PASS: sector bearing geometry, contiguous demo arcs and expanded range.');

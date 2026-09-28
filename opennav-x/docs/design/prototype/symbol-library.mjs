// Offline importer for the pinned OpenCPN chart portrayal assets (see vendor/opencpn).
import fs from 'node:fs';
import crypto from 'node:crypto';
const base='vendor/opencpn/';
const decode=s=>s.replace(/&lt;/g,'<').replace(/&gt;/g,'>').replace(/&quot;/g,'"').replace(/&apos;/g,"'").replace(/&amp;/g,'&');
const tag=(s,n)=>decode(s.match(new RegExp('<'+n+'(?:\\s[^>]*)?>([\\s\\S]*?)</'+n+'>'))?.[1]?.trim()||'');
const attrs=s=>Object.fromEntries([...s.matchAll(/([\w-]+)="([^"]*)"/g)].map(m=>[m[1],m[2]]));
const xy=(s,n)=>{const a=attrs(s.match(new RegExp('<'+n+'\\b[^>]*'))?.[0]||'');return {x:+a.x||0,y:+a.y||0};};

export const symbolCategories=[
 ['seamarks','Buoys & beacons','Bojar, prickar, sjömärken, topptecken'],
 ['lights','Lights & signals','Fyrar, fyrsken, mistsignaler, radio'],
 ['hazards','Rocks & wrecks','Grund, bränning, vrak, hinder'],
 ['depths','Depth & survey','Djup, lodning, sjömätning, kvalitet'],
 ['routes','Routes & limits','Farleder, trafikseparering, gränser, förbud'],
 ['harbours','Harbours & services','Hamn, förtöjning, ankring, service'],
 ['infrastructure','Offshore & crossings','Kablar, rör, broar, vindkraft, plattformar'],
 ['coast','Coast & landmarks','Kust, landmärken, byggnader, vegetation'],
 ['seabed','Seabed & fishing','Botten, sand, lera, fiske, vattenbruk'],
 ['water','Tides, currents & ice','Tidvatten, ström, is, väder'],
 ['vessels','Vessels & navigation','Fartyg, AIS, position, kurs'],
 ['inland','Inland signs','Inland, kanaler, sjövägmärken'],
 ['other','Other chart objects','Övriga symboler och kartobjekt']
].map(([id,name,sv])=>({id,name,sv}));

function category(code,description){
 const s=code+' '+description;
 if(/^(?:BOY|BCN|TOPM|TOPSH|TMAR|DAY|PRICKE|DIRBOY|RETRFL)/.test(code))return 'seamarks';
 if(/^(?:LIGHT|LIT|FOG|RTP|RAD|RDO)/.test(code)||/light flare|signal station|radio station/i.test(description))return 'lights';
 if(/^(?:WRECK|UWTROC|DANGER|OBSTRN|FOULAR|ISODGR|DNGHILIT|RCKLDG|HULKES)/.test(code)||/wreck|underwater rock/i.test(description))return 'hazards';
 if(/^(?:SOUND|DEP|DRGARE|DQUAL|QUAPOS|QUAPNT|QUALIN|M_QUAL|PRTSUR|NODATA|OVERSC|LOWACC|DIAMOND|UNIT|SWPARE|RSCSTA)/.test(code)||/survey|sounding|depth contour/i.test(description))return 'depths';
 if(/^(?:NOTMRK|NOTBRD|ADDMRK|INL|BNK|WTW|SIST|NMK|CLRLIN|DISMAR|HECMTR)/.test(code)||/notice mark|inland|waterway|canal|lock gate/i.test(description))return 'inland';
 if(/^(?:ACH|BERTH|BRTH|BUNSTA|CARTRF|CUSTOM|MOR|HRB|SMC|PIL|RDOCAL|ROLROL|TERMNL|TRNBSN|CRANES)/.test(code)||/anchorage|harbour|harbor|marina|mooring|pilot boarding/i.test(description))return 'harbours';
 if(/^(?:TSS|DWR|DWL|TWRT|RCR|RECTRC|NAVLNE|FAIRWY|RESARE|CTNARE|CTYARE|RESTRN|RSRDEF|INFARE|FERYRT|FRYARE)/.test(code)||/traffic|restricted|prohibited|boundary|routeing|recommended track/i.test(description))return 'routes';
 if(/^(?:CBL|PIP|BRIDGE|OFS|WIND|WND|WIM|PYL|SILTNK|TNK|FLASTK|RFNERY)/.test(code)||/cable|pipeline|platform|bridge|wind turbine|overhead|power transmission/i.test(description))return 'infrastructure';
 if(/^(?:SBDARE|FSH|MARCUL|SNDWAV|WEDKLP)/.test(code)||/seabed|sand|mud|gravel|fish|aquaculture|marine farm/i.test(description))return 'seabed';
 if(/^(?:CUR|TID|ICE|EBBST|FLDST|WATTUR|WTLVGG|HGWTMK|SPRING)/.test(code)||/tidal|current|iceberg|glacier/i.test(description))return 'water';
 if(/^(?:AIS|ARP|OWNSHP|VESS|PASTRK|PLNPOS|APTS|NORTHAR|SCALEB|OSP|VEC|WAYPNT|PLNSPD|POSITN|EBL|ERBL|REFPNT|EVENTS|MAGVAR|LOCMAG)/.test(code)||/own ship|vessel|target|position line|scale bar/i.test(description))return 'vessels';
 if(/^(?:LND|COALNE|SLCONS|BUA|BUI|VEG|CHIM|TOW|CST|MARSH|CAIRNS|DOMES|DSHAER|FLGSTF|FORSTC|HILTOP|MONUMT|MSTCON|PRDINS|QUARRY|REFDMP|SILBUI|TMBYRD|TREPNT)/.test(code)||/landmark|building|tower|church|coast|shore|vegetation|wooded|airport/i.test(s))return 'coast';
 return 'other';
}

// The line/pattern subset uses PU, PD, CI and one polygon. Reject new commands
// instead of silently rendering an incomplete example after an upstream update.
export function hpglGeometry(hpgl,colorRef){
 const pens={};for(let i=0;i<colorRef.length;i+=6)pens[colorRef[i]]=colorRef.slice(i+1,i+6);
 let pos=[0,0],pen=Object.keys(pens)[0],width=1,path='',polygon=null,completed=null;
 const shapes=[],bounds=[];
 const record=(x,y)=>{bounds.push([x,y]);};
 const add=shape=>{shape.color=pens[pen]||'CHBLK';shape.width=width;(polygon||shapes).push(shape);};
 const flush=()=>{if(path){add({type:'path',d:path});path='';}};
 for(const command of hpgl.split(';').filter(Boolean)){
  const op=command.slice(0,2),arg=command.slice(2),n=arg.split(',').map(Number);
  if(op==='SP'){flush();pen=arg;}
  else if(op==='SW'){flush();width=+arg||1;}
  else if(op==='ST'){if(+arg!==0)throw Error('Unsupported HPGL transparency: '+command);}
  else if(op==='PU'||op==='PD'){
   if(op==='PU')flush();
   if(op==='PD'&&!arg){record(...pos);add({type:'circle',cx:pos[0],cy:pos[1],r:width*5,fill:true});}
   for(let i=0;i<n.length-1;i+=2){const next=[n[i],n[i+1]];record(...next);if(op==='PD'){if(!path)path='M'+pos.join(' ');path+='L'+next.join(' ');}pos=next;}
  }else if(op==='CI'){flush();const r=n[0];record(pos[0]-r,pos[1]-r);record(pos[0]+r,pos[1]+r);add({type:'circle',cx:pos[0],cy:pos[1],r});}
  else if(op==='PM'){flush();if(+arg===0)polygon=[];else if(+arg===2){completed=polygon;polygon=null;}else throw Error('Unsupported HPGL polygon mode: '+command);}
  else if(op==='FP'){if(completed){for(const shape of completed)shapes.push({...shape,fill:true});completed=null;}}
  else throw Error('Unsupported HPGL command: '+command);
 }
 flush();if(!bounds.length||!shapes.length)throw Error('Empty vector graphic');
 const xs=bounds.map(p=>p[0]),ys=bounds.map(p=>p[1]),x=Math.min(...xs),y=Math.min(...ys),w=Math.max(...xs)-x,h=Math.max(...ys)-y;
 const pad=Math.max(w,h,100)*.055;
 return {viewBox:[x-pad,y-pad,Math.max(w,1)+2*pad,Math.max(h,1)+2*pad],shapes};
}

export function loadSymbolLibrary(){
 const xml=fs.readFileSync(base+'chartsymbols.xml','utf8');
 const provenance=JSON.parse(fs.readFileSync(base+'SOURCE.json','utf8').replace(/^\uFEFF/,''));
 const palettes={};
 for(const [key,name] of [['day','DAY_BRIGHT'],['dusk','DUSK'],['night','NIGHT']]){
  const body=xml.match(new RegExp('<color-table name="'+name+'">([\\s\\S]*?)</color-table>'))[1];
  palettes[key]=Object.fromEntries([...body.matchAll(/<color\s+[^>]*>/g)].map(m=>{const a=attrs(m[0]);return [a.name,`rgb(${a.r} ${a.g} ${a.b})`];}));
 }
 const png=fs.readFileSync(base+'rastersymbols-day.png'),sheet={width:png.readUInt32BE(16),height:png.readUInt32BE(20)};
 const items=[];
 for(const [kind,section,element] of [['point','symbols','symbol'],['line','line-styles','line-style'],['pattern','patterns','pattern']]){
  const content=tag(xml,section),unique=new Map();
  for(const match of content.matchAll(new RegExp('<'+element+'\\b[^>]*>([\\s\\S]*?)</'+element+'>','g'))){
   const body=match[1],code=tag(body,'name'),prior=unique.get(code),description=tag(body,'description')||prior?.description||'';
   const bitmapTag=body.match(/<bitmap\s+([^>]+)>([\s\S]*?)<\/bitmap>/),vector=tag(body,'vector');
   const entry={id:kind+':'+code,code,kind,description,category:category(code,description)};
   if(bitmapTag){const a=attrs(bitmapTag[1]),p=xy(bitmapTag[2],'graphics-location');entry.bitmap={x:p.x,y:p.y,width:+a.width,height:+a.height,pivot:xy(bitmapTag[2],'pivot')};
    const b=entry.bitmap;if(!b.width||!b.height||b.x<0||b.y<0||b.x+b.width>sheet.width||b.y+b.height>sheet.height)throw Error('Invalid sprite crop '+code);
   }else{try{entry.vector=hpglGeometry(tag(vector,'HPGL')||tag(body,'HPGL'),tag(body,'color-ref'));}catch(e){throw Error(kind+' '+code+': '+e.message);}}
   unique.set(code,entry);
  }
  items.push(...unique.values());
 }
 items.sort((a,b)=>a.code.localeCompare(b.code)||a.kind.localeCompare(b.kind));
 return {version:1,source:{...provenance,xmlSha256:crypto.createHash('sha256').update(xml).digest('hex'),deduplication:'Last definition per name and type; earlier description retained only when the final definition has none.'},sheet,palettes,categories:symbolCategories,items};
}

export function symbolSpriteCss(){
 return ':root{'+[['day','day'],['dusk','dusk'],['night','dark']].map(([key,file])=>'--symbol-sheet-'+key+':url("data:image/png;base64,'+fs.readFileSync(base+'rastersymbols-'+file+'.png').toString('base64')+'");').join('')+'}';
}

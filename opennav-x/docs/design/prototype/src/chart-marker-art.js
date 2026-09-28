// Original OpenNav chart artwork. Source glyphs stay untouched in the atlas.
// Only explicitly mapped definitions receive a designed portrayal. Other source
// definitions use a labelled reference pin, never a guessed navigation symbol.
const chartMarkerGuide=new Map();
for(const g of seamarkGuide)for(const region of ['A','B']){
 const code=g['code'+region]||g.code;
 if(code)chartMarkerGuide.set('point:'+code,{...g,colors:g['colors'+region]||g.colors});
}
const chartMarkerArtwork={
 'point:BCNGEN01':'<path d="M-5 8H5M-3 7-2-5H2L3 7M-4-5H4L0-11Z"/><path d="M-2 2H2"/>',
 'point:LIGHTS13':()=>lighthouseArtwork(),
 'point:RTPBCN02':'<circle r="2"/><path d="M-5-5a7 7 0 0 0 0 10M5-5a7 7 0 0 1 0 10M-8-8a11 11 0 0 0 0 16M8-8a11 11 0 0 1 0 16"/>',
 'point:UWTROC03':'<path d="M-4-4 4 4M-4 4 4-4"/><g class="marker-dot"><circle cy="-7" r=".7"/><circle cy="7" r=".7"/><circle cx="-7" r=".7"/><circle cx="7" r=".7"/></g>',
 'point:UWTROC04':'<path d="M-5 0H5M0-5V5"/><path d="M-8-3a9 9 0 0 1 5-5M3-8a9 9 0 0 1 5 5M8 3a9 9 0 0 1-5 5M-3 8a9 9 0 0 1-5-5" stroke-dasharray="1 3"/>',
 'point:WRECKS05':'<path d="m-9 2 4 4H5L9 2ZM-2 1V-7L4-4H-2M-7-1h3M4-1h3"/><path d="M-11 9q3-2 6 0t6 0t6 0t5 0" opacity=".55"/>',
 'point:ACHARE51':'<circle cy="-7" r="2"/><path d="M0-5V9M-4-2H4M-8 2c0 5 4 7 8 7s8-2 8-7M-8 2l-2 3M-8 2l3 1M8 2l2 3M8 2l-3 1"/>',
 'point:SMCFAC02':'<path d="M-9-3 0-10 9-3M-7-4V7H7V-4M0-4V5M-3 1q0 4 3 4t3-4M-2-2H2"/>',
 'point:PILBOP02':'<path d="m0-11 9 11-9 11-9-11Z"/><path d="M-2.5 4V-4H1a3 3 0 0 1 0 6h-3.5"/>',
 'line:CBLSUB06':'<path d="M-12 0q3-5 6 0t6 0t6 0t6 0"/>',
 'pattern:FSHFAC03':'<path d="m0-7 6 7-6 7-6-7ZM-6 0H6M0-7V7"/>'
};
function lighthouseArtwork(){return '<circle class="lighthouse-point" r="3.5"/><path class="lighthouse-rays" d="M0-7V-10M7 0H10M0 7V10M-7 0H-10"/>';}
function chartMarkerDesigned(s){return chartMarkerGuide.has(s.id)||Object.hasOwn(chartMarkerArtwork,s.id);}
function chartMarkerArt(s){
 const g=chartMarkerGuide.get(s.id);
 if(!g){const art=chartMarkerArtwork[s.id];return typeof art==='function'?art():art||'<path d="M0 11C-3 6-8 2-8-3a8 8 0 0 1 16 0c0 5-5 9-8 14Z"/><circle cy="-3" r="2"/>';}
 const c=g.colors.map(color=>'var(--mark-'+color+')'),top=g.top;
 const cone=(y,up)=>`<path d="${up?`M0 ${y}l-3 4.5h6Z`:`M-3 ${y}h6L0 ${y+4.5}Z`}"/>`;
 let head='';
 if(top==='can')head='<rect x="-2.6" y="-11" width="5.2" height="4.5" rx=".4"/>';
 else if(top==='cone')head=cone(-12,true);
 else if(['north','east','south','west'].includes(top))head=cone(-14,['north','east'].includes(top))+cone(-7,['north','west'].includes(top));
 else if(top==='spheres')head='<circle cy="-12" r="2.2"/><circle cy="-6" r="2.2"/>';
 else if(top==='sphere')head='<circle cy="-9" r="3"/>';
 else if(top==='x')head='<path d="M-3-12 3-6M3-12-3-6" fill="none"/>';
 const topColor=['north','east','south','west','spheres'].includes(top)?'var(--mark-black)':c[0];
 const stem=c.map((color,i)=>`<path d="M0 ${-3+i*11/c.length}v${11/c.length}" stroke="${color}" stroke-width="1.6"/>`).join('');
 return `<g stroke="${c[0]}"><path d="M0-7V8"/>${stem}<g fill="${topColor}" fill-opacity=".13" stroke="${topColor}">${head}</g><path d="M-2.5 6H2.5"/><circle cy="9" r="1.7" fill="var(--water)"/></g>`;
}
function chartMarkerTone(s){const g=chartMarkerGuide.get(s.id);return g?'buoy':s.code==='LIGHTS13'?'light':['UWTROC03','UWTROC04','WRECKS05'].includes(s.code)?'hazard':s.kind==='line'||s.kind==='pattern'?'area':'service';}
function chartSymbolGraphic(s,size=27){
 const designed=chartMarkerDesigned(s);
 return `<g class="chart-marker-art marker-${chartMarkerTone(s)}${designed?'':' marker-reference'}" transform="scale(${size/32})" aria-hidden="true">${chartMarkerArt(s)}${designed?'':`<text class="chart-reference-code" x="12" y="0">${esc(s.code)}</text>`}</g>`;
}
function chartSymbolPreview(s){return `<svg class="charted-symbol-vector" viewBox="-26 -26 52 52" aria-hidden="true">${chartSymbolGraphic(s,42)}</svg>`;}

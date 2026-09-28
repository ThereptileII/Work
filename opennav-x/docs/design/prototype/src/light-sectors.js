// Fictional sector bearings are clockwise from chart north, looking out from the light.
// Geometry lives in chart space, so sector bearings turn with the chart.
Object.assign(suite,{lightSectorHover:'',lightSectorPinned:'',lightSectorSuppressed:''});
function lightSectorPoint(bearing,radius){const a=bearing*Math.PI/180;return {x:Math.sin(a)*radius,y:-Math.cos(a)*radius};}
function lightSectorPath(start,end,radius){const a=lightSectorPoint(start,radius),b=lightSectorPoint(end,radius),large=((end-start+360)%360)>180?1:0;return {wedge:`M0 0L${a.x} ${a.y}A${radius} ${radius} 0 ${large} 1 ${b.x} ${b.y}Z`,arc:`M${a.x} ${a.y}A${radius} ${radius} 0 ${large} 1 ${b.x} ${b.y}`,edges:`M${a.x} ${a.y}L0 0 ${b.x} ${b.y}`};}
function lightSectorExpanded(item){return suite.lightSectorPinned===item.id||(suite.lightSectorHover===item.id&&suite.lightSectorSuppressed!==item.id);}
function syncLightSectors(){
 const layer=$('#chartLightSectorLayer');if(!layer)return;
 const lights=suite.chartSymbolsData.filter(item=>item.sectors);
 layer.style.display=suite.chartSymbols?'':'none';
 const key=[suite.chartSymbols,state.orientation,state.heading,state.zoom,...lights.map(item=>item.id+':'+lightSectorExpanded(item))].join('|');
 if(layer.dataset.renderKey!==key){
  layer.dataset.renderKey=key;
  const angle=[0,-43,-state.heading][state.orientation];
  layer.innerHTML=lights.map(item=>{const expanded=lightSectorExpanded(item),radius=expanded?item.sectorExtent.expanded:item.sectorExtent.compact;
   return `<g class="light-sector-fan ${expanded?'expanded':''}" data-light-sectors="${item.id}" data-extent="${radius}" transform="translate(${item.x} ${item.y})">${item.sectors.map(sector=>{const p=lightSectorPath(sector.from,sector.to,radius),label=lightSectorPoint((sector.from+sector.to)/2,radius+12/state.zoom);return `<g class="light-sector sector-${sector.color}"><path class="sector-wash" d="${p.wedge}"/><path class="sector-boundary" d="${p.edges}"/><path class="sector-arc" d="${p.arc}"/>${expanded?`<g transform="translate(${label.x} ${label.y}) rotate(${-angle}) scale(${1/state.zoom})"><text class="sector-letter" text-anchor="middle" dominant-baseline="middle">${sector.code}</text></g>`:''}</g>`;}).join('')}</g>`;
  }).join('');
 }
 for(const item of lights){const marker=$(`[data-chart-symbol="${item.id}"]`);if(marker){marker.setAttribute('aria-pressed',String(suite.lightSectorPinned===item.id));marker.setAttribute('aria-label',`${item.name} · ${suite.lightSectorPinned===item.id?'Collapse':'Extend'} light sectors`);marker.classList.toggle('sectors-active',lightSectorExpanded(item));}}
 const pinned=lights.find(item=>item.id===suite.lightSectorPinned),readout=$('#lightSectorReadout');
 const visible=!!pinned&&suite.chartSymbols&&state.view==='chart'&&!state.panel&&!state.tool;
 readout.hidden=!visible;$('#chartView').classList.toggle('has-light-selection',visible);
 if(visible){$('#lightSectorName').textContent=pinned.sv||pinned.name;}
}
function clearLightSectors(restoreFocus=false){const id=suite.lightSectorPinned||suite.lightSectorHover;suite.lightSectorSuppressed=id;suite.lightSectorPinned='';suite.lightSectorHover='';syncLightSectors();if(restoreFocus)$(`[data-chart-symbol="${id}"]`)?.focus({preventScroll:true});}
function toggleLightSectors(item){if(suite.lightSectorPinned===item.id)clearLightSectors();else {suite.lightSectorPinned=item.id;suite.chartSymbolId=item.id;syncLightSectors();}}
function sectorMarker(target){const marker=target?.closest?.('[data-chart-symbol]');return marker&&suite.chartSymbolsData.some(item=>item.id===marker.dataset.chartSymbol&&item.sectors)?marker:null;}
document.addEventListener('pointerover',e=>{const marker=sectorMarker(e.target);if(!marker||state.tool||e.pointerType==='touch')return;if(sectorMarker(e.relatedTarget)===marker)return;suite.lightSectorHover=marker.dataset.chartSymbol;syncLightSectors();});
document.addEventListener('pointerout',e=>{const marker=sectorMarker(e.target);if(!marker||sectorMarker(e.relatedTarget)===marker)return;suite.lightSectorHover='';suite.lightSectorSuppressed='';syncLightSectors();});
document.addEventListener('focusin',e=>{const marker=sectorMarker(e.target);if(marker&&marker.matches(':focus-visible')&&!state.tool){suite.lightSectorHover=marker.dataset.chartSymbol;syncLightSectors();}});
document.addEventListener('focusout',e=>{if(sectorMarker(e.target)){suite.lightSectorHover='';suite.lightSectorSuppressed='';syncLightSectors();}});
Object.assign(actions,{collapseLightSectors:()=>clearLightSectors(true),lightSectorDetails:()=>{suite.chartSymbolId=suite.lightSectorPinned;openPanel('chartSymbol');}});

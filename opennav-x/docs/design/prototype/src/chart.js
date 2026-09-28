/* Original illustrative cartography. Coordinates and contours are fictional. */
function createChart() {
 const islands = [
  [-40,175,160,265,1],[95,110,140,122,2],[216,20,110,108,3],
  [122,340,113,92,4],[267,282,57,83,5],[333,226,45,34,6],[220,465,87,45,7],
  [107,575,153,106,8],[348,598,82,56,9],[383,473,35,23,10],
  [510,-10,159,120,11],[397,71,38,41,12],[678,32,52,50,13],
  [740,122,34,32,14],[888,35,77,46,15],[991,42,49,84,16],
  [940,222,80,59,17],[1000,391,100,118,18],[846,415,51,62,19],
  [667,512,69,39,20],[550,596,41,34,21],[731,645,112,81,22],
  [908,597,99,94,23],[560,208,47,21,24],[610,140,22,17,25],
  [466,138,18,24,26],[691,227,23,32,27],[746,393,22,17,28],
  [360,366,19,11,29],[288,397,11,16,30],[505,500,11,18,31],
  [790,551,26,17,32],[848,325,15,16,33],[406,292,12,10,34],
  [328,114,12,20,35],[190,206,14,10,36],[668,362,9,13,37],
  [793,207,8,12,38],[974,524,24,17,39],[452,565,13,14,40]
 ];
 const coast = ([cx,cy,rx,ry,seed]) => {
  const points=[];
  for(let i=0;i<54;i++) { const a=i/54*Math.PI*2;const r=1+.10*Math.sin(a*7+seed)+.07*Math.cos(a*11-seed)+.035*Math.sin(a*17+seed*2);points.push([cx+Math.cos(a)*rx*r,cy+Math.sin(a)*ry*r]); }
  const midpoint=(a,b)=>[(a[0]+b[0])/2,(a[1]+b[1])/2];
  const start=midpoint(points.at(-1),points[0]);
  return 'M'+start.join(',')+points.map((p,i)=>'Q'+p.join(',')+' '+midpoint(p,points[(i+1)%points.length]).join(',')).join('')+'Z';
 };
 let svg='<svg id="chartSvg" viewBox="0 0 1000 630" preserveAspectRatio="xMidYMid slice" role="group" aria-label="Illustrative St. Anna coastal chart with route to Arkösund"><defs><pattern id="chartGrid" width="125" height="105" patternUnits="userSpaceOnUse"><path d="M125 0H0V105" class="grid-line"/></pattern><pattern id="landTexture" width="7" height="7" patternUnits="userSpaceOnUse"><circle cx="2" cy="2" r=".45" fill="var(--shore)" opacity=".3"/></pattern><radialGradient id="shipGlow"><stop stop-color="#267c76" stop-opacity=".09"/><stop offset="1" stop-color="#267c76" stop-opacity="0"/></radialGradient></defs><g id="mapTransform"><rect x="-2000" y="-2000" width="5000" height="5000" fill="var(--water)"/><rect width="1000" height="630" fill="url(#chartGrid)"/>';
 islands.forEach(island=>{const [x,y]=island;const d=coast(island);svg+=`<path d="${d}" class="chart-halo"/><path d="${d}" class="chart-contour" transform="translate(${x} ${y}) scale(1.26) translate(${-x} ${-y})"/><path d="${d}" class="chart-contour" opacity=".5" transform="translate(${x} ${y}) scale(1.5) translate(${-x} ${-y})"/>`;});
 svg+='<g id="contourLayer"><path d="M357 -30C330 70 472 125 431 194S441 272 500 302 615 345 627 415 581 509 625 639M466 -30C414 74 514 97 506 152S476 295 578 312 715 369 706 432 739 541 792 634M824 -40C862 81 809 180 853 255S848 414 773 465 837 564 840 662" class="chart-contour" stroke-dasharray="3 4" opacity=".8"/></g>';
 islands.forEach(island=>{const d=coast(island);svg+=`<path class="chart-land" d="${d}"/><path d="${d}" fill="url(#landTexture)"/>`;});
 svg+='<g fill="none" stroke="var(--shore)" stroke-width="1" opacity=".65"><path d="M-30 55Q35 102 67 109T152 142M88 265Q115 300 114 345T147 401M55 522Q88 556 124 566T175 624M420 -15Q470 5 499 57T551 48M854 7 884 34 914 40"/></g>';
 const labels=[[81,158,'NORRA FINNÖ'],[105,349,'TYRISLÖT'],[255,285,'Aspöja'],[510,30,'LÅNGHOLMEN'],[230,465,'Missjö'],[682,518,'Kråkmarö'],[905,25,'ARKÖ'],[892,605,'Jungfruskär'],[855,412,'Äspskär'],[329,613,'Kopparholmen'],[561,211,'Bockskär'],[948,219,'Gränsö']];
 labels.forEach(([x,y,t])=>svg+=`<text class="chart-label" x="${x}" y="${y}" text-anchor="middle" font-size="${t===t.toUpperCase()?11:10}">${t}</text>`);
 svg+='<text class="chart-water-label" x="460" y="350" transform="rotate(-22 460 350)">SANKT ANNA</text><text class="chart-water-label" x="630" y="570" font-size="12" transform="rotate(-15 630 570)">ÖSTERSJÖN</text>';
 svg+='<g id="depthLayer">';
 for(let iy=0;iy<10;iy++)for(let ix=0;ix<15;ix++){let x=ix*74+((iy*29)%48),y=iy*70+((ix*21)%37);if(!islands.some(([cx,cy,rx,ry])=>((x-cx)/(rx+12))**2+((y-cy)/(ry+10))**2<1.3)){const val=(5+((ix*73+iy*31)%214)/10).toFixed(1);svg+=`<text class="chart-depth" x="${x}" y="${y}">${val}</text>`;}}
 svg+='</g><g stroke="var(--chart-text)" stroke-width=".8" opacity=".65"><path d="m416 446 6 6m0-6-6 6m204-172 6 6m0-6-6 6m148 51 6 6m0-6-6 6m-311-129 6 6m0-6-6 6"/></g>';
 svg+='<g id="chartLightSectorLayer" aria-hidden="true" pointer-events="none"></g><g id="chartSymbolLayer"></g><g id="routeLayer"><path class="chart-route-corridor" d="M500 420 596 330 795 278 820 134 884 149"/><path class="chart-route-under" d="M500 420 596 330 795 278 820 134 884 149"/><path class="chart-route" d="M500 420 596 330 795 278 820 134 884 149"/><path class="chart-track" d="M373 673 405 552Q430 495 500 420"/>';
 [[596,330,'01'],[795,278,'02'],[820,134,'03'],[884,149,'04']].forEach(([x,y,n])=>svg+=`<g class="map-waypoint" tabindex="0" role="button" aria-label="Waypoint ${n}" data-map-waypoint="${Number(n)-1}"><circle cx="${x}" cy="${y}" r="10"/><text x="${x}" y="${y+.5}">${n}</text></g>`);
 svg+='<g transform="translate(596 330)"><rect class="map-pop-bg" x="-19" y="18" width="94" height="25" rx="5"/><text class="map-pop-label" x="-8" y="34">Långholmen</text></g><g transform="translate(884 149)"><rect class="map-pop-bg" x="-102" y="18" width="90" height="25" rx="5"/><text class="map-pop-label" x="-91" y="34">⚑ Arkösund</text></g></g>';
 svg+='<g id="aisLayer">';
 [[690,307,220,'Freja',0],[430,366,55,'S/Y Liv',1],[718,454,-20,'Baltic Pearl',2],[784,77,175,'Saga',3]].forEach(([x,y,r,name,n])=>svg+=`<g class="ais-ship" id="ais-${n}" role="button" tabindex="0" aria-label="AIS target ${name}" data-target="${n}" transform="translate(${x} ${y})"><g transform="rotate(${r})"><line x1="0" y1="-13" x2="0" y2="-65"/><path d="M0-12 6 9 0 5-6 9Z"/></g><text x="14" y="-4">${name}</text></g>`);
 svg+='</g><g id="ownShip" transform="translate(500 420)"><circle r="85" fill="url(#shipGlow)"/><circle r="49" fill="none" stroke="var(--route)" stroke-width=".7" opacity=".18"/><circle r="87" fill="none" stroke="var(--route)" stroke-width=".6" stroke-dasharray="3 5" opacity=".18"/><g transform="rotate(43)"><path d="M0-21V-100" stroke="var(--route)" stroke-width="1.2" stroke-dasharray="5 5" opacity=".65"/></g><g id="ownShipHeading" transform="rotate(41)"><path d="M0-19 11 16 0 10-11 16Z" fill="var(--route)" stroke="var(--floating)" stroke-width="3"/></g><g transform="translate(22 26)"><rect class="map-pop-bg" x="0" y="0" width="86" height="28" rx="6"/><circle cx="12" cy="14" r="3" fill="var(--route)"/><text class="map-pop-label" x="22" y="18" font-weight="600">Reptil · 6.3 kn</text></g></g><g id="customWaypoints"></g><g id="measureLayer"></g><g id="anchorMapLayer" hidden><circle cx="500" cy="420" r="70" stroke="var(--route)" stroke-width="1.5" stroke-dasharray="5 5" fill="var(--route)" fill-opacity=".06"/><text x="485" y="425" font-size="24" fill="var(--route)">⚓</text></g><g id="radarMapLayer" hidden class="map-radar"><circle cx="500" cy="420" r="200" fill="none" stroke="var(--route)"/><circle cx="500" cy="420" r="100" fill="none" stroke="var(--route)"/><path d="M500 420 320 200A290 290 0 0 1 610 150Z" fill="var(--route)" opacity=".15"/></g></g></svg>';
 return svg;
}

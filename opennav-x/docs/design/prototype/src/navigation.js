// A shared return path for sheets, screens, and nested dialogs.
const navHistory=[];
const modalHistory=[];
let restoringNavigation=false;
const panelNames={chartSymbol:'Chart symbol',symbols:'Symbol atlas',symbolDetail:'Symbol details',chart:'Chart',anchor:'Anchor watch',alerts:'Alerts',search:'Chart search',point:'Chart position',object:'Chart object',lookahead:'Route advisory',radarControl:'Radar echo controls',settings:'Settings',layers:'Chart presentation',health:'Source health',autopilot:'Autopilot',rail:'Instruments',route:'Passage',ais:'Traffic',target:'Vessel details',sensors:'Sensors',sensor:'Sensor details',sensorAdd:'Add sensor',system:'Software centre',installation:'Installation & recovery',updates:'Software updates',backups:'Backup & restore',diagnostics:'Diagnostics',help:'Help & guides',guide:'Guide',charts:'Chart library',chartPack:'Chart details',alarms:'Alarms',plugins:'Plugins',about:'About OpenNav X',library:'Passage library',adapter:'Adapter setup',display:'Display',welcome:'Vessel setup',installer:'Installation',energy:'Energy',instruments:'Instruments',radar:'Radar',maintenance:'Recovery',setup:'Vessel setup'};
function inputSnapshot(root){return [...root.querySelectorAll('input,select,textarea')].map((el,i)=>({i,id:el.id,value:el.value,checked:el.checked}));}
function restoreInputs(root,values=[]){const inputs=[...root.querySelectorAll('input,select,textarea')];values.forEach(v=>{const el=v.id?root.querySelector('#'+CSS.escape(v.id)):inputs[v.i];if(el&&el.type!=='file'){el.value=v.value;el.checked=v.checked;}});}
function captureContext(){return {focus:document.activeElement,view:state.view,panel:state.panel,settingsTab:state.settingsTab,target:state.target,scroll:state.panel?$('#drawerBody').scrollTop:$('#fullView').scrollTop,fields:inputSnapshot(state.panel?$('#drawerBody'):$('#fullView')),ui:typeof suite!=='undefined'?{sensorId:suite.sensorId,sensorStep:suite.sensorStep,installerStep:suite.installerStep,setupStep:suite.setupStep,guide:suite.guide,packId:suite.packId,maintenanceStep:suite.maintenanceStep,diagnosticTab:suite.diagnosticTab,symbolMode:suite.symbolMode,symbolCategory:suite.symbolCategory,symbolQuery:suite.symbolQuery,symbolKind:suite.symbolKind,symbolPalette:suite.symbolPalette,symbolPage:suite.symbolPage,symbolId:suite.symbolId,symbolGuide:suite.symbolGuide,buoyRegion:suite.buoyRegion,chartSymbolId:suite.chartSymbolId}:null};}
function contextName(c){if(!c)return 'Chart';if(c.view==='installer'&&!c.panel)return 'Installation · '+['Welcome','Detect host','Options','Review','Install','Ready'][c.ui?.installerStep||0];if(c.view==='setup'&&!c.panel)return 'Vessel setup · '+['Your vessel','Display','Sources','Energy','Helm control','Ready'][c.ui?.setupStep||0];if(c.panel==='sensorAdd')return 'Add sensor · '+['Connection','Discovery','Assignment','Verify'][c.ui?.sensorStep||0];return c.panel==='settings'?'Settings · '+c.settingsTab:panelNames[c.panel||c.view]||'Chart';}
function pushContext(){if(!restoringNavigation)navHistory.push(captureContext());}
// A sheet over a workspace closes; a drilled-in page returns to its parent.
// Wizards own their single return action in the footer instead of the heading.
function returnLabel(previous,overlay=false){return previous&&(previous.panel||['installer','setup','maintenance','symbols'].includes(previous.view)||(!overlay&&previous.view!=='chart'))?'Back':'Close';}
function focusContextReturn(){const root=state.panel?$('#drawer'):$('#fullView');root.querySelector('[data-context-return]:not([hidden])')?.focus({preventScroll:true});}
function updateReturnControl(el,label,previous){if(!el)return;el.dataset.contextReturn='';el.dataset.returnKind=label.toLowerCase();el.innerHTML=icon(label==='Back'?'arrow':'close')+'<span>'+label+'</span>';el.setAttribute('aria-label',label==='Back'?'Back to '+contextName(previous):'Close '+(panelNames[state.panel||state.view]||'panel'));el.title=label==='Back'?'Back to '+contextName(previous):'Return to '+contextName(previous);}
function updateNavigation(){
 syncChartSymbolSelection();$('#app').classList.toggle('symbols-focus',state.view==='symbols');$('#app').classList.toggle('energy-focus',state.view==='energy');$('#chartView').hidden=state.view!=='chart';$('#app').classList.toggle('radar-focus',state.view==='radar');
 const previous=navHistory.at(-1),chartBack=$('#chartReturn');
 chartBack.hidden=state.view!=='chart'||!!state.panel||!previous||!!state.tool;chartBack.innerHTML=icon('arrow')+'Back to '+esc(contextName(previous));chartBack.dataset.contextReturn='';$('#chartView').classList.toggle('chart-context',!chartBack.hidden);
 const drawerReturn=$('#drawerBack');drawerReturn.hidden=state.panel==='sensorAdd';updateReturnControl(drawerReturn,returnLabel(previous,true),previous);
 $('#fullView').inert=state.view==='symbols'&&!!state.panel;const atlasReturn=$('#fullView .view-back');if(atlasReturn)atlasReturn.hidden=state.view==='symbols'&&!!state.panel;
 $('#app').classList.toggle('service-mode',['installer','setup','maintenance'].includes(state.view));updateReturnControl($('#fullView .view-back'),returnLabel(previous),previous);
 $('#drawer').classList.toggle('drawer-wide',['settings','system','sensors','sensor','sensorAdd','installation','updates','backups','diagnostics','help','guide','charts','chartPack','alarms','plugins','adapter','about','display','library','symbolDetail','chartSymbol'].includes(state.panel));
}
function wizardReturn(step,complete=false){const label=complete?'Done':step===0?'Cancel':'Back';return `<button class="btn" data-action="back" data-context-return>${label==='Back'?icon('arrow'):''}${label}</button>`;}
function exitFlow(){
 const flow=state.panel==='sensorAdd'?'sensorAdd':state.view;
 if(flow==='installer'){suite.installBusy=false;suite.installRun=(suite.installRun||0)+1;}
 if(flow==='maintenance'){suite.maintenanceBusy=false;suite.maintenanceRun=(suite.maintenanceRun||0)+1;}
 while(navHistory.length&&(flow==='sensorAdd'?navHistory.at(-1).panel==='sensorAdd':navHistory.at(-1).view===flow&&!navHistory.at(-1).panel))navHistory.pop();
 const origin=navHistory.pop();if(origin)restoreContext(origin);else showChart();
}
function showChart(){if(suite.installBusy){suite.installBusy=false;suite.installProgress=0;}navHistory.length=0;modalHistory.length=0;$('#modalScrim').hidden=true;state.view='chart';state.panel=null;$('#fullView').hidden=true;$('#chartView').hidden=false;$('#drawer').hidden=true;activeNav('chart');updateNavigation();}
function closePanel(){state.panel=null;$('#drawer').hidden=true;navHistory.length=0;$$('.ais-ship').forEach(el=>el.classList.remove('selected'));activeNav(state.view);updateNavigation();if(drawerFocus?.isConnected)drawerFocus.focus({preventScroll:true});}
function openPanel(name,options={}){if(options.root){navHistory.length=0;if(state.view!=='chart'&&!['settings','health','alerts'].includes(name)){state.view='chart';$('#fullView').hidden=true;}state.panel=null;}if(state.panel!==name)pushContext();if($('#drawer').hidden)drawerFocus=document.activeElement;state.panel=name;$('#drawer').hidden=false;activeNav(['ais','target'].includes(name)?'ais':['route','anchor'].includes(name)?name:'settings');renderPanel();$('#drawerBody').scrollTop=0;updateNavigation();focusContextReturn();}
function setPanel(eyebrow,title,body){$('#drawerEyebrow').textContent=eyebrow;$('#drawerTitle').textContent=title;$('#drawerBody').innerHTML=body;hydrate();updateNavigation();}
function fullView(name,options={}){if(options.root){navHistory.length=0;state.view='chart';state.panel=null;}if(state.view!==name||state.panel)pushContext();state.panel=null;state.view=name;$('#drawer').hidden=true;$('#fullView').hidden=false;$('#fullView').scrollTop=0;activeNav(['setup','installer','maintenance'].includes(name)?'settings':name);renderView();updateNavigation();}
function restoreContext(c){restoringNavigation=true;state.view=c.view;state.panel=c.panel;state.settingsTab=c.settingsTab;state.target=c.target;if(c.ui&&typeof suite!=='undefined')Object.assign(suite,c.ui);$('#fullView').hidden=c.view==='chart';if(c.view!=='chart')renderView();$('#drawer').hidden=!c.panel;if(c.panel)renderPanel();restoreInputs(c.panel?$('#drawerBody'):$('#fullView'),c.fields);(c.panel?$('#drawerBody'):$('#fullView')).scrollTop=c.scroll;activeNav(c.panel==='settings'?'settings':c.panel||c.view);updateNavigation();restoringNavigation=false;if(c.focus?.isConnected&&c.focus.getClientRects().length)c.focus.focus({preventScroll:true});else focusContextReturn();}
function goBack(){
 if(!$('#modalScrim').hidden){if(modalHistory.length){const m=modalHistory.pop();$('#modal').innerHTML=m.html;restoreInputs($('#modal'),m.fields);$('#modal').scrollTop=m.scroll;hydrate();$('#modal [data-context-return]')?.focus();}else closeModal();return;}
 if(!state.panel&&state.view==='installer'){if(suite.installBusy){actions.cancelInstallation();return;}if(suite.installerStep===0||suite.installerStep>=4){exitFlow();return;}}
 if(!state.panel&&state.view==='maintenance'){exitFlow();return;}
 if((state.panel==='sensorAdd'&&suite.sensorStep===0)||(!state.panel&&state.view==='setup'&&suite.setupStep===0)){exitFlow();return;}
 if(state.tool){state.appendRoute=false;setTool(null);}
 const c=navHistory.pop();if(c)restoreContext(c);else showChart();
}
function openModal(title,body,eyebrow='OPENNAV X'){
 if(!$('#modalScrim').hidden)modalHistory.push({html:$('#modal').innerHTML,fields:inputSnapshot($('#modal')),scroll:$('#modal').scrollTop});else {modalHistory.length=0;modalFocus=document.activeElement;}
 $('#modal').innerHTML=`<div class="modal-top"><span class="eyebrow mint">${eyebrow}</span></div><h2 id="modalTitle">${title}</h2>${body}`;
 const bodyReturn=$('#modal [data-action="back"],#modal [data-action="closeModal"]');
 if(bodyReturn){bodyReturn.dataset.action='back';bodyReturn.dataset.contextReturn='';if(bodyReturn.textContent.trim()==='Back'&&!modalHistory.length)bodyReturn.textContent='Cancel';}
 else {const label=modalHistory.length?'Back':'Close';$('#modal .modal-top').insertAdjacentHTML('beforeend',`<button class="modal-back" data-action="back" data-context-return aria-label="${label==='Back'?'Back to previous dialog':'Close dialog'}">${icon(label==='Back'?'arrow':'close')}<span>${label}</span></button>`);}
 $('#modalScrim').hidden=false;$('#modal').scrollTop=0;$('#modal').querySelector('input,select,button')?.focus();hydrate();
}
function closeModal(){$('#modalScrim').hidden=true;modalHistory.length=0;if(modalFocus?.isConnected)modalFocus.focus({preventScroll:true});}

function chartContext(){pushContext();state.panel=null;state.view='chart';$('#drawer').hidden=true;$('#fullView').hidden=true;activeNav('chart');updateNavigation();}

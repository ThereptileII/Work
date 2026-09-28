// Validate mock backups before showing the review step or applying any values.
function validateBackup(d){
 const fail=()=>{throw Error('The backup contains invalid or unsupported configuration. Nothing was changed.');};
 const obj=x=>!!x&&typeof x==='object'&&!Array.isArray(x);
 const str=(x,n=120)=>typeof x==='string'&&x.trim().length>0&&x.length<=n;
 const num=(x,min,max)=>typeof x==='number'&&Number.isFinite(x)&&x>=min&&x<=max;
 const choice=(x,a)=>a.includes(x);
 const points=(x,min=0)=>Array.isArray(x)&&x.length>=min&&x.length<=500&&x.every(w=>obj(w)&&str(w.name)&&num(w.x,0,1000)&&num(w.y,0,630));
 const vessel=v=>obj(v)&&str(v.name)&&num(v.draft,.1,30)&&num(v.safetyDepth,v.draft,100)&&num(v.capacity,1,10000)&&num(v.reserve,5,90);
 if(!obj(d)||d.format!=='OpenNavX-design-backup'||!choice(d.version,[1,2])||d.mock!==true)fail();
 if(d.version===1){if(!vessel({name:d.vessel,draft:d.draft,safetyDepth:d.safetyDepth,capacity:d.capacity,reserve:d.reserve})||!choice(d.theme,['day','dusk','night'])||!points(d.waypoints))fail();return;}
 if(!obj(d.sections)||Object.keys(d.sections).some(k=>!choice(k,['settings','vessel','sensors','routes']))||Object.values(d.sections).some(v=>typeof v!=='boolean')||!Object.values(d.sections).some(Boolean))fail();
 for(const key of ['settings','vessel','sensors','routes'])if(!!d.sections[key]!==Object.hasOwn(d,key))fail();
 if(d.vessel&&!vessel(d.vessel))fail();
 if(d.settings){const s=d.settings,a=s.alarm,p=s.preferences;if(!obj(s)||!choice(s.theme,['day','dusk','night'])||!choice(s.units,['Nautical','Metric'])||!obj(a)||!num(a.cpa,.1,20)||!num(a.tcpa,1,120)||!num(a.depth,.1,100)||!num(a.energy,5,90)||['sound','gps','anchor'].some(k=>typeof a[k]!=='boolean')||!obj(p)||!choice(p.route,['Coastal','Shortest passage','Avoid restricted areas'])||!num(p.corridor,.05,2)||!choice(p.orientation,['North up','Course up','Head up']))fail();}
 if(d.sensors){const s=d.sensors;if(!obj(s)||!choice(s.adapter,['Auto detect','Generic NMEA heading adapter','Signal K autopilot adapter'])||!Array.isArray(s.profiles)||s.profiles.length>100)fail();const ids=new Set();for(const p of s.profiles){if(!obj(p)||!str(p.id)||!/^[a-z0-9-]+$/.test(p.id)||ids.has(p.id)||!str(p.name)||!str(p.endpoint,240)||!choice(p.signal,['GPS','Heading','Depth','Wind','Motor','Battery','Tanks','AIS','Rudder','Water temperature'])||!choice(p.transport,['NMEA 2000','NMEA 0183','Signal K'])||!choice(p.priority,['Primary','Secondary','Fallback'])||!choice(p.status,['Current','Aging','Stale','Invalid','Unavailable','Disconnected'])||!str(p.age,30)||!str(p.rate,30)||!str(p.pgn,80)||!num(p.offset,-100,100))fail();ids.add(p.id);}}
 if(d.routes){const r=d.routes;if(!obj(r)||!str(r.name)||!str(r.destination)||!points(r.waypoints)||!points(r.passage,1))fail();}
}

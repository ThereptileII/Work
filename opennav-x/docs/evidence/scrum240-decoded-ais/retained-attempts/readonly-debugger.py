import gdb,json,time
result={'observed_unix':time.time(),'targets':[],'globals':{}}
for k in ('g_bInlandEcdis','g_bDrawAISRealtime','g_bShowScaled','g_bAIS_CPA_Alert'):
 result['globals'][k]=bool(gdb.parse_and_eval(k))
m=gdb.parse_and_eval('g_pAIS->AISTargetList')
v=gdb.default_visualizer(m)
if v is None:raise RuntimeError('No standard library map reader')
children=list(v.children())
for index in range(0,len(children),2):
 key=int(children[index][1]);sp=children[index+1][1];t=sp['_M_ptr'].dereference()
 row={'mmsi':key}
 for k in ('Class','NavStatus','ShipType','n_alert_state','PositionReportTicks','HDG','COG','SOG','Lat','Lon','blue_paddle'):
  row[k]=float(t[k]) if k in ('HDG','COG','SOG','Lat','Lon') else int(t[k])
 for k in ('b_active','b_lost','b_positionOnceValid','b_positionDoubtful','b_nameValid','b_nameFromCache','b_isEuroInland','b_SarAircraftPosnReport','b_isFollower'):
  row[k]=bool(t[k])
 row['name']=t['ShipName'].string().rstrip('@ ')
 row['age_seconds']=result['observed_unix']-row['PositionReportTicks']
 result['targets'].append(row)
print('AIS240_MODEL_JSON='+json.dumps(result,sort_keys=True))

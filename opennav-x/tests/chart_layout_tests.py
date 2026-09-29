"""Pure geometry contracts; these do not claim native rendering acceptance."""
import copy
import importlib.util
from pathlib import Path

spec = importlib.util.spec_from_file_location('chartcheck', Path(__file__).resolve().parents[1] / 'tools/chart-render-check.py')
chartcheck = importlib.util.module_from_spec(spec)
spec.loader.exec_module(chartcheck)


def rect(x, y, width, height, label=None):
    value = dict(x=x, y=y, width=width, height=height)
    if label:
        value.update(label=label, visible=True, enabled=True)
    return value


frame = rect(0, 0, 1280, 800)
client = rect(8, 31, 1264, 761)
def composition(client):
    x,y,w,h = [client[k] for k in ('x','y','width','height')]
    right,bottom = x+w-186,y+h-166
    tx,ty = right-211,bottom-89
    return {
        'chart_region': rect(x+80,y+68,w-266,h-234),
        'footer_region': rect(x,y+h-34,w,34),
        'footer_middle_visible': True,
        'rail_regions': [rect(right,y+110+i*120,186,120,name)
                         for i,name in enumerate(('sog','depth','aws','heading'))],
        'interaction_controls': [rect(x+9,y+82,62,61,'Chart'),
             rect(x+9,y+h-34-61,62,61,'Settings'),
             rect(tx+4,ty+4,44,44,'Measure'),rect(tx+48,ty+4,44,44,'Waypoint'),
             rect(tx+97,ty+4,44,44,'+'),rect(tx+141,ty+4,44,44,'−'),
             rect(right-90,y+90,68,90,'North'),
             rect(x+108,bottom-81,142,44,'Follow boat'),
             rect(x+w-180,y+h-24,164,14,'Source health')],
    }

native = composition(client)
assert chartcheck.navigation_layout(native, frame, client)['primary_rail_values_visible'] == 4
linux = composition(frame)
chartcheck.navigation_layout(linux, frame, frame)
primary_client = rect(8,31,1280,800)
chartcheck.navigation_layout(composition(primary_client),rect(0,0,1296,839),primary_client)


def reject(display, outer=frame, inner=client):
    try:
        chartcheck.navigation_layout(display, outer, inner)
    except AssertionError:
        return
    raise AssertionError('Invalid startup geometry was accepted')


# Actual c8ff99ea native failure: desktop 1280x800, application still 896x532.
reject(native, rect(0, 0, 896, 532), rect(8, 31, 880, 493))
small = copy.deepcopy(native)
small['chart_region'] = rect(65, 88, 686, 379)
reject(small)  # An outer resize alone is insufficient; wx layout must catch up.
for change in ('chart-outside', 'rail-hidden', 'rail-clipped', 'rail-overlap',
               'rail-missing', 'control-chart', 'control-clipped', 'settings-missing',
               'float-missing', 'float-duplicate', 'float-moved', 'float-oversize',
               'rail-duplicate', 'rail-outside-strip', 'orientation-missing', 'old-layout',
               'footer-missing', 'footer-clipped', 'footer-inset', 'footer-tall',
               'footer-middle-hidden', 'health-missing', 'health-hidden', 'health-disabled',
               'health-outside', 'health-duplicate', 'old-system'):
    bad = copy.deepcopy(native)
    if change == 'chart-outside': bad['chart_region']['x'] = 300
    if change == 'rail-hidden': bad['rail_regions'][3]['visible'] = False
    if change == 'rail-clipped': bad['rail_regions'][3]['y'] = 750
    if change == 'rail-overlap': bad['rail_regions'][3]['y'] = bad['rail_regions'][2]['y']
    if change == 'rail-missing': bad['rail_regions'].pop()
    if change == 'control-chart': bad['interaction_controls'][0]['x'] = 300
    if change == 'control-clipped': bad['interaction_controls'][1]['x'] = 1250
    if change == 'settings-missing': bad['interaction_controls'].pop(1)
    if change == 'float-missing': bad['interaction_controls'].pop(2)
    if change == 'float-duplicate': bad['interaction_controls'].append(copy.deepcopy(bad['interaction_controls'][2]))
    if change == 'float-moved': bad['interaction_controls'][2]['x'] -= 10
    if change == 'float-oversize': bad['interaction_controls'][2]['width'] += 10
    if change == 'rail-duplicate': bad['rail_regions'][3]['label'] = 'sog'
    if change == 'rail-outside-strip': bad['rail_regions'][3]['x'] -= 10
    if change == 'orientation-missing': bad['interaction_controls'].pop(6)
    if change == 'old-layout': bad['chart_region'] = rect(65,88,1070,647)
    if change == 'footer-missing': bad.pop('footer_region')
    if change == 'footer-clipped': bad['footer_region']['y'] += 2
    if change == 'footer-inset': bad['footer_region']['x'] += 2; bad['footer_region']['width'] -= 2
    if change == 'footer-tall': bad['footer_region']['y'] -= 2; bad['footer_region']['height'] += 2
    if change == 'footer-middle-hidden': bad['footer_middle_visible'] = False
    if change == 'health-missing': bad['interaction_controls'].pop()
    if change == 'health-hidden': bad['interaction_controls'][-1]['visible'] = False
    if change == 'health-disabled': bad['interaction_controls'][-1]['enabled'] = False
    if change == 'health-outside': bad['interaction_controls'][-1]['y'] -= 20
    if change == 'health-duplicate': bad['interaction_controls'].append(copy.deepcopy(bad['interaction_controls'][-1]))
    if change == 'old-system': bad['interaction_controls'].append(rect(1170,758,90,32,'System'))
    reject(bad)
print('Prototype native-client/outer and Linux composition pass; 29 small/unsettled/clipped/overlapping/obsolete layouts rejected')

# Independent exact ink fixtures: failure must not relearn a water-only canvas
# or accept a Standard palette while XNav is requested (or the reverse).
palettes={
    ('XNav','Day'):('eeeee2','d5e5e5'),
    ('XNav','Dusk'):('4e615d','344f59'),
    ('XNav','Night'):('25342f','121e24'),
    ('Standard','Day'):('aaaf50','aac3f0'),
    ('Standard','Dusk'):('555728','556178'),
    ('Standard','Night'):('2a2b14','2a303c'),
}
color_checks=0
for (style,light),(land,water) in palettes.items():
    row=bytes.fromhex(land)*640+bytes.fromhex(water)*640
    image=row*800
    assert chartcheck.presentation(image,style,light,'unit fixture')['coastline_visible']
    color_checks+=1
    for bad in (bytes(1280*800*3), b'\xff'*(1280*800*3),
                bytes.fromhex(water)*1280*800, bytes.fromhex(land)*1280*800,
                image.replace(bytes.fromhex(land),bytes.fromhex('ff00ff'))):
        try:chartcheck.presentation(bad,style,light,'invalid fixture')
        except AssertionError:pass
        else:raise AssertionError('Missing coastline or incorrect ink was accepted')
        color_checks+=1
    try:chartcheck.presentation(image,'Standard' if style=='XNav' else 'XNav',light,'wrong style')
    except AssertionError:pass
    else:raise AssertionError('Opposite chart style was accepted')
    color_checks+=1
    for wrong_light in {'Day','Dusk','Night'}-{light}:
        try:chartcheck.presentation(image,style,wrong_light,'wrong light')
        except AssertionError:pass
        else:raise AssertionError('Wrong Day/Dusk/Night chart palette was accepted')
        color_checks+=1
print(f'{color_checks} exact Day/Dusk/Night XNav/Standard coastline checks passed')

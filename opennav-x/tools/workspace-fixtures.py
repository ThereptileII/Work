"""Actual Dashboard AUI persistence fixture; only marked disconnected profiles."""
import configparser

NAME = 'OpenNavTestDashboard'
CAPTION = 'SIMULATED workspace persistence'


def seed(profile):
    assert (profile / 'OPENNAV_TEST_PROFILE').is_file()
    chart = ('name=ChartCanvas;caption=;state=768;dir=5;layer=0;row=0;pos=0;'
             'prop=100000;bestw=5;besth=5;minw=256;minh=800;maxw=-1;maxh=-1;'
             'floatx=-1;floaty=-1;floatw=-1;floath=-1')
    pane = (f'name={NAME};caption={CAPTION};state=2098125;dir=4;layer=0;row=0;'
            'pos=7;prop=73121;bestw=220;besth=220;minw=150;minh=80;maxw=-1;maxh=-1;'
            'floatx=820;floaty=120;floatw=220;floath=220')
    with (profile / 'opencpn.conf').open('a', encoding='utf-8') as stream:
        stream.write('\n[PlugIns/Dashboard]\nVersion=2\nDashboardCount=1\n'
                     f'[PlugIns/Dashboard/Dashboard1]\nName={NAME}\nCaption={CAPTION}\n'
                     'Orientation=V\nPersistence=1\nInstrumentCount=1\nInstrument1=1\n'
                     'BestSizeX=220\nBestSizeY=220\nPersistSizeX=220\nPersistSizeY=220\n'
                     f'[AUI]\nAUIPerspective=layout2|{chart}|{pane}|dock_size(5,0,0)=1280|\n')


def pane(perspective):
    # These fixture names contain no escaped delimiters; unknown real profiles
    # are never supplied to this helper.
    rows = [dict(field.split('=', 1) for field in row.split(';'))
            for row in perspective.split('|') if row.startswith('name=')]
    matches = [row for row in rows if row['name'] == NAME]
    assert len(matches) == 1, ('Dashboard pane missing or duplicated', matches)
    return matches[0]


def assert_restored(perspective):
    found = pane(perspective)
    assert found['caption'] == CAPTION, found
    assert int(found['state']) & 3 == 1, ('Expected visible floating Dashboard', found)
    for key, value in {'dir': '4', 'layer': '0', 'row': '0', 'pos': '7',
                       'prop': '73121', 'floatx': '820', 'floaty': '120',
                       'floatw': '220', 'floath': '220'}.items():
        assert found[key] == value, (key, value, found)
    return found


def saved(profile):
    config = configparser.RawConfigParser(strict=False)
    config.read(profile / 'opencpn.conf', encoding='utf-8-sig')
    perspective = config.get('AUI', 'AUIPerspective')
    names = [row.split(';', 1)[0][5:] for row in perspective.split('|')
             if row.startswith('name=')]
    assert not any(name in names for name in ('OpenNavTop', 'OpenNavTools',
                   'OpenNavData', 'OpenNavBottom', 'OpenNavPage', 'OpenNavProduct')), names
    return assert_restored(perspective)

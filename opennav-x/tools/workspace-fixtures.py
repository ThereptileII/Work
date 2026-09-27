"""Actual Dashboard AUI persistence fixture; only marked disconnected profiles."""
import configparser

NAME = 'OpenNavTestDashboard'
CAPTION = 'SIMULATED workspace persistence'
HIDDEN_NAME = 'OpenNavTestHiddenDashboard'


def seed(profile):
    assert (profile / 'OPENNAV_TEST_PROFILE').is_file()
    chart = ('name=ChartCanvas;caption=;state=768;dir=5;layer=0;row=0;pos=0;'
             'prop=100000;bestw=5;besth=5;minw=256;minh=800;maxw=-1;maxh=-1;'
             'floatx=-1;floaty=-1;floatw=-1;floath=-1')
    pane = (f'name={NAME};caption={CAPTION};state=2098125;dir=4;layer=0;row=0;'
            'pos=7;prop=73121;bestw=220;besth=220;minw=150;minh=80;maxw=-1;maxh=-1;'
            'floatx=820;floaty=120;floatw=220;floath=220')
    hidden = (f'name={HIDDEN_NAME};caption=SIMULATED hidden instruments;state=2098127;dir=4;layer=0;row=0;'
              'pos=8;prop=43210;bestw=180;besth=180;minw=150;minh=80;maxw=-1;maxh=-1;'
              'floatx=940;floaty=180;floatw=180;floath=180')
    with (profile / 'opencpn.conf').open('a', encoding='utf-8') as stream:
        stream.write('\n[PlugIns/Dashboard]\nVersion=2\nDashboardCount=2\n'
                     f'[PlugIns/Dashboard/Dashboard1]\nName={NAME}\nCaption={CAPTION}\n'
                     'Orientation=V\nPersistence=1\nInstrumentCount=1\nInstrument1=1\n'
                     'BestSizeX=220\nBestSizeY=220\nPersistSizeX=220\nPersistSizeY=220\n'
                     f'[PlugIns/Dashboard/Dashboard2]\nName={HIDDEN_NAME}\nCaption=SIMULATED hidden instruments\n'
                     'Orientation=V\nPersistence=0\nInstrumentCount=1\nInstrument1=1\n'
                     'BestSizeX=180\nBestSizeY=180\nPersistSizeX=180\nPersistSizeY=180\n'
                     f'[AUI]\nAUIPerspective=layout2|{chart}|{pane}|{hidden}|dock_size(5,0,0)=1280|\n')


def pane(perspective, name=NAME):
    # These fixture names contain no escaped delimiters; unknown real profiles
    # are never supplied to this helper.
    rows = [dict(field.split('=', 1) for field in row.split(';'))
            for row in perspective.split('|') if row.startswith('name=')]
    matches = [row for row in rows if row['name'] == name]
    assert len(matches) == 1, ('Dashboard pane missing or duplicated', matches)
    return matches[0]


def assert_restored(perspective, suppressed=False):
    found = pane(perspective)
    assert found['caption'] == CAPTION, found
    assert int(found['state']) & 3 == (3 if suppressed else 1), ('Expected temporary XNav hiding or restored Legacy visibility', found)
    for key, value in {'dir': '4', 'layer': '0', 'row': '0', 'pos': '7',
                       'prop': '73121', 'floatx': '820', 'floaty': '120',
                       'floatw': '220', 'floath': '220'}.items():
        assert found[key] == value, (key, value, found)
    hidden = pane(perspective, HIDDEN_NAME)
    assert int(hidden['state']) & 3 == 3, ('Originally hidden pane became visible', hidden)
    for key, value in {'prop': '43210', 'floatx': '940', 'floaty': '180', 'floatw': '180', 'floath': '180'}.items():
        assert hidden[key] == value, (key, value, hidden)
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

"""Synthetic, disconnected fixtures for mode persistence; never a live profile."""
from pathlib import Path
from contextlib import closing
import configparser
import shutil
import sqlite3
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
NS = {'g': 'http://www.topografix.com/GPX/1/1', 'o': 'http://www.opencpn.org'}
CONNECTION = '1;0;127.0.0.1;10110;0;;4800;1;0;0;;0;;0;0;0;0;0;SIMULATED disabled input;0;;0;1;'
PLUGIN = 'dashboard_pi.dll' if sys.platform == 'win32' else 'libdashboard_pi.so'

def seed(profile):
    assert (profile / 'OPENNAV_TEST_PROFILE').is_file(), 'Only disposable test profiles are allowed'
    assert not (profile / 'navobj.xml').exists() and not (profile / 'navobj.db').exists(), 'Never replace existing navigation data'
    shutil.copyfile(ROOT / 'tests/fixtures/mode-persistence.gpx', profile / 'navobj.xml')
    with (profile / 'opencpn.conf').open('a') as stream:
        stream.write('\n[Settings/NMEADataSource]\nDataConnections=' + CONNECTION + '\n')
        stream.write('[Settings/AIS]\nbCPAWarn=1\nCPAWarnNMi=0.75\n')
        stream.write(f'[PlugIns/{PLUGIN}]\nbEnabled=1\n')
        # Exercise DLL initialization without opening a separate instrument pane
        # over the canonical XNav layout screenshots.
        stream.write('[PlugIns/Dashboard]\nVersion=2\nDashboardCount=0\n')

def snapshot(profile):
    assert (profile / 'OPENNAV_TEST_PROFILE').is_file()
    config = configparser.RawConfigParser(strict=False)
    config.read(profile / 'opencpn.conf', encoding='utf-8-sig')
    result = {}
    nav = ET.parse(profile / 'navobj.xml').getroot() if not (profile / 'navobj.db').exists() else None
    for tag in (() if nav is None else ('wpt', 'rte', 'trk')):
        objects = []
        for item in nav.findall('g:' + tag, NS):
            name = item.findtext('g:name', namespaces=NS)
            if not name or not name.startswith('SIMULATED persistence'):
                continue
            points = [item] if tag == 'wpt' else item.findall('.//g:rtept' if tag == 'rte' else './/g:trkpt', NS)
            objects.append({'name': name, 'guid': item.findtext('g:extensions/o:guid', namespaces=NS),
                            'points': [(float(p.attrib['lat']), float(p.attrib['lon']), p.findtext('g:time', namespaces=NS)) for p in points]})
        assert len(objects) == 1, f'{tag} fixture missing or duplicated: {objects}'
        result[tag] = objects
    if nav is None:
        # 5.12.4 imports legacy GPX and removes navobj.xml; validate the actual
        # SQLite store, not an untouched backup file. Never write to this DB.
        with closing(sqlite3.connect((profile / 'navobj.db').resolve().as_uri() + '?mode=ro', uri=True)) as db:
            assert db.execute('PRAGMA integrity_check').fetchone() == ('ok',)
            assert not db.execute('PRAGMA foreign_key_check').fetchall()
            for tag, table in [('wpt', 'routepoints'), ('rte', 'routes'), ('trk', 'tracks')]:
                rows = db.execute(f"SELECT guid, name FROM {table} WHERE name LIKE 'SIMULATED persistence%' ORDER BY guid").fetchall()
                assert len(rows) == 1, f'{tag} fixture missing or duplicated in navobj.db'
                guid, name = rows[0]
                if tag == 'wpt':
                    points = db.execute('SELECT lat, lon, Time FROM routepoints WHERE guid=?', (guid,)).fetchall()
                elif tag == 'rte':
                    points = db.execute('SELECT p.lat, p.lon, p.Time FROM routepoints p JOIN routepoints_link l ON p.guid=l.point_guid WHERE l.route_guid=? ORDER BY l.point_order', (guid,)).fetchall()
                else:
                    points = db.execute('SELECT latitude, longitude, timestamp FROM trk_points WHERE track_guid=? ORDER BY point_order', (guid,)).fetchall()
                result[tag] = [{'name': name, 'guid': guid, 'points': points}]
    serialized = config.get('Settings/NMEADataSource', 'DataConnections')
    matches = [s.split(';') for s in serialized.split('|') if 'SIMULATED disabled input' in s]
    assert len(matches) == 1, 'Connection fixture was removed or duplicated'
    connection = matches[0]
    assert connection[17] == '0', 'Test connection became enabled'
    result['connection'] = connection
    result['ais_cpa_warn'] = config.getboolean('Settings/AIS', 'bCPAWarn')
    result['ais_cpa_nm'] = config.getfloat('Settings/AIS', 'CPAWarnNMi')
    result['dashboard_enabled'] = config.getboolean('PlugIns/' + PLUGIN, 'bEnabled')
    assert result['dashboard_enabled'], 'Safe Mode changed the normal plugin preference'
    assert result['ais_cpa_warn'] and result['ais_cpa_nm'] == .75
    return result

"""Synthetic, disconnected fixtures for mode persistence; never a live profile."""
from pathlib import Path
import configparser
import shutil
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
NS = {'g': 'http://www.topografix.com/GPX/1/1', 'o': 'http://www.opencpn.org'}
CONNECTION = '1;0;127.0.0.1;10110;0;;4800;1;0;0;;0;;0;0;0;0;0;SIMULATED disabled input;0;;0;1;'

def seed(profile):
    assert (profile / 'OPENNAV_TEST_PROFILE').is_file(), 'Only disposable test profiles are allowed'
    assert not (profile / 'navobj.xml').exists(), 'Never replace existing navigation data'
    shutil.copyfile(ROOT / 'tests/fixtures/mode-persistence.gpx', profile / 'navobj.xml')
    with (profile / 'opencpn.conf').open('a') as stream:
        stream.write('\n[Settings/NMEADataSource]\nDataConnections=' + CONNECTION + '\n')
        stream.write('[Settings/AIS]\nbCPAWarn=1\nCPAWarnNMi=0.75\n')

def snapshot(profile):
    assert (profile / 'OPENNAV_TEST_PROFILE').is_file()
    config = configparser.RawConfigParser(strict=False)
    config.read(profile / 'opencpn.conf', encoding='utf-8-sig')
    nav = ET.parse(profile / 'navobj.xml').getroot()
    result = {}
    for tag in ('wpt', 'rte', 'trk'):
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
    serialized = config.get('Settings/NMEADataSource', 'DataConnections')
    matches = [s.split(';') for s in serialized.split('|') if 'SIMULATED disabled input' in s]
    assert len(matches) == 1, 'Connection fixture was removed or duplicated'
    connection = matches[0]
    assert connection[17] == '0', 'Test connection became enabled'
    result['connection'] = connection
    result['ais_cpa_warn'] = config.getboolean('Settings/AIS', 'bCPAWarn')
    result['ais_cpa_nm'] = config.getfloat('Settings/AIS', 'CPAWarnNMi')
    assert result['ais_cpa_warn'] and result['ais_cpa_nm'] == .75
    return result

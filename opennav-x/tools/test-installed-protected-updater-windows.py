#!/usr/bin/env python3
"""One explicit installed protected-updater phase on a disposable Windows host.

No enrollment, publication, network changes, profile writes, forced termination,
or transaction retry. Independently authenticate the fixture manifest first.
Broker diagnostics are observations, never launch or mutation authorization.
"""
import argparse
import configparser
from contextlib import closing
import ctypes
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import secrets
import socket
import ssl
import sqlite3
import stat
import subprocess
import sys
import time
import urllib.error
import urllib.request
import uuid
import zipfile
import xml.etree.ElementTree as ET

import updater_package as updater
from product_version import windows_product_version
from hardware_output_policy import require_product_output_policy

OPT_IN = 'SCRUM-311-disposable-installed-protected-updater'
PHASES = ('no-update', 'later', 'now', 'offline')
PROGRAM_DATA = '62ab5d82-fdc1-4dc3-a9dd-070d1d495d97'
NAV_LIMIT = 128 << 20
NAV_ROWS = 250000
NAV_TABLES = ('routepoints', 'routes', 'routepoints_link', 'tracks', 'trk_points',
              'track_html_links', 'route_html_links', 'routepoint_html_links')
# OpenCPN 37fd0cddb7334fe489e9f18aa163977a9c5c84f7 model/src/navobj_db.cpp:
# CreateTables (67-174), route links (1185/1262), track ordering (809-812/988),
# point Time binding (1458). Full table shape prevents accepting a reduced DB
# which the actual LoadAllRoutes/LoadAllTracks readers cannot load.
NAV_SCHEMA = {'routepoints': [('guid', 'TEXT', 1, None, 1),
                 ('lat', 'REAL', 0, None, 0),
                 ('lon', 'REAL', 0, None, 0),
                 ('Symbol', 'TEXT', 0, None, 0),
                 ('Name', 'TEXT', 0, None, 0),
                 ('description', 'TEXT', 0, None, 0),
                 ('TideStation', 'TEXT', 0, None, 0),
                 ('plan_speed', 'REAL', 0, None, 0),
                 ('etd', 'INTEGER', 0, None, 0),
                 ('Type', 'TEXT', 0, None, 0),
                 ('Time', 'TEXT', 0, None, 0),
                 ('ArrivalRadius', 'REAL', 0, None, 0),
                 ('RangeRingsNumber', 'INTEGER', 0, None, 0),
                 ('RangeRingsStep', 'REAL', 0, None, 0),
                 ('RangeRingsStepUnits', 'INTEGER', 0, None, 0),
                 ('RangeRingsVisible', 'INTEGER', 0, None, 0),
                 ('RangeRingsColour', 'TEXT', 0, None, 0),
                 ('ScaleMin', 'INTEGER', 0, None, 0),
                 ('ScaleMax', 'INTEGER', 0, None, 0),
                 ('UseScale', 'INTEGER', 0, None, 0),
                 ('visibility', 'INTEGER', 0, None, 0),
                 ('viz_name', 'INTEGER', 0, None, 0),
                 ('shared', 'INTEGER', 0, None, 0),
                 ('isolated', 'INTEGER', 0, None, 0),
                 ('created_at', 'DATETIME', 0, 'CURRENT_TIMESTAMP', 0)],
 'routes': [('guid', 'TEXT', 1, None, 1),
            ('name', 'TEXT', 0, None, 0),
            ('start_string', 'TEXT', 0, None, 0),
            ('end_string', 'TEXT', 0, None, 0),
            ('description', 'TEXT', 0, None, 0),
            ('planned_departure', 'TEXT', 0, None, 0),
            ('plan_speed', 'REAL', 0, None, 0),
            ('time_format', 'TEXT', 0, None, 0),
            ('style', 'INTEGER', 0, None, 0),
            ('width', 'INTEGER', 0, None, 0),
            ('color', 'TEXT', 0, None, 0),
            ('visibility', 'INTEGER', 0, None, 0),
            ('shared_wp_viz', 'INTEGER', 0, None, 0),
            ('created_at', 'DATETIME', 0, 'CURRENT_TIMESTAMP', 0)],
 'routepoints_link': [('route_guid', 'TEXT', 0, None, 1),
                      ('point_guid', 'TEXT', 0, None, 0),
                      ('point_order', 'INTEGER', 0, None, 2)],
 'tracks': [('guid', 'TEXT', 1, None, 1),
            ('name', 'TEXT', 0, None, 0),
            ('description', 'TEXT', 0, None, 0),
            ('visibility', 'INTEGER', 0, None, 0),
            ('start_string', 'TEXT', 0, None, 0),
            ('end_string', 'TEXT', 0, None, 0),
            ('width', 'INTEGER', 0, None, 0),
            ('style', 'INTEGER', 0, None, 0),
            ('color', 'TEXT', 0, None, 0),
            ('created_at', 'DATETIME', 0, 'CURRENT_TIMESTAMP', 0)],
 'trk_points': [('track_guid', 'TEXT', 1, None, 0),
                ('latitude', 'REAL', 1, None, 0),
                ('longitude', 'REAL', 1, None, 0),
                ('timestamp', 'TEXT', 1, None, 0),
                ('point_order', 'INTEGER', 0, None, 0)],
 'track_html_links': [('guid', 'TEXT', 0, None, 1),
                      ('track_guid', 'TEXT', 1, None, 0),
                      ('html_link', 'TEXT', 0, None, 0),
                      ('html_description', 'TEXT', 0, None, 0),
                      ('html_type', 'TEXT', 0, None, 0)],
 'route_html_links': [('guid', 'TEXT', 0, None, 1),
                      ('route_guid', 'TEXT', 1, None, 0),
                      ('html_link', 'TEXT', 0, None, 0),
                      ('html_description', 'TEXT', 0, None, 0),
                      ('html_type', 'TEXT', 0, None, 0)],
 'routepoint_html_links': [('guid', 'TEXT', 0, None, 1),
                           ('routepoint_guid', 'TEXT', 1, None, 0),
                           ('html_link', 'TEXT', 0, None, 0),
                           ('html_description', 'TEXT', 0, None, 0),
                           ('html_type', 'TEXT', 0, None, 0)]}
NAV_FOREIGN_KEYS = {'routepoints': [],
 'routes': [],
 'routepoints_link': [('routes', 'route_guid', 'guid', 'NO ACTION', 'CASCADE', 'NONE')],
 'tracks': [],
 'trk_points': [('tracks', 'track_guid', 'guid', 'NO ACTION', 'CASCADE', 'NONE')],
 'track_html_links': [('tracks', 'track_guid', 'guid', 'NO ACTION', 'CASCADE', 'NONE')],
 'route_html_links': [('routes', 'route_guid', 'guid', 'NO ACTION', 'CASCADE', 'NONE')],
 'routepoint_html_links': [('routepoints', 'routepoint_guid', 'guid', 'NO ACTION', 'CASCADE', 'NONE')]}
CONTROLLED_XML_SHA256 = '24be1789c06aa08cbca0cd470ece1fc664b8e5f74fac8bdf27b403314e5021dc'
CONTROLLED_XML_CRLF_SHA256 = '90915745d6dae09851a6cdd5e87086b9f0152e52b6d959427b8f372bedff3627'
CONTROLLED_NAV_SHA256 = '7f4fbafa24803022f4503465672d103f1492244aad5d0818bb98cb7792e31082'
HEX = {n: re.compile('[0-9a-f]{' + str(n) + '}\\Z') for n in (32, 40, 64)}
PREFIX = 'SKAGER-Beta2-Portable-Recovery/'
FAILURE = 'Installed protected updater acceptance refused; preserve state and evidence. No automatic retry or cleanup.'


def require(condition):
    if not condition:
        raise ValueError(FAILURE)


def module(name):
    spec = importlib.util.spec_from_file_location('_installed_' + name.replace('-', '_'),
                                                 Path(__file__).with_name(name + '.py'))
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def strict(raw):
    return json.loads(raw.decode('utf-8'), object_pairs_hook=updater._unique,
                      parse_constant=lambda _: (_ for _ in ()).throw(ValueError(FAILURE)))


def keys(value, names):
    require(isinstance(value, dict) and set(value) == set(names))


def pin(value, size=64):
    require(isinstance(value, str) and HEX[size].fullmatch(value))
    return value


def stamp(path):
    value = path.stat()
    return (value.st_dev, value.st_ino, value.st_size, value.st_mtime_ns, value.st_ctime_ns, value.st_nlink)


def approved_file(record, limit):
    keys(record, ('path', 'sha256'))
    pin(record['sha256'])
    path = Path(record['path'])
    require(path.is_absolute() and os.path.normpath(str(path)) == str(path))
    updater._plain(path)
    before = stamp(path)
    require(path.is_file() and path.stat().st_nlink == 1 and 0 < before[2] <= limit)
    require(updater._hash(path) == record['sha256'] and stamp(path) == before)
    return path, before


def entries(archive, limit=20000):
    result, folded = {}, set()
    for item in archive.infolist():
        name = item.filename
        parts = PurePosixPath(name)
        require(len(result) < limit and item.orig_filename == name and name and len(name) <= 1024 and
                '\\' not in name and ':' not in name and not parts.is_absolute() and
                parts.as_posix() == name and all(p not in ('.', '..') and not p.endswith((' ', '.')) for p in parts.parts) and
                not item.is_dir() and not item.flag_bits & 1 and
                stat.S_IFMT(item.external_attr >> 16) in (0, stat.S_IFREG) and name.casefold() not in folded)
        result[name] = item
        folded.add(name.casefold())
    require(result)
    return result


def member(archive, index, name, limit):
    require(name in index and 0 < index[name].file_size <= limit)
    return archive.read(index[name])


def fixture(record, retained):
    keys(record, ('version', 'commit', 'recovery', 'source', 'policy'))
    windows_product_version(record['version'])
    pin(record['commit'], 40)
    paths = {}
    for name, limit in (('recovery', 4 << 30), ('source', 4 << 30), ('policy', 32 << 10)):
        paths[name], identity = approved_file(record[name], limit)
        retained.append((paths[name], identity, record[name]['sha256']))
    policy = strict(paths['policy'].read_bytes())
    require(policy['schema'] == 1 and policy['product'] == 'skager' and policy['channel'] == 'beta' and
            policy['version'] == record['version'] and policy['commit'] == record['commit'])
    for key, name, filename in (('recovery', 'recovery', 'SKAGER-Beta2-Portable-Recovery.zip'),
                                ('correspondingSource', 'source', 'SKAGER-Beta2-source.zip')):
        item = policy[key]
        require(item['path'] == 'artifacts/' + record['commit'] + '/' + filename and
                item['sha256'] == record[name]['sha256'] and item['bytes'] == paths[name].stat().st_size)
    with zipfile.ZipFile(paths['recovery']) as archive:
        index = entries(archive)
        require(all(name.startswith(PREFIX) for name in index))
        raw = member(archive, index, PREFIX + 'FILE_SHA256.json', 16 << 20)
        inventory = strict(raw)
        require(set(inventory) == {name[len(PREFIX):] for name in index if name != PREFIX + 'FILE_SHA256.json'})
        total = 0
        for name, digest in inventory.items():
            pin(digest)
            item = index[PREFIX + name]
            total += item.file_size
            require(total <= 4 << 30 and item.file_size <= 1 << 30)
            with archive.open(item) as stream:
                require(hashlib.file_digest(stream, 'sha256').hexdigest() == digest)
        product_raw = member(archive, index, PREFIX + 'docs/PRODUCT_BUILD.json', 32 << 10)
        product = strict(product_raw)
        require(product['version'] == record['version'] and product['commit'] == record['commit'] and
                product['test_fixtures'] is False and product['build_purpose'] == 'INSTALLED PRODUCT' and
                product['update_startup_health'] == 1 and product['commissioning_restart_protocol'] == 1)
        # Accept the existing product capability contract; this driver never
        # enables or exercises equipment outputs.
        require_product_output_policy(product)
        for field, name in (('executable_sha256', 'opencpn.exe'), ('startup_launcher_sha256', 'skager-start.exe'),
                            ('restart_helper_sha256', 'opennav-restart.exe')):
            require(product[field] == inventory['app/' + name])
        capability = strict(member(archive, index, PREFIX + 'app/update-qualification.json', 4096))
        require(capability == {'schema': 1, 'purpose': 'skager-qualified-update-v1', 'commit': record['commit'],
                'executableSha256': product['executable_sha256'], 'configurationProtocol': 1,
                'coreOutputSuppression': 1, 'pluginPolicy': 'deferred-until-ordinary-start'})
        trust_raw = member(archive, index, PREFIX + 'app/update-trust.json', 576 << 10)
        trust_sha = hashlib.sha256(trust_raw).hexdigest()
        require(product['update_trust']['configSha256'] == product['update_trust']['sourceSha256'] == trust_sha and
                product['update_trust']['productCommit'] == record['commit'])
        package_raw = member(archive, index, PREFIX + 'installer/package.json', 16 << 20)
        package = strict(package_raw)
        require(package['commit'] == record['commit'] and package['version'] == record['version'])
    with zipfile.ZipFile(paths['source']) as archive:
        # Source archives may intentionally contain links elsewhere. Read only
        # these exact regular members; never extract/dereference source links.
        index = {item.filename: item for item in archive.infolist()}
        for name in ('SOURCE_REFERENCE.json', 'opennav-x/src/application/Version.h',
                     'opennav-x/installer/windows/staging-update-trust.json'):
            matches = [item for item in archive.infolist() if item.filename.casefold() == name.casefold()]
            require(len(matches) == 1 and matches[0].filename == name and not matches[0].is_dir() and
                    stat.S_IFMT(matches[0].external_attr >> 16) in (0, stat.S_IFREG))
        reference = strict(member(archive, index, 'SOURCE_REFERENCE.json', 16 << 20))
        require(reference['productCommit'] == record['commit'])
        version_name = 'opennav-x/src/application/Version.h'
        version_raw = member(archive, index, version_name, 4096)
        require(hashlib.sha256(version_raw).hexdigest() == reference['files'][version_name]['sha256'])
        require(re.findall(rb'inline\s+constexpr\s+char\s+Version\[\]\s*=\s*"([^"\r\n]+)"\s*;', version_raw) == [record['version'].encode()])
        name = 'opennav-x/installer/windows/staging-update-trust.json'
        source_trust = member(archive, index, name, 576 << 10)
        require(source_trust == trust_raw and hashlib.sha256(source_trust).hexdigest() == reference['files'][name]['sha256'])
    return dict(record, paths=paths, inventory=inventory, product=product, trust=strict(trust_raw), trustSha256=trust_sha,
                packageSha256=hashlib.sha256(package_raw).hexdigest(), productSha256=hashlib.sha256(product_raw).hexdigest(),
                inventorySha256=hashlib.sha256(raw).hexdigest())


def prepare(path, digest):
    path, identity = approved_file({'path': str(path), 'sha256': digest}, 64 << 10)
    retained = [(path, identity, digest)]
    value = strict(path.read_bytes())
    keys(value, ('schema', 'b0', 'b1', 'operatorKit', 'clientRequest', 'trustSha256', 'preserveSettings'))
    require(type(value['schema']) is int and value['schema'] == 1)
    b0, b1 = fixture(value['b0'], retained), fixture(value['b1'], retained)
    require(windows_product_version(b1['version']) > windows_product_version(b0['version']) and b0['commit'] != b1['commit'])
    require(b0['trustSha256'] == b1['trustSha256'] == pin(value['trustSha256']))
    request_path, identity = approved_file(value['clientRequest'], 1 << 20)
    retained.append((request_path, identity, value['clientRequest']['sha256']))
    request = strict(request_path.read_bytes())
    keys(request, ('schema', 'purpose', 'clientSid', 'installed', 'stock', 'trustConfiguration', 'trustConfigSha256'))
    keys(request['stock'], ('path', 'version', 'arch', 'executableSha256', 'upstreamCommit'))
    require(type(request['schema']) is int and request['schema'] == 1 and request['purpose'] == 'client-registration' and
            request['trustConfigSha256'] == value['trustSha256'] and request['trustConfiguration'] == b0['trust'])
    installed = request['installed']
    keys(installed, ('version', 'commit', 'generation', 'packageSha256', 'productBuildSha256', 'executableSha256', 'inventorySha256'))
    require(installed['version'] == b0['version'] and installed['commit'] == b0['commit'] and
            installed['packageSha256'] == b0['packageSha256'] and installed['productBuildSha256'] == b0['productSha256'] and
            installed['executableSha256'] == b0['product']['executable_sha256'] and installed['inventorySha256'] == b0['inventorySha256'])
    pin(installed['generation'], 32)
    require(re.fullmatch(r'S-1-5-21-[1-9][0-9]*-[1-9][0-9]*-[1-9][0-9]*-[1-9][0-9]*', request['clientSid']))
    settings = value['preserveSettings']
    require(isinstance(settings, list) and 1 <= len(settings) <= 128)
    seen = set()
    for item in settings:
        keys(item, ('section', 'key'))
        require(all(isinstance(s, str) and 0 < len(s) <= 256 and not any(c in s for c in '\r\n\x00') for s in item.values()))
        pair = item['section'], item['key']
        require(pair not in seen)
        seen.add(pair)
    kit_path, identity = approved_file(value['operatorKit'], 768 << 20)
    retained.append((kit_path, identity, value['operatorKit']['sha256']))
    return dict(b0=b0, b1=b1, request=request, kit=kit_path, settings=settings, retained=retained)


def unchanged(prepared):
    for path, identity, digest in prepared['retained']:
        updater._plain(path)
        require(stamp(path) == identity and updater._hash(path) == digest and stamp(path) == identity)


def profile_preserved(before, after):
    """Compare stores; only the exact controlled fixture can cross XML→DB."""
    for observation in (before, after):
        keys(observation, ('navigationStore', 'navigationSha256', 'navigationDetailsSha256', 'navigationMigrationFixtureSha256', 'settingsSha256'))
        require(observation['navigationStore'] in ('xml', 'sqlite') and isinstance(observation['settingsSha256'], list) and
                1 <= len(observation['settingsSha256']) <= 128)
        for digest in (observation['navigationSha256'], observation['navigationDetailsSha256'], *observation['settingsSha256']):
            pin(digest)
        require(observation['navigationMigrationFixtureSha256'] in ('', CONTROLLED_XML_SHA256))
    if before['navigationStore'] == 'sqlite' and after['navigationStore'] != 'sqlite':
        return False
    if before['navigationSha256'] != after['navigationSha256'] or before['settingsSha256'] != after['settingsSha256']:
        return False
    if before['navigationStore'] != after['navigationStore']:
        return (before['navigationMigrationFixtureSha256'] == after['navigationMigrationFixtureSha256'] == CONTROLLED_XML_SHA256 and
                before['navigationSha256'] == CONTROLLED_NAV_SHA256)
    return before['navigationDetailsSha256'] == after['navigationDetailsSha256']


def _nav_text(value, required=False):
    if value is None and not required:
        return ''
    require(type(value) is str and len(value) <= 1 << 20 and '\x00' not in value and (not required or 0 < len(value) <= 256))
    return value


def _nav_time(value):
    value = _nav_text(value)
    if not value:
        return ''
    require(len(value) <= 64)
    parsed = datetime.fromisoformat(value.replace('Z', '+00:00'))
    require(parsed.tzinfo is not None)
    return parsed.astimezone(timezone.utc).isoformat()


def _nav_position(lat, lon, when):
    require(type(lat) in (str, int, float) and type(lon) in (str, int, float))
    lat, lon = float(lat), float(lon)
    require(math.isfinite(lat) and math.isfinite(lon) and -90 <= lat <= 90 and -180 <= lon <= 180)
    return [lat if lat else 0.0, lon if lon else 0.0, _nav_time(when)]


def _nav_digest(value):
    raw = json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
    require(len(raw) <= NAV_LIMIT)
    return hashlib.sha256(raw).hexdigest()


def _xml_navigation(raw):
    text = raw.decode('utf-8-sig')
    require('\x00' not in text and '<!DOCTYPE' not in text.upper() and '<!ENTITY' not in text.upper())
    root = ET.fromstring(text)
    g, o = '{http://www.topografix.com/GPX/1/1}', '{http://www.opencpn.org}'
    require(root.tag == g + 'gpx')
    points, routes, tracks = {}, {}, {}
    def identity(node):
        values = node.findall(g + 'extensions/' + o + 'guid')
        require(len(values) == 1)
        return _nav_text(values[0].text, True)
    def text(node, key):
        values = node.findall(g + key)
        require(len(values) <= 1)
        return _nav_text(values[0].text if values else None)
    def point(node):
        guid = identity(node)
        value = [text(node, 'name'), text(node, 'desc'), *_nav_position(node.attrib['lat'], node.attrib['lon'], text(node, 'time'))]
        require(guid not in points or points[guid] == value)
        points[guid] = value
        return guid
    for node in root.findall(g + 'wpt'):
        point(node)
    for node in root.findall(g + 'rte'):
        guid = identity(node)
        require(guid not in routes)
        routes[guid] = [text(node, 'name'), text(node, 'desc'), [point(p) for p in node.findall(g + 'rtept')]]
    for node in root.findall(g + 'trk'):
        guid = identity(node)
        require(guid not in tracks)
        # The supported DB has no segment-boundary column. Refuse a projection
        # that could silently hide loss of a gap between original GPX segments.
        require(len(node.findall(g + 'trkseg')) <= 1)
        ordered = [_nav_position(p.attrib['lat'], p.attrib['lon'], text(p, 'time'))
                   for p in node.findall(g + 'trkseg/' + g + 'trkpt')]
        tracks[guid] = [text(node, 'name'), text(node, 'desc'), ordered]
    require(len(points) + len(routes) + len(tracks) + sum(len(t[2]) for t in tracks.values()) <= NAV_ROWS)
    return {'points': points, 'routes': routes, 'tracks': tracks}


def _sqlite_navigation(path):
    # Immutable prevents even journal/SHM creation. Refuse every sidecar so it
    # cannot ignore committed WAL content or observe an in-flight transaction.
    for suffix in ('-wal', '-shm', '-journal'):
        sidecar = path.with_name(path.name + suffix)
        require(not sidecar.exists() and not sidecar.is_symlink())
    deadline = time.monotonic() + 10
    with closing(sqlite3.connect(path.as_uri() + '?mode=ro&immutable=1', uri=True, timeout=0)) as db:
        db.set_progress_handler(lambda: int(time.monotonic() >= deadline), 1000)
        db.execute('PRAGMA query_only=ON')
        db.execute('PRAGMA trusted_schema=OFF')
        require(db.execute('PRAGMA integrity_check').fetchall() == [('ok',)])
        require(not db.execute('PRAGMA foreign_key_check').fetchall())
        schema = db.execute("SELECT type,name,sql FROM sqlite_schema WHERE name NOT GLOB 'sqlite_*'").fetchall()
        require({row[1] for row in schema} == set(NAV_TABLES) and
                all(row[0] == 'table' and row[2].lstrip().upper().startswith('CREATE TABLE') for row in schema))
        tables, details, total = {}, {}, 0
        for table in NAV_TABLES:  # Fixed supported names; no input-derived SQL.
            require([tuple(row[1:]) for row in db.execute('PRAGMA table_info(' + table + ')')] == NAV_SCHEMA[table])
            require([tuple(row[2:]) for row in db.execute('PRAGMA foreign_key_list(' + table + ')')] == NAV_FOREIGN_KEYS[table])
            cursor = db.execute('SELECT * FROM ' + table)
            names = [column[0] for column in cursor.description]
            rows = cursor.fetchmany(NAV_ROWS + 1)
            total += len(rows)
            require(total <= NAV_ROWS)
            tables[table] = [dict(zip(names, row)) for row in rows]
            # Native loaders construct std::string directly from TEXT values;
            # declared nullable columns do not make a NULL payload loadable.
            text_columns = [column[0] for column in NAV_SCHEMA[table] if column[1] in ('TEXT', 'DATETIME')]
            for row in tables[table]:
                for column in text_columns:
                    require(type(row[column]) is str)
                    _nav_text(row[column])
            # created_at is DB bookkeeping, assigned during import. Retain all
            # other columns/links when comparing two observations of this DB.
            details[table] = sorted((json.dumps({k: v for k, v in row.items() if k != 'created_at'},
                                      sort_keys=True, allow_nan=False) for row in tables[table]))
    points, routes, tracks = {}, {}, {}
    for row in tables['routepoints']:
        guid = _nav_text(row['guid'], True)
        require(guid not in points)
        points[guid] = [_nav_text(row['Name']), _nav_text(row['description']), *_nav_position(row['lat'], row['lon'], row['Time'])]
    for table, result in (('routes', routes), ('tracks', tracks)):
        for row in tables[table]:
            guid = _nav_text(row['guid'], True)
            require(guid not in result)
            result[guid] = [_nav_text(row['name']), _nav_text(row['description']), []]
    links = {guid: [] for guid in routes}
    for row in tables['routepoints_link']:
        require(row['route_guid'] in routes and row['point_guid'] in points and type(row['point_order']) is int)
        links[row['route_guid']].append((row['point_order'], row['point_guid']))
    for guid, ordered in links.items():
        ordered.sort()
        require([p[0] for p in ordered] == list(range(1, len(ordered) + 1)))
        routes[guid][2] = [p[1] for p in ordered]
    track_points = {guid: [] for guid in tracks}
    for row in tables['trk_points']:
        require(row['track_guid'] in tracks and type(row['point_order']) is int)
        track_points[row['track_guid']].append((row['point_order'], _nav_position(row['latitude'], row['longitude'], row['timestamp'])))
    for guid, ordered in track_points.items():
        ordered.sort(key=lambda p: p[0])
        require([p[0] for p in ordered] == list(range(len(ordered))))
        tracks[guid][2] = [p[1] for p in ordered]
    semantic = {'points': points, 'routes': routes, 'tracks': tracks}
    # Only properties explicitly represented in the pinned controlled GPX are
    # migration evidence. Never discard arbitrary XML metadata/links to pass.
    fixture = (_nav_digest(semantic) == CONTROLLED_NAV_SHA256 and
               all(row['Symbol'] == 'diamond' for row in tables['routepoints']) and
               all(row['visibility'] == row['shared'] == 1 for row in tables['routepoints']
                   if row['guid'] == 'ae43f488-caa5-445b-88e3-0112875a3471') and
               all(row['visibility'] == 1 and row['plan_speed'] == 5 for row in tables['routes']) and
               all(row['visibility'] == 1 for row in tables['tracks']) and
               not any(tables[table] for table in ('track_html_links', 'route_html_links', 'routepoint_html_links')))
    return semantic, _nav_digest(details), CONTROLLED_XML_SHA256 if fixture else ''


def navigation_observation(root):
    """Read the active store, never a stale XML backup beside a current DB."""
    database = root / 'navobj.db'
    path = database if database.exists() or database.is_symlink() else root / 'navobj.xml'
    updater._plain(path)
    entry = path.stat()
    require(stat.S_ISREG(entry.st_mode) and entry.st_nlink == 1 and 0 < entry.st_size <= NAV_LIMIT)
    with path.open('rb') as held:
        identity = stamp(path)
        require(stat.S_ISREG(os.fstat(held.fileno()).st_mode) and path.stat().st_nlink == 1 and 0 < path.stat().st_size <= NAV_LIMIT)
        before = hashlib.file_digest(held, 'sha256').hexdigest()
        if path == database:
            semantic, details, migration = _sqlite_navigation(path)
            store = 'sqlite'
        else:
            held.seek(0)
            raw = held.read(NAV_LIMIT + 1)
            require(len(raw) <= NAV_LIMIT)
            semantic = _xml_navigation(raw)
            details = hashlib.sha256(ET.canonicalize(raw).encode()).hexdigest()
            # Windows checkout may convert this text fixture to CRLF. Accept
            # only these two exact known byte forms, never arbitrary XML edits.
            migration = CONTROLLED_XML_SHA256 if before in (CONTROLLED_XML_SHA256, CONTROLLED_XML_CRLF_SHA256) else ''
            store = 'xml'
        require(any(semantic.values()))  # Empty data cannot prove preservation.
        updater._plain(path)
        require(stamp(path) == identity and os.fstat(held.fileno()).st_ino == path.stat().st_ino)
        held.seek(0)
        require(hashlib.file_digest(held, 'sha256').hexdigest() == before and updater._hash(path) == before)
        if store == 'sqlite':
            for suffix in ('-wal', '-shm', '-journal'):
                sidecar = path.with_name(path.name + suffix)
                require(not sidecar.exists() and not sidecar.is_symlink())
    return {'navigationStore': store, 'navigationSha256': _nav_digest(semantic), 'navigationDetailsSha256': details,
            'navigationMigrationFixtureSha256': migration}


def stage_operator(prepared, directory):
    """Verify the kit with its exact B0 launcher prerequisite; execute nothing."""
    kit = module('package-protected-operator-kit')
    with zipfile.ZipFile(prepared['kit']) as archive:
        index = entries(archive)
        require(set(index) == set(kit.PAYLOAD_LIMITS) | {kit.MANIFEST})
        manifest = strict(member(archive, index, kit.MANIFEST, updater.MAX_RECORD_BYTES))
        keys(manifest, ('schema', 'productCommit', 'files'))
        require(manifest['schema'] == 1 and manifest['productCommit'] == prepared['b0']['commit'] and
                set(manifest['files']) == set(kit.PAYLOAD_LIMITS))
        require(sum(item.file_size for item in index.values()) <= kit.MAX_KIT_BYTES)
        for name, limit in kit.PAYLOAD_LIMITS.items():
            record = manifest['files'][name]
            keys(record, ('bytes', 'sha256'))
            pin(record['sha256'])
            require(type(record['bytes']) is int and record['bytes'] == index[name].file_size and 0 < record['bytes'] <= limit)
            require(index[name].external_attr >> 16 == stat.S_IFREG | 0o644)
            target = directory / name
            target.parent.mkdir(parents=True, exist_ok=True)
            with archive.open(index[name]) as source, target.open('xb') as output:
                while block := source.read(1 << 20):
                    output.write(block)
            require(updater._hash(target) == record['sha256'])
    with zipfile.ZipFile(prepared['b0']['paths']['recovery']) as archive:
        index = entries(archive)
        for name, limit in updater.TRANSFER_FILES.items():
            if name in kit.PAYLOAD_LIMITS:
                require(prepared['b0']['inventory']['app/' + name] == updater._hash(directory / name))
                continue
            item = index[PREFIX + 'app/' + name]
            require(0 < item.file_size <= limit)
            target = directory / name
            target.parent.mkdir(parents=True, exist_ok=True)
            with archive.open(item) as source, target.open('xb') as output:
                while block := source.read(1 << 20):
                    output.write(block)
    expected = updater.verify_updater_package(directory, prepared['b0']['commit'])
    kit._verify_components(directory, prepared['b0']['commit'], expected)
    with zipfile.ZipFile(directory / updater.SOURCE_RELATIVE) as archive:
        require((directory / kit.LICENSE).read_bytes() == archive.read('project/LICENSE') and
                (directory / kit.REFERENCE).read_bytes() == archive.read('SOURCE_REFERENCE.json'))
    prepared['provisionerSha256'] = manifest['files']['skager-update-provisioner.exe']['sha256']
    prepared['brokerSha256'] = manifest['files']['skager-update-broker.exe']['sha256']
    unchanged(prepared)
    return directory / 'skager-update-provisioner.exe'


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, *unused):
        raise ValueError(FAILURE)


def public_get(url):
    require(re.fullmatch(r'https://[a-z0-9.-]+/(?:timestamp\.json|[1-9][0-9]*\.(?:snapshot|targets)\.json)', url))
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), NoRedirect())
    with opener.open(urllib.request.Request(url, headers={'Cache-Control': 'no-cache'}), timeout=5) as response:
        require(response.status == 200 and response.geturl() == url)
        raw = response.read((1 << 20) + 1)
        require(0 < len(raw) <= 1 << 20)
        return raw


def publication(prepared, phase, timestamp_sha, get=public_get):
    origin = prepared['b0']['trust']['metadataUrl']
    require(re.fullmatch(r'https://[a-z0-9.-]+', origin))
    if phase == 'offline':
        require(timestamp_sha is None)
        try:
            get(origin + '/timestamp.json')
        except urllib.error.HTTPError:
            raise ValueError(FAILURE) from None
        except urllib.error.URLError as error:
            require(isinstance(error.reason, (TimeoutError, ConnectionError, socket.gaierror)) and
                    not isinstance(error.reason, ssl.SSLError))
            return
        except (TimeoutError, ConnectionError):
            return
        raise ValueError(FAILURE)  # HTTP errors, TLS failure and redirects are not offline proof.
    pin(timestamp_sha)
    raw = get(origin + '/timestamp.json')
    require(hashlib.sha256(raw).hexdigest() == timestamp_sha)
    value = strict(raw)
    for name in ('snapshot', 'targets'):
        metadata = value['signed']['meta'][name + '.json']
        require(type(metadata['version']) is int and metadata['version'] > 0)
        raw = get(origin + '/' + str(metadata['version']) + '.' + name + '.json')
        require(len(raw) == metadata['length'] and hashlib.sha256(raw).hexdigest() == metadata['hashes']['sha256'])
        value = strict(raw)
    selected = prepared['b0' if phase == 'no-update' else 'b1']
    target = value['signed']['targets']['releases/beta.json']
    require(target['length'] == selected['paths']['policy'].stat().st_size and
            target['hashes']['sha256'] == selected['policy']['sha256'])


def diagnostic(provisioner, run=subprocess.run, clock=time.monotonic):
    pages, expected, page, deadline = [], None, 0, clock() + 30
    while True:
        require(clock() < deadline)
        request = secrets.token_hex(16)
        command = [str(provisioner), '--maintenance=selected-history', '--request-id=' + request, '--page=' + str(page)]
        if expected is not None:
            command += ['--expected-selected-head-sha256=' + expected['selectedHeadSha256'],
                        '--inventory-sha256=' + expected['inventorySha256']]
        result = run(command, capture_output=True, timeout=10, check=False)
        require(result.returncode == 0 and not result.stderr and len(result.stdout) <= 4096)
        current = strict(result.stdout)
        required = {'schema', 'requestId', 'state', 'generation', 'completionSha256', 'selectedHeadSha256', 'version',
                    'commit', 'qualificationSha256', 'qualificationClosedSha256', 'inventorySha256', 'page', 'ancestors'}
        require(set(current) in (required, required | {'nextPage'}) and type(current['schema']) is int and
                current['schema'] == 1 and current['state'] == 'selected-history' and current['requestId'] == request and
                type(current['page']) is int and current['page'] == page and isinstance(current['ancestors'], list) and len(current['ancestors']) <= 16)
        for field in ('completionSha256', 'selectedHeadSha256', 'qualificationSha256', 'qualificationClosedSha256', 'inventorySha256'):
            pin(current[field])
        pin(current['generation'], 32)
        pin(current['commit'], 40)
        windows_product_version(current['version'])
        for ancestor in current['ancestors']:
            keys(ancestor, ('generation', 'completionSha256'))
            pin(ancestor['generation'], 32)
            pin(ancestor['completionSha256'])
        if expected is None:
            expected = current
        else:
            require(all(current[k] == expected[k] for k in required - {'requestId', 'page', 'ancestors'}))
        pages += current['ancestors']
        if 'nextPage' not in current:
            return dict(expected, ancestors=pages)
        require(type(current['nextPage']) is int and current['nextPage'] == page + 1)
        page += 1


def run_phase(prepared, phase, expected_head, timestamp_sha, native, report):
    pin(expected_head)
    before = native.history()
    initial = prepared['b1' if phase == 'offline' else 'b0']
    require(before['selectedHeadSha256'] == expected_head and before['version'] == initial['version'] and before['commit'] == initial['commit'])
    if phase != 'offline':
        require(before['generation'] == prepared['request']['installed']['generation'])
    report['before'] = before
    native.publication(phase, timestamp_sha)
    profile = native.profile()
    unchanged(prepared)
    native.start_and_close(phase, before)
    after = native.history()
    report['after'] = after
    if phase == 'now':
        require(after['version'] == prepared['b1']['version'] and after['commit'] == prepared['b1']['commit'] and
                after['selectedHeadSha256'] != before['selectedHeadSha256'] and after['generation'] != before['generation'] and
                after['qualificationSha256'] != before['qualificationSha256'] and
                after['qualificationClosedSha256'] != before['qualificationClosedSha256'] and
                {'generation': before['generation'], 'completionSha256': before['completionSha256']} in after['ancestors'])
    else:
        require(all(after[k] == before[k] for k in ('generation', 'completionSha256', 'selectedHeadSha256', 'version', 'commit',
                                                   'qualificationSha256', 'qualificationClosedSha256', 'inventorySha256')))
    require(profile_preserved(profile, native.profile()))
    native.publication(phase, timestamp_sha)
    unchanged(prepared)
    report['preservation'] = profile
    report['status'] = 'passed'


class Native:
    def __init__(self, prepared, provisioner):
        from ctypes import wintypes as w
        self.w, self.p, self.provisioner = w, prepared, provisioner
        self.ui = module('windows-ui')
        self.runtime = module('test-packaged-updater-windows')
        self.handles = []
        self.query = self.ui.declare(self.ui.kernel, 'QueryFullProcessImageNameW', w.BOOL, w.HANDLE, w.DWORD, w.LPWSTR, ctypes.POINTER(w.DWORD))
        self.times = self.ui.declare(self.ui.kernel, 'GetProcessTimes', w.BOOL, w.HANDLE, *([ctypes.POINTER(w.FILETIME)] * 4))
        self.program = self.folder('7c5a40ef-a0fb-4bfc-874a-c0f2e0b9fa8e')
        self.profile_root = self.folder(PROGRAM_DATA) / 'opencpn'
        self.sid = self.actual_sid()
        require(self.sid == prepared['request']['clientSid'])
        try:
            self.pin_image(provisioner, prepared['provisionerSha256'])
            self.pin_image(self.program / 'SKAGER' / 'UpdateBroker' / 'skager-update-broker.exe', prepared['brokerSha256'])
        except BaseException:
            self.close()
            raise

    def folder(self, identifier):
        guid = (ctypes.c_ubyte * 16).from_buffer_copy(uuid.UUID(identifier).bytes_le)
        value = ctypes.c_void_p()
        shell = ctypes.WinDLL('shell32', use_last_error=True)
        call = self.ui.declare(shell, 'SHGetKnownFolderPath', ctypes.c_long, ctypes.c_void_p, ctypes.c_ulong, ctypes.c_void_p, ctypes.POINTER(ctypes.c_void_p))
        require(call(ctypes.byref(guid), 0, None, ctypes.byref(value)) == 0 and value.value)
        try:
            path = Path(ctypes.wstring_at(value.value))
            updater._plain(path)
            return path
        finally:
            self.ui.declare(ctypes.WinDLL('ole32'), 'CoTaskMemFree', None, ctypes.c_void_p)(value)

    def actual_sid(self):
        w, ui = self.w, self.ui
        adv = ctypes.WinDLL('advapi32', use_last_error=True)
        current = ui.declare(ui.kernel, 'GetCurrentProcess', w.HANDLE)()
        thread = ui.declare(ui.kernel, 'GetCurrentThread', w.HANDLE)()
        token = w.HANDLE()
        if ui.declare(adv, 'OpenThreadToken', w.BOOL, w.HANDLE, w.DWORD, w.BOOL, ctypes.POINTER(w.HANDLE))(
                thread, 8, True, ctypes.byref(token)):
            ui.CloseHandle(token)
            raise ValueError(FAILURE)
        require(ctypes.get_last_error() == 1008)  # ERROR_NO_TOKEN only.
        require(ui.declare(adv, 'OpenProcessToken', w.BOOL, w.HANDLE, w.DWORD, ctypes.POINTER(w.HANDLE))(current, 8, ctypes.byref(token)))
        try:
            query = ui.declare(adv, 'GetTokenInformation', w.BOOL, w.HANDLE, ctypes.c_int, ctypes.c_void_p, w.DWORD, ctypes.POINTER(w.DWORD))
            used, elevated = w.DWORD(), w.DWORD()
            require(query(token, 20, ctypes.byref(elevated), 4, ctypes.byref(used)) and elevated.value == 1)
            query(token, 1, None, 0, ctypes.byref(used))
            buffer = ctypes.create_string_buffer(used.value)
            require(query(token, 1, buffer, used.value, ctypes.byref(used)))
            text = w.LPWSTR()
            require(ui.declare(adv, 'ConvertSidToStringSidW', w.BOOL, ctypes.c_void_p, ctypes.POINTER(w.LPWSTR))(
                ctypes.cast(buffer, ctypes.POINTER(ctypes.c_void_p))[0], ctypes.byref(text)))
            try:
                return text.value
            finally:
                ui.declare(ui.kernel, 'LocalFree', w.HANDLE, ctypes.c_void_p)(ctypes.cast(text, ctypes.c_void_p))
        finally:
            require(ui.CloseHandle(token))

    def image(self, generation, name):
        pin(generation, 32)
        return self.program / 'SKAGER' / 'Applications' / self.sid / 'generations' / generation / 'app' / name

    def pin_image(self, path, digest):
        import msvcrt
        updater._plain(path)
        w, ui = self.w, self.ui
        create = ui.declare(ui.kernel, 'CreateFileW', w.HANDLE, w.LPCWSTR, w.DWORD, w.DWORD, ctypes.c_void_p, w.DWORD, w.DWORD, w.HANDLE)
        handle = create(str(path), 0x80000000, 1, None, 3, 0x00200000, None)
        require(handle not in (None, ctypes.c_void_p(-1).value))
        try:
            descriptor = msvcrt.open_osfhandle(handle, os.O_RDONLY | os.O_BINARY)
        except BaseException:
            require(ui.CloseHandle(handle))
            raise
        try:
            stream = os.fdopen(descriptor, 'rb')
        except BaseException:
            os.close(descriptor)
            raise
        self.handles.append(stream)
        require(os.fstat(stream.fileno()).st_nlink == 1 and hashlib.file_digest(stream, 'sha256').hexdigest() == digest)
        return stream

    def history(self):
        return diagnostic(self.provisioner)

    def publication(self, phase, timestamp_sha):
        publication(self.p, phase, timestamp_sha)

    def profile(self):
        self.no_existing_clients()
        root = updater._plain(self.profile_root)
        result = navigation_observation(root)
        ini = updater._plain(root / 'opencpn.ini')
        require(ini.stat().st_size <= 8 << 20 and ini.stat().st_nlink == 1)
        config = configparser.RawConfigParser(strict=True, interpolation=None)
        config.optionxform = str
        config.read_string(ini.read_text(encoding='utf-8-sig'))
        result['settingsSha256'] = []
        for item in self.p['settings']:
            require(config.has_option(item['section'], item['key']))
            result['settingsSha256'].append(hashlib.sha256(config.get(item['section'], item['key'], raw=True).encode()).hexdigest())
        return result

    def process(self, pid, path, digest):
        handle = self.ui.monitor_process(pid)
        try:
            size = self.w.DWORD(32768)
            value = ctypes.create_unicode_buffer(size.value)
            require(self.query(handle, 0, value, ctypes.byref(size)) and Path(value.value).samefile(path))
            times = [self.w.FILETIME() for _ in range(4)]
            require(self.times(handle, *(ctypes.byref(t) for t in times)) and self.ui.WaitForSingleObject(handle, 0) == 258)
            created = (times[0].dwHighDateTime << 32) | times[0].dwLowDateTime
            require(created >= self.started)
            self.pin_image(path, digest)
            return handle
        except BaseException:
            self.ui.CloseHandle(handle)
            raise

    def start_and_close(self, phase, before):
        ui = self.ui
        self.no_existing_clients()
        require(not any(title in ('SKAGER / OpenCPN', 'SKAGER Legacy / OpenCPN', 'SKAGER Safe Mode / OpenCPN', 'SKAGER software update')
                        for _, _, title in ui.windows()))
        baseline = self.p['request']['installed']['generation']
        launcher = self.image(baseline, 'skager-start.exe')
        self.pin_image(launcher, self.p['b0']['product']['startup_launcher_sha256'])
        before_log = self.runtime.startup_log(self.profile_root)
        started = self.w.FILETIME()
        ui.declare(ui.kernel, 'GetSystemTimeAsFileTime', None, ctypes.POINTER(self.w.FILETIME))(ctypes.byref(started))
        self.started = (started.dwHighDateTime << 32) | started.dwLowDateTime
        parent = subprocess.Popen([str(launcher), '--broker-start=' + baseline], cwd=launcher.parent,
                                  stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        # Never kill on failure. The exact installed state and original processes
        # remain available for operator recovery on this disposable machine.
        if phase in ('later', 'now'):
            deadline, prompt = time.monotonic() + 30, None
            while time.monotonic() < deadline:
                matches = [(h, pid) for h, pid, title in ui.windows() if title == 'SKAGER software update']
                require(len(matches) <= 1)
                if matches:
                    prompt = matches[0]
                    break
                time.sleep(.1)
            require(prompt is not None)
            window, pid = prompt
            image = self.image(before['generation'], 'skager-update-prompt.exe')
            handle = self.process(pid, image, self.p['b0']['inventory']['app/skager-update-prompt.exe'])
            try:
                label = 'LATER' if phase == 'later' else 'UPDATE NOW'
                matches = [h for h, caption in ui.children(window) if caption == label and ui.IsWindowEnabled(h)]
                require(len(matches) == 1 and ui.IsWindowEnabled(window))
                owner = self.w.DWORD()
                ui.GetWindowThreadProcessId(window, ctypes.byref(owner))
                require(owner.value == pid and ui.WaitForSingleObject(handle, 0) == 258)
                # Actual custom-button mouse messages, matching the existing
                # native popup fixture; never synthesize an exit/consent result.
                for message, flags in ((0x0201, 1), (0x0202, 0)):
                    require(ui.PostMessageW(matches[0], message, flags, (20 << 16) | 20))
                require(self.wait_exit(handle, 30000) == (0 if phase == 'later' else 10))
            finally:
                if handle is not None:
                    require(ui.CloseHandle(handle))
        deadline = time.monotonic() + 180
        while parent.poll() is None and time.monotonic() < deadline:
            if phase in ('no-update', 'offline'):
                require(not any(title == 'SKAGER software update' for _, _, title in ui.windows()))
            time.sleep(.1)
        require(parent.poll() == 0)
        selected = self.history()  # Actual Now health+closed facts precede ordinary selection.
        target = self.p['b1' if phase in ('now', 'offline') else 'b0']
        require(selected['version'] == target['version'] and selected['commit'] == target['commit'])
        deadline, found = time.monotonic() + 30, None
        while time.monotonic() < deadline:
            matches = [(h, pid) for h, pid, title in ui.windows() if title == 'SKAGER / OpenCPN']
            require(len(matches) <= 1)
            if matches:
                found = matches[0]
                break
            time.sleep(.1)
        require(found is not None)
        window, pid = found
        app = self.image(selected['generation'], 'opencpn.exe')
        handle = self.process(pid, app, target['product']['executable_sha256'])
        try:
            self.runtime.wait_startup_ready(self.profile_root, before_log)
            require(ui.IsWindowEnabled(window) and ui.WaitForSingleObject(handle, 0) == 258)
            owner = self.w.DWORD()
            ui.GetWindowThreadProcessId(window, ctypes.byref(owner))
            require(owner.value == pid)
            require(ui.PostMessageW(window, 0x0010, 0, 0))
            require(self.wait_exit(handle, 30000) == 0)
        finally:
            if handle is not None:
                require(ui.CloseHandle(handle))

    def wait_exit(self, handle, timeout):
        # Keep ownership until our checked CloseHandle in the caller's finally.
        require(self.ui.WaitForSingleObject(handle, timeout) == 0)
        code = self.w.DWORD()
        require(self.ui.GetExitCodeProcess(handle, ctypes.byref(code)))
        return code.value

    def no_existing_clients(self):
        w, ui = self.w, self.ui
        class ProcessEntry(ctypes.Structure):
            _fields_ = [('size', w.DWORD), ('usage', w.DWORD), ('pid', w.DWORD), ('heap', ctypes.c_size_t),
                        ('module', w.DWORD), ('threads', w.DWORD), ('parent', w.DWORD),
                        ('priority', w.LONG), ('flags', w.DWORD), ('name', w.WCHAR * 260)]
        create = ui.declare(ui.kernel, 'CreateToolhelp32Snapshot', w.HANDLE, w.DWORD, w.DWORD)
        first = ui.declare(ui.kernel, 'Process32FirstW', w.BOOL, w.HANDLE, ctypes.POINTER(ProcessEntry))
        following = ui.declare(ui.kernel, 'Process32NextW', w.BOOL, w.HANDLE, ctypes.POINTER(ProcessEntry))
        snapshot = create(2, 0)
        require(snapshot not in (None, ctypes.c_void_p(-1).value))
        try:
            item = ProcessEntry()
            item.size = ctypes.sizeof(item)
            require(first(snapshot, ctypes.byref(item)))
            while True:
                require(item.name.casefold() not in ('opencpn.exe', 'skager-start.exe', 'skager-update-prompt.exe'))
                if not following(snapshot, ctypes.byref(item)):
                    require(ctypes.get_last_error() == 18)
                    break
        finally:
            require(ui.CloseHandle(snapshot))

    def close(self):
        failure = False
        for handle in reversed(self.handles):
            try:
                handle.close()
            except Exception:
                failure = True
        require(not failure)


class Once(argparse.Action):
    def __call__(self, parser, namespace, values, option_string=None):
        seen = getattr(namespace, '_seen', set())
        require(self.dest not in seen)
        namespace._seen = seen | {self.dest}
        setattr(namespace, self.dest, values)


def main(argv=None):
    class Parser(argparse.ArgumentParser):
        def error(self, message):
            raise ValueError(FAILURE)
    parser = Parser(description=__doc__, allow_abbrev=False)
    for name in ('fixtures', 'fixtures-sha256', 'evidence', 'disposable-opt-in', 'phase', 'expected-selected-head-sha256'):
        parser.add_argument('--' + name, required=True, action=Once)
    parser.add_argument('--timestamp-sha256', action=Once)
    report, output, native = {'schema': 1, 'status': 'refused', 'nativeProductAcceptance': False,
                              'nativeInstalledPhaseAcceptance': False}, None, None
    result_code = 1
    try:
        args = parser.parse_args(argv)
        require(sys.platform == 'win32' and args.disposable_opt_in == OPT_IN and args.phase in PHASES and
                os.environ.get('GITHUB_ACTIONS') == 'true' and os.environ.get('RUNNER_ENVIRONMENT') == 'github-hosted' and
                os.environ.get('GITHUB_REPOSITORY') == 'ThereptileII/Work')
        pin(args.fixtures_sha256)
        pin(args.expected_selected_head_sha256)
        require((args.phase == 'offline') == (args.timestamp_sha256 is None))
        if args.timestamp_sha256 is not None:
            pin(args.timestamp_sha256)
        prepared = prepare(Path(args.fixtures), args.fixtures_sha256)
        evidence = Path(args.evidence)
        require(evidence.is_absolute() and not evidence.exists() and not evidence.is_symlink())
        updater._plain(evidence.parent)
        # Evidence cannot be placed inside an approved input tree or any fixed
        # installed machine/profile tree. No caller-supplied installed roots exist.
        require(all(evidence != p and p not in evidence.parents for p, _, _ in prepared['retained']))
        require('skager' not in {p.casefold() for p in evidence.parts} and 'opencpn' not in {p.casefold() for p in evidence.parts})
        evidence.mkdir()
        output = evidence / 'result.json'
        report.update(phase=args.phase, fixturesSha256=args.fixtures_sha256,
                      harnessSha256=updater._hash(Path(__file__)), scope='installed selection, actual normal app and scoped navigation/settings preservation')
        # Staging is private and retained after every outcome; it is never an
        # installed namespace and no automatic cleanup runs on an error.
        operator = evidence / 'verified-operator'
        operator.mkdir()
        provisioner = stage_operator(prepared, operator)
        native = Native(prepared, provisioner)
        run_phase(prepared, args.phase, args.expected_selected_head_sha256, args.timestamp_sha256, native, report)
        native.close()
        native = None
        report['nativeInstalledPhaseAcceptance'] = True
        result_code = 0
    except (Exception, KeyboardInterrupt):
        report['status'] = 'refused'
        report['nativeProductAcceptance'] = False
        report['nativeInstalledPhaseAcceptance'] = False
        print(FAILURE, file=sys.stderr)
    finally:
        if native is not None:
            try:
                native.close()
            except Exception:
                report['status'] = 'refused'
                report['nativeProductAcceptance'] = False
                report['nativeInstalledPhaseAcceptance'] = False
                result_code = 1
        if output is not None:
            try:
                with output.open('x', encoding='utf-8') as stream:
                    json.dump(report, stream, sort_keys=True, indent=2)
                    stream.write('\n')
            except (Exception, KeyboardInterrupt):
                print(FAILURE, file=sys.stderr)
                result_code = 1
    return result_code


if __name__ == '__main__':
    raise SystemExit(main())

"""Observed font formats and exact pinned OpenCPN font-source identities.

These are source/byte identities, not license grants. In particular SSFN's
renderer license must not be mistaken for the converted FreeSans font's terms.
"""
from pathlib import PurePosixPath
import re

FREESANS_SOURCE = 'OpenCPN-5.12.4-integrated/data/ssfn/FreeSans.sfn'
FREESANS_PACKAGE = 'app/ssfndata/FreeSans.sfn'
FREESANS_SHA256 = '4b34341dbfd211b646807bda719e81f2ab2938bcb31f9eaf9c3f26b371566a46'
HELVETICA_SOURCE = 'OpenCPN-5.12.4-integrated/data/s57data/Helvetica.txf'
HELVETICA_SHA256 = '1c22a8277dd2ef1c0524978f4a3610fd56015107aa90cedd8990129625dde852'
FONT_SUFFIXES = {'.ttf', '.otf', '.ttc', '.otc', '.woff', '.woff2', '.sfn', '.txf',
                 '.fon', '.fnt', '.pfa', '.pfb', '.bdf', '.pcf', '.eot', '.dfont', '.sfnt'}


def font_candidate(name, header):
    """A font candidate needs review; an extension alone does not prove format."""
    signatures = {b'SSFN': 'SSFN', b'SFNC': 'SSFN collection', b'\xfftxf': 'TXF',
                  b'OTTO': 'OpenType', b'\x00\x01\x00\x00': 'TrueType',
                  b'ttcf': 'TrueType collection', b'wOFF': 'WOFF', b'wOF2': 'WOFF2'}
    if header[:4] in signatures:
        return {'basis': 'header', 'format': signatures[header[:4]]}
    suffix = PurePosixPath(name).suffix.lower()
    if suffix in FONT_SUFFIXES:
        return {'basis': 'extension-only', 'format': 'unverified ' + suffix}
    return None


def inspect_install_lists(cmake):
    """Check the reviewed simple upstream lists, not arbitrary CMake evaluation."""
    def entries(variable):
        matches = re.findall(r'(?m)^set\(\s*' + variable + r'\s+([^)]*)\)', cmake)
        if len(matches) != 1:
            raise ValueError('Font install list is missing or ambiguous: ' + variable)
        return matches[0].split()
    if entries('ssfnData') != ['data/ssfn/FreeSans.sfn']:
        raise ValueError('SSFN install source inventory changed')
    if 'data/s57data/Helvetica.txf' in entries('s57data'):
        raise ValueError('Texture font has entered the reviewed install list')
    normalized = ' '.join(cmake.split())
    for variable, destination in (('ssfnData', 'ssfndata'), ('s57data', 's57data')):
        if f'install(FILES ${{{variable}}} DESTINATION ${{PREFIX_PKGDATA}}/{destination})' not in normalized:
            raise ValueError('Font installation mapping changed')
    return {'installedSource': 'data/ssfn/FreeSans.sfn',
            'sourceOnlyUnderReviewedList': 'data/s57data/Helvetica.txf'}

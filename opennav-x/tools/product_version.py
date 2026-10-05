"""The reviewed Beta 2 product series and its exact Windows resource mapping."""
from pathlib import Path
import re


def validate_product_version(value):
    if not isinstance(value, str):
        raise ValueError('Product version must be a string')
    match = re.fullmatch(r'0\.4\.0-beta2(?:\.([1-9][0-9]{0,4}))?', value)
    if not match or (match[1] is not None and int(match[1]) > 65535):
        raise ValueError('Expected canonical 0.4.0-beta2 or 0.4.0-beta2.N (1..65535)')
    return value


def read_product_version(header):
    text = Path(header).read_text(encoding='utf-8')
    versions = re.findall(r'inline\s+constexpr\s+char\s+Version\[\]\s*=\s*"([^"\r\n]+)"\s*;', text)
    editions = re.findall(r'inline\s+constexpr\s+char\s+Edition\[\]\s*=\s*"([^"\r\n]+)"\s*;', text)
    if len(versions) != 1 or editions != ['Beta 2']:
        raise ValueError('One exact Beta 2 Version.h identity is required')
    return validate_product_version(versions[0])


def windows_product_version(value):
    version = validate_product_version(value)
    return (0, 4, 0, int(version.rsplit('.', 1)[1]) if '-beta2.' in version else 0)

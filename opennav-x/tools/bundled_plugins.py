"""Positive discovery check for the four normal Windows bundled plugins."""
import time

WINDOWS_BUNDLED_DLLS = ('chartdldr_pi.dll', 'dashboard_pi.dll', 'grib_pi.dll', 'wmm_pi.dll')


def require_bundled_windows_plugins(snapshot):
    # RuntimeDiagnostics copies the upstream PluginLoader array. Initial normal
    # loading sets this filename only after successful load/factory creation;
    # disabled plugins may have enabled/initialized=false. Not ABI qualification.
    records = snapshot['runtime']['plugins']
    assert isinstance(records, list) and all(
        isinstance(record, dict) and isinstance(record.get('file'), str)
        for record in records), 'Malformed native plugin records'
    for filename in WINDOWS_BUNDLED_DLLS:
        # Windows filenames are case-insensitive. Require the whole basename,
        # never a common-name match, suffix, path or an arbitrary replacement.
        count = sum(record['file'].lower() == filename for record in records)
        assert count == 1, ('Expected one loaded bundled plugin record', filename, count)


def _publication_epoch(snapshot):
    # PreviewDiagnostics writes steady_clock milliseconds as a decimal string.
    value = snapshot.get('publication_monotonic_ms')
    assert (snapshot.get('publication_clock') == 'live monotonic clock' and
            isinstance(value, str) and 1 <= len(value) <= 19 and
            value.isascii() and value.isdecimal()), 'Invalid native publication epoch'
    epoch = int(value)
    assert epoch <= 9223372036854775807, 'Invalid native publication epoch'
    return epoch


def require_fresh_bundled_windows_plugins(read, after_ready):
    """Check the first newer publication, never poll until plugins succeed."""
    baseline = _publication_epoch(after_ready)
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline:
        snapshot = read()
        if _publication_epoch(snapshot) > baseline:
            require_bundled_windows_plugins(snapshot)
            return snapshot
        time.sleep(.1)
    raise AssertionError('Newer native diagnostic publication did not arrive')

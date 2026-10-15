"""Positive discovery check for the four normal Windows bundled plugins."""

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

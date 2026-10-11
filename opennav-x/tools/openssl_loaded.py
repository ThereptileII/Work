"""Compare existing native module observations with installed OpenSSL provenance."""
from pathlib import Path

from openssl_package import EXPECTED_VERSION, verify_packaged_openssl


def verify_loaded_openssl(install, manifest, loaded):
    """Bind an existing native module observation to the verified install.

    This does not enumerate a process or authenticate the observation. The
    native caller must supply its complete observed path/byte identities and
    retain the installed manifest/binaries unchanged around that observation.
    """
    install = Path(install).resolve()
    if (manifest.get('library') != 'OpenSSL' or
            manifest.get('version') != EXPECTED_VERSION or
            manifest.get('architecture') != 'Win32' or manifest.get('abi') != 'x86'):
        raise ValueError('Loaded OpenSSL requires maintained Win32 build provenance')
    verify_packaged_openssl(install, manifest)
    result = {}
    for name in ('libssl-3.dll', 'libcrypto-3.dll'):
        matches = [(Path(path), record) for path, record in loaded.items()
                   if Path(path).name.casefold() == name]
        if len(matches) != 1:
            raise ValueError('Loaded OpenSSL module missing or duplicated: ' + name)
        path, record = matches[0]
        expected = manifest['outputs']['bin/' + name]
        if (not path.is_absolute() or path.is_symlink() or
                path.resolve() != install / name or record != expected):
            raise ValueError('Loaded OpenSSL path or bytes differ from installed build: ' + name)
        # Outputs and cache/install provenance must agree, not merely match two
        # independently supplied observations.
        if {key: manifest['cacheBuildwin'][name][key] for key in ('bytes', 'sha256')} != expected:
            raise ValueError('Loaded OpenSSL output/cache provenance differs: ' + name)
        result[name] = {'path': str(path), **record}
    return result

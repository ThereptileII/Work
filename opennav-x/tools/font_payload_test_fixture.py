"""Inert font-source fixture; never used for product packaging."""
import hashlib
from unittest.mock import patch
import freefont_payload


def install(test, add_source, add_notice=None):
    payloads = {name: ('inert font fixture ' + name).encode() for name in freefont_payload.SOURCE_FILES}
    override = patch.object(freefont_payload, 'SOURCE_FILES',
                            {name: hashlib.sha256(data).hexdigest() for name, data in payloads.items()})
    override.start()
    test.addCleanup(override.stop)
    for name, data in payloads.items():
        add_source(name, data)
        if add_notice and name in freefont_payload.NOTICE_FILES:
            add_notice(freefont_payload.PACKAGE_ROOT + name.removeprefix(freefont_payload.NOTICE_ROOT), data)

"""Verify a same-commit native baseline handoff console beside its provisioner.

This checks exact PE, source and prerequisite bytes. It does not authenticate
remote producer provenance, approve operator pins, enroll a broker or run output.
"""
import broker_package as broker
import provisioner_package as provisioner
import updater_package as updater

BINARY = 'skager-baseline-package.exe'
RECORD_RELATIVE = 'opennav/third-party/baseline-handoff/build.json'
BUILD_RECIPE = ('CGO_ENABLED=0 GOOS=windows GOARCH=386 GO386=sse2 go build '
                '-mod=readonly -trimpath -buildvcs=true ./cmd/skager-baseline-package')

# The provisioner source closure covers the registration/importer and native
# prerequisites. Add the actual baseline producer and fixed packaging entry
# points; all retained module Go members are also hash checked by Component.
REQUIRED_SOURCES = frozenset(provisioner.REQUIRED_SOURCES | {
    broker.PREFIX + 'baselinepackage/package.go',
    broker.PREFIX + 'cmd/skager-baseline-package/main.go',
    'project/tools/baseline_handoff_package.py',
    'project/tools/package-baseline-handoff.py',
})
BASELINE_HANDOFF = broker.Component('Baseline handoff', BINARY, RECORD_RELATIVE,
                                    'cmd/skager-baseline-package', BUILD_RECIPE, 3,
                                    REQUIRED_SOURCES, broker.PREFIX)


def provisioner_prerequisite(app, commit):
    """Pin the exact provisioner bytes/record; it pins broker and launcher."""
    app = updater._plain(app)

    def snapshot():
        return {'binary': {'path': provisioner.BINARY,
                           'sha256': updater._hash(broker._regular(
                               app / provisioner.BINARY, updater.MAX_BINARY_BYTES))},
                'buildRecord': {'path': provisioner.RECORD_RELATIVE,
                                'sha256': updater._hash(broker._regular(
                                    app / provisioner.RECORD_RELATIVE,
                                    updater.MAX_RECORD_BYTES))}}

    before = snapshot()
    provisioner.verify_provisioner_package(app, commit)
    if snapshot() != before:
        raise ValueError('Provisioner prerequisite changed during verification')
    return before


def verify_baseline_handoff_package(app, commit):
    """Require exact console producer and unchanged native prerequisite chain."""
    prerequisite = provisioner_prerequisite(app, commit)
    descriptor = broker._verify_component_package(app, commit, BASELINE_HANDOFF,
                                                   prerequisite=prerequisite)
    if provisioner_prerequisite(app, commit) != prerequisite:
        raise ValueError('Baseline handoff prerequisite changed during verification')
    return descriptor

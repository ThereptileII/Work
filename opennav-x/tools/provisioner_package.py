"""Verify an operator-only provisioner beside its native launcher/broker closure.

The caller authenticates original native producer provenance before first
execution. Hashes and embedded metadata bind bytes and source; this verifier
never elevates, executes, installs, provisions trust, or activates a service.
"""
import broker_package as broker

BINARY = 'skager-update-provisioner.exe'
RECORD_RELATIVE = 'opennav/third-party/update-provisioner/build.json'
BUILD_RECIPE = ('CGO_ENABLED=0 GOOS=windows GOARCH=386 GO386=sse2 go build '
                '-mod=readonly -trimpath -buildvcs=true ./cmd/skager-update-provisioner')
REQUIRED_SOURCES = frozenset(broker.REGISTRATION_SOURCES | {broker.PREFIX + name for name in (
    'cmd/skager-update-provisioner/main.go',
    'cmd/skager-update-provisioner/operator_maintenance.go',
    'cmd/skager-update-provisioner/operator_decommission.go',
    'cmd/skager-update-provisioner/installed_maintenance.go',
    'cmd/skager-update-provisioner/installed_elevation.go',
    'cmd/skager-update-provisioner/installed_elevation_other.go',
    'cmd/skager-update-provisioner/installed_elevation_windows.go',
    'cmd/skager-update-provisioner/operator_selected_diagnostics.go',
    'cmd/skager-update-provisioner/customer_rollback.go',
    'cmd/skager-update-provisioner/customer_rollback_other.go',
    'cmd/skager-update-provisioner/customer_rollback_windows.go',
    'cmd/skager-update-provisioner/customer_repair.go',
    'cmd/skager-update-provisioner/customer_repair_other.go',
    'cmd/skager-update-provisioner/customer_repair_windows.go',
    'cmd/skager-update-provisioner/operator_manual_rollback.go', 'go.mod', 'go.sum')} | {
        'project/LICENSE', 'project/tools/package-update-launcher.py',
        'project/tools/package-update-broker.py', 'project/tools/package-update-provisioner.py',
        'project/tools/broker_package.py', 'project/tools/provisioner_package.py',
        'project/tools/updater_package.py'})
PROVISIONER = broker.Component('Provisioner', BINARY, RECORD_RELATIVE,
                               'cmd/skager-update-provisioner', BUILD_RECIPE, 3,
                               REQUIRED_SOURCES, broker.PREFIX)


def verify_provisioner_package(app, commit):
    """Require exact console provisioner and unchanged native prerequisites."""
    prerequisite = broker.broker_prerequisite(app, commit)
    descriptor = broker._verify_component_package(app, commit, PROVISIONER,
                                                  prerequisite=prerequisite)
    if broker.broker_prerequisite(app, commit) != prerequisite:
        raise ValueError('Provisioner broker prerequisite changed during verification')
    return descriptor

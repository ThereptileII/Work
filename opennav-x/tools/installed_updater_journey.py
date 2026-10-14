"""One genuine disposable installed lifecycle; no approval or host provisioning.

Call prepare on the existing admission/phase drivers with independently approved
declaration pins before run_joined. The coordinating operator owns native host
authorization and the handoff callback. This module never publishes, changes
connectivity, seeds a profile, synthesizes approval, retries or removes state.
"""
import copy
import importlib.util
import json
from pathlib import Path


_spec = importlib.util.spec_from_file_location(
    '_joined_initial_b0', Path(__file__).with_name('test-installed-protected-b0-startup-windows.py'))
admission = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(admission)
phase = admission.phase
PHASES = ('no-update', 'later', 'now', 'offline')
STEPS = (*PHASES, 'manual-rollback')


def _same_inputs(initial, phases):
    """Both independently prepared declarations must retain the SAME originals."""
    admission.require(all(initial[key] == phases[key] for key in ('b0', 'request', 'kit', 'settings')))
    for path in (*initial['b0']['paths'].values(), initial['requestPath'], initial['kit']):
        left = [item for item in initial['retained'] if item[0] == path]
        right = [item for item in phases['retained'] if item[0] == path]
        admission.require(len(left) == len(right) == 1 and left == right)
    admission.require(phase.windows_product_version(phases['b1']['version']) >
                      phase.windows_product_version(phases['b0']['version']) and
                      phases['b1']['commit'] != phases['b0']['commit'] and
                      phases['b1']['trustSha256'] == phases['b0']['trustSha256'])
    admission.unchanged(initial)
    phase.unchanged(phases)


class _OriginalBroker:
    """Retain the fixture's already authenticated original query owner only."""
    def __init__(self, fixture):
        self.ui, self.handle = fixture.n.ui, fixture.service_process()
        self.closed = False

    def check(self):
        # A restart cannot make this original retained process live again.
        admission.require(not self.closed and self.ui.WaitForSingleObject(self.handle, 0) == 258)

    def close(self):
        if not self.closed:
            self.closed = True
            admission.require(self.ui.CloseHandle(self.handle))


def _write_result(path, report):
    with path.open('x', encoding='utf-8') as stream:
        json.dump(report, stream, sort_keys=True, indent=2)
        stream.write('\n')


def _evidence(output, initial, phases):
    output = Path(output)
    evidence = output.parent
    admission.require(output.is_absolute() and output.name == 'result.json' and not output.exists() and
                      not output.is_symlink() and not evidence.is_symlink())
    admission.updater._plain(evidence.parent)
    admission.require(all(evidence != p and p not in evidence.parents
                          for p, _, _ in (*initial['retained'], *phases['retained'])) and
                      all(evidence != p and p not in evidence.parents and evidence not in p.parents
                          for p, _ in initial['directories']))
    admission.require(not {'skager', 'opencpn'} & {part.casefold() for part in evidence.parts})
    if evidence.exists():
        admission.updater._plain(evidence)
        admission.require(evidence.is_dir() and next(evidence.iterdir(), None) is None)
    else:
        evidence.mkdir()
    return output


def run_joined(initial, phases, pins, output, *, handoff,
               initial_native_factory=phase.Native, phase_native_factory=phase.Native,
               preflight_factory=admission.BeforeEnrollment, stage=phase.stage_operator,
               publish=phase.publication, enroll=admission.enroll_once,
               fixture_factory=admission.InstalledB0Fixture, broker_factory=_OriginalBroker):
    """Setup/enroll once, four startup phases, explicit B0 rollback, Maintain LAST.

    handoff(phase_name, expected_selected_head) performs only the independently
    authorized operator's publication/connectivity work. Return its independently
    approved timestamp SHA-256 for each online phase and exactly None for offline
    and manual-rollback. The latter keeps the authorized offline condition through
    explicit rollback and restored B0 ordinary startup; it cannot publish internally.
    This return value is a pin, never an approval manufactured by the journey.
    Original publication/offline observers verify the actual configured endpoint.

    output is result.json in a fresh evidence directory. Every phase retains its
    original scoped result, including a refused partial phase. A failure raises
    after retaining evidence and draining native query/image owners; no later
    phase or uninstall runs. Portable injected seams are not native acceptance.
    """
    admission.require(callable(handoff))
    _same_inputs(initial, phases)
    # Own the context without changing caller records or the initial observer's
    # B0-only p. Native hashes are added only after authenticated B0 staging.
    phase_context = copy.deepcopy(phases)
    output = _evidence(output, initial, phase_context)
    reports = []
    result = dict(schema=1, state='refused', nativeInitialB0Acceptance=False,
                  nativeInstalledUninstallAcceptance=False, nativeInstalledManualRollbackAcceptance=False,
                  customerUacAcceptance=False,
                  nativeProductAcceptance=False, phases=reports)
    result['journeyHarnessSha256'] = admission.updater._hash(Path(__file__))
    result['admissionHarnessSha256'] = admission.updater._hash(Path(admission.__file__))
    result['declarationSha256'] = initial['retained'][0][2]
    result['fixturesSha256'] = phase_context['retained'][0][2]

    def before_uninstall(authenticated, provisioner, selected, original_profile, fixture):
        _same_inputs(authenticated, phase_context)
        # These observations came from successful original kit verification and
        # the admission's independent policy/pin comparisons BEFORE enrollment.
        phase_context.update(provisionerSha256=authenticated['provisionerSha256'],
                             brokerSha256=authenticated['brokerSha256'])
        expected_head = selected['selectedHeadSha256']
        broker = broker_factory(fixture)
        try:
            for name in STEPS:
                broker.check()
                admission.unchanged(authenticated)
                phase.unchanged(phase_context)
                directory = output.parent / name
                directory.mkdir()
                acceptance = ('nativeInstalledManualRollbackAcceptance' if name == 'manual-rollback'
                              else 'nativeInstalledPhaseAcceptance')
                report = dict(schema=1, status='refused', phase=name, nativeProductAcceptance=False,
                              expectedSelectedHeadSha256=expected_head,
                              fixturesSha256=result['fixturesSha256'],
                              harnessSha256=admission.updater._hash(Path(phase.__file__)),
                              scope=('explicit protected B0 rollback, restored ordinary app and current-profile preservation'
                                     if name == 'manual-rollback' else
                                     'installed selection, actual normal app and scoped navigation/settings preservation'))
                report[acceptance] = False
                reports.append(report)
                native = None
                try:
                    timestamp = handoff(name, expected_head)
                    admission.require(timestamp is None if name in ('offline', 'manual-rollback') else phase.pin(timestamp))
                    if timestamp is not None:
                        report['timestampSha256'] = timestamp
                    broker.check()
                    admission.unchanged(authenticated)
                    phase.unchanged(phase_context)
                    native = phase_native_factory(phase_context, provisioner)
                    admission.require(native.sid == fixture.n.sid and
                                      native.profile_root == fixture.n.profile_root)
                    admission.require(phase.profile_preserved(original_profile, native.profile()))
                    if name == 'manual-rollback':
                        phase.run_manual_rollback(phase_context, expected_head, selected, native, report)
                    else:
                        phase.run_phase(phase_context, name, expected_head, timestamp, native, report)
                    admission.require(phase.profile_preserved(original_profile, native.profile()))
                    broker.check()
                    # Drain generation image owners before the next handoff,
                    # update transaction or customer Maintain retirement.
                    closed, native = native, None
                    closed.close()
                    admission.unchanged(authenticated)
                    phase.unchanged(phase_context)
                    report[acceptance] = True
                    if name == 'manual-rollback':
                        result[acceptance] = True
                    expected_head = report['after']['selectedHeadSha256']
                except BaseException:
                    report['status'] = 'refused'
                    report[acceptance] = False
                    raise
                finally:
                    try:
                        if native is not None:
                            native.close()
                    finally:
                        _write_result(directory / 'result.json', report)
            broker.check()
        finally:
            broker.close()

    try:
        installed = admission.run(initial, pins, output, initial_native_factory, preflight_factory,
                                  stage, publish, enroll, fixture_factory, before_uninstall=before_uninstall)
        result.update(installed)
        result['state'] = 'genuine-setup-b0-four-phases-manual-rollback-maintain-uninstalled'
        return result
    finally:
        _write_result(output, result)

"""Schema 3: one reviewed full native retest of the frozen bf candidate.

No failed prefix is promoted. Every runtime report comes from the fresh scope=all
job; the old archive identifies only the reviewed stale-publication failure.
The closed Git delta leaves the qualifier, workflow, product, SDK and recipes
unchanged. Composition itself never launches or rebuilds the application.
"""
import hashlib
import re

import release_manifest as manifest
import staging_build_inputs as inputs

require = manifest.require
PRODUCER = dict(commit='bf4896833fda8389cfb03924cde7d6c89de5e5e6', runId='39320631961', runAttempt='1')
HARNESS = '43a088a2d15fc33f3fd13956dcc61f0e52754b8c'
HARNESS_DELTA = {
    'opennav-x/tools/smoke-navigation.py': '6830eed4bdee5ef0b6e31b0212c4671154f5cd7c',
    'opennav-x/tools/test-smoke-native-waypoint-name.py': 'ed15a338cb2eae93524127c5ab2fcf34e61aa018',
    'opennav-x/tests/smoke_navigation_surface_tests.py': '37de7b4da792cd2d25792c7da86a19ca180403e0',
}
BUILD = dict(artifactId='12222529458',
    artifactName='staging-build-' + PRODUCER['commit'] + '-run39320631961-attempt1',
    artifactDigest='sha256:60e3d7d7371ffdec7d8538f38bb38fd61fc42227f66f7fe40fbebc577fae5b5e',
    archiveSha256='2ff939ab78c7b1286cf68ead235489e6c6148d7528fff86dda26cac4131d85e0')
MANIFEST_SHA = 'ae74d3e05d034a576a3cc4e5340a41a3eaf7b15ca3fc5d8f715359aef9e7ff2a'
SOURCE_SHA = '6b5dbaa7228bd7ad074af7f166b5739176b62bc49c2a6b76a4c96bfc98bd2e89'
ORIGINAL = dict(jobId='118321923727', artifactId='12223183895',
    artifactName='windows-qualification-' + PRODUCER['commit'] + '-attempt1',
    artifactDigest='sha256:fd754f83c7beb21eaa5992a3ffee0b1f380c45f27a1a02c982d42c1d18006c2d')
ORIGINAL_REPORTS = {
    'staging-inputs.json': '3f044a24310db753ec43a7ab7865193e973f8da591d2e0c01ffbe111df7c9d8b',
    'objects-input-results.json': 'd10dac5af24f5efca471b0c2089d10c3e1be1d78fca4c26a11366244d21174b8',
    'objects-input-profile/objects-fixture-results.json': 'ec18a48fff4bddc7bba8076b7428a2fafc004f4b72d4399876c4c4163bcde894',
    'objects-input-profile/opennav-diagnostics.json': 'bcaf5666b7696d44628f9ad91f4e545dd4bad32a49f38ed461f5ef5ddd3fced6',
}
RESTART = dict(artifactId='12221495624',
    artifactName='commissioning-restart-qualified-' + PRODUCER['commit'] + '-attempt1',
    artifactDigest='sha256:f542105e17f6ceca8246c12957e6ff171da70903db09ba69c31f8495bad9448d',
    reportSha256='be5b24a5501a6dd7cc0d318dd0c75cfcc7985ffdaa3c4ea817e4d4f4aaaa04c8')
EXTRA_JOBS = ('pilot-serial-contracts / serial',
              'updater-contracts / Native protected mode routing and ordinary restart')
SIGNED_LOGS = frozenset('signed-update-custody/' + arch + suffix + '.log'
                        for arch in ('x64', 'x86') for suffix in ('', '-previous-restore'))
EXTRA_REPORTS = frozenset({'peer-two-instances-results.json',
                          'objects-input-profile/objects-fixture-results.json'}) | SIGNED_LOGS
NATIVE_EDIT_CONFIRMATION = 'Themed property sheet saves and refreshes; delete cancellation preserves mark'
MANUAL_PILOT_TAIL = [
    'TRACK stays unavailable without qualified native route output; disabled click creates no request or wire bytes',
    'WIND UI follows reported capability only; no wind readiness, WIND output or mode confirmation claimed',
]


def validate_request(request):
    require(request['producer'] == PRODUCER and request['build'] == BUILD and
            request['retest']['commit'] == HARNESS and request['restart'] == RESTART and
            {k: v for k, v in request['original'].items() if k != 'prefixReports'} == ORIGINAL,
            'Full retest requires the exact reviewed producer, failure, build, restart and corrected harness')
    require({i['path']: i['sha256'] for i in request['original']['prefixReports']} == ORIGINAL_REPORTS,
            'Original stale-publication failure bytes differ')


def verify_harness(gh):
    comparison = gh.api(f"{gh.base}/compare/{PRODUCER['commit']}...{HARNESS}")
    require(comparison.get('base_commit', {}).get('sha') == PRODUCER['commit'] and
            comparison.get('merge_base_commit', {}).get('sha') == PRODUCER['commit'] and
            comparison.get('status') == 'ahead' and comparison.get('total_commits') == 1 and
            [item.get('sha') for item in comparison.get('commits', [])] == [HARNESS],
            'Full retest harness is not the reviewed direct child')
    files = comparison.get('files', [])
    require(len(files) == len(HARNESS_DELTA) and
            all(item.get('status') == 'modified' and not item.get('previous_filename') for item in files) and
            {item.get('filename'): item.get('sha') for item in files} == HARNESS_DELTA,
            'Full retest changed unreviewed helper, product, qualifier, workflow, SDK or recipe bytes')


def validate_restart(record):
    require(isinstance(record, dict) and set(record) == {'schema', 'owner', 'commit', 'runId', 'runAttempt',
                           'transport', 'maintenance', 'broker', 'prepareArm', 'actualBoat'} and
            type(record['schema']) is int and record['schema'] == 1 and
            record['owner'] == 'OpenNavX.CI.RestartGates.1' and
            all(str(record[k]) == v for k, v in PRODUCER.items()) and
            all(record[k] == 'success' for k in ('transport', 'maintenance', 'broker', 'prepareArm')) and
            record['actualBoat'] is False, 'Same-producer restart qualification differs')


def package_binding(tree):
    """Metadata from the already hash-authenticated retained package tree."""
    raw = {name: inputs.plain_file(tree, inputs.PACKAGE_ROOT + '/' + name).read_bytes()
           for name in ('docs/PRODUCT_BUILD.json', 'FILE_SHA256.json')}
    product = inputs.strict_json(raw['docs/PRODUCT_BUILD.json'])
    inventory = inputs.strict_json(raw['FILE_SHA256.json'])
    require(isinstance(inventory, dict) and len(inventory) > 0 and
            inventory.get('docs/PRODUCT_BUILD.json') == hashlib.sha256(raw['docs/PRODUCT_BUILD.json']).hexdigest() and
            inventory.get('app/opencpn.exe') == product.get('executable_sha256') and
            inventory.get('app/skager-start.exe') == product.get('startup_launcher_sha256'),
            'Frozen inventory does not bind product and launcher')
    require(product.get('commit') == PRODUCER['commit'] and product.get('test_fixtures') is False and
            product.get('build_purpose') == 'INSTALLED PRODUCT' and
            product.get('xnav_hardware_output_policy') == 'manual-commissioning' and
            type(product.get('xnav_manual_control_contract')) is int and product['xnav_manual_control_contract'] == 1,
            'Frozen full-retest product policy differs')
    return dict(build=product, files=len(inventory))


def _checks(record, result, count, label):
    require(record.get('result') == result and isinstance(record.get('checks'), list) and
            len(record['checks']) == count, label + ' checks incomplete')


def _restored(record, harness):
    require(record['status'] == 'restored' and record['qualification'] == 'not-run' and
            record['producer'] == inputs.producer(PRODUCER['commit'], PRODUCER['runId'], PRODUCER['runAttempt']) and
            record['harnessCommit'] == harness and record['archiveSha256'] == BUILD['archiveSha256'] and
            record['sourceArchiveSha256'] == SOURCE_SHA and record['manifestSha256'] == MANIFEST_SHA,
            'Full retest restored another source, manifest, archive or harness')


def _native_names(record):
    require(record.get('native_edit_confirmation') == NATIVE_EDIT_CONFIRMATION,
            'Native waypoint save/cancel confirmation absent')
    names = record.get('native_waypoint_name')
    expected = [('after-save', 'ALPHA TEST UI edited'), ('after-delete-cancel', 'ALPHA TEST UI edited'),
                ('after-restore', 'ALPHA TEST edited')]
    require(isinstance(names, list) and len(names) == len(expected), 'Native waypoint read stages incomplete')
    for item, (stage, value) in zip(names, expected):
        require(set(item) == {'stage', 'page_handle', 'edit_handle', 'bounds', 'value',
                              'diagnostic_field_published', 'reader'} and
                item['stage'] == stage and item['value'] == value and item['reader'] == 'WM_GETTEXT' and
                type(item['diagnostic_field_published']) is bool and
                all(type(item[k]) is int and 0 < item[k] < 2**64 for k in ('page_handle', 'edit_handle')) and
                item['page_handle'] != item['edit_handle'], 'Native waypoint read identity/value differs')
        bounds = item['bounds']
        require(isinstance(bounds, dict) and set(bounds) == {'x', 'y', 'width', 'height'} and
                all(type(v) is int for v in bounds.values()) and bounds['width'] > 0 and bounds['height'] > 0,
                'Native waypoint read has invalid geometry')


def _signed_reports(reports):
    # These are corroborating output from the exact unchanged qualifier command
    # list, whose authenticated full native job must also have succeeded.
    for arch, bits in (('x64', 64), ('x86', 32)):
        lines = reports[f'signed-update-custody/{arch}.log'].splitlines()
        expected = (
            'Signed update session: 61 pure contract checks passed; no launch permission or native qualification claimed.',
            'Signed update operations: 129 pure checks passed; no broker, mutation or launch authority.',
            'Signed update store: 245 inert checks passed; nativeWindows=True. No launch authority or resumable recovery claimed.',
        )
        require(all(lines.count(line) == 1 for line in expected), 'Signed update session checks incomplete')
        json_lines = [line for line in lines if line.strip().startswith('{')]
        require(len(json_lines) == 1, 'Signed update installed barrier report missing or ambiguous')
        barrier = inputs.strict_json(json_lines[0])
        require(barrier.get('status') == 'passed' and barrier.get('native') is True and
                type(barrier.get('count')) is int and barrier['count'] == 42 and
                barrier.get('scope') == 'inert installed barrier and private store only' and
                barrier.get('applicationLaunched') is False and barrier.get('profileAccess') is False,
                'Signed update installed barrier did not pass inertly')
        previous = inputs.strict_json(reports[f'signed-update-custody/{arch}-previous-restore.log'])
        require(previous.get('status') == 'passed' and previous.get('processBits') == bits and
                re.fullmatch(r'5\.1\.[0-9.]+', str(previous.get('powerShellVersion', ''))) and
                type(previous.get('count')) is int and previous['count'] == 106 and
                isinstance(previous.get('checks'), list) and previous['count'] == len(previous['checks']) and
                isinstance(previous.get('sourceSha256'), dict) and len(previous['sourceSha256']) == 17 and
                all(isinstance(v, str) and re.fullmatch('[a-f0-9]{64}', v) for v in previous['sourceSha256'].values()) and
                all(previous.get(k) is False for k in ('continuousNamespaceCustodyAccepted',
                    'installedProductAccepted', 'productWired', 'applicationLaunched', 'boatAccess')),
                'Native previous-generation restore checks incomplete or wrong host')


def validate_reports(original, fresh, q, frozen, package, restart, *, nav_reports,
                     report_set, manual_checks, passive_checks, updater_checks):
    require(set(original) == set(ORIGINAL_REPORTS) and set(fresh) == report_set,
            'Full retest report coverage incomplete')
    validate_restart(restart)
    require(frozen['sourceArchiveSha256'] == SOURCE_SHA and frozen['manifestSha256'] == MANIFEST_SHA,
            'Restored frozen input identity differs')
    require(isinstance(package, dict) and set(package) == {'build', 'files'}, 'Frozen package binding required')
    product = package['build']; commit = PRODUCER['commit']
    require(product.get('commit') == commit and product.get('test_fixtures') is False and
            product.get('build_purpose') == 'INSTALLED PRODUCT' and
            product.get('xnav_hardware_output_policy') == 'manual-commissioning' and
            type(product.get('xnav_manual_control_contract')) is int and product['xnav_manual_control_contract'] == 1 and
            type(package['files']) is int and package['files'] > 0, 'Frozen product metadata differs')
    _restored(original['staging-inputs.json'], commit)
    _restored(fresh['staging-inputs.json'], HARNESS)
    failed = original['objects-input-results.json']
    require('result' not in failed and failed.get('native_waypoint_name') == [] and
            failed.get('object_contract') == original['objects-input-profile/objects-fixture-results.json'] and
            failed['object_contract']['result'] == 'passed' and failed['object_contract']['phase'] == 'done',
            'Original failure no longer matches the reviewed post-save boundary')
    result = fresh['staging-retest.json']
    require(result['status'] == 'passed' and result['scope'] == 'all' and result['productCommit'] == commit and
            result['producer'] == fresh['staging-inputs.json']['producer'] and result['harnessCommit'] == HARNESS and
            result['archiveSha256'] == BUILD['archiveSha256'] and result['publicAccess'] is False and
            result['releaseQualification'] is False and result['checks'] == [dict(name='all', status='passed')],
            'Fresh evidence is not the unchanged complete native qualifier')
    feedback = fresh['boat-feedback-windows/result.json']
    require(feedback['passed'] is True and feedback['platform'] == 'win32' and
            feedback['source_commit'] == commit and feedback['harness_commit'] == HARNESS,
            'Fresh component gate identity failed')
    tests = feedback['tests']; binaries = {b['name']: b['sha256'] for b in frozen['boatFeedback']['binaries']}
    require(len(tests) == len(inputs.FEEDBACK_TESTS) and {t['name'] for t in tests} == inputs.FEEDBACK_TESTS and
            all(t['passed'] is True and type(t['exit_code']) is int and t['exit_code'] == 0 and
                t['timed_out'] is False and t['binary_sha256'] == binaries[t['name']] for t in tests) and
            feedback['manifest_sha256'] == frozen['boatFeedback']['manifestSha256'], 'Fresh component checks/binaries differ')
    loader = fresh['installer-loader-selftest.json']
    require(type(loader['checks']) is int and loader['checks'] == 5 and loader['status'] == 'passed' and
            loader['loader']['passed'] is True and loader['loader']['profile_initialized'] is False and
            loader['loader']['plugins_loaded'] is False and loader['loader']['commit'] == commit and
            loader['loader']['test_fixtures'] is True and loader['loader']['build_purpose'] == 'DEVELOPER TEST BUILD' and
            loader['loader']['upstream'] == manifest.UPSTREAM and
            loader['loader']['xnav_hardware_output_policy'] == 'test-loopback-only', 'Fresh loader gate differs')
    mode = fresh['mode-cycle-results.json']
    require(mode['result'] == 'interaction and fixture persistence passed; visual review required' and
            len(mode['steps']) == 6 and len(mode['peer_boundary']) == 5 and
            all(s.get('interaction', s.get('persistence')) == 'pass' for s in mode['steps']) and
            all(s['profile_preserved'] is True and s['owned_tcp_listener_ports'] == [] for s in mode['peer_boundary']) and
            len(mode['chart_rendering']) == 6 and all(s['coastline_visible'] is True for s in mode['chart_rendering']),
            'Fresh mode gate incomplete')
    for name in nav_reports:
        r = fresh[name + '-input-results.json']
        require(r['result'] == 'loopback transport and lifecycle passed; numeric and stale screenshot review required' and
                r['transport_errors'] == [] and r['visual_review'] == 'not requested', 'Fresh navigation gate differs')
    for name in ('route', 'route-standard'):
        r = fresh[name + '-input-results.json']['route_contract']
        _checks(r, 'passed', 26, 'Route'); require(r['phase'] == 'done', 'Route lifecycle incomplete')
    objects = fresh['objects-input-results.json']; r = objects['object_contract']
    _checks(r, 'passed', 28, 'Objects')
    require(r['phase'] == 'done' and r['late_connection_added_after_deferred'] is True and
            r == fresh['objects-input-profile/objects-fixture-results.json'], 'Object lifecycle reports differ')
    _native_names(objects)
    for name, result, count in (
        ('recording-results.json', 'passed; screenshot review required', 6),
        ('pilot-results.json', 'passed; screenshot review required', 8),
        ('recovery-results.json', 'passed; native screenshot review required', 5),
        ('user-flows-results.json', 'passed; native screenshots require review', 9),
        ('pilot-status-only/pilot-results.json', 'passed; screenshot review required', 5)):
        _checks(fresh[name], result, count, name)
    require(fresh['pilot-results.json']['checks'] == manual_checks[:6] + MANUAL_PILOT_TAIL and
            fresh['pilot-status-only/pilot-results.json']['checks'] == passive_checks and
            fresh['pilot-results.json']['transport_failures'] == [] and
            fresh['user-flows-results.json']['transport_errors'] == [], 'Fresh pilot/user-flow assertions differ')
    signal = fresh['signalk-results.json']
    require(signal['status'] == 'passed' and signal['build_commit'] == commit and
            signal['transport_errors'] == [] and len(signal['checks']) == 5, 'Fresh Signal K gate differs')
    pilot = fresh['pilot-status-only/pilot-results.json']
    require(pilot['sent'] == pilot['wire_output'] == pilot['transport_failures'] == [] and
            type(pilot['received_bytes']) is int and pilot['received_bytes'] == 0, 'Installed product emitted bytes')
    diag = fresh['pilot-status-only/pilot-opennav-diagnostics.json']; passive = diag['runtime']['pilot']
    require(diag['build_commit'] == commit and diag['test_fixtures'] is False and
            diag['build_purpose'] == 'INSTALLED PRODUCT' and diag['xnav_hardware_output_policy'] == 'manual-commissioning' and
            type(diag['xnav_manual_control_contract']) is int and diag['xnav_manual_control_contract'] == 1 and
            diag['runtime']['test_fixtures'] is False and passive['configured_permission'] is True and
            all(passive[k] is False for k in ('output_unavailable', 'enabled', 'serial_session_enabled',
                'control_capability', 'simulated', 'track_capability', 'wind_capability')) and
            passive['command_id'] == '0' and passive['command_state'] == 'None' and passive['fresh'] is True and
            passive['mode'] == 'STANDBY', 'Fresh installed pilot gate was not passive/fresh')
    recovery = fresh['production-recovery-results.json']
    require(recovery['status'] == 'passed' and len(recovery['checks']) == 7 and recovery['build'] == product and
            recovery['design_validation'] == 'not requested' and recovery['files_verified'] == package['files'] and
            len(recovery['chart_rendering']) == 10 and all(s['coastline_visible'] is True for s in recovery['chart_rendering']),
            'Fresh recovery does not bind the frozen product/inventory')
    installer = fresh['installer-staging.json']; updater = fresh['packaged-updater.json']
    require(installer['status'] == 'passed' and installer['mode'] == 'staging' and
            installer['product_commit'] == commit and installer['harness_commit'] == HARNESS and
            installer.get('packaged_updater') == updater and
            fresh['charts-results.json']['result'] == 'passed; native screenshot review and physical GPU gate remain separate',
            'Fresh installer/charts gates differ')
    require(updater['status'] == 'passed' and updater['commit'] == commit and updater['checks'] == updater_checks and
            updater['setupSha256'] == installer['setup_sha256'] and updater['executableSha256'] == product['executable_sha256'] and
            updater['faultInjection']['restored'] is True, 'Fresh packaged updater did not complete all mechanics')
    peer = fresh['peer-two-instances-results.json']
    require(peer['result'] == 'PASS' and peer['authority'] == 'native Windows CI' and
            peer['test_source_commit'] == HARNESS and peer['product_commit'] == commit and
            len(peer['profiles']) == len(peer['observations']) == 2 and not peer.get('finalization_errors') and
            all(p['build_commit'] == commit and p['cert_preserved_at_exit'] is True and
                p['key_preserved_at_exit'] is True for p in peer['profiles']), 'Fresh peer containment gate differs')
    require(peer['test_script_sha256'] == 'bf73bbfc80c2e33a2f1711831ff354bc543a5b2d85254a4a2c2c62cdb896f2d3',
            'Peer report used another source helper')
    for index, (profile, observed) in enumerate(zip(peer['profiles'], peer['observations']), 1):
        require(profile['index'] == index and profile['launch_mode'] == ('ordinary' if index == 1 else 'portable') and
                profile['first_start_setup']['status'] == 'deferred-with-Later' and
                profile['first_start_setup']['after_ticks'] > profile['first_start_setup']['before_ticks'] and
                observed['profile_preserved'] is True and observed['owned_tcp_listener_ports'] == [] and
                type(profile['pid']) is int and profile['pid'] > 0 and
                observed['pid'] == observed['startup_pid'] == profile['pid'], 'Peer containment observation differs')
    _signed_reports(fresh)


def bind_runtime_bytes(tree, raw, reports):
    receipt_sha = hashlib.sha256(raw['staging-inputs.json']).hexdigest()
    require(reports['boat-feedback-windows/result.json']['restore_receipt_sha256'] == receipt_sha and
            reports['peer-two-instances-results.json']['restore_receipt_sha256'] == receipt_sha,
            'Fresh components/peer do not bind their actual restoration')
    fixture_sha = inputs.sha(tree / 'build/xnav-install/opencpn.exe')
    product_sha = inputs.sha(tree / 'build/production-install/opencpn.exe')
    setup_sha = inputs.sha(tree / 'build/beta-installer/SKAGER-Beta2-Setup.exe')
    require(reports['installer-loader-selftest.json']['app_sha256'] == fixture_sha and
            reports['peer-two-instances-results.json']['executable_sha256'] == fixture_sha and
            reports['production-recovery-results.json']['build']['executable_sha256'] == product_sha and
            reports['packaged-updater.json']['executableSha256'] == product_sha and
            reports['installer-staging.json']['setup_sha256'] == setup_sha and
            reports['packaged-updater.json']['setupSha256'] == setup_sha and
            reports['production-recovery-results.json']['package_sha256'] ==
                inputs.sha(tree / 'build/developer-preview/SKAGER-Beta2-Portable-Recovery.zip'),
            'Fresh native gates tested different retained binary/package bytes')

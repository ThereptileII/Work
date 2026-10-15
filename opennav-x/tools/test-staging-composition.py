#!/usr/bin/env python3
"""Focused offline composition trust tests. No network, builds or applications."""
import base64
import copy
import hashlib
import io
import json
import os
from pathlib import Path
import stat
import tempfile
import unittest
from unittest.mock import patch
import zipfile

import staging_composition as c

PRODUCT = '1' * 40
HARNESS = '2' * 40
COMPOSER = '3' * 40


def fixture():
    p = dict(commit=PRODUCT, runId='100', runAttempt='1')
    r = dict(commit=HARNESS, runId='200', runAttempt='2', jobId='201', artifactId='202',
             artifactName=f'staging-retest-evidence-100-attempt1-harness{HARNESS}-run200-attempt2',
             artifactDigest='sha256:' + 'b' * 64,
             reports=[dict(path=n, sha256='a' * 64) for n in sorted(c.RETEST_REPORTS)])
    request = dict(schema=1, repository=c.REPOSITORY, producer=p,
                   build=dict(artifactId='101', artifactName=f'staging-build-{PRODUCT}-run100-attempt1',
                              artifactDigest='sha256:' + 'c' * 64, archiveSha256='d' * 64),
                   original=dict(jobId='102', artifactId='103',
                                 artifactName=f'windows-qualification-{PRODUCT}-attempt1',
                                 artifactDigest='sha256:' + 'e' * 64,
                                 prefixReports=[dict(path=n, sha256='a' * 64) for n in sorted(c.PREFIX_REPORTS)]), retest=r)
    raw = json.dumps(request).encode()
    execution = dict(commit=COMPOSER, runId='300', runAttempt='3')
    q = c.qualification(request, c.digest(raw), execution)
    record = dict(p, channel='staging')
    support = dict(artifactName=f'staging-retest-{PRODUCT}-attempt3', runId='300', runAttempt='3',
                   commit=PRODUCT, archiveName='SKAGER-Beta2-Retest-Support.zip', sha256='f' * 64, size=123)
    return request, raw, q, record, support


def report_fixture(q):
    p = q['composition']['producer']
    frozen = dict(status='restored', qualification='not-run', producer=c.inputs.producer(PRODUCT, p['runId'], p['runAttempt']),
                  harnessCommit=PRODUCT, archiveSha256='d'*64, sourceArchiveSha256='5'*64,
                  manifestSha256='6'*64, boatFeedback=dict(manifestSha256='7'*64,
                    binaries=[dict(name=n, sha256='8'*64) for n in sorted(c.inputs.FEEDBACK_TESTS)]))
    original = {'staging-inputs.json': copy.deepcopy(frozen)}
    original['boat-feedback-windows/result.json'] = dict(passed=True, platform='win32', source_commit=PRODUCT,
        harness_commit=PRODUCT, manifest_sha256='7'*64,
        tests=[dict(name=n, passed=True, exit_code=0, timed_out=False, binary_sha256='8'*64) for n in sorted(c.inputs.FEEDBACK_TESTS)])
    original['installer-loader-selftest.json'] = dict(status='passed', checks=5, loader=dict(passed=True,
        profile_initialized=False, plugins_loaded=False, commit=PRODUCT, test_fixtures=True,
        build_purpose='DEVELOPER TEST BUILD', upstream=c.manifest.UPSTREAM, xnav_hardware_output_policy='test-loopback-only'))
    original['mode-cycle-results.json'] = dict(result='interaction and fixture persistence passed; visual review required',
        steps=[dict(interaction='pass')]*6, peer_boundary=[dict(profile_preserved=True, owned_tcp_listener_ports=[])]*5,
        chart_rendering=[dict(coastline_visible=True)]*6)
    for name in c.NAV_REPORTS:
        original[name+'-input-results.json'] = dict(result='loopback transport and lifecycle passed; numeric and stale screenshot review required',
            transport_errors=[], visual_review='not requested')
    for name in ('route', 'route-standard'):
        original[name+'-input-results.json']['route_contract'] = dict(result='passed', phase='done', checks=[{}]*26)
    original['objects-input-results.json']['object_contract'] = dict(result='passed', phase='done', checks=[{}]*28,
                                                                 late_connection_added_after_deferred=True)
    original['signalk-results.json'] = dict(status='passed', build_commit=PRODUCT, transport_errors=[], checks=['x']*5)
    for name, result, count in [('recording-results.json','passed; screenshot review required',6),
                               ('pilot-results.json','passed; screenshot review required',6),
                               ('recovery-results.json','passed; native screenshot review required',5),
                               ('user-flows-results.json','passed; native screenshots require review',8),
                               ('pilot-status-only/pilot-results.json','passed; screenshot review required',5)]:
        original[name] = dict(result=result, checks=['x']*count)
    original['pilot-results.json']['transport_failures'] = []
    original['user-flows-results.json']['transport_errors'] = []
    original['pilot-status-only/pilot-results.json'].update(sent=[], wire_output=[], received_bytes=0, transport_failures=[])
    original['pilot-status-only/pilot-opennav-diagnostics.json'] = dict(build_commit=PRODUCT, test_fixtures=False,
        build_purpose='INSTALLED PRODUCT', xnav_hardware_output_policy='status-only', runtime=dict(pilot=dict(
            output_unavailable=True, control_capability=False, enabled=False, command_id='0', command_state='None',
            fresh=True, mode='STANDBY')))
    original['production-recovery-results.json'] = dict(status='passed', checks=['x']*7, build=dict(commit=PRODUCT,
        test_fixtures=False, build_purpose='INSTALLED PRODUCT', xnav_hardware_output_policy='status-only',
        executable_sha256='9'*64), design_validation='not requested', files_verified=1019,
        chart_rendering=[dict(coastline_visible=True)]*10)
    original['installer-staging.json'] = dict(status='failed', error=c.FAILURE)
    original['packaged-updater.json'] = dict(status='failed', error=c.FAILURE, commit=PRODUCT,
        checks=c.UPDATER_CHECKS[:3], faultInjection=dict(restored=True))
    retest = {'staging-inputs.json': dict(frozen, harnessCommit=HARNESS),
        'staging-retest.json': dict(status='passed', scope='installer-charts', productCommit=PRODUCT,
            producer=frozen['producer'], harnessCommit=HARNESS, archiveSha256='d'*64, publicAccess=False,
            releaseQualification=False, checks=[dict(name=n,status='passed') for n in ('installer','charts')]),
        'installer-staging.json': dict(status='passed', mode='staging', product_commit=PRODUCT, harness_commit=HARNESS, setup_sha256='0'*64),
        'charts-results.json': dict(result='passed; native screenshot review and physical GPU gate remain separate'),
        'packaged-updater.json': dict(status='passed', commit=PRODUCT, checks=c.UPDATER_CHECKS,
            setupSha256='0'*64, executableSha256='9'*64, faultInjection=dict(restored=True))}
    return original, retest, frozen


def full_fixture():
    request, _, old_q, record, support = fixture()
    request['schema'] = 2
    request['original']['prefixReports'] = [dict(path=n, sha256='a'*64) for n in sorted(c.ALL_ORIGINAL)]
    request['retest']['reports'] = [dict(path=n, sha256='a'*64) for n in sorted(c.ALL_REPORTS)]
    request['helperClosure'] = dict(producerTree='a'*40, harnessTree='b'*40,
        edits=[dict(path=n, before=f'{i+1:040x}', after=f'{i+11:040x}') for i,n in enumerate(sorted(c.ALL_EDITS))])
    request['restart'] = dict(artifactId='104', artifactName=f'commissioning-restart-qualified-{PRODUCT}-attempt1',
                             artifactDigest='sha256:'+'e'*64, reports=[dict(path='qualified.json',sha256='a'*64)])
    raw = json.dumps(request).encode()
    q = c.qualification(request, c.digest(raw), old_q['composition']['execution'])
    return request, raw, q, record, support


def full_reports(q):
    old, suffix, frozen = report_fixture(q)
    reports = copy.deepcopy(old); reports.update(suffix)
    original = {n: copy.deepcopy(old[n]) for n in c.ALL_ORIGINAL}
    del original['objects-input-results.json']['result']
    reports['staging-retest.json'].update(scope='all',checks=[dict(name='all',status='passed')])
    feedback = reports['boat-feedback-windows/result.json']
    feedback.update(harness_commit=HARNESS, source_dirty=None, harness_dirty=False, restore_receipt_sha256='a'*64)
    reports['installer-loader-selftest.json']['app_sha256'] = '1'*64
    reports['objects-input-results.json']['native_edit_confirmation'] = (
        'Themed property sheet saves and refreshes; delete cancellation preserves mark')
    reports['pilot-results.json']['checks'] = ['x']*7
    sources = {'opennav-x/tools/boat/'+n: 'a'*64 for n in c.CUSTODY_SOURCES}
    sources.update({p:'a'*64 for p in ('opennav-x/tools/smoke-peer-two-instances.py',
                    'opennav-x/installer/windows/Lifecycle.ps1','opennav-x/tools/test-installer-missing-dll-selftest.ps1')})
    product = dict(commit=PRODUCT,test_fixtures=False,build_purpose='INSTALLED PRODUCT',
                   xnav_hardware_output_policy='manual-commissioning',xnav_manual_control_contract=1,
                   executable_sha256='9'*64)
    frozen['runtime'] = dict(product=product, recoveryFiles=37, sourceHashes=sources,
                            hashes=dict(fixture='1'*64,product='9'*64,setup='0'*64,recovery='2'*64))
    reports['peer-two-instances-results.json'] = dict(result='PASS',authority='native Windows CI',
        product_commit=PRODUCT,test_source_commit=HARNESS,executable_sha256='1'*64,
        restore_receipt_sha256='a'*64,test_script_sha256='a'*64,
        profiles=[dict(build_commit=PRODUCT,pid=p,profile=f'D:/fixture/profile-{p}',launch_mode=mode,
                       cert_preserved_at_exit=True,key_preserved_at_exit=True)
                  for p,mode in ((10,'ordinary'),(11,'portable'))],observations=[dict(pid=p,startup_pid=p,
            profile_preserved=True,owned_tcp_listener_ports=[]) for p in (10,11)])
    diag = reports['pilot-status-only/pilot-opennav-diagnostics.json']
    diag.update(xnav_hardware_output_policy='manual-commissioning',xnav_manual_control_contract=1)
    diag['runtime']['test_fixtures'] = False
    diag['runtime']['pilot'].update(configured_permission=True,output_unavailable=False,
        serial_session_enabled=False,simulated=False,track_capability=False,wind_capability=False)
    reports['production-recovery-results.json'].update(build=product,harness_commit=HARNESS,
                                                       package_sha256='2'*64,files_verified=37)
    reports['installer-staging.json'].update(source_archive_sha256=frozen['sourceArchiveSha256'],
        compiled_input_receipt_sha256='a'*64,compiled_input_producer=frozen['producer'],
        installer_source_sha256='a'*64,installer_helper_sha256='a'*64)
    for arch,bits in (('x64',64),('x86',32)):
        reports[f'signed-update-custody/{arch}.log'] = '\n'.join([
            'Signed update session: 61 pure contract checks passed; no launch permission or native qualification claimed.',
            'Signed update operations: 129 pure checks passed; no broker, mutation or launch authority.',
            'Signed update store: 245 inert checks passed; nativeWindows=True. No launch authority or resumable recovery claimed.',
            json.dumps(dict(status='passed',count=42,native=True,scope='inert installed barrier and private store only',
                            applicationLaunched=False,profileAccess=False))])+'\n'
        reports[f'signed-update-custody/{arch}-previous-restore.log'] = json.dumps(dict(status='passed',count=106,
            checks=['x']*106,processBits=bits,powerShellVersion='5.1.0',
            sourceSha256={n:'a'*64 for n in c.CUSTODY_SOURCES},continuousNamespaceCustodyAccepted=False,
            installedProductAccepted=False,productWired=False,applicationLaunched=False,boatAccess=False))
    return original,reports,frozen


class FakeGitHub:
    repo = c.REPOSITORY
    base = 'repos/' + c.REPOSITORY

    def __init__(self, request, raw, q):
        self.data = {}
        identities = [(request['producer'], c.BASELINE, 'failure'),
                      (request['retest'], c.RETEST, 'success'),
                      (q['composition']['execution'], c.WORKFLOW, 'success')]
        for identity, workflow, result in identities:
            prefix = f"{self.base}/actions/runs/{identity['runId']}/attempts/{identity['runAttempt']}"
            self.data[prefix] = dict(id=int(identity['runId']), run_attempt=int(identity['runAttempt']),
                                    head_sha=identity['commit'], path=workflow, status='completed', conclusion=result,
                                    head_repository=dict(full_name=self.repo), event='push', head_branch='skager-staging-compose')
            if workflow == c.BASELINE:
                jobs = [(name, 'success', '1') for name in (c.ALL_JOBS if request['schema']==2 else c.REQUIRED_JOBS)] + [(c.RUNTIME_JOB, 'failure', '102')]
            elif workflow == c.RETEST:
                jobs = [('retest', 'success', '201')]
            else:
                jobs = [(c.ASSEMBLE_JOB, 'success', '301')]
            self.data[prefix + '/jobs'] = [dict(name=name, status='completed', conclusion=result, id=int(jobid),
                                               run_id=int(identity['runId']), run_attempt=int(identity['runAttempt']),
                                               head_sha=identity['commit']) for name, result, jobid in jobs]
        for pin, identity in ((request['build'], request['producer']), (request['original'], request['producer']),
                              (request['retest'], request['retest'])):
            self.data[f"{self.base}/actions/artifacts/{pin['artifactId']}"] = dict(
                id=int(pin['artifactId']), name=pin['artifactName'], digest=pin['artifactDigest'], expired=False,
                workflow_run=dict(id=int(identity['runId']), head_sha=identity['commit']), size_in_bytes=100)
        self.data[f'{self.base}/contents/{c.REQUEST_PATH}?ref={COMPOSER}'] = dict(
            type='file', encoding='base64', size=len(raw), content=base64.b64encode(raw).decode())
        if request['schema'] == 2:
            pin=request['restart']
            self.data[f"{self.base}/actions/artifacts/{pin['artifactId']}"] = dict(id=int(pin['artifactId']),
                name=pin['artifactName'],digest=pin['artifactDigest'],expired=False,
                workflow_run=dict(id=100,head_sha=PRODUCT),size_in_bytes=100)
            common=[]
            for path in c.ALL_RECIPE:
                source=Path(c.__file__).parents[2]/path; data=source.read_bytes()
                blob=hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()
                common.append(dict(path=path,type='blob',mode='100644',sha=blob))
                self.data[f'{self.base}/git/blobs/{blob}']=dict(sha=blob,encoding='base64',size=len(data),
                                                              content=base64.b64encode(data).decode())
            for commit,tree_key,side in ((PRODUCT,'producerTree','before'),(HARNESS,'harnessTree','after')):
                tree=request['helperClosure'][tree_key]
                entries=copy.deepcopy(common)+[dict(path=e['path'],type='blob',mode='100644',sha=e[side])
                                               for e in request['helperClosure']['edits']]
                self.data[f'{self.base}/git/commits/{commit}']=dict(sha=commit,tree=dict(sha=tree))
                self.data[f'{self.base}/git/trees/{tree}?recursive=1']=dict(sha=tree,truncated=False,tree=entries)

    def api(self, endpoint):
        return self.data[endpoint]

    def pages(self, endpoint):
        return self.data[endpoint]


class CompositionTests(unittest.TestCase):
    def setUp(self):
        self.request, self.raw, self.q, self.record, self.support = fixture()
        self.gh = FakeGitHub(self.request, self.raw, self.q)

    def test_original_failed_run_and_distinct_completed_composition_are_valid(self):
        c.validate_qualification(self.q, self.record, self.support)
        c.verify_provenance(self.gh, self.q, complete=True)
        self.assertEqual(self.gh.data[self.gh.base + '/actions/runs/100/attempts/1']['conclusion'], 'failure')

    def test_only_current_composition_can_publish_after_assembly(self):
        env = dict(GITHUB_REPOSITORY=c.REPOSITORY, GITHUB_SHA=COMPOSER, GITHUB_RUN_ID='300',
                   GITHUB_RUN_ATTEMPT='3', GITHUB_REF='refs/heads/skager-staging-compose')
        run = self.gh.data[self.gh.base + '/actions/runs/300/attempts/3']
        run.update(status='in_progress', conclusion=None)
        with patch.dict(os.environ, env):
            c.verify_provenance(self.gh, self.q)
            with self.assertRaises(ValueError): c.verify_provenance(self.gh, self.q, complete=True)
            for key, wrong in [('GITHUB_SHA', PRODUCT), ('GITHUB_RUN_ID', '100'), ('GITHUB_REF', 'refs/heads/main')]:
                with patch.dict(os.environ, {key: wrong}), self.assertRaises(ValueError):
                    c.verify_provenance(self.gh, self.q)

    def test_no_failed_or_missing_gate_can_be_substituted(self):
        base = self.gh.base + '/actions/runs/100/attempts/1/jobs'
        for index in range(len(c.REQUIRED_JOBS)):
            gh = copy.deepcopy(self.gh); gh.data[base][index]['conclusion'] = 'failure'
            with self.subTest(index=index), self.assertRaises(ValueError): c.verify_provenance(gh, self.q, True)
        for run, attempt in [('200', '2'), ('300', '3')]:
            gh = copy.deepcopy(self.gh); gh.data[f'{gh.base}/actions/runs/{run}/attempts/{attempt}/jobs'] = []
            with self.assertRaises(ValueError): c.verify_provenance(gh, self.q, True)
        gh = copy.deepcopy(self.gh); gh.data[base][-1]['conclusion'] = 'success'
        with self.assertRaises(ValueError): c.verify_provenance(gh, self.q, True)

    def test_request_hash_and_artifact_provenance_cannot_be_rewritten(self):
        for section, key, bad in [('build', 'artifactDigest', 'sha256:' + '9'*64),
                                  ('original', 'jobId', '888'), ('retest', 'commit', '9'*40)]:
            q = copy.deepcopy(self.q); q['composition'][section][key] = bad
            with self.assertRaises(ValueError): c.verify_provenance(self.gh, q, True)
        for field, bad in [('head_sha', HARNESS), ('path', c.RETEST), ('run_attempt', 9)]:
            gh = copy.deepcopy(self.gh); gh.data[gh.base + '/actions/runs/100/attempts/1'][field] = bad
            with self.assertRaises(ValueError): c.verify_provenance(gh, self.q, True)
        gh = copy.deepcopy(self.gh)
        gh.data[gh.base + '/actions/artifacts/103']['workflow_run']['id'] = 999
        with self.assertRaises(ValueError): c.verify_provenance(gh, self.q, True)

    def test_expired_input_metadata_can_verify_release_but_not_download(self):
        pin = self.request['original']; a = self.gh.data[self.gh.base + '/actions/artifacts/103']; a['expired'] = True
        c.verify_provenance(self.gh, self.q, complete=True)
        with self.assertRaises(ValueError): c._artifact(self.gh, pin, self.request['producer'], downloadable=True)

    def test_support_origin_is_composition_not_original_or_retest(self):
        for changes in [dict(runId='100'), dict(runId='200'), dict(runAttempt='1'), dict(commit=COMPOSER),
                        dict(artifactName=f'staging-retest-{PRODUCT}-attempt1')]:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                c.validate_qualification(self.q, self.record, dict(self.support, **changes))
        for path in ('prefixReports',):
            request = copy.deepcopy(self.request); request['original'][path].pop()
            with self.assertRaises(ValueError): c.validate_request(request)
        request = copy.deepcopy(self.request); request['retest']['reports'][0]['path'] = '../evil.json'
        with self.assertRaises(ValueError): c.validate_request(request)

    def test_gate_policy_and_source_identity_are_not_relabeled(self):
        for field, value in [('schemaVersion', True), ('publicAccess', True), ('commit', COMPOSER),
                             ('runId', '300'), ('designReview', 'requested'), ('endurance', 'passed')]:
            q = copy.deepcopy(self.q); q[field] = value
            with self.subTest(field=field), self.assertRaises(ValueError): c.validate_qualification(q, self.record, self.support)
        q = copy.deepcopy(self.q); q['gates']['installer'] = 'failed'
        with self.assertRaises(ValueError): c.validate_qualification(q, self.record, self.support)

    def test_archive_paths_aliases_links_and_collisions_refused_before_read(self):
        for malicious in ['../out', 'C:/out', 'safe\\out', 'SAFE.json', 'CON.json', 'safe.json/child']:
            with tempfile.TemporaryDirectory() as tmp:
                archive = Path(tmp)/'in.zip'
                with zipfile.ZipFile(archive, 'w') as z:
                    z.writestr('safe.json', '{}'); z.writestr(malicious, 'evil')
                with self.subTest(malicious=malicious), self.assertRaises(ValueError):
                    c.selected_zip(archive, ['safe.json'])
        for mode in (stat.S_IFLNK, stat.S_IFIFO):
            archive = io.BytesIO()
            with zipfile.ZipFile(archive, 'w') as z:
                info = zipfile.ZipInfo('safe.json'); info.external_attr = (mode | 0o644) << 16
                z.writestr(info, '{}')
            archive.seek(0)
            with self.assertRaises(ValueError): c.selected_zip(archive, ['safe.json'])

    def test_source_only_opaque_link_is_hash_bound_and_never_selected(self):
        name = 'OpenCPN-5.12.4-integrated/data/opaque-link'
        target = b'../../outside-source'

        def source_zip(*, text=target, record=None, member=name, mode=stat.S_IFLNK, extra=None):
            archive = io.BytesIO()
            if record is None: record = dict(gitMode='120000', sha256=c.digest(target))
            with zipfile.ZipFile(archive, 'w') as z:
                z.writestr('safe.json', '{"frozen":true}')
                z.writestr('SOURCE_REFERENCE.json', json.dumps(dict(files={member:record})))
                info = zipfile.ZipInfo(member); info.external_attr = (mode | 0o777) << 16
                z.writestr(info, text)
                if extra: z.writestr(extra, 'unexpected')
            archive.seek(0)
            return archive

        wanted = ['SOURCE_REFERENCE.json', 'safe.json']
        result = c.selected_source_zip(source_zip(), wanted)
        self.assertEqual(set(result), set(wanted))
        self.assertEqual(result['safe.json'], b'{"frozen":true}')
        # The exact same link is still forbidden at the ordinary evidence boundary.
        with self.assertRaisesRegex(ValueError, 'Linked, special or oversized ZIP member:.*opaque-link'):
            c.selected_zip(source_zip(), wanted)
        with self.assertRaisesRegex(ValueError, 'Source symlink cannot be selected:.*opaque-link'):
            c.selected_source_zip(source_zip(), wanted+[name])
        for overrides in [dict(text=b'changed target'), dict(record=dict(gitMode='100644',sha256=c.digest(target))),
                          dict(record={}), dict(text=b'x'*4097), dict(mode=stat.S_IFIFO),
                          dict(member='../escape'), dict(extra=name+'/child'), dict(extra=name.upper())]:
            with self.subTest(overrides=overrides), self.assertRaises(ValueError):
                c.selected_source_zip(source_zip(**overrides), wanted)

    def test_report_pins_are_checked_before_json_semantics(self):
        with tempfile.TemporaryDirectory() as tmp:
            archive = Path(tmp)/'reports.zip'
            with zipfile.ZipFile(archive, 'w') as z: z.writestr('safe.json', '{"status":"passed"}')
            with self.assertRaises(ValueError): c._reports(archive, [dict(path='safe.json', sha256='0'*64)])
            raw, reports = c._reports(archive, [dict(path='safe.json', sha256=c.digest(b'{"status":"passed"}'))])
            self.assertEqual(reports['safe.json']['status'], 'passed')

    def test_fixed_semantics_do_not_treat_reviewed_hashes_as_passes(self):
        original, retest, frozen = report_fixture(self.q)
        c.validate_reports(original, retest, self.q, frozen)
        mutations = [
            ('boat-feedback-windows/result.json', ('tests',0,'passed'), False),
            ('boat-feedback-windows/result.json', ('tests',0,'binary_sha256'), '0'*64),
            ('installer-loader-selftest.json', ('loader','profile_initialized'), True),
            ('mode-cycle-results.json', ('steps',0,'interaction'), 'fail'),
            ('route-input-results.json', ('route_contract','checks'), [{}]*25),
            ('objects-input-results.json', ('object_contract','late_connection_added_after_deferred'), False),
            ('signalk-results.json', ('transport_errors',), ['error']),
            ('pilot-status-only/pilot-results.json', ('wire_output',), ['unexpected']),
            ('pilot-status-only/pilot-opennav-diagnostics.json', ('runtime','pilot','control_capability'), True),
            ('production-recovery-results.json', ('build','test_fixtures'), True),
            ('production-recovery-results.json', ('chart_rendering',0,'coastline_visible'), False),
            ('installer-staging.json', ('status',), 'passed'),
            ('packaged-updater.json', ('error',), 'some different failure'),
        ]
        for name, path, bad in mutations:
            changed = copy.deepcopy(original); target = changed[name]
            for key in path[:-1]: target = target[key]
            target[path[-1]] = bad
            with self.subTest(name=name, path=path), self.assertRaises(ValueError):
                c.validate_reports(changed, retest, self.q, frozen)
        for name, key, bad in [('staging-retest.json','scope','installer'),
                               ('staging-retest.json','harnessCommit',PRODUCT),
                               ('installer-staging.json','status','failed'),
                               ('charts-results.json','result','not run'),
                               ('packaged-updater.json','checks',c.UPDATER_CHECKS[:3])]:
            changed = copy.deepcopy(retest); changed[name][key] = bad
            with self.subTest(name=name, key=key), self.assertRaises(ValueError):
                c.validate_reports(original, changed, self.q, frozen)

    def test_frozen_assembler_preserves_payloads_and_uses_frozen_notes_without_tokens(self):
        with tempfile.TemporaryDirectory() as tmp:
            tree = Path(tmp)
            preview = tree/'build/developer-preview'; preview.mkdir(parents=True)
            setup = tree/'build/beta-installer/SKAGER-Beta2-Setup.exe'; setup.parent.mkdir(); setup.write_bytes(b'unchanged inert Setup')
            recovery = preview/'SKAGER-Beta2-Portable-Recovery.zip'; recovery.write_bytes(b'unchanged inert recovery')
            product = tree/c.inputs.PACKAGE_ROOT/'docs/PRODUCT_BUILD.json'; product.parent.mkdir(parents=True)
            product.write_text(json.dumps(dict(commit=PRODUCT, test_fixtures=False, build_purpose='INSTALLED PRODUCT',
                                               xnav_hardware_output_policy='status-only')))
            tools = Path(c.__file__).parent
            members = {'opennav-x/tools/alpha-artifacts.py': (tools/'alpha-artifacts.py').read_bytes(),
                       'opennav-x/tools/hardware_output_policy.py': (tools/'hardware_output_policy.py').read_bytes(),
                       'opennav-x/release/qualification.json': b'{"enduranceEnabled":false}'}
            for name in c.manifest.PRODUCT_FILES:
                if name.endswith('.md'): members['opennav-x/docs/beta2/'+name] = ('FROZEN PRODUCT '+name).encode()
            source = preview/'SKAGER-Beta2-source.zip'
            references = dict(productCommit=PRODUCT, files={n:dict(sha256=c.digest(b)) for n,b in members.items()})
            link_name = 'OpenCPN-5.12.4-integrated/data/frozen-link'
            link_text = b'../../unselected-and-never-followed'
            references['files'][link_name] = dict(gitMode='120000', sha256=c.digest(link_text))
            with zipfile.ZipFile(source,'w') as z:
                for name, data in members.items(): z.writestr(name,data)
                z.writestr('SOURCE_REFERENCE.json',json.dumps(references))
                link = zipfile.ZipInfo(link_name); link.external_attr = (stat.S_IFLNK | 0o777) << 16
                z.writestr(link, link_text)
            before = {p.name:p.read_bytes() for p in (source,setup,recovery)}
            old_raw = {'production-recovery-results.json':json.dumps(dict(status='passed',package_sha256=c.inputs.sha(recovery))).encode()}
            new_raw = {'installer-staging.json':json.dumps(dict(status='passed',setup_sha256=c.inputs.sha(setup))).encode()}
            real_run = c.subprocess.run
            captured = []
            def run(command, **kwargs):
                captured.append(kwargs['env'])
                return real_run(command, **kwargs, stdout=c.subprocess.PIPE)
            with patch.dict(os.environ, dict(GH_TOKEN='must-not-cross', GITHUB_TOKEN='must-not-cross', GITHUB_SHA=COMPOSER)), \
                    patch.object(c.subprocess,'run',side_effect=run):
                c.assemble_frozen_payloads(tree, old_raw, new_raw, PRODUCT)
                self.assertEqual(os.environ['GITHUB_SHA'],COMPOSER)
            self.assertEqual(captured[0]['GITHUB_SHA'],PRODUCT)
            self.assertNotIn('GH_TOKEN',captured[0]); self.assertNotIn('GITHUB_TOKEN',captured[0])
            release = tree/'build/beta-artifacts'
            for name,data in before.items(): self.assertEqual((release/name).read_bytes(),data)
            for name in c.manifest.PRODUCT_FILES:
                if name.endswith('.md'):
                    self.assertEqual((release/name).read_bytes(),members['opennav-x/docs/beta2/'+name])
            self.assertFalse((tree/'build/developer-preview/CHANGED').exists())
            self.assertFalse((tree/'OpenCPN-5.12.4-integrated').exists())


class NativeAllCompositionTests(unittest.TestCase):
    def setUp(self):
        self.request,self.raw,self.q,self.record,self.support=full_fixture()
        self.gh=FakeGitHub(self.request,self.raw,self.q)
        self.original,self.reports,self.frozen=full_reports(self.q)

    def test_full_replacement_keeps_product_failure_and_distinct_harness(self):
        c.validate_qualification(self.q,self.record,self.support)
        c.verify_provenance(self.gh,self.q,complete=True)
        c.validate_reports(self.original,self.reports,self.q,self.frozen)
        self.assertNotIn('result',self.original['objects-input-results.json'])
        self.assertEqual(self.q['commit'],PRODUCT)
        self.assertEqual(self.q['composition']['retest']['commit'],HARNESS)
        self.assertEqual(self.gh.data[self.gh.base+'/actions/runs/100/attempts/1']['conclusion'],'failure')

    def test_every_current_report_is_required_including_native_custody(self):
        for name in c.ALL_REPORTS:
            changed=copy.deepcopy(self.reports);del changed[name]
            with self.subTest(name=name),self.assertRaises(ValueError):
                c.validate_reports(self.original,changed,self.q,self.frozen)
        for group in ('original','retest'):
            request=copy.deepcopy(self.request)
            request[group]['prefixReports' if group=='original' else 'reports'].pop()
            with self.assertRaises(ValueError):c.validate_request(request)

    def test_full_semantics_refuse_false_passes_and_other_bytes(self):
        mutations=[
            ('staging-retest.json',('scope',),'installer-charts'),
            ('staging-retest.json',('checks',),[dict(name='all',status='running')]),
            ('staging-retest.json',('harnessCommit',),PRODUCT),
            ('staging-inputs.json',('sourceArchiveSha256',),'0'*64),
            ('staging-inputs.json',('manifestSha256',),'0'*64),
            ('boat-feedback-windows/result.json',('source_commit',),HARNESS),
            ('boat-feedback-windows/result.json',('harness_commit',),PRODUCT),
            ('boat-feedback-windows/result.json',('restore_receipt_sha256',),'0'*64),
            ('boat-feedback-windows/result.json',('tests',0,'binary_sha256'),'0'*64),
            ('installer-loader-selftest.json',('app_sha256',),'0'*64),
            ('peer-two-instances-results.json',('restore_receipt_sha256',),'0'*64),
            ('peer-two-instances-results.json',('test_script_sha256',),'0'*64),
            ('peer-two-instances-results.json',('profiles',1,'launch_mode'),'ordinary'),
            ('peer-two-instances-results.json',('profiles',1,'profile'),'d:/FIXTURE/PROFILE-10'),
            ('peer-two-instances-results.json',('profiles',0,'cert_preserved_at_exit'),False),
            ('peer-two-instances-results.json',('profiles',0,'key_preserved_at_exit'),False),
            ('peer-two-instances-results.json',('observations',0,'owned_tcp_listener_ports'),[8443]),
            ('peer-two-instances-results.json',('profiles',0,'pid'),999),
            ('objects-input-results.json',('native_edit_confirmation',),'not run'),
            ('route-standard-input-results.json',('route_contract','checks'),['x']*25),
            ('pilot-results.json',('checks',),['x']*6),
            ('pilot-status-only/pilot-results.json',('received_bytes',),1),
            ('pilot-status-only/pilot-opennav-diagnostics.json',('xnav_manual_control_contract',),True),
            ('pilot-status-only/pilot-opennav-diagnostics.json',('runtime','pilot','serial_session_enabled'),True),
            ('production-recovery-results.json',('files_verified',),1019),
            ('production-recovery-results.json',('package_sha256',),'0'*64),
            ('production-recovery-results.json',('build','xnav_hardware_output_policy'),'status-only'),
            ('installer-staging.json',('source_archive_sha256',),'0'*64),
            ('installer-staging.json',('installer_source_sha256',),'0'*64),
            ('installer-staging.json',('installer_helper_sha256',),'0'*64),
            ('packaged-updater.json',('executableSha256',),'0'*64),
            ('packaged-updater.json',('checks',),c.UPDATER_CHECKS[:3]),
            ('charts-results.json',('result',),'skipped'),
        ]
        for name,path,value in mutations:
            changed=copy.deepcopy(self.reports);target=changed[name]
            for key in path[:-1]:target=target[key]
            target[path[-1]]=value
            with self.subTest(name=name,path=path),self.assertRaises(ValueError):
                c.validate_reports(self.original,changed,self.q,self.frozen)

    def test_dirty_flag_and_unrelated_ports_are_not_extra_product_gates(self):
        self.reports['boat-feedback-windows/result.json']['harness_dirty']=True
        self.reports['peer-two-instances-results.json']['observations'][0]['owned_tcp_listener_ports']=[12345]
        c.validate_reports(self.original,self.reports,self.q,self.frozen)

    def test_original_failure_cannot_be_rewritten_to_success(self):
        changed=copy.deepcopy(self.original);changed['objects-input-results.json']['result']='passed'
        with self.assertRaises(ValueError):c.validate_reports(changed,self.reports,self.q,self.frozen)

    def test_native_custody_uses_both_hosts_and_exact_consumed_sources(self):
        for arch in ('x64','x86'):
            name=f'signed-update-custody/{arch}-previous-restore.log'
            for key,value in [('status','failed'),('processBits',16),('powerShellVersion','7.6.7'),
                              ('sourceSha256',{}),('boatAccess',True)]:
                changed=copy.deepcopy(self.reports);record=json.loads(changed[name]);record[key]=value
                changed[name]=json.dumps(record)
                with self.subTest(arch=arch,key=key),self.assertRaises(ValueError):
                    c.validate_reports(self.original,changed,self.q,self.frozen)
            changed=copy.deepcopy(self.reports);changed[f'signed-update-custody/{arch}.log']='passed'
            with self.assertRaises(ValueError):c.validate_reports(self.original,changed,self.q,self.frozen)

    def test_full_tree_refuses_added_shadow_inputs_mode_changes_and_other_helpers(self):
        endpoint=f"{self.gh.base}/git/trees/{self.request['helperClosure']['harnessTree']}?recursive=1"
        for mutation in ('added','removed','mode','changed','truncated'):
            gh=FakeGitHub(self.request,self.raw,self.q);tree=gh.data[endpoint]
            if mutation=='added':tree['tree'].append(dict(path='opennav-x/tools/json.py',mode='100644',type='blob',sha='f'*40))
            elif mutation=='removed':tree['tree'].pop()
            elif mutation=='mode':tree['tree'][-1]['mode']='120000'
            elif mutation=='changed':tree['tree'][0]['sha']='f'*40
            else:tree['truncated']=True
            with self.subTest(mutation=mutation),self.assertRaises(ValueError):c.verify_helper_closure(gh,self.request)
        gh=FakeGitHub(self.request,self.raw,self.q)
        gh.data[endpoint]['tree'].append(dict(path='opennav-x/docs/evidence/new/receipt.json',mode='100644',type='blob',sha='f'*40))
        c.verify_helper_closure(gh,self.request)

    def test_recipe_hash_and_commit_tree_cannot_be_substituted(self):
        gh=FakeGitHub(self.request,self.raw,self.q)
        endpoint=next(p for p in gh.data if '/git/blobs/' in p)
        gh.data[endpoint].update(size=7,content=base64.b64encode(b'no test').decode())
        with self.assertRaises(ValueError):c.verify_helper_closure(gh,self.request)
        gh=FakeGitHub(self.request,self.raw,self.q)
        gh.data[f'{gh.base}/git/commits/{HARNESS}']['tree']['sha']='0'*40
        with self.assertRaises(ValueError):c.verify_helper_closure(gh,self.request)

    def test_current_pilot_serial_and_original_restart_authority_are_mandatory(self):
        endpoint=f'{self.gh.base}/actions/runs/100/attempts/1/jobs'
        self.gh.data[endpoint]=[j for j in self.gh.data[endpoint] if j['name']!='pilot-serial-contracts / serial']
        with self.assertRaises(ValueError):c.verify_provenance(self.gh,self.q,complete=True)
        self.gh=FakeGitHub(self.request,self.raw,self.q)
        self.gh.data[f'{self.gh.base}/actions/artifacts/104']['workflow_run']['head_sha']=HARNESS
        with self.assertRaises(ValueError):c.verify_provenance(self.gh,self.q,complete=True)

    def test_restart_receipt_cannot_transfer_or_claim_boat_acceptance(self):
        p=self.request['producer']
        good=dict(schema=1,owner='OpenNavX.CI.RestartGates.1',**p,transport='success',maintenance='success',
                  broker='success',prepareArm='success',actualBoat=False)
        c.validate_restart_receipt(good,p)
        for key,value in [('commit',HARNESS),('runAttempt','2'),('prepareArm','skipped'),('actualBoat',True)]:
            with self.subTest(key=key),self.assertRaises(ValueError):c.validate_restart_receipt(dict(good,**{key:value}),p)

    def test_runtime_inventory_count_and_source_hashes_come_from_frozen_files(self):
        with tempfile.TemporaryDirectory() as tmp:
            tree=Path(tmp);preview=tree/'build/developer-preview';preview.mkdir(parents=True)
            sources={'opennav-x/tools/smoke-peer-two-instances.py':dict(sha256='a'*64)}
            with zipfile.ZipFile(preview/'SKAGER-Beta2-source.zip','w') as z:
                z.writestr('SOURCE_REFERENCE.json',json.dumps(dict(productCommit=PRODUCT,files=sources)))
            product=dict(commit=PRODUCT)
            metadata=tree/c.inputs.PACKAGE_ROOT/'docs/PRODUCT_BUILD.json';metadata.parent.mkdir(parents=True)
            metadata.write_text(json.dumps(product))
            for path in ('build/xnav-install/opencpn.exe','build/production-install/opencpn.exe',
                         'build/beta-installer/SKAGER-Beta2-Setup.exe'):
                file=tree/path;file.parent.mkdir(parents=True,exist_ok=True);file.write_bytes(path.encode())
            for count in (2,9):
                with zipfile.ZipFile(preview/'SKAGER-Beta2-Portable-Recovery.zip','w') as z:
                    z.writestr('SKAGER-Beta2-Portable-Recovery/FILE_SHA256.json',
                               json.dumps({f'file-{n}':'b'*64 for n in range(count)}))
                result=c.runtime_binding(tree)
                self.assertEqual(result['recoveryFiles'],count)
                self.assertEqual(result['sourceHashes'],{n:r['sha256'] for n,r in sources.items()})
                self.assertEqual(result['product'],product)
                self.assertEqual(result['hashes']['product'],c.digest(b'build/production-install/opencpn.exe'))

    def test_full_assembly_consumes_retest_recovery_without_rewriting_negative(self):
        # External custody/restoration is mocked here; its trust checks have
        # independent negatives above. Exercise the real assembly routing and
        # report validation, with distinct original and replacement inventories.
        contents={'build/xnav-install/opencpn.exe':b'fixture',
            'build/production-install/opencpn.exe':b'product',
            'build/beta-installer/SKAGER-Beta2-Setup.exe':b'setup',
            'build/developer-preview/SKAGER-Beta2-Portable-Recovery.zip':b'recovery',
            'build/developer-preview/SKAGER-Beta2-source.zip':b'source',
            'build/beta-installer/package.json':b'{}','build/beta-installer/payload.zip':b'payload',
            'build/production-windows/include/config.h':b'production',
            'build/xnav-windows/include/config.h':b'fixture'}
        facts=self.frozen['runtime'];facts['hashes']={k:c.digest(v) for k,v in
                                                    dict(fixture=b'fixture',product=b'product',setup=b'setup',recovery=b'recovery').items()}
        facts['product']['executable_sha256']=facts['hashes']['product']
        self.reports['installer-loader-selftest.json']['app_sha256']=facts['hashes']['fixture']
        self.reports['peer-two-instances-results.json']['executable_sha256']=facts['hashes']['fixture']
        self.reports['production-recovery-results.json']['package_sha256']=facts['hashes']['recovery']
        self.reports['installer-staging.json']['setup_sha256']=facts['hashes']['setup']
        self.reports['packaged-updater.json'].update(setupSha256=facts['hashes']['setup'],executableSha256=facts['hashes']['product'])
        restore_raw=json.dumps(self.reports['staging-inputs.json']).encode();restore_hash=c.digest(restore_raw)
        for name,key in [('boat-feedback-windows/result.json','restore_receipt_sha256'),
                         ('peer-two-instances-results.json','restore_receipt_sha256'),
                         ('installer-staging.json','compiled_input_receipt_sha256')]:self.reports[name][key]=restore_hash
        next(x for x in self.request['retest']['reports'] if x['path']=='staging-inputs.json')['sha256']=restore_hash
        raw=json.dumps(self.request).encode();q=c.qualification(self.request,c.digest(raw),self.q['composition']['execution'])
        gh=FakeGitHub(self.request,raw,q)
        old_raw={n:json.dumps(v).encode() for n,v in self.original.items()}
        new_raw={n:(v.encode() if isinstance(v,str) else json.dumps(v).encode()) for n,v in self.reports.items()}
        restart=dict(schema=1,owner='OpenNavX.CI.RestartGates.1',**self.request['producer'],transport='success',
                     maintenance='success',broker='success',prepareArm='success',actualBoat=False)
        groups={'original.zip':(old_raw,self.original),'retest.zip':(new_raw,self.reports),
                'restart.zip':({'qualified.json':json.dumps(restart).encode()},{'qualified.json':restart})}
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)/'checkout/opennav-x';request_file=root/'tools/staging-composition-request.json'
            request_file.parent.mkdir(parents=True);request_file.write_bytes(raw)
            def restore(tree,*args):
                for name,data in contents.items():
                    path=tree/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(data)
                meta=tree/c.inputs.PACKAGE_ROOT/'docs/PRODUCT_BUILD.json';meta.parent.mkdir(parents=True)
                meta.write_text(json.dumps(facts['product']))
                return copy.deepcopy(self.frozen)
            def assemble_payloads(tree,accepted,suffix,commit):
                self.assertEqual(accepted,new_raw);self.assertEqual(suffix,new_raw);self.assertEqual(commit,PRODUCT)
                release=tree/'build/beta-artifacts';release.mkdir()
                for name in ('build/beta-installer/SKAGER-Beta2-Setup.exe',
                             'build/developer-preview/SKAGER-Beta2-Portable-Recovery.zip',
                             'build/developer-preview/SKAGER-Beta2-source.zip'):
                    (release/Path(name).name).write_bytes(contents[name])
            def create(release,commit,run,attempt,version):
                (release/'RELEASE.json').write_text('{}');return dict(commit=commit,runId=run,runAttempt=attempt)
            facts['product']['version']='0.4.0-beta2.4'
            env=dict(GITHUB_SHA=COMPOSER,GITHUB_RUN_ID='300',GITHUB_RUN_ATTEMPT='3',
                     GITHUB_REPOSITORY=c.REPOSITORY,GITHUB_REF='refs/heads/skager-staging-compose')
            real_spec=c.importlib.util.spec_from_file_location
            retain_path=Path(c.__file__).parent/'retain-release-inputs.py'
            def spec(name,path):return real_spec(name,retain_path)
            with patch.object(c,'__file__',str(root/'tools/staging_composition.py')),patch.dict(os.environ,env),\
                 patch.object(c.subprocess,'check_output',side_effect=[str(root.parent)+'\n',COMPOSER+'\n',raw]),\
                 patch.object(c,'_download'),patch('fetch_ci_inputs.unpack'),patch.object(c.inputs,'restore',side_effect=restore),\
                 patch.object(c,'runtime_binding',return_value=facts),\
                 patch.object(c,'_reports',side_effect=lambda path,pins:groups[path.name]),\
                 patch.object(c,'assemble_frozen_payloads',side_effect=assemble_payloads),\
                 patch.object(c.importlib.util,'spec_from_file_location',side_effect=spec),\
                 patch.object(c.manifest,'create',side_effect=create),patch.object(c.manifest,'verify'):
                proof=c.assemble(gh,request_file,Path(tmp)/'output')
            self.assertFalse(proof['productRebuilt']);self.assertFalse(proof['applicationsExecuted'])
            saved=Path(tmp)/'output/evidence/original/objects-input-results.json'
            self.assertEqual(saved.read_bytes(),old_raw['objects-input-results.json'])
            self.assertNotIn('result',json.loads(saved.read_bytes()))



if __name__ == '__main__':
    unittest.main()

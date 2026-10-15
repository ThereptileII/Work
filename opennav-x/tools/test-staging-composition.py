#!/usr/bin/env python3
"""Focused offline composition trust tests. No network, builds or applications."""
import base64
import copy
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
                jobs = [(name, 'success', '1') for name in c.REQUIRED_JOBS] + [(c.RUNTIME_JOB, 'failure', '102')]
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


if __name__ == '__main__':
    unittest.main()

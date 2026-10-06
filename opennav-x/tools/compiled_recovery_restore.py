#!/usr/bin/env python3
"""Restore only the reviewed 0db prepackage recovery, never qualify a failed run.

The new packaging workflow owns its own identity. This helper performs no build,
package, application launch, SDK installation or release publication. Inert SDK
source archives are selected from the authenticated original dependency artifact;
one pinned openssl.exe is restored solely for the existing packager byte/PE check,
never executed. No compiler payload or SDK environment receipts are imported.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path, PureWindowsPath
import re
import stat
import tempfile
import zipfile

import compiled_recovery as recovery
import staging_build_inputs as sealed
from fetch_ci_inputs import authenticated_artifact
from github_release_delivery import GitHub

REPOSITORY = 'ThereptileII/Work'
COMMIT = '0db45cb92509c29ef4d1fc33d0c76dc7cf511b97'
RUN = '37424595899'
ATTEMPT = '1'
PRODUCER = sealed.producer(COMMIT, RUN, ATTEMPT)
KIND = 'skager-exact-compiled-recovery-restoration'
FAILURE_STEP = 'Compile exact installer without running its desktop qualification yet'
WORKFLOW = '.github/workflows/opennav-baseline.yml'
REQUIRED_STEPS = (
    'Native AIS observation and transport with verified maintained TLS',
    'Compile fixture application and run compiled security and unit checks',
    'Verify packaged source reproduces the curl certificate-tool patch',
    'Installed native peer CLI refuses key changes on the disposable runner',
    'Compile fixture-free product and run installed security and unit checks',
    'Check prepackage recovery retention with disposable real Git trees',
    'Retain both native applications and exact tracked source before packaging',
    'Upload unqualified compiled recovery before packaging can fail',
    'Resolve private chart imports in the real host without plugin initialization',
    'Native public Downloader Windows trust and rejection gate',
    'Exact peer response buffer on native MSVC Win32',
    'Stage exact qualified startup updater and retained dependency source',
    'Package fixture-free recovery before desktop qualification',
)
ARTIFACTS = {
    'compiled': dict(artifactId='11396343771', artifactDigest='sha256:e89cc630a362a8cd142ae3a68d21018b299e9a9a1b7aa3bdefe7872a1294793f',
        artifactName=f'compiled-recovery-{COMMIT}-run{RUN}-attempt1'),
    'updater': dict(artifactId='11394646331', artifactDigest='sha256:62a907e9710a48b18929693882b882e0a00278f183911a34c4e4e2747d59cd9f',
        artifactName=f'update-verifier-windows-2022-386-{COMMIT}'),
    'evidence': dict(artifactId='11396920456', artifactDigest='sha256:6a8b4b429fe5f584b60b654e9d6f024da0764de145ee7794ea03a63159f9bb82',
        artifactName=f'windows-build-evidence-{COMMIT}-attempt1'),
    'sdk': dict(repository=REPOSITORY, runId='37230581131', runAttempt='1',
        headSha='1b25542aea3f9ab62c83c5d7a3ecdac9652f7d3e', artifactId='11314817073',
        artifactName='windows-dependencies-cf9d9024a09681e06bc77a5e8a7b93ba4bd171191a68a5fceee3ff6f4bf15cf4-run37230581131-attempt1',
        artifactDigest='sha256:27f689d55a6827e52826fb64f7f1c5081e17ee19ab35beaa219d406b43bfaf2e'),
}
# Derived from the independently downloaded, hash-verified fixed artifacts, not
# caller metadata. These pins prevent this recovery becoming a general waiver.
INNER_SHA = 'f93ce616a0040e31dbdbce76bc4f0a40960cfd00e676a4f2d5bf6799e64b7f45'
MANIFEST_SHA = '240828402b346974bd281e7ac18dc34de1b5bc6b7cc2bb43a2f17985add0f44a'
SOURCES_SHA = '223636b925fe5efde6afeedc4d0dd61c4ce6dd297fea377101d09adc88575259'
COMPILED_FILES_SHA = '1a9aec7386a83397fd35ac329a8b1829117846325e0c19e6e89282156d824395'
LATE_FILES_SHA = '78e4a596fdc5801586e6f0aecd907f43d11a53b3eebb6ec82375baea7a7d504d'
UPDATER_FILES_SHA = 'b8f89868a11b8e0a7f7bda47d76584da6dce19a305a143537535c9497028cfcf'
ALL_FILES_SHA = 'c243bd9e1379873461eaa6888532db1bb5c266ebb92c26d85bb4e2a1a1bb61e1'
ALL_FILES_COUNT = 3484
# The original curl provenance checker reads this exact path; do not rewrite its
# historical manifest or import a runnable SDK environment to satisfy that check.
VERIFICATION_TOOL = dict(
    path=r'D:\a\Work\Work\opennav-x\build\windows-openssl-3.5.9\install\bin\openssl.exe',
    relativePath='build/windows-openssl-3.5.9/install/bin/openssl.exe',
    sdkMember='payload/build/windows-openssl-3.5.9/install/bin/openssl.exe',
    size=722944, sha256='e538ab95debb62adb94914e61144690965ea7db2251d95085471a59454250b6b',
    purpose='packager byte/PE verification only; never executed')
DEPENDENCY_SOURCES = {
    'openssl-3.5.9.tar.gz': dict(size=53279637, sha256='603f5602e2eef00d77fbd429d34dcd5822bb301757a1bc9cdb24c670f1eb859a'),
    'curl-8.22.0.tar.xz': dict(size=2953092, sha256='f7ef3ae8a22e521f289803fe93543eb64c329b58aa73a9e224dfd915a2a5f4f7'),
    'zlib-1.3.2.tar.gz': dict(size=1502830, sha256='bb329a0a2cd0274d05519d61c667c062e06990d72e125ee2dfa8de64f0119d16'),
}
UPDATER_NAMES = ('skager-start.exe', 'opennav/third-party/updater/build.json',
                 'opennav/third-party/updater/updater-source.zip')
MAX_OUTER = 8 * 1024**3
require = sealed.require


def digest(data):
    return hashlib.sha256(data).hexdigest()


def object_digest(value):
    return digest(sealed.canonical(value))


def strict_json(data):
    require(len(data) <= sealed.MAX_MANIFEST, 'Bounded JSON required')
    def pairs(items):
        result, seen = {}, set()
        for key, value in items:
            require(key.casefold() not in seen, 'Duplicate/case-folded JSON field')
            seen.add(key.casefold()); result[key] = value
        return result
    return json.loads(data, object_pairs_hook=pairs,
                      parse_constant=lambda _: (_ for _ in ()).throw(ValueError('Nonfinite JSON')))


def fields(value, names):
    require(isinstance(value, dict) and set(value) == set(names), 'Closed recovery schema differs')


def safe_name(name):
    sealed.safe_name(name)
    require(not re.search(r'[\x00-\x1f<>"|?*]', name) and
            all(p.casefold() != '.git' for p in name.split('/')), 'Unsafe recovery path')
    return name


def plain(path, *, missing=False):
    path = Path(path).absolute()
    for part in (path, *path.parents):
        try: info = part.lstat()
        except FileNotFoundError:
            require(missing, 'Missing recovery path'); continue
        require(not stat.S_ISLNK(info.st_mode) and not getattr(info, 'st_file_attributes', 0) & 0x400,
                'Linked/reparse recovery path')
    return path


def zip_index(stream):
    entries = stream.infolist()
    require(0 < len(entries) <= sealed.MAX_FILES and sum(e.file_size for e in entries) <= sealed.MAX_TOTAL,
            'Recovery ZIP count/size bound')
    result, folded = {}, set()
    for entry in entries:
        name = entry.orig_filename
        # ZipInfo normalizes backslashes on Windows (and truncates NULs).
        # Reject the raw/normalized alias before considering its safe spelling.
        require(name == entry.filename, 'ZIP reader normalized an unsafe raw name')
        require(not entry.is_dir() and not entry.flag_bits & 1 and
                stat.S_IFMT(entry.external_attr >> 16) in (0, stat.S_IFREG) and
                0 <= entry.file_size <= sealed.MAX_FILE, 'Nonregular/encrypted/oversized ZIP input')
        safe_name(name)
        require(name.casefold() not in folded, 'Aliased ZIP input')
        folded.add(name.casefold()); result[name] = entry
    require(all('/'.join(n.split('/')[:i]) not in folded for n in folded
                for i in range(1, len(n.split('/')))), 'ZIP file/directory collision')
    return result


def read_bounded(stream, entry, limit=sealed.MAX_MANIFEST):
    require(entry.file_size <= limit, 'Selected JSON exceeds bound')
    return stream.read(entry)


def file_record(stream, entry):
    sha = hashlib.sha256(); count = 0
    with stream.open(entry) as source:
        for block in iter(lambda: source.read(1024**2), b''):
            count += len(block); require(count <= entry.file_size, 'ZIP grew past declared size'); sha.update(block)
    require(count == entry.file_size, 'Truncated ZIP member')
    return dict(size=count, sha256=sha.hexdigest())


def authenticate(gh):
    require(gh.repo == REPOSITORY, 'Recovery repository differs')
    endpoint = f'{gh.base}/actions/runs/{RUN}/attempts/{ATTEMPT}'
    run = gh.api(endpoint)
    require(str(run.get('id')) == RUN and str(run.get('run_attempt')) == ATTEMPT and
            run.get('head_sha') == COMMIT and run.get('path') == WORKFLOW and
            run.get('head_repository', {}).get('full_name') == REPOSITORY and
            run.get('status') == 'completed' and run.get('conclusion') == 'failure' and
            run.get('event') == 'push' and run.get('head_branch') == 'staging',
            'Only the reviewed original failed Staging attempt is recoverable')
    jobs = gh.pages(endpoint + '/jobs')
    def job(name, conclusion):
        matches = [j for j in jobs if j.get('name') == name]
        require(len(matches) == 1, 'Missing/duplicate original producer job')
        j = matches[0]
        require(j.get('status') == 'completed' and j.get('conclusion') == conclusion and
                str(j.get('run_id')) == RUN and str(j.get('run_attempt')) == ATTEMPT and
                j.get('head_sha') == COMMIT, 'Original producer job differs')
        return j
    native = job('windows-integration', 'failure')
    require(str(native.get('id')) == '112143482519', 'Original native job differs')
    steps = native.get('steps', [])
    require([s.get('name') for s in steps if s.get('conclusion') == 'failure'] == [FAILURE_STEP] and
            all(s.get('status') == 'completed' and s.get('conclusion') in ('success', 'failure', 'skipped')
                for s in steps), 'Failure occurred outside the reviewed packaging boundary')
    for name in REQUIRED_STEPS:
        selected = [s for s in steps if s.get('name') == name]
        require(len(selected) == 1 and selected[0].get('conclusion') == 'success', 'Original prerequisite did not pass: ' + name)
    job('updater-contracts / verify (windows-2022, 386)', 'success')
    # Qualification never started; do not reuse the pilot-tail composition schema.
    qualifier = job('Qualify retained native Staging inputs', 'skipped')
    require(str(qualifier.get('id')) == '112158432202', 'Original skipped qualifier differs')
    failed = [s for s in steps if s.get('name') == FAILURE_STEP]
    seal = [s for s in steps if s.get('name') == 'Seal compiled inputs before any desktop test can fail']
    require(len(failed) == 1 and failed[0].get('number') == 25 and len(seal) == 1 and
            seal[0].get('number') == 26 and seal[0].get('conclusion') == 'skipped', 'Original interruption position differs')
    metadata = {}
    for key, pin in ARTIFACTS.items():
        if key == 'sdk': authenticated_artifact(gh, pin, kind='dependencies')
        record = gh.api(f"{gh.base}/actions/artifacts/{pin['artifactId']}")
        expected_run = pin.get('runId', RUN); expected_commit = pin.get('headSha', COMMIT)
        require(str(record.get('id')) == pin['artifactId'] and record.get('name') == pin['artifactName'] and
                record.get('digest') == pin['artifactDigest'] and record.get('expired') is False and
                str(record.get('workflow_run', {}).get('id')) == expected_run and
                record.get('workflow_run', {}).get('head_sha') == expected_commit and
                type(record.get('size_in_bytes')) is int and 0 < record['size_in_bytes'] <= MAX_OUTER,
                'Fixed recovery artifact metadata differs: ' + key)
        metadata[key] = record['size_in_bytes']
    return metadata


def downloads(gh, directory, sizes):
    directory = plain(directory, missing=True); directory.mkdir(parents=True, exist_ok=True)
    result = {}
    for key, pin in ARTIFACTS.items():
        target = plain(directory / ('artifact-' + pin['artifactId'] + '.zip'), missing=True)
        if not target.exists():
            with target.open('xb') as output:
                gh._run(['api', f"{gh.base}/actions/artifacts/{pin['artifactId']}/zip"], output=output)
        require(target.is_file() and target.stat().st_size == sizes[key] and
                'sha256:' + sealed.sha(target) == pin['artifactDigest'], 'Recovery artifact bytes differ: ' + key)
        result[key] = target
    return result


def validate_compiled(outer, temporary):
    with zipfile.ZipFile(outer) as z:
        index = zip_index(z); require(set(index) == {'COMPILED_RECOVERY.zip', 'receipt.json'}, 'Recovery outer closure differs')
        receipt = strict_json(read_bounded(z, index['receipt.json']))
        fields(receipt, ('schema', 'kind', 'status', 'qualification', 'packaging', 'producer', 'archive',
                         'archiveSha256', 'manifestSha256', 'files', 'bytes', 'trackedSourceDirty'))
        require(type(receipt['schema']) is int and receipt['schema'] == 1 and receipt['kind'] == recovery.KIND and
                receipt['status'] == 'retained' and receipt['qualification'] == receipt['packaging'] == 'not-run' and
                receipt['producer'] == PRODUCER and receipt['archive'] == recovery.ARCHIVE and
                receipt['archiveSha256'] == INNER_SHA and receipt['manifestSha256'] == MANIFEST_SHA and
                receipt['trackedSourceDirty'] is True, 'Original retention receipt differs')
        inner = Path(temporary) / recovery.ARCHIVE
        with z.open(index[recovery.ARCHIVE]) as source, inner.open('xb') as target:
            while True:
                data = source.read(1024**2)
                if not data: break
                target.write(data)
        require(sealed.sha(inner) == INNER_SHA, 'Inner recovery archive differs')
    with zipfile.ZipFile(inner) as z:
        index = zip_index(z)
        raw = read_bounded(z, index[recovery.MANIFEST]); require(digest(raw) == MANIFEST_SHA, 'Recovery manifest differs')
        manifest = strict_json(raw)
        fields(manifest, ('schema', 'kind', 'producer', 'qualification', 'packaging', 'scope', 'sources', 'files'))
        require(type(manifest['schema']) is int and manifest['schema'] == 1 and manifest['kind'] == recovery.KIND and
                manifest['producer'] == PRODUCER and manifest['qualification'] == manifest['packaging'] == 'not-run' and
                manifest['scope'] == 'diagnostic recovery only; all original gates required' and
                object_digest(manifest['sources']) == SOURCES_SHA, 'Original source/retention provenance differs')
        files = {}
        for record in manifest['files']:
            fields(record, ('path', 'size', 'sha256')); name = safe_name(record['path'])
            require(name not in files and type(record['size']) is int and 0 <= record['size'] <= sealed.MAX_FILE and
                    isinstance(record['sha256'], str) and re.fullmatch('[a-f0-9]{64}', record['sha256']), 'Invalid recovery file record')
            files[name] = {k: record[k] for k in ('size', 'sha256')}
        require(set(index) == set(files) | {recovery.MANIFEST} and receipt['files'] == len(files) and
                receipt['bytes'] == sum(r['size'] for r in files.values()), 'Recovery inventory/count differs')
        for name, expected in files.items():
            require(file_record(z, index[name]) == expected, 'Retained byte differs: ' + name)
        compiled = {n: r for n, r in files.items() if not n.startswith('source/')}
        require(all(sealed.allowed(n) and not n.startswith(('build/developer-preview/', 'build/beta-installer/')) for n in compiled) and
                object_digest(compiled) == COMPILED_FILES_SHA, 'Unapproved compiled recovery closure')
    return inner, manifest, files, compiled


def verify_sources(root, manifest, files):
    # Caller reconstructs Git using original checkout + ORIGINAL prepare-integration
    # recipe. We neither fabricate a repository nor patch/overwrite tracked source.
    sources, references = recovery.source_inputs(root, COMMIT)
    require(references == manifest['sources'] and object_digest(references) == SOURCES_SHA,
            'Reconstructed Git/source references differ from actual producer')
    expected = {n: r for n, r in files.items() if n.startswith('source/')}
    require(set(sources) == set(expected), 'Reconstructed source file closure differs')
    for name, path in sources.items():
        require(path.stat().st_size == expected[name]['size'] and sealed.sha(recovery.ordinary(path)) == expected[name]['sha256'],
                'Reconstructed source byte differs: ' + name)


def selected_records(archive, select):
    with zipfile.ZipFile(archive) as z:
        index = zip_index(z); mapping = {dest: name for name in index if (dest := select(name)) is not None}
        require(len(mapping) == len(set(mapping.values())), 'Selected destination collision')
        records = {dest: file_record(z, index[name]) for dest, name in mapping.items()}
    return mapping, records


def select_late(name):
    return 'evidence/local/' + name if name == 'peer-buffer-windows.log' or name.startswith(('downloader-trust-windows/', 'ocharts-real-host-module/')) else None


def validate_additions(paths):
    late_map, late = selected_records(paths['evidence'], select_late)
    require(object_digest(late) == LATE_FILES_SHA, 'Post-retention security evidence differs')
    updater_map, updater = selected_records(paths['updater'], lambda n: 'build/updater-qualified/' + n if n in {'install/' + p for p in UPDATER_NAMES} else None)
    require(object_digest(updater) == UPDATER_FILES_SHA, 'Original qualified updater closure differs')
    sdk_map, sdk = selected_records(paths['sdk'], lambda n: n[len('payload/'):] if n in {'payload/build/dependency-downloads/' + p for p in DEPENDENCY_SOURCES} else None)
    require(sdk == {'build/dependency-downloads/' + n: r for n, r in DEPENDENCY_SOURCES.items()}, 'Pinned inert dependency source differs')
    # Bind selected sources to the SDK's own authenticated inventory as well as
    # independent reviewed source digests. No SDK environment receipt is reused.
    with zipfile.ZipFile(paths['sdk']) as z:
        index = zip_index(z); bundle = strict_json(read_bounded(z, index['bundle.json']))
        for name, record in sdk.items():
            require(bundle['files'][name] == dict(bytes=record['size'], sha256=record['sha256']), 'SDK source inventory differs')
    return [('evidence', late_map, late), ('updater', updater_map, updater), ('sdk', sdk_map, sdk)]


def certificate_tool_target(harness_root):
    target = plain(plain(harness_root) / VERIFICATION_TOOL['relativePath'], missing=True)
    require(target == Path(VERIFICATION_TOOL['path']), 'Certificate tool is not at the fixed original harness path')
    return target


def validate_certificate_tool(inner, sdk, harness_root):
    target = certificate_tool_target(harness_root)
    require(not target.exists(), 'Certificate tool would overwrite an existing file')
    expected = dict(bytes=VERIFICATION_TOOL['size'], sha256=VERIFICATION_TOOL['sha256'])
    with zipfile.ZipFile(inner) as z:
        index = zip_index(z)
        for variant in sealed.VARIANTS:
            prefix = f'build/{variant}-install/'
            raw = read_bounded(z, index[prefix + 'openssl-build.json'])
            openssl = strict_json(raw)
            curl = strict_json(read_bounded(z, index[prefix + 'curl-build.json']))
            tool = curl['buildSteps']['certificateTool']
            fields(tool, ('path', 'bytes', 'sha256', 'versionOutput'))
            require(tool['path'] == VERIFICATION_TOOL['path'] and
                    {k: tool[k] for k in expected} == expected and
                    openssl['outputs']['bin/openssl.exe'] == expected and
                    isinstance(tool['versionOutput'], str) and tool['versionOutput'].startswith('OpenSSL 3.5.9 ') and
                    tool['versionOutput'] == openssl['versionOutput'] and
                    curl['dependencies']['openssl']['manifestSha256'] == digest(raw) and
                    PureWindowsPath(curl['dependencies']['openssl']['prefix']) / 'bin/openssl.exe' == PureWindowsPath(tool['path']),
                    'Original installed certificate-tool provenance differs')
    with zipfile.ZipFile(sdk) as z:
        index = zip_index(z)
        bundle = strict_json(read_bounded(z, index['bundle.json']))
        require(bundle['files'][VERIFICATION_TOOL['relativePath']] == expected and
                file_record(z, index[VERIFICATION_TOOL['sdkMember']]) ==
                dict(size=expected['bytes'], sha256=expected['sha256']), 'SDK certificate-tool bytes/inventory differ')
    return target


def verify_certificate_tool(receipt, harness_root):
    # Separate filesystem check: validate_receipt remains a pure offline parser.
    require(receipt['verificationOnlyTool'] == VERIFICATION_TOOL, 'Verification-only tool receipt differs')
    target = plain(certificate_tool_target(harness_root))
    require(target.is_file() and target.stat().st_size == VERIFICATION_TOOL['size'] and
            sealed.sha(target) == VERIFICATION_TOOL['sha256'], 'Verification-only certificate tool changed')
    return target


def verify_files(root, records):
    for name, record in records.items():
        path = plain(root / safe_name(name))
        require(path.is_file() and path.stat().st_size == record['size'] and sealed.sha(path) == record['sha256'], 'Restored bytes changed: ' + name)


def validate_identity(identity):
    fields(identity, ('commit', 'runId', 'runAttempt'))
    require(isinstance(identity['commit'], str) and re.fullmatch('[a-f0-9]{40}', identity['commit']) and identity['commit'] != COMMIT and
            all(isinstance(identity[k], str) and re.fullmatch('[1-9][0-9]{0,19}', identity[k]) for k in ('runId', 'runAttempt')) and
            identity['runId'] != RUN, 'Recovery must retain a separate execution identity')


def validate_receipt(record):
    fields(record, ('schema', 'kind', 'status', 'qualification', 'packaging', 'producer', 'recovery',
                    'originalConclusion', 'originalFailureStep', 'artifacts', 'archiveSha256', 'manifestSha256',
                    'sourceManifestSha256', 'files', 'reusedEvidence', 'boatFeedback', 'workspaceRoot', 'verificationOnlyTool'))
    require(type(record['schema']) is int and record['schema'] == 1 and record['kind'] == KIND and
            record['status'] == 'restored' and record['qualification'] == record['packaging'] == 'not-run' and
            record['producer'] == PRODUCER and record['originalConclusion'] == 'failure' and
            record['originalFailureStep'] == FAILURE_STEP and record['artifacts'] == ARTIFACTS and
            record['archiveSha256'] == INNER_SHA and record['manifestSha256'] == MANIFEST_SHA and
            record['sourceManifestSha256'] == SOURCES_SHA, 'Exact recovery receipt authority differs')
    require(record['verificationOnlyTool'] == VERIFICATION_TOOL, 'Verification-only tool receipt differs')
    validate_identity(record['recovery'])
    files = record['files']
    require(isinstance(files, dict) and len(files) == ALL_FILES_COUNT and object_digest(files) == ALL_FILES_SHA,
            'Recovery immutable file closure differs')
    require(record['reusedEvidence'] == sorted(n for n in files if select_late(n.removeprefix('evidence/local/')) == n) and
            isinstance(record['workspaceRoot'], str) and len(record['workspaceRoot']) <= 1024 and '\x00' not in record['workspaceRoot'],
            'Recovery evidence/workspace binding differs')
    expected_feedback = dict(manifestSha256=files[sealed.FEEDBACK_MANIFEST]['sha256'], binaries=[
        dict(name=n, path=sealed.FEEDBACK_BINARIES[n], sha256=files[sealed.FEEDBACK_BINARIES[n]]['sha256']) for n in sorted(sealed.FEEDBACK_TESTS)])
    require(record['boatFeedback'] == expected_feedback, 'Recovery component manifest/binaries differ')
    return record


def restore(root, paths, receipt_path, identity):
    validate_identity(identity)
    root = plain(root); receipt_path = plain(receipt_path, missing=True)
    require(set(paths) == set(ARTIFACTS), 'Complete authenticated input closure required')
    for key, path in paths.items():
        path = plain(path)
        require(path.is_file() and path.stat().st_size <= MAX_OUTER and
                'sha256:' + sealed.sha(path) == ARTIFACTS[key]['artifactDigest'], 'Pinned archive differs: ' + key)
    require(root.is_dir() and not receipt_path.exists(), 'Fresh recovery receipt required')
    with tempfile.TemporaryDirectory(prefix='skager-exact-recovery-') as temporary:
        inner, manifest, files, compiled = validate_compiled(paths['compiled'], temporary)
        verify_sources(root, manifest, files)
        additions = validate_additions(paths)
        harness_root = Path(__file__).resolve().parents[1]
        tool_target = validate_certificate_tool(inner, paths['sdk'], harness_root)
        retained = dict(compiled)
        for _, _, records in additions:
            require(not set(retained) & set(records), 'Recovery overlays an original byte')
            retained.update(records)
        require(len(retained) == ALL_FILES_COUNT and object_digest(retained) == ALL_FILES_SHA, 'Recovery closure differs')
        # Validate every target before the first restore mutation. Exclusive file
        # creation refuses races; a partial failure stays unqualified/unreceipted.
        for name in retained:
            require(not plain(root / safe_name(name), missing=True).exists(), 'Recovery would overwrite an existing file: ' + name)
        groups = [(inner, {n: n for n in compiled})] + [(paths[key], mapping) for key, mapping, _ in additions]
        for archive, mapping in groups:
            with zipfile.ZipFile(archive) as z:
                for name, member in mapping.items():
                    target = plain(root / name, missing=True); target.parent.mkdir(parents=True, exist_ok=True)
                    with z.open(member) as source, target.open('xb') as output:
                        while True:
                            block = source.read(1024**2)
                            if not block: break
                            output.write(block)
        verify_sources(root, manifest, files); verify_files(root, retained)
        # These are local source/PE checks only, never an updater executable call.
        from updater_package import verify_updater_package
        verify_updater_package(root / 'build/updater-qualified/install', COMMIT)
        record = dict(schema=1, kind=KIND, status='restored', qualification='not-run', packaging='not-run',
            producer=PRODUCER, recovery=identity, originalConclusion='failure', originalFailureStep=FAILURE_STEP,
            artifacts=ARTIFACTS, archiveSha256=INNER_SHA, manifestSha256=MANIFEST_SHA,
            sourceManifestSha256=SOURCES_SHA, files=retained,
            reusedEvidence=sorted(n for _, _, records in additions[:1] for n in records),
            boatFeedback=sealed.feedback_binding(root, COMMIT), workspaceRoot=str(root),
            verificationOnlyTool=dict(VERIFICATION_TOOL))
        validate_receipt(record)
        # Only this one authenticated SDK executable is materialized, without
        # dependencies or execution. Exclusive creation never replaces a file.
        created = receipt_created = False
        try:
            tool_target = plain(tool_target, missing=True)
            tool_target.parent.mkdir(parents=True, exist_ok=True)
            with zipfile.ZipFile(paths['sdk']) as z:
                index = zip_index(z)
                data = read_bounded(z, index[VERIFICATION_TOOL['sdkMember']], VERIFICATION_TOOL['size'])
                require(len(data) == VERIFICATION_TOOL['size'] and digest(data) == VERIFICATION_TOOL['sha256'],
                        'SDK certificate tool changed before creation')
            with tool_target.open('xb') as output:
                created = True
                output.write(data); output.flush(); os.fsync(output.fileno())
            verify_certificate_tool(record, harness_root)
            receipt_path.parent.mkdir(parents=True, exist_ok=True)
            with receipt_path.open('xb') as output:
                receipt_created = True
                output.write(sealed.canonical(record)); output.flush(); os.fsync(output.fileno())
            return record
        except BaseException:
            if receipt_created:
                plain(receipt_path).unlink()
            if created:
                plain(tool_target).unlink()
            raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--downloads', type=Path, required=True)
    parser.add_argument('--receipt', type=Path, required=True)
    for name in ('harness-commit', 'run-id', 'run-attempt'): parser.add_argument('--' + name, required=True)
    args = parser.parse_args()
    identity = dict(commit=args.harness_commit, runId=args.run_id, runAttempt=args.run_attempt)
    validate_identity(identity)
    require(os.name == 'nt' and os.environ.get('GITHUB_ACTIONS') == 'true' and
            os.environ.get('RUNNER_ENVIRONMENT') == 'github-hosted' and os.environ.get('GITHUB_REPOSITORY') == REPOSITORY and
            all(os.environ.get(env) == identity[k] for env, k in (('GITHUB_SHA', 'commit'), ('GITHUB_RUN_ID', 'runId'), ('GITHUB_RUN_ATTEMPT', 'runAttempt'))),
            'Actual disposable native recovery identity required')
    require(os.environ.get('GITHUB_JOB') == 'recover' and
            os.environ.get('GITHUB_REF') == 'refs/heads/skager-compiled-recovery' and
            os.environ.get('GITHUB_EVENT_NAME') in ('push', 'workflow_dispatch') and
            recovery.git(Path(__file__).resolve().parents[1], 'rev-parse', 'HEAD').decode().strip() == identity['commit'],
            'Only the exact reviewed recovery workflow checkout may restore')
    gh = GitHub(REPOSITORY); sizes = authenticate(gh); paths = downloads(gh, args.downloads, sizes)
    result = restore(args.root, paths, args.receipt, identity)
    print(json.dumps(dict(status=result['status'], producer=PRODUCER, recovery=identity,
                         files=len(result['files']), qualification='not-run', packaging='not-run')))


if __name__ == '__main__':
    main()

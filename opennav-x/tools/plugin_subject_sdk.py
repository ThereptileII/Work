"""Inert subject-SDK authentication for retained PluginHandler tests only.

The host SDK still passes the unchanged current-image restore/native reprobe.
This module never restores, builds or stages the subject SDK into that workspace.
"""
import copy
import json
from pathlib import Path, PureWindowsPath
import re
import shutil
import tempfile

import fetch_ci_inputs as fetch
import windows_dependency_bundle as bundle_api
import windows_dependency_evidence as evidence
import windows_dependency_receipt as receipt
import windows_dependency_reuse as reuse
import windows_dependency_stage as stage
from release_manifest import require


def dependency_authority(document):
    return {'mode': 'cross-run-bundle', 'producer': document['producer'],
            'fingerprint': document['fingerprint'],
            'toolchainSha256': document['toolchainSha256']}


def select_subject(lock, captured):
    # Validate the entire unchanged lock against the real current host first.
    # A capture may select only an already reviewed immutable pin, never a URL
    # or a replacement authority taken from its own claims.
    fetch.resolve_selection(lock, kind='dependencies')
    require(set(lock) == {'schemaVersion', 'repository', 'images'},
            'Retained subject requires the reviewed image selection map')
    require(isinstance(captured, dict) and set(captured) ==
            {'mode', 'producer', 'fingerprint', 'toolchainSha256'} and
            captured['mode'] == 'cross-run-bundle', 'Captured SDK authority missing')
    bundle_api._validate_identity(captured['producer'])
    matches = [entry for entry in lock['images'] if all(
        entry['selection'][key] == captured['producer'][key]
        for key in ('repository', 'runId', 'runAttempt', 'headSha'))]
    require(len(matches) == 1, 'Captured SDK has no unique reviewed selection')
    return matches[0]


def interface_files(document):
    # Complete headers and import libraries, including extra/missing entries.
    result = {}
    for prefix in stage.PREFIX.values():
        selected = {name: value for name, value in document['files'].items()
                    if name.startswith(prefix + '/include/') or
                    (name.startswith(prefix + '/lib/') and name.endswith('.lib'))}
        require(any('/include/' in name for name in selected) and
                any(name.endswith('.lib') for name in selected),
                'Subject SDK header/import closure missing')
        result.update(selected)
    return result


def compare_host_subject(host, subject):
    require(subject['workspaceRoot'] == host['workspaceRoot'], 'Subject workspace identity differs')
    require(subject['python'] == host['python'], 'Subject/host Python identity differs')
    require(subject['fingerprint'] == host['fingerprint'] and
            subject['fingerprint']['abi'] == bundle_api.ABI,
            'Subject/host source recipes or ABI differ')
    require(all(subject['files'][name] == host['files'][name]
                for name in reuse.PRODUCER_FACTS.values()),
            'Subject/host native tool facts differ')
    require(interface_files(subject) == interface_files(host),
            'Subject/host headers or import libraries differ')
    # Deliberately do not compare or normalize DLL/executable bytes. Subject
    # runtime identity is bound independently and exercised by the native gate.


def verify_subject(root, subject, provenance, host_bundle, host_provenance, captured):
    host = bundle_api.verify_restored(root, host_bundle, host_provenance)
    lock = evidence._strict_json(receipt._plain_path(root, 'tools/windows-dependency-bundle.lock.json'))
    entry = select_subject(lock, captured)
    selection = entry['selection']
    subject = receipt._workspace_root(subject)
    require(not provenance.resolve().is_relative_to(subject.resolve()),
            'Subject provenance must be outside its artifact')
    authority = evidence._strict_json(provenance)
    expected = dict(selection, schemaVersion=1, workflowPath=bundle_api.WORKFLOW,
                    job=bundle_api.JOB, conclusion='success')
    require(set(authority) == set(expected) | {'bundleSha256'} and
            all(authority[key] == value for key, value in expected.items()) and
            isinstance(authority['bundleSha256'], str) and
            receipt.SHA256.fullmatch(authority['bundleSha256']),
            'Subject authenticated producer envelope differs from reviewed selection')
    manifest = receipt._plain_path(subject, 'bundle.json')
    require(receipt._digest(manifest) == authority['bundleSha256'],
            'Subject manifest differs from authenticated artifact')
    document = evidence._strict_json(manifest)
    require(set(document) == {'schemaVersion', 'producer', 'workspaceRoot', 'fingerprint',
            'roots', 'files', 'toolchainSha256', 'python', 'runner'} and
            type(document['schemaVersion']) is int and document['schemaVersion'] == 1,
            'Unsupported subject bundle schema')
    bundle_api._validate_identity(document['producer'])
    require(all(document['producer'][key] == authority[key] for key in document['producer']) and
            authority['artifactName'] == f"windows-dependencies-{document['fingerprint']['sha256']}-run{authority['runId']}-attempt{authority['runAttempt']}",
            'Subject manifest producer differs')
    require(document['runner'] == entry['runner'], 'Subject original runner differs')
    require(dependency_authority(document) == captured, 'Subject differs from original capture authority')
    payload = receipt._plain_path(subject, 'payload')
    require(document['roots'] == bundle_api._roots(payload), 'Subject roots differ')
    files = receipt._inventory(payload, document['roots'])
    whole = receipt._inventory(subject, ['payload'])
    require(files == document['files'] and
            {name[len('payload/'):]: value for name, value in whole.items()} == files,
            'Subject payload has missing, changed, extra or aliased files')
    require(bundle_api.fingerprint(payload) == document['fingerprint'], 'Subject recipe fingerprint differs')
    require(bundle_api._toolchain_digest(document) == document['toolchainSha256'],
            'Subject original toolchain digest differs')
    compare_host_subject(host, document)
    verify_subject_producers(payload, document)
    return document



def verify_subject_producers(payload, document):
    """Run existing inert producer checks on an explicit path-only view.

    The original curl receipt points into the original workspace, where the
    host SDK now lives. Validate those logical paths first; never read runtime
    through them. Only a temporary validation copy receives subject locations.
    Original manifests, hashes, runtime bytes and upstream evidence stay intact.
    """
    manifests = {kind: evidence._strict_json(receipt._plain_path(payload, f'{prefix}/{kind}-build.json'))
                 for kind, prefix in stage.PREFIX.items()}
    workspace = PureWindowsPath(document['workspaceRoot'])
    require(workspace.is_absolute(), 'Subject original workspace must be absolute')
    curl = manifests['curl']
    for kind in ('openssl', 'zlib'):
        require(PureWindowsPath(curl['dependencies'][kind]['prefix']) == workspace / stage.PREFIX[kind],
                'Subject original dependency prefix differs: ' + kind)
    require(PureWindowsPath(curl['buildSteps']['certificateTool']['path']) ==
            workspace / stage.PREFIX['openssl'] / 'bin/openssl.exe',
            'Subject original certificate tool path differs')
    for kind, manifest in manifests.items():
        for name, record in manifest['outputs'].items():
            bundle_api.curl_package.verify_file(receipt._plain_path(payload, f'{stage.PREFIX[kind]}/{name}'), record)
    with tempfile.TemporaryDirectory(prefix='plugin-subject-validate-') as temporary:
        # Same flat package view as verify_producers, with exactly three path
        # replacements. Dependency manifest hashes and all other fields remain
        # original. No validation view is used by the native probe/runtime.
        flat = Path(temporary)
        derived = copy.deepcopy(curl)
        prefixes = {kind: receipt._plain_path(payload, stage.PREFIX[kind]) for kind in ('openssl', 'zlib')}
        for kind, prefix in prefixes.items():
            derived['dependencies'][kind]['prefix'] = str(prefix)
        derived['buildSteps']['certificateTool']['path'] = str(prefixes['openssl'] / 'bin/openssl.exe')
        for kind, prefix in stage.PREFIX.items():
            if kind == 'curl':
                (flat / 'curl-build.json').write_text(json.dumps(derived), encoding='utf-8')
            else:
                shutil.copyfile(payload / prefix / f'{kind}-build.json', flat / f'{kind}-build.json')
            for source in (payload / prefix / 'bin').glob('*.dll'):
                shutil.copyfile(source, flat / source.name)
        bundle_api.openssl_package.verify_openssl_package_inputs(flat, payload / 'tools/windows-openssl.lock.json',
            payload / 'build/dependency-downloads/openssl-3.5.9.tar.gz', payload / 'docs/third-party/OpenSSL-3.5.9')
        bundle_api.curl_package.verify_curl_package_inputs(flat, payload / 'build/dependency-downloads',
                                                        payload / 'docs/third-party')
        bundle_api.curl_package.verify_manifest(flat, 'curl', dependency_prefixes=prefixes)
    imports = curl['importOutput']
    require(all(re.search(rf'(?im)^\s*{re.escape(name)}\s*$', imports)
                for name in ('libssl-3.dll', 'libcrypto-3.dll', 'zlib1.dll')) and
            not re.search(r'(?i)ssleay32\.dll|libeay32\.dll', imports), 'Subject curl closure incomplete or legacy')
    evidence._require_upstream_tests(payload, manifests, evidence.DEFAULT_ZLIB_SOURCE_VERIFICATION)
    for kind, name in reuse.PRODUCER_FACTS.items():
        facts = evidence._strict_json(receipt._plain_path(payload, name))
        require(facts.get('schemaVersion') == 1 and facts.get('kind') == kind,
                'Subject producer native tool receipt missing')


def verify_probe_tools(paths, subject, payload):
    """Compare actual CMake-selected tools to both SDKs' equal native facts."""
    records = {}
    def collect(value):
        if isinstance(value, dict):
            if {'path', 'sha256', 'bytes'} <= set(value):
                key = str(PureWindowsPath(value['path'])).casefold()
                record = {k: value[k] for k in ('bytes', 'sha256')}
                require(key not in records or records[key] == record, 'Conflicting native tool identities')
                records[key] = record
            for child in value.values():
                collect(child)
        elif isinstance(value, list):
            for child in value:
                collect(child)
    for name in reuse.PRODUCER_FACTS.values():
        collect(evidence._strict_json(receipt._plain_path(payload, name)))
    python = subject['python']
    # Python's bundle identity has its own schema, rather than a native fact.
    require(len(paths) == 5, 'Expected compiler, dumpbin, CMake, Python and PowerShell')
    for path in paths:
        actual = {'bytes': path.stat().st_size, 'sha256': receipt._digest(path)}
        key = str(PureWindowsPath(path)).casefold()
        if key == str(PureWindowsPath(python['executable'])).casefold():
            require(actual['sha256'] == python['sha256'], 'Selected Python differs from subject')
        else:
            require(records.get(key) == actual, 'Selected native tool differs from subject: ' + str(path))

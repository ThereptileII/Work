#!/usr/bin/env python3
"""Immutable cross-run dependency transport, separate from same-job receipts.

The trusted CI downloader must authenticate the run/job/artifact using GitHub's
API and write the provenance envelope OUTSIDE the downloaded bundle. This module
checks that binding; a JSON file delivered inside an artifact is not authority.
Paths are intentionally not relocatable: native tool reprobes remain mandatory.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import zlib

import curl_package
import openssl_package
import windows_dependency_evidence as evidence
import windows_dependency_receipt as receipt
import windows_dependency_reuse as reuse
import windows_dependency_stage as stage

WORKFLOW = '.github/workflows/skager-windows-dependencies.yml'
JOB = 'windows-dependencies'
VERIFIER = 'tools/windows_dependency_bundle.py'
COMPATIBILITY = 'tools/windows-dependency-verifier-compatibility.json'
ABI = {'architecture': 'Win32', 'abi': 'x86', 'runtime': 'MultiThreadedDLL (/MD)',
       'openssl': '3.5.9', 'zlib': '1.3.2', 'curl': '8.22.0'}
# Only producer-relevant bytes: application/UI patches do not invalidate this
# closure. Options and generated wrapper recipes live in the producer scripts.
INPUTS = tuple(sorted({
    'tools/' + name for name in (
        'build-pristine-windows.ps1', 'build-openssl-windows.ps1',
        'build-zlib-windows.ps1', 'build-curl-windows.ps1',
        'windows-parent-environment.ps1', 'windows_gettext.py',
        'windows-curl-environment.ps1', 'windows-curl-import-layout.cmake',
        'windows-native-tool-facts.ps1', 'windows-native-tool-facts.cmake',
        'windows-openssl.lock.json', 'windows-zlib.lock.json', 'windows-curl.lock.json',
        'test-curl-source-preflight.ps1', 'test-zlib-source-verification.ps1',
        'patch-curl-test-openssl.py', 'openssl_package.py', 'curl_package.py',
        'windows_dependency_bundle.py', 'windows_dependency_receipt.py',
        'windows_dependency_evidence.py', 'windows_dependency_stage.py',
        'windows_dependency_reuse.py',
    )
} | {f'docs/third-party/{kind}/{name}'
     for kind in ('OpenSSL-3.5.9', 'zlib-1.3.2', 'curl-8.22.0')
     for name in ('LICENSE.txt', 'provenance.json')}))
LOGS = (
    'evidence/local/windows-openssl-native-output.log',
    'evidence/local/windows-zlib-1.3.2/windows-zlib-native-output.log',
    'evidence/local/windows-curl-native-output.log',
    evidence.DEFAULT_ZLIB_SOURCE_VERIFICATION,
    'evidence/local/windows-curl-source-preflight',
    'evidence/local/windows-curl-certificate-patch.json',
    'evidence/local/windows-curl-import-Release.txt',
)
CMAKE_BUILDS = ('build/windows-zlib-1.3.2/cmake-build', 'build/windows-curl-8.22.0/build')


def _workflow_path(root: Path) -> Path:
    # Published monorepo stores the software under opennav-x; never let a
    # restored nested .github snapshot shadow its authoritative parent recipe.
    owner = root.parent if root.name == 'opennav-x' else root
    return receipt._plain_path(receipt._workspace_root(owner), WORKFLOW)


def _current_inventory(root: Path, roots: list[str]) -> dict:
    files = receipt._inventory(root, [name for name in roots if name != WORKFLOW])
    workflow = _workflow_path(root)
    files[WORKFLOW] = {'bytes': workflow.stat().st_size, 'sha256': receipt._digest(workflow)}
    return dict(sorted(files.items()))


def fingerprint(root: Path) -> dict:
    root = receipt._workspace_root(root)
    inputs = {name: receipt._digest(receipt._plain_path(root, name)) for name in INPUTS}
    value = {'abi': ABI, 'inputs': inputs, 'workflowSha256': receipt._digest(_workflow_path(root))}
    return {**value, 'sha256': hashlib.sha256(_encode(value)).hexdigest()}


def _python_identity():
    executable = Path(sys.executable).resolve(strict=True)
    return {'executable': str(executable), 'sha256': receipt._digest(executable),
            'version': sys.version, 'zlibCompile': zlib.ZLIB_VERSION,
            'zlibRuntime': zlib.ZLIB_RUNTIME_VERSION,
            'zlibNg': getattr(zlib, 'ZLIBNG_VERSION', None)}


def _runner_identity():
    values = {name: os.environ.get(name, '') for name in
              ('ImageOS', 'ImageVersion', 'RUNNER_OS', 'RUNNER_ARCH')}
    if (values['RUNNER_OS'] != 'Windows' or values['RUNNER_ARCH'] != 'X64' or
            not re.fullmatch(r'win[0-9]+', values['ImageOS']) or
            not re.fullmatch(r'[0-9]+(?:\.[0-9]+)+', values['ImageVersion'])):
        raise ValueError('exact hosted Windows image identity is required')
    return values


def _toolchain_digest(document):
    return hashlib.sha256(_encode({
        'facts': {name: document['files'][name] for name in reuse.PRODUCER_FACTS.values()},
        'python': document['python'], 'runner': document['runner']})).hexdigest()


def _encode(value):
    return (json.dumps(value, sort_keys=True, separators=(',', ':')) + '\n').encode()


def _identity(root: Path) -> dict:
    if os.environ.get('GITHUB_ACTIONS') != 'true' or os.name != 'nt':
        raise ValueError('sealing requires the native Windows GitHub producer')
    identity = {name: os.environ.get(env, '') for name, env in (
        ('repository', 'GITHUB_REPOSITORY'), ('runId', 'GITHUB_RUN_ID'),
        ('runAttempt', 'GITHUB_RUN_ATTEMPT'), ('headSha', 'GITHUB_SHA'), ('job', 'GITHUB_JOB'))}
    workflow_ref = os.environ.get('GITHUB_WORKFLOW_REF', '')
    if not workflow_ref.startswith(identity['repository'] + '/' + WORKFLOW + '@'):
        raise ValueError('unapproved dependency producer workflow')
    identity['workflowPath'] = WORKFLOW
    _validate_identity(identity)
    head = subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip()
    if head != identity['headSha']:
        raise ValueError('producer checkout differs from GitHub SHA')
    return identity


def _validate_identity(value):
    if not isinstance(value, dict) or set(value) != {
        'repository', 'runId', 'runAttempt', 'headSha', 'job', 'workflowPath'}:
        raise ValueError('incomplete producer identity')
    if (not re.fullmatch(r'[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+', value['repository']) or
            value['workflowPath'] != WORKFLOW or value['job'] != JOB or
            not re.fullmatch(r'[0-9a-f]{40}', value['headSha']) or
            any(not re.fullmatch(r'[1-9][0-9]{0,19}', value[key]) for key in ('runId', 'runAttempt'))):
        raise ValueError('unapproved producer identity')


def _roots(root: Path) -> list[str]:
    roots = [WORKFLOW, *evidence.PREFIXES.values(), *LOGS, *reuse.PRODUCER_FACTS.values()]
    roots += list(INPUTS)
    for name in ('openssl', 'zlib', 'curl'):
        lock = evidence._strict_json(root / f'tools/windows-{name}.lock.json')
        roots.append('build/dependency-downloads/' + lock['archive'])
    # NASM can already be supplied by the image. If a local fallback was used,
    # transport the reviewed host-tool archive, license and executable together.
    local_nasm = root / 'build/dependency-tools/nasm-3.02'
    if local_nasm.exists():
        roots.append('build/dependency-tools/nasm-3.02')
        nasm = evidence._strict_json(root / 'tools/windows-openssl.lock.json')['buildTools']['nasm']
        roots.append('build/dependency-downloads/' + nasm['archive'])
    for build in CMAKE_BUILDS:
        roots += [f'{build}/CMakeCache.txt', f'{build}/xnav-native-cmake-tools.txt']
        directory = receipt._plain_path(root, build)
        for pattern in ('CMakeCCompiler.cmake', 'CMakeCXXCompiler.cmake',
                        'zlib.vcxproj' if 'zlib' in build else 'libcurl_shared.vcxproj'):
            matches = sorted(directory.rglob(pattern))
            if len(matches) != 1 and not (pattern == 'CMakeCXXCompiler.cmake' and not matches):
                raise ValueError(f'missing or ambiguous native compiler/project metadata: {pattern}')
            roots += [p.relative_to(root).as_posix() for p in matches]
    return sorted(roots)


def verify_producers(root: Path) -> dict:
    """Use the existing packaging validators without an application install."""
    root = receipt._workspace_root(root)
    for kind, relative in evidence.PREFIXES.items():
        manifest = evidence._strict_json(receipt._plain_path(root, f'{relative}/{kind}-build.json'))
        for name, record in manifest['outputs'].items():
            curl_package.verify_file(receipt._plain_path(root, f'{relative}/{name}'), record)
    # Packaging validators expect a flat install. This temporary validation view
    # contains dependency files only and never becomes an application install.
    with tempfile.TemporaryDirectory(prefix='dependency-validate-') as temporary:
        flat = Path(temporary)
        for kind, relative in evidence.PREFIXES.items():
            shutil.copyfile(root / relative / f'{kind}-build.json', flat / f'{kind}-build.json')
            for source in (root / relative / 'bin').glob('*.dll'):
                shutil.copyfile(source, flat / source.name)
        opened = openssl_package.verify_openssl_package_inputs(
            flat, root / 'tools/windows-openssl.lock.json',
            root / 'build/dependency-downloads/openssl-3.5.9.tar.gz',
            root / 'docs/third-party/OpenSSL-3.5.9')
        curled = curl_package.verify_curl_package_inputs(flat, root / 'build/dependency-downloads',
                                                       root / 'docs/third-party')
    manifests = {'openssl': opened['manifest'], **curled['manifests']}
    curl_package.verify_manifest(root / evidence.PREFIXES['curl'], 'curl', dependency_prefixes={
        name: root / evidence.PREFIXES[name] for name in ('openssl', 'zlib')})
    imports = manifests['curl']['importOutput']
    if any(not re.search(rf'(?im)^\s*{re.escape(name)}\s*$', imports)
           for name in ('libssl-3.dll', 'libcrypto-3.dll', 'zlib1.dll')) or re.search(
               r'(?i)ssleay32\.dll|libeay32\.dll', imports):
        raise ValueError('producer curl closure incomplete or legacy')
    evidence._require_upstream_tests(root, manifests, evidence.DEFAULT_ZLIB_SOURCE_VERIFICATION)
    for kind, name in reuse.PRODUCER_FACTS.items():
        facts = evidence._strict_json(receipt._plain_path(root, name))
        if facts.get('schemaVersion') != 1 or facts.get('kind') != kind:
            raise ValueError('missing producer native tool receipt')
    return manifests


def seal(root: Path, output: Path, *, producer_success: bool):
    root = receipt._workspace_root(root)
    if not producer_success:
        raise ValueError('explicit successful native producer completion required')
    identity = _identity(root)
    if output.exists() or output.is_symlink():
        raise ValueError('immutable bundle destination already exists')
    verify_producers(root)
    roots = _roots(root)
    files = _current_inventory(root, roots)
    document = {'schemaVersion': 1, 'producer': identity, 'workspaceRoot': str(root),
                'fingerprint': fingerprint(root), 'roots': roots, 'files': files,
                'python': _python_identity(), 'runner': _runner_identity()}
    # Toolchain/tool observations and all payload bytes are bound into the
    # sealed digest; restore compares current input bytes before any execution.
    document['toolchainSha256'] = _toolchain_digest(document)
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.dependency-seal-', dir=output.parent) as temporary:
        temp = Path(temporary)
        payload = temp / 'payload'
        payload.mkdir()
        for name, record in files.items():
            destination = payload / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(_workflow_path(root) if name == WORKFLOW else receipt._plain_path(root, name), destination)
            curl_package.verify_file(destination, record)
        if receipt._inventory(payload, roots) != files:
            raise ValueError('producer files changed during sealing')
        (temp / 'bundle.json').write_bytes(_encode(document))
        # Move out of the temporary container, preserving the no-overwrite rule.
        if output.exists():
            raise ValueError('immutable bundle destination appeared during seal')
        shutil.move(str(temp), str(output))
    print(json.dumps({'bundleSha256': receipt._digest(output / 'bundle.json'),
                      'fingerprint': document['fingerprint']['sha256'], 'producer': identity}))


def _verifier_compatibility(root: Path, document: dict) -> dict:
    """Return the one explicitly reviewed current verifier record, if needed.

    Producer receipts always retain the original verifier bytes. This exception
    authorizes a stricter consumer boundary; it never changes compiler recipes,
    producer evidence, SDK files, or the original artifact fingerprint.
    """
    actual = receipt._plain_path(root, VERIFIER)
    record = {'bytes': actual.stat().st_size, 'sha256': receipt._digest(actual)}
    recorded = document['files'][VERIFIER]
    if record == recorded:
        return {}
    policy = evidence._strict_json(receipt._plain_path(root, COMPATIBILITY))
    if (set(policy) != {'schemaVersion', 'entries'} or type(policy['schemaVersion']) is not int or
            policy['schemaVersion'] != 1 or not isinstance(policy['entries'], list)):
        raise ValueError('invalid explicit verifier compatibility policy')
    matches = []
    for entry in policy['entries']:
        if (not isinstance(entry, dict) or set(entry) != {
                'producerCommit', 'originalVerifierSha256', 'currentVerifierSha256', 'reason'} or
                not re.fullmatch(r'[0-9a-f]{40}', entry['producerCommit']) or
                any(not receipt.SHA256.fullmatch(entry[key]) for key in
                    ('originalVerifierSha256', 'currentVerifierSha256')) or not entry['reason']):
            raise ValueError('invalid explicit verifier compatibility entry')
        if (entry['producerCommit'] == document['producer']['headSha'] and
                entry['originalVerifierSha256'] == recorded['sha256'] and
                entry['currentVerifierSha256'] == record['sha256']):
            matches.append(entry)
    if len(matches) != 1:
        raise ValueError('current verifier differs without exact reviewed compatibility')
    return {VERIFIER: record}


def _expected_current_files(root: Path, document: dict) -> dict:
    return {**document['files'], **_verifier_compatibility(root, document)}


def verify_bundle(root: Path, bundle: Path, provenance: Path) -> dict:
    root = receipt._workspace_root(root)
    bundle = receipt._workspace_root(bundle)
    if provenance.resolve().is_relative_to(bundle.resolve()):
        raise ValueError('provenance must be authenticated outside the artifact')
    authority = evidence._strict_json(provenance)
    expected_keys = {'schemaVersion', 'repository', 'workflowPath', 'runId', 'runAttempt',
                     'headSha', 'job', 'conclusion', 'artifactId', 'artifactName',
                     'artifactDigest', 'bundleSha256'}
    if (set(authority) != expected_keys or type(authority['schemaVersion']) is not int or
            authority['schemaVersion'] != 1 or authority['conclusion'] != 'success' or
            not re.fullmatch(r'[1-9][0-9]{0,19}', str(authority['artifactId'])) or
            not re.fullmatch(r'sha256:[0-9a-f]{64}', authority['artifactDigest']) or
            not receipt.SHA256.fullmatch(authority['bundleSha256'])):
        raise ValueError('incomplete authenticated producer envelope')
    expected_repository = os.environ.get('GITHUB_REPOSITORY')
    if not expected_repository or authority['repository'] != expected_repository:
        raise ValueError('producer belongs to a different repository')
    manifest = receipt._plain_path(bundle, 'bundle.json')
    if receipt._digest(manifest) != authority['bundleSha256']:
        raise ValueError('bundle manifest differs from authenticated artifact')
    document = receipt._read_json(manifest)
    if (set(document) != {'schemaVersion', 'producer', 'workspaceRoot', 'fingerprint',
                          'roots', 'files', 'toolchainSha256', 'python', 'runner'} or
            type(document['schemaVersion']) is not int or document['schemaVersion'] != 1):
        raise ValueError('unsupported immutable bundle schema')
    _validate_identity(document['producer'])
    if any(document['producer'][key] != authority[key] for key in document['producer']):
        raise ValueError('bundle producer differs from authenticated producer')
    if authority['artifactName'] != f"windows-dependencies-{document['fingerprint']['sha256']}-run{authority['runId']}-attempt{authority['runAttempt']}":
        raise ValueError('dependency artifact name does not bind inputs and run attempt')
    if document['workspaceRoot'] != str(root):
        raise ValueError('dependency workspace relocation is unsupported; rebuild required')
    if document['runner'] != _runner_identity():
        raise ValueError('current Windows runner image differs; fresh producer required')
    if document['python'] != _python_identity():
        raise ValueError('current Python build-helper toolchain differs')
    current_fingerprint = fingerprint(root)
    compatibility = _verifier_compatibility(root, document)
    if compatibility:
        # Compare every actual binary-producing input normally; substitute only
        # the exact authenticated original verifier identity for this comparison.
        current_fingerprint['inputs'][VERIFIER] = document['files'][VERIFIER]['sha256']
        comparable = {key: value for key, value in current_fingerprint.items() if key != 'sha256'}
        current_fingerprint['sha256'] = hashlib.sha256(_encode(comparable)).hexdigest()
    if document['fingerprint'] != current_fingerprint:
        raise ValueError('dependency source/recipe/configuration/ABI inputs changed')
    payload = receipt._plain_path(bundle, 'payload')
    # Derive allowed payload roots from the actual verified metadata, never
    # permit the artifact to choose arbitrary checkout destinations.
    if document['roots'] != _roots(payload):
        raise ValueError('bundle roots differ from complete dependency closure')
    files = receipt._inventory(payload, document['roots'])
    if fingerprint(payload) != document['fingerprint']:
        raise ValueError('payload recipes differ from the input fingerprint')
    # A complete whole-payload inventory additionally rejects unlisted extras.
    all_files = receipt._inventory(bundle, ['payload'])
    if ({name[len('payload/'):]: value for name, value in all_files.items()} != files or
            files != document['files']):
        raise ValueError('bundle has missing, changed, extra or aliased files')
    if document['toolchainSha256'] != _toolchain_digest(document):
        raise ValueError('bundle toolchain fingerprint differs')
    return document


def restore(root: Path, bundle: Path, provenance: Path):
    document = verify_bundle(root, bundle, provenance)
    expected_current = _expected_current_files(root, document)
    compatibility = _verifier_compatibility(root, document)
    # Validate every destination before copying. Refuse conflicting existing
    # files; routine current checkout notices may already have identical bytes.
    for name, record in document['files'].items():
        if name == WORKFLOW:
            continue  # Current authoritative workflow was checked by fingerprint.
        destination = stage._safe_destination(root, name)
        if destination.exists():
            curl_package.verify_file(destination, expected_current[name])
    for name, record in document['files'].items():
        if name == WORKFLOW or name in compatibility:
            continue  # Keep the independently verified current workflow/verifier.
        destination = stage._safe_destination(root, name)
        destination.parent.mkdir(parents=True, exist_ok=True)
        if not destination.exists():
            with destination.open('xb') as output, receipt._plain_path(bundle / 'payload', name).open('rb') as source:
                shutil.copyfileobj(source, output)
        curl_package.verify_file(destination, record)
    if _current_inventory(root, document['roots']) != _expected_current_files(root, document):
        raise ValueError('restored dependency inventory differs')
    verify_producers(root)
    print('Immutable dependency payload verified and restored; live native reprobes still required')


def verify_restored(root: Path, bundle: Path, provenance: Path) -> dict:
    document = verify_bundle(root, bundle, provenance)
    if _current_inventory(root, document['roots']) != _expected_current_files(root, document):
        raise ValueError('dependency payload changed before staging')
    verify_producers(root)
    return document


def stage_bundle(root: Path, bundle: Path, provenance: Path):
    document = verify_restored(root, bundle, provenance)
    receipt._plain_path(root, stage.CACHE)
    for kind, subdir in stage.HEADER_TREES:
        prefix = f'{stage.PREFIX[kind]}/{subdir}/'
        target_root = stage._safe_destination(root, f'{stage.CACHE}/{subdir}')
        if target_root.exists():
            stage._assert_plain_tree(target_root)
            shutil.rmtree(target_root)
        for name in document['files']:
            if name.startswith(prefix):
                stage._copy_record(root, document['files'], name,
                                   f'{stage.CACHE}/{subdir}/{name[len(prefix):]}')
    for kind, source, target in stage.FILES:
        stage._copy_record(root, document['files'], f'{stage.PREFIX[kind]}/{source}', f'{stage.CACHE}/{target}')
    for name in ('libeay32.dll', 'ssleay32.dll', 'libeay32.lib', 'ssleay32.lib'):
        target = stage._safe_destination(root, f'{stage.CACHE}/{name}')
        if target.exists():
            if not target.is_file():
                raise ValueError('legacy TLS cache path is not a file')
            target.unlink()
    print('Restaged verified cross-run Win32 dependency prefixes')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('fingerprint', 'seal', 'verify', 'restore', 'stage'))
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--bundle', type=Path)
    parser.add_argument('--provenance', type=Path)
    parser.add_argument('--producer-success', action='store_true')
    args = parser.parse_args(argv)
    try:
        if args.mode == 'fingerprint':
            print(fingerprint(args.root)['sha256'])
        elif args.mode == 'seal':
            if not args.output:
                raise ValueError('--output required')
            seal(args.root, args.output, producer_success=args.producer_success)
        else:
            if not args.bundle or not args.provenance:
                raise ValueError('--bundle and --provenance required')
            {'verify': verify_bundle, 'restore': restore, 'stage': stage_bundle}[args.mode](
                args.root, args.bundle, args.provenance)
    except (OSError, ValueError, TypeError, KeyError, subprocess.CalledProcessError) as error:
        print(f'Immutable Windows dependency bundle rejected: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

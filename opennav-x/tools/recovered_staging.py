"""Exact 0db prepackage recovery, with distinct compilation and packaging origins.

This is not the historical successful-producer staging contract. Its archive is
unqualified until the complete existing native qualifier succeeds. No release or
publication action is provided here.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import zipfile

import staging_build_inputs as sealed

KIND = 'skager-recovered-prepackage-staging-inputs'
ARCHIVE = 'RECOVERED_STAGING_INPUTS.zip'
MANIFEST = 'RECOVERED_STAGING_INPUTS.json'
PRODUCT = '0db45cb92509c29ef4d1fc33d0c76dc7cf511b97'
ROOT = Path(__file__).resolve().parents[1]
RESTORE_RECORD = 'evidence/local/compiled-recovery-origin.json'
RECEIPT = 'evidence/local/staging-inputs.json'
REPORT = 'evidence/local/compiled-recovery-qualification.json'


def read(path):
    path = Path(path)
    sealed.require(path.is_file() and not path.is_symlink() and path.stat().st_size <= sealed.MAX_MANIFEST,
                   'Regular bounded recovery record required')
    return sealed.strict_json(path.read_bytes())


def write(path, record):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x', encoding='utf-8') as stream:
        stream.write(json.dumps(record, indent=2) + '\n')


def current_identity(harness):
    sealed.require(os.name == 'nt' and os.environ.get('GITHUB_ACTIONS') == 'true' and
                   os.environ.get('RUNNER_ENVIRONMENT') == 'github-hosted' and
                   os.environ.get('GITHUB_REPOSITORY') == 'ThereptileII/Work' and
                   os.environ.get('GITHUB_JOB') == 'recover', 'Exact disposable recovery job required')
    head = subprocess.check_output(['git', '-C', str(harness), 'rev-parse', 'HEAD'], text=True).strip()
    sealed.require(head == os.environ.get('GITHUB_SHA') and re.fullmatch('[a-f0-9]{40}', head),
                   'Recovery harness differs from actual workflow source')
    sealed.require(subprocess.run(['git', '-C', str(harness), 'diff', '--quiet', 'HEAD', '--', '.']).returncode == 0,
                   'Tracked recovery helper source changed')
    result = dict(commit=head, runId=os.environ.get('GITHUB_RUN_ID'), runAttempt=os.environ.get('GITHUB_RUN_ATTEMPT'))
    sealed.require(all(re.fullmatch('[1-9][0-9]{0,19}', result[key] or '') for key in ('runId', 'runAttempt')),
                   'Canonical recovery run identity required')
    return result


def validate_origin(record):
    from compiled_recovery_restore import validate_receipt
    validate_receipt(record)
    sealed.require(record['producer'] == sealed.producer(PRODUCT, '37424595899', '1'),
                   'Only the specifically reviewed prepackage failure is supported')
    return record


def unchanged(root, files):
    for name, record in files.items():
        sealed.safe_name(name)
        path = sealed.plain_file(root, name)
        sealed.require(path.stat().st_size == record['size'] and sealed.sha(path) == record['sha256'],
                       'Original retained input changed: ' + name)


def validated_receipt(root, receipt_path, product_commit, harness_commit):
    root = Path(root).resolve()
    sealed.require(Path(receipt_path).resolve() == root / RECEIPT, 'Unexpected recovered input receipt location')
    record = read(receipt_path)
    sealed.require(record.get('schema') == 1 and type(record.get('schema')) is int and
                   record.get('kind') == KIND and record.get('status') == 'restored' and
                   record.get('qualification') == 'not-run' and
                   record.get('workspaceRoot') == str(root) and record.get('harnessCommit') == harness_commit and
                   product_commit == PRODUCT and record.get('packaging') == current_identity(root),
                   'Recovery qualification identity differs')
    origin_path = sealed.plain_file(root, RESTORE_RECORD)
    sealed.require(sealed.sha(origin_path) == record.get('originReceiptSha256'), 'Original recovery receipt changed')
    origin = validate_origin(read(origin_path))
    sealed.require(record.get('producer') == origin['producer'] and record['packaging'] == origin['recovery'],
                   'Compilation and packaging origins differ')
    records = record.get('files')
    sealed.require(isinstance(records, dict) and records, 'Missing recovered input inventory')
    sealed.validate_inventory(list(records))
    sealed.require(all(isinstance(item, dict) and set(item) == {'size', 'sha256'} and
                       type(item['size']) is int and 0 <= item['size'] <= sealed.MAX_FILE and
                       re.fullmatch('[a-f0-9]{64}', item['sha256'] or '') for item in records.values()),
                   'Invalid recovered input inventory')
    unchanged(root, records)
    sealed.require(record.get('boatFeedback') == sealed.feedback_binding(root, PRODUCT) and
                   record.get('sourceArchiveSha256') == sealed.sha(root / 'build/developer-preview/SKAGER-Beta2-source.zip'),
                   'Recovered compiled components or corresponding source changed')
    return record


def prepare(product_root, origin_path, harness_root=ROOT):
    product_root, harness_root = Path(product_root).resolve(), Path(harness_root).resolve()
    identity = current_identity(harness_root)
    origin = validate_origin(read(origin_path))
    sealed.require(origin['workspaceRoot'] == str(product_root) and origin['recovery'] == identity,
                   'Restoration belongs to another source tree or workflow run')
    unchanged(product_root, origin['files'])
    names = sealed.inventory(product_root)
    sealed.validate_inventory(names)
    sealed.validate_content(product_root, names, origin['producer'])
    product = read(product_root / sealed.PACKAGE_ROOT / 'docs/PRODUCT_BUILD.json')
    sealed.require(product.get('compiled_ci_run') == origin['producer']['runId'] and
                   product.get('packaging_ci_run') == identity['runId'] and
                   product.get('packaging_helper_commit') == identity['commit'],
                   'Package metadata conflates compiled and packaging origins')
    for name in names:
        target = harness_root / name
        sealed.require(not target.exists() and not target.is_symlink(), 'Qualification inputs must be fresh')
    output = harness_root / 'build/recovered-staging'
    sealed.require(not output.exists(), 'Recovery archive output must be fresh')
    output.mkdir(parents=True)
    records, total = {}, 0
    with zipfile.ZipFile(output / ARCHIVE, 'x', zipfile.ZIP_DEFLATED, compresslevel=1) as archive:
        for name in names:
            source = sealed.plain_file(product_root, name)
            size = source.stat().st_size; total += size
            sealed.require(size <= sealed.MAX_FILE and total <= sealed.MAX_TOTAL, 'Recovery staging size bound exceeded')
            digest = sealed.sha(source)
            target = harness_root / name; target.parent.mkdir(parents=True, exist_ok=True)
            with source.open('rb') as src, target.open('xb') as dest:
                shutil.copyfileobj(src, dest, 1024 * 1024)
            sealed.require(target.stat().st_size == size and sealed.sha(target) == digest,
                           'Recovery input changed during preparation')
            records[name] = dict(size=size, sha256=digest)
            with target.open('rb') as src, archive.open(sealed.archive_entry(name), 'w', force_zip64=True) as dest:
                shutil.copyfileobj(src, dest, 1024 * 1024)
        manifest = dict(schema=1, kind=KIND, qualification='not-run', producer=origin['producer'],
                        packaging=identity, originReceiptSha256=sealed.sha(origin_path), files=records)
        encoded = sealed.canonical(manifest)
        sealed.require(len(encoded) <= sealed.MAX_MANIFEST, 'Recovered staging manifest exceeds bound')
        archive.writestr(sealed.archive_entry(MANIFEST), encoded)
    unchanged(product_root, origin['files'])
    origin_target = harness_root / RESTORE_RECORD
    origin_target.parent.mkdir(parents=True, exist_ok=True)
    with Path(origin_path).open('rb') as src, origin_target.open('xb') as dest:
        shutil.copyfileobj(src, dest)
    receipt = dict(manifest, status='restored', workspaceRoot=str(harness_root), harnessCommit=identity['commit'],
                   archiveSha256=sealed.sha(output / ARCHIVE), manifestSha256=hashlib.sha256(encoded).hexdigest(), sourceArchiveSha256=sealed.sha(harness_root / 'build/developer-preview/SKAGER-Beta2-source.zip'),
                   boatFeedback=sealed.feedback_binding(harness_root, PRODUCT))
    write(harness_root / RECEIPT, receipt)
    write(output / 'receipt.json', receipt)
    validated_receipt(harness_root, harness_root / RECEIPT, PRODUCT, identity['commit'])
    return receipt


def qualify(root=ROOT):
    root = Path(root).resolve()
    sealed.require(not any(os.environ.get(name) for name in ('GH_TOKEN', 'GITHUB_TOKEN', 'OPENNAV_ARTIFACT_TOKEN')),
                   'Remove Actions credentials before application qualification')
    identity = current_identity(root)
    receipt = validated_receipt(root, root / RECEIPT, PRODUCT, identity['commit'])
    report = dict(schema=1, kind=KIND, status='running', scope='complete-native-staging', productCommit=PRODUCT,
                  compiledOrigin=receipt['producer'], packagingOrigin=receipt['packaging'],
                  archiveSha256=receipt['archiveSha256'], receiptSha256=sealed.sha(root / RECEIPT),
                  publicAccess=False, releaseQualification=False, actualBoat=False)
    path = root / REPORT
    write(path, report)
    try:
        subprocess.run(['pwsh', '-NoProfile', '-File', str(root / 'tools/qualify-staging-windows.ps1'),
                        '-ProductCommit', PRODUCT, '-CompiledRetest'], check=True)
        validated_receipt(root, root / RECEIPT, PRODUCT, identity['commit'])
        report['status'] = 'passed'
        report['files'] = receipt['files']
    except BaseException:
        report['status'] = 'failed'
        raise
    finally:
        path.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('phase', choices=('prepare', 'qualify'))
    parser.add_argument('--product-root', type=Path)
    parser.add_argument('--origin-receipt', type=Path)
    args = parser.parse_args()
    if args.phase == 'prepare':
        sealed.require(args.product_root is not None and args.origin_receipt is not None, 'Source and restoration receipt required')
        prepare(args.product_root, args.origin_receipt)
    else:
        sealed.require(args.product_root is None and args.origin_receipt is None, 'Qualification uses only prepared inputs')
        qualify()


if __name__ == '__main__':
    main()

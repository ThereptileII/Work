"""Pinned Gitleaks checkout audit with value-free reports and an explicit gate.

Never forward subprocess output: even a scanner error may contain input bytes.
Only repository paths, scanner rule IDs, counts and fixed status text leave this
module. The report-only CI bridge is not a successful clean qualification.
"""
from __future__ import annotations

import argparse
import hashlib
import io
import json
import os
from pathlib import Path, PurePosixPath
import platform
import re
import subprocess
import tarfile
import tempfile
import urllib.request
import zipfile

TOOLS = Path(__file__).resolve().parent
# Only these two fields are written by Gitleaks; no match, secret, line, author,
# commit message or fingerprint is retained, even in its temporary report.
REPORT_TEMPLATE = ('[{{range $i, $f := .}}{{if $i}},{{end}}'
                   '{"path":{{toJson $f.File}},"rule":{{toJson $f.RuleID}}}{{end}}]')


class ScanError(Exception):
    """An error whose message is constant and safe to publish."""


def load_scanner(directory: Path, archive_path: Path | None = None) -> Path:
    lock = json.loads((TOOLS / 'secret-scanner.lock.json').read_text())
    system = {'Linux': 'linux', 'Windows': 'windows'}.get(platform.system())
    if not system or platform.machine().lower() not in ('x86_64', 'amd64'):
        raise ScanError('Pinned scanner is unavailable for this platform.')
    record = lock['platforms'][system + '-x64']
    if archive_path:
        if archive_path.is_symlink() or not archive_path.is_file():
            raise ScanError('Scanner archive must be a regular file.')
        data = archive_path.read_bytes()
    else:
        url = ('https://github.com/gitleaks/gitleaks/releases/download/v' +
               lock['version'] + '/' + record['archive'])
        try:
            with urllib.request.urlopen(url, timeout=45) as response:
                data = response.read(record['bytes'] + 1)
        except Exception:
            raise ScanError('Pinned scanner download failed.') from None
    if len(data) != record['bytes'] or hashlib.sha256(data).hexdigest() != record['sha256']:
        raise ScanError('Pinned scanner archive checksum or size differs.')
    # Extract exactly one regular member; never extract an archive path to disk.
    if record['archive'].endswith('.zip'):
        with zipfile.ZipFile(io.BytesIO(data)) as archive:
            members = [m for m in archive.infolist() if m.filename == record['binary']]
            if len(members) != 1 or members[0].is_dir():
                raise ScanError('Pinned scanner archive member is ambiguous.')
            binary = archive.read(members[0])
    else:
        with tarfile.open(fileobj=io.BytesIO(data), mode='r:gz') as archive:
            members = [m for m in archive.getmembers() if m.name == record['binary']]
            if len(members) != 1 or not members[0].isfile():
                raise ScanError('Pinned scanner archive member is ambiguous.')
            binary = archive.extractfile(members[0]).read()
    executable = directory / record['binary']
    executable.write_bytes(binary)
    executable.chmod(0o700)
    result = subprocess.run([str(executable), 'version'], capture_output=True, timeout=15)
    if result.returncode or result.stdout.decode().strip() != lock['version']:
        raise ScanError('Pinned scanner version differs.')
    return executable


def scan_files(scanner: Path, files: dict[str, bytes], directory: Path) -> list[dict]:
    """Scan an inert snapshot. Opaque names prevent path-based default skipping.

    No source .gitignore/config/inline allowance or filename-extension exclusion
    can hide a tracked credential. Symlink text is supplied by snapshot(), never
    followed. Original paths are restored only in the sanitized result.
    """
    directory.mkdir()
    inputs = directory / 'inputs'
    inputs.mkdir()
    paths = {}
    for index, (name, data) in enumerate(sorted(files.items())):
        scan_name = f'{index:08d}.scan'
        (inputs / scan_name).write_bytes(data)
        paths[scan_name] = name
    report = directory / 'report.json'
    template = directory / 'report.tmpl'
    template.write_text(REPORT_TEMPLATE, encoding='utf-8')
    ignore = directory / 'empty.ignore'
    ignore.write_text('', encoding='utf-8')
    env = {key: value for key, value in os.environ.items() if not key.startswith('GITLEAKS_')}
    command = [str(scanner), 'dir', str(inputs), '--config', str(TOOLS / 'secret-scanner.toml'),
               '--report-format', 'template', '--report-template', str(template),
               '--report-path', str(report), '--redact=100', '--no-banner', '--no-color',
               '--log-level', 'error', '--ignore-gitleaks-allow',
               '--gitleaks-ignore-path', str(ignore), '--timeout', '120',
               '--max-decode-depth', '5', '--max-archive-depth', '0']
    result = subprocess.run(command, capture_output=True, timeout=135, env=env, cwd=directory)
    if result.returncode not in (0, 1) or not report.is_file():
        raise ScanError('Secret scanner failed; raw diagnostics suppressed.')
    try:
        findings = json.loads(report.read_text(encoding='utf-8'))
        sanitized = set()
        if not isinstance(findings, list):
            raise ValueError()
        for finding in findings:
            if set(finding) != {'path', 'rule'} or not re.fullmatch('[a-z0-9-]+', finding['rule']):
                raise ValueError()
            path = Path(finding['path'])
            # Actual Gitleaks emits absolute names. Require exact snapshot paths.
            if path.parent != inputs or path.name not in paths:
                raise ValueError()
            sanitized.add((paths[path.name], finding['rule']))
        if bool(findings) != (result.returncode == 1):
            raise ValueError()
    except (ValueError, KeyError, TypeError, OSError):
        raise ScanError('Secret scanner report is invalid; raw diagnostics suppressed.') from None
    return [{'path': path, 'rule': rule} for path, rule in sorted(sanitized)]


def snapshot(repo: Path) -> tuple[dict[str, bytes], list[str]]:
    result = subprocess.run(['git', '-C', str(repo), 'ls-files', '--stage', '-z'],
                            capture_output=True, timeout=30)
    if result.returncode:
        raise ScanError('Cannot enumerate tracked software checkout.')
    files, gitlinks = {}, []
    for entry in result.stdout.split(b'\0'):
        if not entry:
            continue
        metadata, name = entry.split(b'\t', 1)
        mode, _, stage = metadata.decode().split()
        name = name.decode('utf-8')
        parts = PurePosixPath(name).parts
        if (stage != '0' or name in files or not parts or '..' in parts or
                PurePosixPath(name).is_absolute() or '\\' in name or ':' in name):
            raise ScanError('Tracked file inventory is ambiguous or unsafe.')
        path = repo / name
        if any(parent.is_symlink() for parent in path.parents if parent != repo and repo in parent.parents):
            raise ScanError('Tracked file parent is a symlink.')
        if mode == '160000':
            gitlinks.append(name)
        elif mode == '120000' and path.is_symlink():
            files[name] = os.readlink(path).encode('utf-8')
        elif mode in ('100644', '100755') and path.is_file() and not path.is_symlink():
            files[name] = path.read_bytes()
        else:
            raise ScanError('Tracked file is missing or its type changed.')
    if not files:
        raise ScanError('Tracked software checkout is empty.')
    return files, sorted(gitlinks)


def self_check(scanner: Path, directory: Path) -> None:
    # Deliberately invalid fixtures generated only in temporary storage. No live
    # key is generated, accessed, printed or used against an external service.
    suffix = 'Ab3dE7gH9jK2mN4pQ6rS8tU0vW1xY5zC0D2F'
    findings = scan_files(scanner, {
        'token.txt': ('token = "ghp_' + suffix + '" # gitleaks:allow\n').encode(),
        'signing.pem': ('-----BEGIN ' + 'RSA PRIVATE KEY-----\n' +
                        'MII' + suffix * 3 + '\n-----END RSA PRIVATE KEY-----\n').encode(),
        'public.pem': ('-----BEGIN PUBLIC KEY-----\n' + suffix + '\n-----END PUBLIC KEY-----\n').encode(),
    }, directory)
    observed = {(item['path'], item['rule']) for item in findings}
    if (('token.txt', 'github-pat') not in observed or
            ('signing.pem', 'private-key') not in observed or
            any(path == 'public.pem' for path, _ in observed)):
        raise ScanError('Pinned scanner seeded-secret self-check failed.')


def audit_repository(repo: Path, archive_path: Path | None = None) -> dict:
    """Return an explicitly qualified audit; findings are never silently ignored."""
    repo = repo.resolve()
    try:
        # The repository has retained evidence; small /tmp quotas cannot hold
        # its snapshot. Use the ignored product-local scratch directory.
        project = repo / 'opennav-x' if (repo / 'opennav-x').is_dir() else repo
        scratch = project / '.local'
        if scratch.is_symlink():
            raise ScanError('Secret scanner scratch directory is a symlink.')
        scratch.mkdir(mode=0o700, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix='secret-scan-', dir=scratch) as temporary:
            directory = Path(temporary)
            scanner = load_scanner(directory, archive_path)
            self_check(scanner, directory / 'self-check')
            files, gitlinks = snapshot(repo)
            findings = scan_files(scanner, files, directory / 'checkout')
        return {'schema': 1, 'scanner': 'gitleaks', 'version': '8.30.1',
                'scope': 'tracked checkout bytes; no Git history or untracked files',
                'status': 'unreviewed-findings' if findings else 'clean-checkout',
                'qualificationPassed': not findings, 'trackedFiles': len(files),
                'gitlinksNotScanned': gitlinks, 'findings': findings}
    except ScanError:
        raise
    except Exception:
        raise ScanError('Secret scan could not complete; raw diagnostics suppressed.') from None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', type=Path, default=Path('.'))
    parser.add_argument('--scanner-archive', type=Path, help='Offline archive; same pinned digest is mandatory')
    parser.add_argument('--report-only', action='store_true', help='Inventory only; does not qualify the checkout')
    args = parser.parse_args()
    try:
        report = audit_repository(args.repo, args.scanner_archive)
    except ScanError as error:
        print(str(error))
        return 2
    print(json.dumps(report, indent=2, ensure_ascii=True))
    return 0 if args.report_only or report['qualificationPassed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

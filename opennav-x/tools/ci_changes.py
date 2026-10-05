#!/usr/bin/env python3
"""Select Staging checks from a Git range, never qualify or promote a candidate.

Local paths are project-relative; the published monorepo uses opennav-x/ and
root .github/workflows/. Workflow/test-helper-only changes run focused checks.
Unknown inputs or incomplete history request the conservative build path.

Corresponding source contains more files than the executable/runtime package.
Skipping a docs/helper-only build does NOT qualify a new source revision or
transfer an older candidate's evidence. Existing source/license gates still
apply whenever a new package is produced.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path, PurePosixPath
import re
import subprocess

FLAGS = ('product', 'helpers', 'dependencies', 'docs')
# Reviewed producer inputs from windows_dependency_reuse.INPUTS. Keep these
# ahead of the test-* rule: two producer preflight scripts also look like tests.
DEPENDENCY_TOOLS = frozenset('''
build-pristine-windows.ps1 build-openssl-windows.ps1 build-zlib-windows.ps1
build-curl-windows.ps1 windows-parent-environment.ps1 windows_gettext.py
windows-curl-environment.ps1 windows-curl-import-layout.cmake
windows-native-tool-facts.ps1 windows-native-tool-facts.cmake
windows_dependency_reuse.py windows_dependency_stage.py
windows_dependency_receipt.py windows_dependency_evidence.py
openssl_package.py curl_package.py prepare-integration.py verify-upstream.py
patch-curl-test-openssl.py test-curl-source-preflight.ps1
test-zlib-source-verification.ps1 prepare-ocharts-adapter.py
windows_dependency_bundle.py build-windows-dependency-bundle.ps1
'''.split())
PRODUCT_TOOLS = frozenset('''
source_package.py hardware_output_policy.py release_manifest.py updater_package.py
arch-gcc-compat.cmake local-env.sh wx-config-local
verify-distribution-inputs.py verify-preview-pe.py verify-skager-brand.py
verify-skager-pe-brand.py verify-ocharts-adapter-package.py
'''.split())
HELPER_TOOLS = frozenset('''
ci_changes.py staging_build_inputs.py github_release_delivery.py production-qualification.py
fetch_ci_inputs.py qualify-staging-windows.ps1 retest-staging-windows.py
staging_composition.py staging-composition-request.json staging-installer-retest.json
staging-publication-request.json
staging-qualification.py retain-release-inputs.py prepare-release-retest.py
beta2_handoff.py alpha-artifacts.py chart-render-check.py diagnostic-geometry.py
diagnostic_snapshot.py installer-welcome.py installer-deny-directory.ps1
install-official-opencpn-windows.py install-official-opencpn-fixture.ps1
inspect-official-installer-windows.py peer_boundary.py peer-cli-receipt.py
preferences-touch.py product-interaction.py profile-fixtures.py
workspace-fixtures.py restart_capability.py startup-log.py windows-ui.py
windows_chart_units.py ocharts_cmake_path_probe.py ocharts_compile_probe_inputs.py
prepare-test-profile.py prepare-stock-review-desktop.py build-installer-prior-fixture.py
extract-peer-buffer.py fetch-st4000-oracle.py soak-runtime.py
'''.split())
DOC_FILES = frozenset('''
README.md AGENTS.md FIRST_TASK.md PROJECT_GOAL.md START_HERE.md
OpenNavX_Codex_Project_Specification.md .gitignore
'''.split())
PRODUCT_PREFIXES = ('src/', 'include/', 'integration/', 'resources/', 'installer/',
                    'plugins/', 'hardware/', 'release/')


def classify_path(raw: str, layout: str) -> tuple[str, str]:
    """Only reviewed non-product scopes may suppress a build."""
    path = PurePosixPath(raw)
    if (not raw or raw.startswith('/') or '\\' in raw or
            any(part in ('', '.', '..') for part in raw.split('/'))):
        return 'unknown', 'ambiguous path'
    if raw == '.github/workflows/skager-windows-dependencies.yml':
        return 'dependency-workflow', 'producer workflow invalidates reuse; focused checks only'
    if raw.startswith('.github/workflows/'):
        return 'helpers', 'workflow scheduling/validation; no candidate qualification'
    if layout == 'monorepo':
        if raw.startswith('opennav-x/'):
            raw = raw[len('opennav-x/'):]
            path = PurePosixPath(raw)
        else:
            return 'unknown', 'unreviewed monorepo scope'
    if raw.startswith(('docs/beta2/', 'docs/third-party/')) or raw == 'LICENSE':
        return 'product', 'packaged release notes or license notices'
    if (raw in ('upstream.lock.json', '.gitmodules', '.gitattributes') or
            raw.startswith(('upstream/', 'cmake/', 'patches/'))):
        return 'dependencies', 'upstream, toolchain or integration producer input'
    if raw == 'CMakeLists.txt' or raw.startswith(PRODUCT_PREFIXES):
        return 'product', 'application, runtime resource or package input'
    if raw.startswith(('docs/', 'evidence/')) or raw in DOC_FILES:
        return 'docs', 'documentation/evidence outside runtime package'
    if raw.startswith('web/'):
        return 'other', 'independent web project; its own checks apply'
    if raw.startswith('tests/'):
        # tests/support is linked into the fixture executable; other compiled
        # tests need their build targets. Until dedicated targets are selected,
        # this directory cannot safely use the delivery-helper-only path.
        return 'product', 'compiled test/fixture scope requires application build checks'
    if raw.startswith('tools/'):
        name = path.name
        if raw.startswith('tools/update-verifier/'):
            return 'product', 'installed secure startup updater and its pinned source'
        if raw.startswith(('tools/boat/', 'tools/prototype/', 'tools/fixtures/',
                           'tools/diagnostics/', 'tools/downloader-trust-shim/')):
            return 'helpers', 'focused test/retest helper; retain candidate identity'
        if name.endswith('.md'):
            return 'docs', 'tool documentation outside runtime package'
        if name in DEPENDENCY_TOOLS or (path.parent == PurePosixPath('tools') and
                                       name.endswith('.lock.json')):
            return 'dependencies', 'reviewed dependency producer/lock input'
        if (name in PRODUCT_TOOLS or name.startswith(('package-', 'build-', 'generate-',
                                                     'derive-', 'chart_'))):
            return 'product', 'build, generated runtime content or packaging recipe'
        if (name in HELPER_TOOLS or name.startswith(('test-', 'smoke-', 'capture-',
                                                    'qualify-', 'check-'))):
            return 'helpers', 'focused test/retest helper; retain candidate identity'
    return 'unknown', 'unreviewed input; conservative product/dependency path'


def classify(paths: list[str], layout: str = 'local', *, force_product: bool = False,
             error: str | None = None) -> dict:
    result = {'schema': 1, **dict.fromkeys(FLAGS, False),
              'changed_paths': sorted(set(paths)), 'reasons': []}
    for path in result['changed_paths']:
        category, reason = classify_path(path, layout)
        result['reasons'].append({'path': path, 'category': category, 'reason': reason})
        if category in FLAGS:
            result[category] = True
        if category == 'dependency-workflow':
            result['helpers'] = result['dependencies'] = True
        if category in ('dependencies', 'unknown'):
            result['product'] = result['dependencies'] = True
        if category == 'unknown':
            result['helpers'] = True
    if error:
        result.update(product=True, dependencies=True, helpers=True)
        result['reasons'].append({'category': 'fallback', 'reason': error})
    if force_product:
        result.update(product=True, dependencies=True)
        result['reasons'].append({'category': 'manual', 'reason': 'explicit product build requested'})
    return result


def git(repo: Path, *args: str) -> bytes:
    return subprocess.run(['git', '-C', str(repo), *args], check=True,
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                          timeout=60).stdout


def resolve(repo: Path, revision: str) -> str:
    if not revision or revision.startswith('-') or '\0' in revision:
        raise ValueError('missing or unsafe revision')
    commit = git(repo, 'rev-parse', '--verify', '--end-of-options',
                 revision + '^{commit}').decode('ascii').strip()
    if re.fullmatch(r'[0-9a-f]{40}|[0-9a-f]{64}', commit) is None:
        raise ValueError('invalid resolved commit')
    return commit


def parse_changes(data: bytes) -> list[str]:
    if not data:
        return []
    if not data.endswith(b'\0'):
        raise ValueError('incomplete Git change record')
    fields = data[:-1].split(b'\0')
    paths = []
    i = 0
    while i < len(fields):
        status = fields[i].decode('ascii')
        i += 1
        if re.fullmatch(r'[ADMT]|[RC][0-9]+', status) is None:
            raise ValueError('ambiguous Git change status')
        count = 2 if status.startswith(('R', 'C')) else 1
        if i + count > len(fields):
            raise ValueError('incomplete Git change paths')
        paths.extend(value.decode('utf-8') for value in fields[i:i + count])
        i += count
    return paths


def select(repo: Path, base: str, head: str, layout: str = 'local', *,
           merge_base: bool = False, force_product: bool = False) -> dict:
    try:
        base_commit, head_commit = resolve(repo, base), resolve(repo, head)
        # A rewritten/disconnected push is ambiguous. For PRs the caller may
        # explicitly compare the merge base; do not silently change push meaning.
        ancestor = git(repo, 'merge-base', base_commit, head_commit).decode('ascii').strip()
        if merge_base:
            base_commit = ancestor
        elif ancestor != base_commit:
            raise ValueError('base is not an ancestor of head')
        changes = parse_changes(git(repo, 'diff', '--no-ext-diff', '--no-textconv',
                                    '--name-status', '-z', '--find-renames',
                                    base_commit, head_commit, '--'))
        return classify(changes, layout, force_product=force_product)
    except (OSError, ValueError, UnicodeError, subprocess.SubprocessError):
        # Avoid echoing commands/revision strings into the workflow output.
        return classify([], layout, force_product=force_product,
                        error='Missing, ambiguous or unreadable Git history; build conservatively')


def staging_build_requested(repo: Path, head: str, event: str, ref: str) -> bool:
    """Only the checked-out commit's exact trailer on a trusted Staging push."""
    if event != 'push' or ref != 'refs/heads/staging':
        return False
    try:
        checked_out = resolve(repo, 'HEAD')
        if resolve(repo, head) != checked_out:
            return False
        message, trailers = git(repo, 'show', '-s', '--format=%B%x00%(trailers:only,unfold=true)',
                                checked_out).decode('utf-8').split('\0', 1)
        final_block = message.rstrip('\n').rsplit('\n\n', 1)[-1].splitlines()
        requests = [line for line in trailers.splitlines()
                    if line.partition(':')[0].lower() == 'skager-staging-build']
        return (requests == ['Skager-Staging-Build: true'] and
                final_block.count('Skager-Staging-Build: true') == 1)
    except (OSError, ValueError, UnicodeError, subprocess.SubprocessError):
        return False


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', type=Path, default=Path('.'))
    parser.add_argument('--base', default='')
    parser.add_argument('--head', default='HEAD')
    parser.add_argument('--layout', choices=('local', 'monorepo'), default='local')
    parser.add_argument('--merge-base', action='store_true', help='PR comparison; push uses exact base/head')
    parser.add_argument('--force-product', action='store_true', help='Explicit manual Staging build only')
    parser.add_argument('--staging-request', action='store_true', help='Honor exact HEAD trailer only on a Staging push')
    parser.add_argument('--github-output', type=Path)
    args = parser.parse_args()
    requested = args.staging_request and staging_build_requested(
        args.repo, args.head, os.environ.get('GITHUB_EVENT_NAME', ''), os.environ.get('GITHUB_REF', ''))
    result = select(args.repo, args.base, args.head, args.layout,
                    merge_base=args.merge_base, force_product=args.force_product or requested)
    print(json.dumps(result, indent=2, ensure_ascii=True))
    if args.github_output:
        with args.github_output.open('a', encoding='utf-8') as output:
            for flag in FLAGS:
                output.write(f'{flag}={str(result[flag]).lower()}\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

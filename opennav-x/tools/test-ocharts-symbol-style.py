#!/usr/bin/env python3
"""Focused private symbol-style source proof and actual-method C++ fixture."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--before', type=Path, required=True)
    p.add_argument('--wx-prefix', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    source, before = args.source.resolve(), args.before.resolve()
    units = ['libs/s52plib/src/s52plib.h', 'libs/s52plib/src/s52plib.cpp', 'src/eSENCChart.cpp']
    spec = importlib.util.spec_from_file_location('extract', ROOT / 'tools/test-chart-selector.py')
    extract = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(extract)
    header = (source / units[0]).read_text()
    enable = extract.body(header, 'void EnablePresentationSimplifiedSymbols(')
    getter = extract.body(header, 'LUPname GetEffectiveSymbolStyle(')
    member = re.search(r'bool m_presentationSimpleSymbols = false;', header)[0]
    block = '  // Display-only policy; the host/saved preference is never overwritten.\n  ' + enable + '\n  ' + getter + '\n\n'
    assert header.count(block) == 1
    restored = header.replace(block, '').replace('  ' + member + '\n', '')
    assert restored == (before / units[0]).read_text()
    library = (source / units[1]).read_text()
    assert library.count('if (SIMPLIFIED == GetEffectiveSymbolStyle())') == 1
    assert library.replace('if (SIMPLIFIED == GetEffectiveSymbolStyle())', 'if (SIMPLIFIED == m_nSymbolStyle)') == (before / units[1]).read_text()
    chart = (source / units[2]).read_text()
    assert chart.count('ps52plib->GetEffectiveSymbolStyle()') == 7
    assert 'ps52plib->m_nSymbolStyle' not in chart
    assert chart.replace('ps52plib->GetEffectiveSymbolStyle()', 'ps52plib->m_nSymbolStyle') == (before / units[2]).read_text()
    assert {p.relative_to(source) for p in source.rglob('*') if p.is_file()} == {p.relative_to(before) for p in before.rglob('*') if p.is_file()}
    for path in before.rglob('*'):
        if path.is_file() and path.relative_to(before).as_posix() not in units:
            assert (source / path.relative_to(before)).read_bytes() == path.read_bytes()
    adapter = (ROOT / 'src/plugin-adapters/ocharts/ChartPresentationAdapter.cpp').read_text()
    assert adapter.count('library->EnablePresentationSimplifiedSymbols();') == 1
    assert re.search(r'if \(library->m_bOK\) \{\s+if \(VerifyCompiledResources\(directory\)\) \{\s+library->EnablePresentationSimplifiedSymbols\(\);', adapter)
    assert 'return new s52plib(stockDirectory,false,path[0]==0,false);' in adapter
    # The exact original enum, actual inline methods and actual UpdateMarinerParams
    # run against the real private S52 mariner store, without constructing a plugin.
    enum = re.search(r'typedef enum _LUPname \{.*?\} LUPname;', (source / 'libs/s52plib/src/s52s57.h').read_text(), re.S)[0]
    update = extract.body(library, 'void s52plib::UpdateMarinerParams(')
    snippet = '#include "s52utils.h"\n' + enum + '\nclass s52plib {\npublic:\n' + enable + '\n' + getter + '\nLUPname m_nSymbolStyle=PAPER_CHART;\nLUPname m_nBoundaryStyle=PLAIN_BOUNDARIES;\nvoid UpdateMarinerParams();\nprivate:\n' + member + '\n};\n' + update
    (out / 'actual-symbol-style.inc').write_text(snippet)
    wx = [str(args.wx_prefix / 'bin/wx-config'), '--prefix=' + str(args.wx_prefix)]
    flags = shlex.split(subprocess.check_output(wx + ['--cxxflags', '--libs'], text=True))
    command = ['c++', '-std=c++17', '-I' + str(out), '-I' + str(source / 'libs/s52plib/src'),
               str(ROOT / 'src/plugin-adapters/ocharts/tests/symbol_style_test.cpp'),
               str(source / 'libs/s52plib/src/s52utils.cpp'), *flags,
               '-Wl,-rpath,' + str(args.wx_prefix / 'lib'), '-o', str(out / 'symbol-style')]
    result = subprocess.run(command, capture_output=True, text=True)
    (out / 'compile.log').write_text(result.stdout + result.stderr)
    result.check_returncode()
    result = subprocess.run([str(out / 'symbol-style')], capture_output=True, text=True, timeout=20,
                            env={**os.environ, 'LD_LIBRARY_PATH': str(args.wx_prefix / 'lib')})
    (out / 'checks.log').write_text(result.stdout + result.stderr)
    result.check_returncode()
    # Mutation negative: remove only the policy override from the extracted actual
    # getter; unchanged assertions must reject the resulting Paper selection.
    mutated = snippet.replace('return m_presentationSimpleSymbols ? SIMPLIFIED : m_nSymbolStyle;', 'return m_nSymbolStyle;')
    assert mutated != snippet
    (out / 'actual-symbol-style.inc').write_text(mutated)
    negative = subprocess.run(command, capture_output=True, text=True)
    negative.check_returncode()
    negative = subprocess.run([str(out / 'symbol-style')], capture_output=True, text=True, timeout=20,
                              env={**os.environ, 'LD_LIBRARY_PATH': str(args.wx_prefix / 'lib')})
    (out / 'negative.log').write_text(negative.stdout + negative.stderr)
    assert negative.returncode == 1 and 'verified style must select Simplified' in negative.stderr
    (out / 'actual-symbol-style.inc').write_text(snippet)
    (out / 'receipt.json').write_text(json.dumps({
        'sourceReverseEquality': True, 'effectiveReads': {'eSENCChart': 7, 'UpdateMarinerParams': 1},
        'unchangedOtherSourceFiles': True, 'command': command,
        'fixtureExit': result.returncode, 'missingOverrideNegativeExit': negative.returncode,
        'sourceHashes': {n: hashlib.sha256((source/n).read_bytes()).hexdigest() for n in units},
        'actualMethodsSha256': hashlib.sha256(snippet.encode()).hexdigest(),
        'limits': 'Method fixture and Linux objects only; no plugin, chart, helper, native Windows or boat execution.',
    }, indent=2) + '\n')
    print(result.stdout, end='')
    print('Full private-source reverse proof passed; missing-override mutation rejected')


if __name__ == '__main__':
    main()

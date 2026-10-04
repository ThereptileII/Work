"""Run only the existing ordering assertion against two in-memory mutations."""
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import unittest
from unittest.mock import patch

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
spec = importlib.util.spec_from_file_location('gettext_contract', ROOT/'tests/windows_gettext_tests.py')
contract = importlib.util.module_from_spec(spec)
spec.loader.exec_module(contract)
build = ROOT/'tools/build-pristine-windows.ps1'
original = build.read_text()
read_text = Path.read_text
gate_start = original.index('    $Gettext = Initialize-WindowsGettext')
gate_end = original.index('\n', original.index('\n', gate_start)+1)+1
gate = original[gate_start:gate_end]
without_gate = original[:gate_start]+original[gate_end:]
after_preflight = without_gate.index('\n', without_gate.index("'-Evidence', $CurlPreflight)"))+1
verify_start = original.index("    Run python @((Join-Path $PSScriptRoot 'windows_gettext.py'), 'verify'")
verify_end = original.index('\n', verify_start)+1
verify = original[verify_start:verify_end]
without_verify = original[:verify_start]+original[verify_end:]
after_cmake = without_verify.index('\n', without_verify.index(') + $OpenNavArgs)'))+1
mutations = {
    'ensure-after-curl-preflight': without_gate[:after_preflight]+gate+without_gate[after_preflight:],
    'verify-after-application-cmake': without_verify[:after_cmake]+verify+without_verify[after_cmake:],
}
results = []
for name, mutated in mutations.items():
    def read(path, *args, **kwargs):
        return mutated if path == build else read_text(path, *args, **kwargs)
    stream = io.StringIO()
    case = contract.GettextPrerequisiteTests('test_script_orders_gate_before_every_expensive_step')
    with patch.object(Path, 'read_text', read):
        result = unittest.TextTestRunner(stream=stream).run(unittest.TestSuite([case]))
    log = stream.getvalue()
    (HERE/(name+'.log')).write_text(log)
    expected = result.testsRun == 1 and len(result.failures) == 1 and not result.errors
    results.append({'mutation': name, 'rejectedByAssertion': expected,
                    'testsRun': result.testsRun, 'failures': len(result.failures),
                    'errors': len(result.errors),
                    'mutatedSourceSha256': hashlib.sha256(mutated.encode()).hexdigest()})
(HERE/'negative-results.json').write_text(json.dumps(results, indent=2)+'\n')
print(json.dumps(results, indent=2))
raise SystemExit(0 if all(r['rejectedByAssertion'] for r in results) else 1)

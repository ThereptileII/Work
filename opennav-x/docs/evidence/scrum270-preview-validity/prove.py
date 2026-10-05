"""Bounded evidence replay; does not import/launch smoke-preview or an app."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import subprocess

BASE = '48be04c35de20c150e885e797eff16e25db208cb'
FAILED_SHA = 'd2d2a125309198258edb53c19b2099a38169bffd796e1ccb75d94578df4eb48c'
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
SOURCES = ['src/vessel/DemoSource.cpp', 'src/vessel/RouteProgress.cpp',
           'src/vessel/VesselState.cpp', 'src/smartnav/VesselEnergy.cpp',
           'src/smartnav/Energy.cpp', 'tests/support/PreviewEnergyModel.cpp']

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def parts(source):
    tree = ast.parse(source)
    assignments = [n for n in ast.walk(tree) if isinstance(n, ast.Assign) and
                   any(isinstance(t, ast.Name) and t.id == 'later' for t in n.targets)]
    assert len(assignments) == 1
    call = assignments[0].value
    assert isinstance(call, ast.Call) and isinstance(call.func, ast.Name) and call.func.id == 'data'
    assert not call.keywords and len(call.args) == 1 and isinstance(call.args[0], ast.Lambda)
    reader = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'data')
    assert ast.literal_eval(reader.args.defaults[-1]) == 12
    item = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'item')
    comparisons = [n for n in ast.walk(tree) if isinstance(n, ast.Assert) and
                   any(isinstance(x, ast.Name) and x.id == 'later' for x in ast.walk(n))]
    assert len(comparisons) == 3
    return call.args[0], item, sorted(comparisons, key=lambda n:n.lineno)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--failed-json', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    assert sha(args.failed_json) == FAILED_SHA, 'Unexpected failed native snapshot'
    args.output.mkdir(parents=True, exist_ok=True)
    # Freshly compile only six small unchanged implementation units, avoiding
    # unproven ABI/source provenance of cached application objects.
    for name in SOURCES:
        original = subprocess.check_output(['git', 'show', BASE + ':' + name], cwd=ROOT)
        assert (ROOT / name).read_bytes() == original, name
    binary = args.output / 'emit'
    depfile = args.output / 'emit.d'
    command = ['c++', '-std=c++17', '-O0', '-I' + str(ROOT / 'src'),
               str(HERE / 'emit.cpp'), *[str(ROOT / p) for p in SOURCES], '-o', str(binary)]
    built = subprocess.run(command, text=True, capture_output=True)
    (args.output / 'compile.log').write_text(built.stdout + built.stderr)
    built.check_returncode()
    generated = subprocess.check_output([str(binary)])
    (args.output / 'samples.json').write_bytes(generated)
    samples = json.loads(generated)
    assert [s['second'] for s in samples] == [0,114,115,116]
    assert [s['route']['state'] for s in samples] == ['Valid','Valid','ActivePointChanged','Valid']
    failed = json.loads(args.failed_json.read_bytes())
    old_source = subprocess.check_output(['git','show',BASE + ':tools/smoke-preview.py'],cwd=ROOT).decode()
    new_source = (ROOT / 'tools/smoke-preview.py').read_text()
    old, old_item, old_assertions = parts(old_source)
    new, item, assertions = parts(new_source)
    assert ast.dump(old_item) == ast.dump(item)
    assert [ast.dump(n) for n in old_assertions] == [ast.dump(n) for n in assertions]
    env = {'first': samples[0]}
    exec(compile(ast.Module(body=[item], type_ignores=[]), '<actual item helper>', 'exec'), env)
    before = eval(compile(ast.Expression(old), '<old predicate>', 'eval'), env)
    after = eval(compile(ast.Expression(new), '<new predicate>', 'eval'), env)
    results = []
    for label, sample, expected in [('initial',samples[0],False), ('114',samples[1],True),
            ('115',samples[2],False), ('116',samples[3],True), ('native failure',failed,False)]:
        previous, current = before(sample), after(sample)
        assert current == expected, label
        if label in ('115','native failure'):
            assert previous and 'remaining_nm' not in sample['route'] and 'arrival_soc' not in sample['energy']
            assert sample['route']['state'] == 'ActivePointChanged'
            env['later'] = sample
            try:
                exec(compile(ast.Module(body=old_assertions,type_ignores=[]),'<old comparisons>','exec'),env)
            except KeyError as error:
                assert error.args == ('remaining_nm',)
            else:
                raise AssertionError('Original failure was not reproduced')
        if current:
            env['later'] = sample
            exec(compile(ast.Module(body=assertions,type_ignores=[]),'<unchanged comparisons>','exec'),env)
        results.append(dict(sample=label,old_predicate=previous,new_predicate=current,
                            original_three_comparisons='passed' if current else 'not evaluated'))
    # Match real native transition telemetry to actual fixture output (native
    # JSON rounds numeric values); this is not a handwritten trip equation.
    for name in ('Latitude','Battery SOC'):
        assert abs(env['item'](failed,name)['value'] - env['item'](samples[2],name)['value']) < 1e-8
    inputs = SOURCES + ['tools/smoke-preview.py', str((HERE/'emit.cpp').relative_to(ROOT)),
                        str(Path(__file__).resolve().relative_to(ROOT))]
    # Include actual local header dependency closure from the same compiler.
    deps = subprocess.check_output(['c++','-std=c++17','-MM','-I'+str(ROOT/'src'),
                                  str(HERE/'emit.cpp'),*[str(ROOT/p) for p in SOURCES]],text=True)
    depfile.write_text(deps)
    headers = set()
    for token in deps.replace('\\\n',' ').split():
        p = Path(token)
        if p.is_absolute() and p.is_file() and p.suffix == '.h':
            headers.add(str(p.relative_to(ROOT)))
    inputs = sorted(set(inputs) | headers)
    receipt = dict(result='passed',base_commit=BASE,failed_snapshot_sha256=FAILED_SHA,
        failed_run=37120549213,failed_artifact=11275626437,
        artifact_sha256='eb11a4ab014832e3304a387ada3b3f8afe8195f2d3850ee24c1ccaed1f064adc',
        old_smoke_preview_sha256=hashlib.sha256(old_source.encode()).hexdigest(),
        compiler=subprocess.check_output(['c++','--version'],text=True).splitlines()[0],
        command=command,emitter_sha256=sha(binary),samples_sha256=sha(args.output/'samples.json'),
        source_sha256={p:sha(ROOT/p) for p in inputs},cases=results,timeout_seconds=12,
        limits=['No native UI rerun','No full app build','Emitter serializes only consumed fields; does not run PreviewDiagnostics'])
    (args.output/'proof.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print(json.dumps(results,indent=2))

if __name__ == '__main__':
    main()

"""Focused source contract for the native 1280x800 navigation surface."""
import ast
from pathlib import Path


root = Path(__file__).resolve().parents[1]
smoke_path = root / 'tools/smoke-navigation.py'
smoke_source = smoke_path.read_text()
tree = ast.parse(smoke_source)
function = next(node for node in tree.body if isinstance(node, ast.FunctionDef)
                and node.name == 'exact_native_reference_desktop')
namespace = {}
exec(compile(ast.Module(body=[function], type_ignores=[]), str(smoke_path), 'exec'), namespace)
is_reference = namespace['exact_native_reference_desktop']

assert is_reference({'after': (1280, 800, 32)})
for display in ({}, {'after': ()}, {'after': (1280,)},
                {'after': (1280, 799, 32)}, {'after': (1279, 800, 32)},
                {'after': (1920, 1080, 32)}):
    assert not is_reference(display), display

print('7 exact native reference desktop selection checks passed')

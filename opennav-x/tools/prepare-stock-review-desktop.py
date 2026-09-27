"""Prepare only the disposable hosted CI desktop for fixed stock-window tests.

Production review helpers never call this and never change display settings.
The existing Windows UI fixture enumerates supported modes and requests a
session-only mode, without registry persistence or taskbar/DPI changes.
"""
import importlib.util
import json
import os
from pathlib import Path
import sys

if sys.platform != 'win32' or os.environ.get('GITHUB_ACTIONS') != 'true':
    raise SystemExit('Disposable native GitHub Actions desktop only')
spec = importlib.util.spec_from_file_location('stock_fixture_windows_ui', Path(__file__).with_name('windows-ui.py'))
ui = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ui)
result = ui.ensure_desktop(1280, 900)
work = ui.W.RECT()
query = ui.declare(ui.user, 'SystemParametersInfoW', ui.W.BOOL,
                   ui.W.UINT, ui.W.UINT, ui.C.POINTER(ui.W.RECT), ui.W.UINT)
if not query(0x30, 0, ui.C.byref(work), 0):
    raise RuntimeError('Cannot inspect disposable desktop work area')
if work.right - work.left < 1280 or work.bottom - work.top < 800:
    raise RuntimeError('Prepared CI work area still cannot hold the fixed frame')
result.update(scope='Disposable CI session only; never a boat operation',
              work_area=[work.left, work.top, work.right, work.bottom],
              persistent_settings_changed=False)
print(json.dumps(result))

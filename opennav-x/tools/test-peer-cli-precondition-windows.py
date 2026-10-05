#!/usr/bin/env python3
"""Native pinned-wx/home-creation excerpt proof, not an installed CLI test."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def load(filename):
    spec = importlib.util.spec_from_file_location(filename.replace('-', '_'), ROOT / 'tools' / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence', type=Path, required=True)
    args = parser.parse_args()
    cli = load('test-peer-cli.py')
    runner = cli.windows_runner_temp()
    if runner is None:
        raise RuntimeError('Native disposable GitHub-hosted Windows required')
    for name in ('CL', '_CL_', 'CXXFLAGS', 'CFLAGS'):
        if os.environ.get(name):
            raise RuntimeError(f'Refusing inherited compiler override: {name}')
    helper = load('test-windows-changed-units.py')
    evidence = args.evidence.resolve()
    evidence.mkdir(parents=True, exist_ok=False)
    report = {'status': 'failed', 'scope': __doc__, 'installedCliExecuted': False}
    directory = cli.common_app_data_dir() / 'opencpn'
    owned = False
    config = directory / 'opencpn.ini'
    try:
        if os.path.lexists(directory):
            raise RuntimeError('Refusing pre-existing common-data directory; no mutation permitted')
        lock = json.loads((ROOT / 'upstream.lock.json').read_text())
        source = subprocess.check_output(['git', '-C', ROOT / 'upstream/OpenCPN', 'show',
                                         lock['commit'] + ':model/src/base_platform.cpp'], text=True)
        # Compile the exact home-creation block rather than a Python imitation.
        start = source.index('  // create the opencpn "home" directory if we need to')
        end = source.index('  // create the opencpn "log" directory if we need to', start)
        excerpt = source[start:end]
        home = source[source.index('wxString& AbstractPlatform::GetHomeDir()'):source.index('wxString& AbstractPlatform::GetHomeDir()') + 1100]
        if not re.search(r'std_path\s*\.GetConfigDir\(\)', home):
            raise RuntimeError('Pinned Windows home resolver changed')
        cli_source = subprocess.check_output(['git', '-C', ROOT / 'upstream/OpenCPN', 'show',
                                             lock['commit'] + ':cli/console.cpp'], text=True)
        if 'SetAppName("opencpn")' not in cli_source:
            raise RuntimeError('Pinned CLI app name changed')
        report['upstream'] = lock['commit']
        report['candidate'] = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
        for name, content in (('base_platform.cpp', source), ('home-creation-excerpt.cpp', excerpt)):
            (evidence / name).write_text(content)
        report['inputs'] = {p: helper.record(ROOT / p) for p in (
            'tools/test-peer-cli.py', 'tools/test-peer-cli-precondition-windows.py',
            'tools/test-windows-changed-units.py', 'tools/windows-wx.lock.json', 'upstream.lock.json')}
        sdk = ROOT / 'build/windows-changed-unit-sdk'
        wx = sdk / 'wx'
        for item in json.loads((ROOT / 'tools/windows-wx.lock.json').read_text())['archives']:
            archive = sdk / item['file']
            helper.fetch(item, archive)
            helper.run(['7z', 'x', '-y', '-o' + str(wx), archive], evidence / (item['file'] + '.log'))
        probe = evidence / 'probe'
        probe.mkdir()
        (probe / 'main.cpp').write_text('''#include <iostream>
#include <wx/app.h>
#include <wx/filename.h>
#include <wx/stdpaths.h>
wxString& GetHomeDir() {
  static wxString home = wxStandardPaths::Get().GetConfigDir();
  if (!home.EndsWith(wxString(wxFileName::GetPathSeparator())))
    home += wxFileName::GetPathSeparator();
  return home;
}
bool CreateHome() {
''' + excerpt + '''  return true;
}
class Probe : public wxAppConsole {
 public:
  bool OnInit() override { SetAppName("opencpn"); return true; }
  int OnRun() override {
    if (argc > 1 && !CreateHome()) return 2;
    std::cout << GetHomeDir().ToStdString() << std::endl;
    return 0;
  }
};
wxIMPLEMENT_APP_CONSOLE(Probe);
''')
        (probe / 'CMakeLists.txt').write_text('''cmake_minimum_required(VERSION 3.20)
project(peer_home_probe LANGUAGES CXX)
find_package(wxWidgets REQUIRED COMPONENTS base)
include(${wxWidgets_USE_FILE})
add_executable(peer-home-probe main.cpp)
target_link_libraries(peer-home-probe PRIVATE ${wxWidgets_LIBRARIES})
''')
        build = evidence / 'build'
        helper.run(['cmake', '-S', probe, '-B', build, '-G', 'Visual Studio 17 2022', '-A', 'Win32',
                    '-DwxWidgets_ROOT_DIR:PATH=' + wx.as_posix(),
                    '-DwxWidgets_LIB_DIR:PATH=' + (wx / 'lib/vc14x_dll').as_posix(),
                    '-DwxWidgets_CONFIGURATION=mswu'], evidence / 'configure.log', timeout=90)
        helper.run(['cmake', '--build', build, '--config', 'Release', '--parallel', '2'], evidence / 'compile.log', timeout=90)
        executable = build / 'Release/peer-home-probe.exe'
        report['runtime'] = helper.stage_native_runtime(executable, wx, ('wxbase32u_vc14x.dll',))
        env = os.environ.copy()
        resolved = subprocess.check_output([executable], env=env, text=True, timeout=20).strip()
        if Path(resolved).resolve() != directory:
            raise RuntimeError('Pinned wx path differs from exact Python known-folder target')
        report['nativeWxHome'] = resolved
        with tempfile.TemporaryDirectory(prefix='peer-precondition-', dir=runner) as temp:
            target, seeded = cli.windows_config_target(Path(temp))
            owned = True
            if target != directory or seeded != config or config.read_bytes() != cli.initial_config():
                raise RuntimeError('Native seed differs from the guarded config')
            report['beforeHomeCreation'] = 'guard accepted absent directory; exact sentinel seeded'
            config.unlink()
            directory.rmdir()
            # Only the known, previously absent directory can be created here.
            subprocess.run([executable, 'create-home'], env=env, check=True, timeout=20)
            if not directory.is_dir() or list(directory.iterdir()):
                raise RuntimeError('Home excerpt did not create exactly the empty directory')
            try:
                cli.windows_config_target(Path(temp))
            except RuntimeError as error:
                if 'already exists' not in str(error):
                    raise
                report['afterHomeCreation'] = str(error)
            else:
                raise AssertionError('Guard accepted the GUI-created home directory')
        report['status'] = 'passed'
    except BaseException as error:
        report['error'] = str(error)
        raise
    finally:
        try:
            if owned:
                if os.path.lexists(config):
                    if config.is_symlink() or config.read_bytes() != cli.initial_config():
                        raise RuntimeError('Unknown config contents retained; refusing cleanup')
                    config.unlink()
                # Never recursively delete; an unexpected file makes cleanup fail.
                if directory.exists():
                    directory.rmdir()
                report['cleanup'] = 'owned directory removed'
        except BaseException as error:
            report['status'] = 'failed'
            report['cleanupError'] = str(error)
            raise
        finally:
            (evidence / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()

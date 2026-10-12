#!/usr/bin/env python3
"""Exercise actual patched MyConfig persistence boundaries, without building OpenCPN.

The qualification flag/constructor observer are test-local restriction stubs.
This does not exercise native CONFIG custody or assert updater health.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def run(args, **kwargs):
    return subprocess.run(args, check=True, **kwargs)


def function(source, signature):
    begin = source.index(signature)
    opening = source.index('{', begin)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[begin:end]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--upstream', required=True, type=Path)
    parser.add_argument('--wx-config', required=True, type=Path)
    parser.add_argument('--wx328-headers', required=True, type=Path)
    parser.add_argument('--evidence-directory', required=True, type=Path)
    parser.add_argument('--cxx', default='c++')
    args = parser.parse_args()
    args.evidence_directory.mkdir(parents=True, exist_ok=False)
    evidence = args.evidence_directory
    lock = json.loads((ROOT / 'upstream.lock.json').read_text())
    revision = subprocess.check_output(['git', '-C', str(args.upstream), 'rev-parse', 'HEAD'], text=True).strip()
    if revision != lock['commit']:
        raise ValueError('Wrong pinned upstream revision')
    # Verify the exact pinned Windows producer headers, not the host wx version.
    wx_lock = json.loads((ROOT / 'tools/windows-wx.lock.json').read_text())
    header_sha = hashlib.sha256(args.wx328_headers.read_bytes()).hexdigest()
    pinned_header = next(a for a in wx_lock['archives'] if a['file'] == 'wxWidgets-3.2.8-headers.7z')
    if header_sha != pinned_header['sha256'] or args.wx328_headers.stat().st_size != pinned_header['bytes']:
        raise ValueError('wx header archive does not match the Windows lock')
    header = subprocess.check_output(['bsdtar', '-xOf', str(args.wx328_headers), 'include/wx/fileconf.h'], text=True)
    assert 'virtual bool Flush(bool bCurrentOnly = false) wxOVERRIDE;' in header
    assert 'void DisableAutoSave() { m_autosave = false; }' in header
    (evidence / 'wx328-fileconf.h').write_text(header)
    recipe = (ROOT / 'tools/prepare-integration.py').read_text()
    patches = [ROOT / 'patches' / name for name in re.findall(r"'patches/([^']+)'", recipe)]
    assert patches and patches[-1].name == 'opencpn-5.12.4-update-qualification.patch'
    with tempfile.TemporaryDirectory(prefix='qualification-config-', dir=evidence) as temporary:
        env = dict(os.environ, GIT_INDEX_FILE=str(Path(temporary) / 'index'))
        run(['git', '-C', str(args.upstream), 'read-tree', revision], env=env)
        for patch in patches:
            run(['git', '-C', str(args.upstream), 'apply', '--cached', '-'],
                input=patch.read_bytes().replace(b'\r\n', b'\n'), env=env)
        source = subprocess.check_output(['git', '-C', str(args.upstream), 'show', ':gui/src/navutil.cpp'], env=env, text=True)
        declarations = subprocess.check_output(['git', '-C', str(args.upstream), 'show', ':gui/include/gui/navutil.h'], env=env, text=True)
    constructor = function(source, 'MyConfig::MyConfig(')
    flush = function(source, 'bool MyConfig::Flush(')
    update = function(source, 'void MyConfig::UpdateSettings()')
    assert 'bool Flush(bool bCurrentOnly = false) override;' in declarations
    assert update.index('QualificationProfileRequired()) return;') < update.index('SetPath(')
    actual = constructor + '\nMyConfig::~MyConfig() {}\n' + flush
    (evidence / 'actual-config-boundaries.cpp').write_text(actual)
    harness = r'''
#include <wx/fileconf.h>
#include <wx/file.h>
#include <wx/filename.h>
#include <wx/init.h>
#include <stdexcept>
#include <iostream>
#define OPENNAV_X 1
namespace opennav::integration {
bool required = false;
unsigned observed = 0;
bool QualificationProfileRequired() noexcept { return required; }
void ObserveQualifiedConfigConstructor(const std::wstring&) noexcept { ++observed; }
}
class MyConfig : public wxFileConfig {
public:
  MyConfig(const wxString& LocalFileName);
  ~MyConfig();
  bool Flush(bool bCurrentOnly = false) override;
};
''' + actual + r'''
void check(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
wxString bytes(const wxString& path) {
  wxFile f(path); wxString value;
  check(f.IsOpened() && f.ReadAll(&value), "cannot read test INI"); return value;
}
int main() {
  wxInitializer init; check(init.IsOk(), "wx initialization");
  const wxString original = "[Settings]\nNavigation=untouched\n";
  const auto path = wxFileName::CreateTempFileName("skager-config-save");
  try {
    for (bool qualified : {false, true}) {
      for (bool explicit_flush : {false, true}) {
        { wxFile f(path, wxFile::write); check(f.Write(original), "seed original"); }
        opennav::integration::required = qualified;
        const auto count = opennav::integration::observed;
        {
          MyConfig config(path);
          check(opennav::integration::observed == count + 1, "actual constructor observation");
          wxString navigation;
          check(config.Read("/Settings/Navigation", &navigation) && navigation == "untouched", "real config read");
          check(config.Write("/Settings/StartupDefault", "changed"), "in-memory write");
          if (explicit_flush) {
            wxConfigBase* base = &config;
            check(base->Flush(), "virtual Flush failed");
            check((bytes(path) == original) == qualified, "wrong explicit Flush disk effect");
          }
        }
        check((bytes(path) == original) == qualified, "wrong destructor disk effect");
        std::cout << "PASS " << (qualified ? "qualification preserves" : "ordinary persists")
                  << (explicit_flush ? " virtual Flush and destruction\n" : " destruction\n");
      }
    }
    check(wxRemoveFile(path), "remove test INI");
    opennav::integration::required = true;
    { MyConfig config(path); check(config.Write("key", "value"), "missing in-memory write"); check(config.Flush(), "missing Flush"); }
    check(!wxFileExists(path), "qualification created missing INI");
    std::cout << "PASS qualification does not create missing INI\n";
  } catch (...) { wxRemoveFile(path); throw; }
}
'''
    cpp = evidence / 'config-save-test.cpp'
    cpp.write_text(harness)
    flags = shlex.split(subprocess.check_output([str(args.wx_config), '--cxxflags', '--libs', 'base'], text=True))
    binary = evidence / 'config-save-test'
    run([args.cxx, '-std=c++17', str(cpp), '-o', str(binary), *flags])
    run([str(binary)])
    # Existing upstream recovery tests exercise the actual store and ordinary controls.
    recovery_binary = evidence / 'recovery-store-test'
    # Observe the actual store's notification call without sending native IPC.
    checkpoint = evidence / 'recovery-checkpoint-test.cpp'
    checkpoint.write_text(r'''#include "integration/RecoveryStore.h"
#include <gtest/gtest.h>
#include <wx/filename.h>
#include <wx/init.h>
using namespace std::chrono_literals;
namespace opennav::integration {
unsigned checkpoints = 0;
void NotifyUpdateStartupHealthy() noexcept { ++checkpoints; }
}
TEST(OpenNavRecovery, QualificationEmitsOnlyTheRealDeferredThirtySecondCheckpoint) {
  wxInitializer init; ASSERT_TRUE(init.IsOk());
  const auto path = wxFileName::CreateTempFileName("skager-checkpoint");
  ASSERT_TRUE(wxRemoveFile(path)); ASSERT_TRUE(wxMkdir(path));
  const auto previous = opennav::integration::checkpoints;
  {
    opennav::integration::RecoveryStore store(path, true);
    ASSERT_TRUE(store.BeginXNav());
    const opennav::vessel::Time t{100s};
    store.ObserveHealthy(false, t);
    store.ObserveHealthy(false, t + 31s);
    EXPECT_EQ(opennav::integration::checkpoints, previous);
    store.ObserveHealthy(true, t + 32s);
    store.ObserveHealthy(true, t + 61s);
    EXPECT_EQ(opennav::integration::checkpoints, previous);
    store.ObserveHealthy(true, t + 62s);
    EXPECT_EQ(opennav::integration::checkpoints, previous + 1);
    store.ObserveHealthy(true, t + 90s);
    EXPECT_EQ(opennav::integration::checkpoints, previous + 1);
    store.CleanClose();
  }
  EXPECT_FALSE(wxFileExists(wxFileName(path, "opennav-startup.state").GetFullPath()));
  EXPECT_TRUE(wxRmdir(path));
}
''')
    run([args.cxx, '-std=c++17', '-I' + str(ROOT / 'src'),
         str(ROOT / 'tests/recovery_store_upstream_tests.cpp'),
         str(ROOT / 'src/integration/RecoveryStore.cpp'),
         str(ROOT / 'src/integration/StartupRecovery.cpp'),
         str(checkpoint),
         '-o', str(recovery_binary), *flags, '-lgtest', '-lgtest_main', '-pthread'])
    run([str(recovery_binary), '--gtest_filter=OpenNavRecovery.*'])
    receipt_binary = evidence / 'startup-receipt-test'
    run([args.cxx, '-std=c++17', '-I' + str(ROOT / 'src'),
         str(ROOT / 'tests/update_startup_receipt_tests.cpp'),
         str(ROOT / 'src/integration/UpdateStartupReceipt.cpp'),
         str(ROOT / 'src/integration/UpdateQualification.cpp'),
         '-o', str(receipt_binary)])
    run([str(receipt_binary)])
    receipt = {'upstream': revision, 'wx328HeadersSHA256': header_sha,
               'hostWxVersion': subprocess.check_output([str(args.wx_config), '--version'], text=True).strip(),
               'patches': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in patches},
               'boundariesSHA256': hashlib.sha256(actual.encode()).hexdigest(),
               'scope': 'actual extracted MyConfig constructor/Flush; UpdateSettings early guard; actual RecoveryStore; not native custody or health'}
    (evidence / 'receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
    print('PASS exact patch chain, pinned wx3.2.8 virtual/auto-save API and early UpdateSettings guard')


if __name__ == '__main__':
    main()

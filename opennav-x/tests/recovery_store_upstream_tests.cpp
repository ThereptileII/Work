#include "integration/RecoveryStore.h"
#include <filesystem>
#include <gtest/gtest.h>
#include <wx/file.h>
#include <wx/filename.h>
#include <wx/init.h>
using namespace opennav;
using namespace std::chrono_literals;
namespace {
struct Directory {
  wxString path;
  Directory() {
    path = wxFileName::CreateTempFileName("opennav-recovery");
    wxRemoveFile(path);
    wxMkdir(path);
  }
  ~Directory() { wxFileName::Rmdir(path, wxPATH_RMDIR_RECURSIVE); }
};
} // namespace
TEST(OpenNavRecovery, ActualJournalCountsCrashesAndPreservesRetryEvidence) {
  wxInitializer init;
  ASSERT_TRUE(init.IsOk());
  Directory dir;
  {
    integration::RecoveryStore s(dir.path);
    EXPECT_FALSE(s.RequiresSafe());
    ASSERT_TRUE(s.BeginXNav());
  }
  {
    integration::RecoveryStore s(dir.path);
    EXPECT_FALSE(s.RequiresSafe());
    ASSERT_TRUE(s.BeginXNav());
  }
  {
    integration::RecoveryStore s(dir.path);
    EXPECT_TRUE(s.RequiresSafe());
    EXPECT_FALSE(s.BeginXNav());
    ASSERT_TRUE(s.Retry());
    ASSERT_TRUE(s.BeginXNav());
    s.CleanClose();
  }
  integration::RecoveryStore clean(dir.path);
  EXPECT_FALSE(clean.RequiresSafe());
  unsigned backups = 0;
  for (const auto &p : std::filesystem::directory_iterator(
           std::filesystem::u8path(dir.path.ToStdString(wxConvUTF8))))
    if (p.path().filename().string().find(".retry-") != std::string::npos)
      ++backups;
  EXPECT_EQ(backups, 1u);
}
TEST(OpenNavRecovery, HealthyRequiresDeferredInitializationAndThirtySeconds) {
  wxInitializer init;
  ASSERT_TRUE(init.IsOk());
  Directory dir;
  const vessel::Time t{100s};
  {
    integration::RecoveryStore s(dir.path);
    ASSERT_TRUE(s.BeginXNav());
    s.ObserveHealthy(false, t);
    s.ObserveHealthy(false, t + 31s);
  }
  {
    integration::RecoveryStore s(dir.path);
    ASSERT_TRUE(s.BeginXNav());
    s.ObserveHealthy(true, t);
    s.ObserveHealthy(true, t + 29s);
  }
  {
    integration::RecoveryStore s(dir.path);
    EXPECT_TRUE(s.RequiresSafe());
    ASSERT_TRUE(s.Retry());
    ASSERT_TRUE(s.BeginXNav());
    s.ObserveHealthy(true, t);
    s.ObserveHealthy(true, t + 30s);
    EXPECT_EQ(s.FailuresObservedAtLaunch(), 2u);
    EXPECT_TRUE(s.PreviousLaunchUnfinished());
    EXPECT_FALSE(s.RequiresSafe());
  }
  integration::RecoveryStore s(dir.path);
  EXPECT_FALSE(s.RequiresSafe());
}
TEST(OpenNavRecovery, CorruptJournalFailsClosedWithoutTouchingNavigationData) {
  wxInitializer init;
  ASSERT_TRUE(init.IsOk());
  Directory dir;
  {
    wxFile f(wxFileName(dir.path, "opennav-startup.state").GetFullPath(),
             wxFile::write);
    ASSERT_TRUE(f.Write("corrupt record"));
  }
  const auto data = wxFileName(dir.path, "navigation-canary").GetFullPath();
  {
    wxFile f(data, wxFile::write);
    ASSERT_TRUE(f.Write("navigation untouched"));
  }
  integration::RecoveryStore s(dir.path);
  EXPECT_TRUE(s.RequiresSafe());
  EXPECT_FALSE(s.BeginXNav());
  wxFile f(data);
  wxString value;
  ASSERT_TRUE(f.ReadAll(&value));
  EXPECT_EQ(value, "navigation untouched");
  f.Close();
  EXPECT_TRUE(s.Retry());
  EXPECT_FALSE(s.RequiresSafe());
}

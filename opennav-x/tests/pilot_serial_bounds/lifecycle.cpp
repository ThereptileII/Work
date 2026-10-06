// Actual driver Open/Close with a deterministic wxThread lifecycle double.
// A created thread starts only in Wait, reproducing Close before Entry begins.
#include <atomic>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

static void Check(bool ok) {
  if (!ok) { std::cerr << "serial lifecycle contract failed\n"; std::abort(); }
}
struct wxString {
  std::string value;
  wxString(const char* v = "") : value(v) {}
  wxString AfterFirst(char) const { return *this; }
  wxString BeforeFirst(char) const { return *this; }
  const char* c_str() const { return value.c_str(); }
  static wxString Format(const char*, const char*) { return {}; }
};
#define _T(x) x
static void wxLogMessage(const wxString&) {}
constexpr int wxTHREAD_NO_ERROR = 0;
constexpr int wxTHREAD_WAIT_BLOCK = 1;
static bool fail_create, fail_run;
static int create_count, run_count, delete_count, wait_count, destroy_count;
static std::vector<std::string> calls;
class CommDriverN2KSerialThread;
struct CommDriverN2KSerial {
  struct Params { wxString GetDSPort() const { return "Serial:fake"; } } m_params;
  struct Timer { void Stop() { calls.push_back("timer stop"); } } m_stats_timer;
  struct State { void Connection(bool connected) { Check(!connected); calls.push_back("disconnect"); } } m_serial_state;
  wxString m_BaudRate, m_portstring;
  std::atomic_int m_Thread_run_flag{-1};
  std::atomic_bool m_bsec_thread_active{false};
  bool m_closing = false;
  CommDriverN2KSerialThread* m_pSecondary_Thread = nullptr;
  void SetSecondaryThread(CommDriverN2KSerialThread* worker) { m_pSecondary_Thread = worker; }
  void SetThreadRunFlag(int value) { m_Thread_run_flag = value; }
  void SetSecThreadInActive() { m_bsec_thread_active = false; }
  bool Open();
  void Close();
};
class CommDriverN2KSerialThread {
  CommDriverN2KSerial* parent;
  bool created = false, run = false, cancelled = false, joined = false;
public:
  CommDriverN2KSerialThread(CommDriverN2KSerial* p, const wxString&, const wxString&) : parent(p) {}
  ~CommDriverN2KSerialThread() {
    Check(!created || joined); // Never destroy a native thread or its parent early.
    ++destroy_count;
    calls.push_back("destroy");
  }
  int Create() { ++create_count; created = !fail_create; return fail_create ? 1 : 0; }
  int Run() { ++run_count; Check(created); run = !fail_run; return fail_run ? 1 : 0; }
  int Delete(void*, int mode) {
    Check(created && mode == wxTHREAD_WAIT_BLOCK);
    cancelled = true;
    ++delete_count;
    calls.push_back("cancel");
    // POSIX NEW state can return an error after cancellation; Wait still joins.
    return 1;
  }
  void* Wait(int mode) {
    Check(created && mode == wxTHREAD_WAIT_BLOCK && (run || cancelled));
    Check(parent->m_Thread_run_flag <= 0);
    if (!cancelled) {
      // Entry finally starts, sees stop and returns with parent still alive.
      parent->m_bsec_thread_active = true;
      parent->m_bsec_thread_active = false;
      parent->m_Thread_run_flag = -1;
    }
    joined = true;
    ++wait_count;
    calls.push_back("join");
    return nullptr;
  }
};
#include "actual_lifecycle.inc"

static void Reset() {
  fail_create = fail_run = false;
  create_count = run_count = delete_count = wait_count = destroy_count = 0;
  calls.clear();
}
int main() {
  Reset();
  {
    CommDriverN2KSerial driver;
    Check(driver.Open());
    Check(!driver.m_bsec_thread_active); // Entry has not begun.
    Check(!driver.Open()); // Never replace a live owned worker.
    driver.Close();
    Check(wait_count == 1 && destroy_count == 1 && delete_count == 0);
    Check(driver.m_pSecondary_Thread == nullptr && driver.m_Thread_run_flag == -1);
    Check(calls == std::vector<std::string>({"timer stop", "disconnect", "join", "destroy"}));
    driver.Close();
    Check(wait_count == 1 && destroy_count == 1);
  }
  Reset();
  fail_create = true;
  {
    CommDriverN2KSerial driver;
    Check(!driver.Open());
    Check(create_count == 1 && run_count == 0 && wait_count == 0 && destroy_count == 1);
    Check(driver.m_pSecondary_Thread == nullptr && driver.m_Thread_run_flag == -1);
    driver.Close();
  }
  Reset();
  fail_run = true;
  {
    CommDriverN2KSerial driver;
    Check(!driver.Open());
    Check(delete_count == 1 && wait_count == 1 && destroy_count == 1);
    Check(calls == std::vector<std::string>({"cancel", "join", "destroy"}));
    Check(driver.m_pSecondary_Thread == nullptr && driver.m_Thread_run_flag == -1);
    driver.Close();
    Check(wait_count == 1 && destroy_count == 1);
  }
  Reset();
  {
    CommDriverN2KSerial driver;
    Check(driver.Open());
    driver.m_bsec_thread_active = true;
    driver.Close();
    Check(wait_count == 1 && destroy_count == 1 && !driver.m_bsec_thread_active);
  }
  std::cout << "actual serial lifecycle: create failure, run failure, delayed Entry, active close, repeated close passed\n";
}

// The Helm Server child process with stdin/stdout pipes (Windows and POSIX).
#include "helm/HelmHost.h"
#include <mutex>

#ifdef _WIN32
#include <windows.h>
#else
#include <cerrno>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <pthread.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>
#endif

namespace opennav::helm {
namespace {

#ifdef _WIN32
std::wstring Wide(const std::string &utf8) {
  if (utf8.empty()) return {};
  const int n = MultiByteToWideChar(CP_UTF8, 0, utf8.data(), static_cast<int>(utf8.size()), nullptr, 0);
  std::wstring out(static_cast<size_t>(n), L'\0');
  MultiByteToWideChar(CP_UTF8, 0, utf8.data(), static_cast<int>(utf8.size()), out.data(), n);
  return out;
}

// CommandLineToArgvW quoting rules.
std::wstring QuoteArg(const std::wstring &arg) {
  if (!arg.empty() && arg.find_first_of(L" \t\n\v\"") == std::wstring::npos) return arg;
  std::wstring out = L"\"";
  for (size_t i = 0;; ++i) {
    size_t slashes = 0;
    while (i < arg.size() && arg[i] == L'\\') { ++i; ++slashes; }
    if (i == arg.size()) { out.append(slashes * 2, L'\\'); break; }
    if (arg[i] == L'"') { out.append(slashes * 2 + 1, L'\\'); out.push_back(L'"'); }
    else { out.append(slashes, L'\\'); out.push_back(arg[i]); }
  }
  return out + L"\"";
}

class WindowsProcess final : public HelmProcess {
public:
  ~WindowsProcess() override {
    Stop();
    if (out_) CloseHandle(out_);
    if (job_) CloseHandle(job_);
  }
  bool Start(const std::string &exe, const std::vector<std::string> &args, const std::string &log, std::string &error) override {
    SECURITY_ATTRIBUTES sa{sizeof(sa), nullptr, TRUE};
    HANDLE child_in = nullptr, child_out = nullptr, child_err = nullptr;
    if (!CreatePipe(&child_in, &in_, &sa, 0) || !CreatePipe(&out_, &child_out, &sa, 0)) {
      error = "pipes unavailable";
      return false;
    }
    SetHandleInformation(in_, HANDLE_FLAG_INHERIT, 0);
    SetHandleInformation(out_, HANDLE_FLAG_INHERIT, 0);
    child_err = CreateFileW(log.empty() ? L"NUL" : Wide(log).c_str(), GENERIC_WRITE, FILE_SHARE_READ, &sa,
                            CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    STARTUPINFOW si{};
    si.cb = sizeof(si);
    si.dwFlags = STARTF_USESTDHANDLES;
    si.hStdInput = child_in;
    si.hStdOutput = child_out;
    si.hStdError = child_err == INVALID_HANDLE_VALUE ? nullptr : child_err;
    std::wstring command = QuoteArg(Wide(exe));
    for (const auto &a : args) command += L" " + QuoteArg(Wide(a));
    PROCESS_INFORMATION pi{};
    const BOOL ok = CreateProcessW(Wide(exe).c_str(), command.data(), nullptr, nullptr, TRUE,
                                   CREATE_NO_WINDOW | CREATE_UNICODE_ENVIRONMENT | CREATE_SUSPENDED, nullptr, nullptr, &si, &pi);
    CloseHandle(child_in);
    CloseHandle(child_out);
    if (child_err != INVALID_HANDLE_VALUE) CloseHandle(child_err);
    if (!ok) {
      error = "could not run " + exe;
      return false;
    }
    // If SKAGER ends abruptly, Windows ends Helm with it.
    job_ = CreateJobObjectW(nullptr, nullptr);
    if (job_) {
      JOBOBJECT_EXTENDED_LIMIT_INFORMATION limits{};
      limits.BasicLimitInformation.LimitFlags = JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
      SetInformationJobObject(job_, JobObjectExtendedLimitInformation, &limits, sizeof(limits));
      AssignProcessToJobObject(job_, pi.hProcess);
    }
    ResumeThread(pi.hThread);
    CloseHandle(pi.hThread);
    process_ = pi.hProcess;
    return true;
  }
  bool WriteLine(const std::string &line) override {
    std::lock_guard<std::mutex> lock(write_);
    if (!in_) return false;
    const std::string data = line + "\n";
    DWORD written = 0;
    return WriteFile(in_, data.data(), static_cast<DWORD>(data.size()), &written, nullptr) && written == data.size();
  }
  bool ReadLine(std::string &line) override {
    for (;;) {
      const auto nl = buffer_.find('\n');
      if (nl != std::string::npos) {
        line = buffer_.substr(0, nl);
        if (!line.empty() && line.back() == '\r') line.pop_back();
        buffer_.erase(0, nl + 1);
        return true;
      }
      char chunk[4096];
      DWORD n = 0;
      if (!ReadFile(out_, chunk, sizeof(chunk), &n, nullptr) || n == 0) return false;
      buffer_.append(chunk, n);
      if (buffer_.size() > (1u << 20)) buffer_.clear();
    }
  }
  void Stop() override {
    {
      std::lock_guard<std::mutex> lock(write_);
      if (in_) { CloseHandle(in_); in_ = nullptr; }
    }
    std::lock_guard<std::mutex> lock(stop_);
    if (!process_) return;
    if (WaitForSingleObject(process_, 3000) != WAIT_OBJECT_0) TerminateProcess(process_, 1);
    WaitForSingleObject(process_, 2000);
    CloseHandle(process_);
    process_ = nullptr;
  }

private:
  HANDLE in_ = nullptr, out_ = nullptr, process_ = nullptr, job_ = nullptr;
  std::mutex write_, stop_;
  std::string buffer_;
};
#else
class PosixProcess final : public HelmProcess {
public:
  ~PosixProcess() override {
    Stop();
    if (out_ >= 0) close(out_);
  }
  bool Start(const std::string &exe, const std::vector<std::string> &args, const std::string &log, std::string &error) override {
    int in[2], out[2];
    if (pipe(in) != 0 || pipe(out) != 0) {
      error = "pipes unavailable";
      return false;
    }
    std::vector<std::string> storage{exe};
    storage.insert(storage.end(), args.begin(), args.end());
    std::vector<char *> argv;
    for (auto &s : storage) argv.push_back(s.data());
    argv.push_back(nullptr);
    const int err = open(log.empty() ? "/dev/null" : log.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0600);
    pid_ = fork();
    if (pid_ == 0) {
      dup2(in[0], 0);
      dup2(out[1], 1);
      if (err >= 0) dup2(err, 2);
      for (int fd : {in[0], in[1], out[0], out[1]}) close(fd);
      execv(exe.c_str(), argv.data());
      _exit(127);
    }
    close(in[0]);
    close(out[1]);
    if (err >= 0) close(err);
    if (pid_ < 0) {
      close(in[1]);
      close(out[0]);
      error = "could not start a process";
      return false;
    }
    fcntl(in[1], F_SETFD, FD_CLOEXEC);
    fcntl(out[0], F_SETFD, FD_CLOEXEC);
    in_ = in[1];
    out_ = out[0];
    // A server that exits at once (missing file, wrong architecture) is a start failure.
    for (int i = 0; i < 10; ++i) {
      int status = 0;
      if (waitpid(pid_, &status, WNOHANG) == pid_) {
        pid_ = -1;
        error = WIFEXITED(status) && WEXITSTATUS(status) == 127 ? "could not run " + exe : "it exited at once";
        return false;
      }
      usleep(10000);
    }
    return true;
  }
  bool WriteLine(const std::string &line) override {
    std::lock_guard<std::mutex> lock(write_);
    if (in_ < 0) return false;
    const std::string data = line + "\n";
    // A server that already exited must not raise SIGPIPE in SKAGER: block it
    // for this thread and swallow one that the write leaves pending.
    sigset_t pipe_set, old;
    sigemptyset(&pipe_set);
    sigaddset(&pipe_set, SIGPIPE);
    pthread_sigmask(SIG_BLOCK, &pipe_set, &old);
    size_t done = 0;
    bool ok = true;
    while (done < data.size()) {
      const ssize_t n = write(in_, data.data() + done, data.size() - done);
      if (n < 0 && errno == EINTR) continue;
      if (n <= 0) { ok = false; break; }
      done += static_cast<size_t>(n);
    }
    if (!ok) {
      const timespec zero{0, 0};
      sigtimedwait(&pipe_set, nullptr, &zero);
    }
    pthread_sigmask(SIG_SETMASK, &old, nullptr);
    return ok;
  }
  bool ReadLine(std::string &line) override {
    for (;;) {
      const auto nl = buffer_.find('\n');
      if (nl != std::string::npos) {
        line = buffer_.substr(0, nl);
        buffer_.erase(0, nl + 1);
        return true;
      }
      char chunk[4096];
      const ssize_t n = read(out_, chunk, sizeof(chunk));
      if (n < 0 && errno == EINTR) continue;
      if (n <= 0) return false;
      buffer_.append(chunk, static_cast<size_t>(n));
      if (buffer_.size() > (1u << 20)) buffer_.clear();
    }
  }
  void Stop() override {
    {
      std::lock_guard<std::mutex> lock(write_);
      if (in_ >= 0) { close(in_); in_ = -1; }
    }
    std::lock_guard<std::mutex> lock(stop_);
    if (pid_ <= 0) return;
    for (int i = 0; i < 300; ++i) {  // Up to 3 s for an orderly exit.
      if (waitpid(pid_, nullptr, WNOHANG) == pid_) { pid_ = -1; return; }
      usleep(10000);
    }
    kill(pid_, SIGKILL);
    waitpid(pid_, nullptr, 0);
    pid_ = -1;
  }

private:
  int in_ = -1, out_ = -1;
  pid_t pid_ = -1;
  std::mutex write_, stop_;
  std::string buffer_;
};
#endif
}  // namespace

std::unique_ptr<HelmProcess> CreateHelmProcess() {
#ifdef _WIN32
  return std::make_unique<WindowsProcess>();
#else
  return std::make_unique<PosixProcess>();
#endif
}

}  // namespace opennav::helm

#include "integration/UpdateQualification.h"
#include <algorithm>
#include <atomic>
#include <utility>
#ifdef _WIN32
#include "platform/windows/UpdateQualificationNative.h"
#endif
namespace opennav::integration {
namespace {
bool Hex(const std::string& s, std::size_t n) {
  return s.size() == n && std::all_of(s.begin(), s.end(), [](char c) {
    return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
  });
}
UpdateQualificationFlag flag;
std::atomic<bool> suppressed{false};
QualifiedConfigLoadOwner* active = nullptr;
bool attempted = false;
std::optional<QualifiedConfigObservation> loaded;
std::unique_ptr<QualifiedConfigLoadOwner> retained;
}
std::optional<std::string> EncodeQualifiedConfigObservation(const QualifiedConfigObservation& o) {
  if (!Hex(o.directory_volume,16) || !Hex(o.directory_id,32) ||
      !Hex(o.directory_created,16) || !Hex(o.file_volume,16) ||
      !Hex(o.file_id,32) || !Hex(o.file_created,16) || !Hex(o.ini_sha256,64)) return {};
  return o.directory_volume + " " + o.directory_id + " " + o.directory_created + " " +
      o.file_volume + " " + o.file_id + " " + o.file_created + " " + o.ini_sha256;
}
void UpdateQualificationFlag::Capture(const std::optional<std::string>& value, bool removed) {
  if (captured_) return;
  captured_ = true;
  required_ = value.has_value() || !removed;
  valid_ = value && *value == "1" && removed;
}
void CaptureUpdateQualificationFlag() noexcept {
#ifdef _WIN32
  try {
    const auto captured = platform::TakeUpdateQualificationFlag();
    flag.Capture(captured.first, captured.second);
  } catch (...) { flag.Capture(std::string{}, false); }
#else
  flag.Capture({});
#endif
  if (flag.Required()) suppressed.store(true, std::memory_order_release);
}
bool QualificationOutputSuppressed() noexcept { return suppressed.load(std::memory_order_acquire); }
bool QualificationProfileRequired() noexcept { return flag.Required(); }
bool QualifiedProfileObserved() noexcept { return !flag.Required() || loaded.has_value(); }
std::optional<QualifiedConfigObservation> LoadedQualifiedConfigObservation() { return loaded; }
struct QualifiedConfigLoadOwner::Impl {
#ifdef _WIN32
  std::unique_ptr<platform::NativeQualifiedConfigLoad> native;
#endif
};
QualifiedConfigLoadOwner::QualifiedConfigLoadOwner(std::unique_ptr<Impl> p) : impl_(std::move(p)) {}
QualifiedConfigLoadOwner::~QualifiedConfigLoadOwner() { if (active == this) active = nullptr; }
std::unique_ptr<QualifiedConfigLoadOwner> BeginQualifiedConfigLoad(const std::wstring& path) {
  if (!flag.Required()) return {};
  if (!flag.Valid() || attempted) return {};
  attempted = true;
#ifdef _WIN32
  try {
    auto p = std::make_unique<QualifiedConfigLoadOwner::Impl>();
    p->native = platform::NativeQualifiedConfigLoad::Open(path);
    auto owner = std::unique_ptr<QualifiedConfigLoadOwner>(new QualifiedConfigLoadOwner(std::move(p)));
    active = owner.get();
    return owner;
  } catch (...) { return {}; }
#else
  (void)path;
  return {};
#endif
}
void ObserveQualifiedConfigConstructor(const std::wstring& path) noexcept {
#ifdef _WIN32
  try { if (active) active->impl_->native->ObserveConstructor(path); }
  catch (...) { active = nullptr; }
#else
  (void)path;
#endif
}
bool CompleteQualifiedConfigLoad(std::unique_ptr<QualifiedConfigLoadOwner> owner,
                                const std::wstring& path) noexcept {
  if (!flag.Required()) return !owner;
  if (!owner || active != owner.get() || loaded) return false;
  active = nullptr;
#ifdef _WIN32
  try {
    auto observation = owner->impl_->native->Complete(path);
    retained = std::move(owner); // Keep original directory/ancestor pins alive.
    loaded = std::move(observation);
    return true;
  }
  catch (...) { return false; }
#else
  (void)path;
  return false;
#endif
}
}  // namespace opennav::integration

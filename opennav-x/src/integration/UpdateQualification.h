#pragma once
#include <memory>
#include <optional>
#include <string>

namespace opennav::integration {
// Observation only. The broker must independently match its original cold
// profile and authenticate the held child; these bytes grant no health/launch.
struct QualifiedConfigObservation {
  std::string directory_volume, directory_id, directory_created;
  std::string file_volume, file_id, file_created, ini_sha256;
};
std::optional<std::string> EncodeQualifiedConfigObservation(
    const QualifiedConfigObservation& observation);

class UpdateQualificationFlag final {
 public:
  void Capture(const std::optional<std::string>& flag, bool removed = true);
  bool Required() const { return required_; }
  bool Valid() const { return captured_ && valid_; }
 private:
  bool captured_ = false, required_ = false, valid_ = false;
};
// Captured once before config construction, then removed from OS and CRT env.
// Any present value latches suppression for this process, including malformed
// values. This restriction is never a server-authentication claim.
void CaptureUpdateQualificationFlag() noexcept;
bool QualificationOutputSuppressed() noexcept;
bool QualificationProfileRequired() noexcept;
bool QualifiedProfileObserved() noexcept;
std::optional<QualifiedConfigObservation> LoadedQualifiedConfigObservation();

class QualifiedConfigLoadOwner final {
 public:
  ~QualifiedConfigLoadOwner();
  QualifiedConfigLoadOwner(const QualifiedConfigLoadOwner&) = delete;
  QualifiedConfigLoadOwner& operator=(const QualifiedConfigLoadOwner&) = delete;
 private:
  struct Impl;
  explicit QualifiedConfigLoadOwner(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
  friend std::unique_ptr<QualifiedConfigLoadOwner> BeginQualifiedConfigLoad(const std::wstring&);
  friend void ObserveQualifiedConfigConstructor(const std::wstring&) noexcept;
  friend bool CompleteQualifiedConfigLoad(std::unique_ptr<QualifiedConfigLoadOwner>, const std::wstring&) noexcept;
};
// These hooks surround the actual wxFileConfig constructor and LoadMyConfig.
// No profile is edited. The load pin ends after observation; qualification-only
// MyConfig suppresses disk commits while ordinary startup keeps normal wx saves.
std::unique_ptr<QualifiedConfigLoadOwner> BeginQualifiedConfigLoad(const std::wstring& actual_path);
void ObserveQualifiedConfigConstructor(const std::wstring& actual_path) noexcept;
bool CompleteQualifiedConfigLoad(std::unique_ptr<QualifiedConfigLoadOwner> owner,
                                const std::wstring& actual_path) noexcept;
}  // namespace opennav::integration

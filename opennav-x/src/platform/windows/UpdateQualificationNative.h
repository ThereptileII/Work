#pragma once
#include "integration/UpdateQualification.h"
#include <memory>
#include <optional>
#include <string>
#include <utility>
namespace opennav::platform {
std::pair<std::optional<std::string>, bool> TakeUpdateQualificationFlag();
class NativeQualifiedConfigLoad final {
 public:
  static std::unique_ptr<NativeQualifiedConfigLoad> Open(const std::wstring& actual_path);
  ~NativeQualifiedConfigLoad();
  NativeQualifiedConfigLoad(const NativeQualifiedConfigLoad&) = delete;
  NativeQualifiedConfigLoad& operator=(const NativeQualifiedConfigLoad&) = delete;
  void ObserveConstructor(const std::wstring& actual_path);
  integration::QualifiedConfigObservation Complete(const std::wstring& actual_path);
 private:
  struct Impl;
  explicit NativeQualifiedConfigLoad(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};
}  // namespace opennav::platform

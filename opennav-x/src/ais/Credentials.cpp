#include "ais/Credentials.h"
#include <algorithm>

namespace opennav::ais {
Secret::~Secret() { Clear(); }
void Secret::Clear() noexcept {
  volatile char *data = bytes_.data();
  for (std::size_t i = 0; i < bytes_.size(); ++i)
    data[i] = 0;
  size_ = 0;
}
bool Secret::Assign(std::string_view value) {
  if (value.empty() || value.size() > bytes_.size()) {
    Clear();
    return false;
  }
  for (unsigned char c : value)
    if (c < 33 || c > 126) {
      Clear();
      return false;
    }
  // Copy before clearing: Assign(View()) and a subview are safe as well.
  Secret replacement;
  std::copy(value.begin(), value.end(), replacement.bytes_.begin());
  replacement.size_ = value.size();
  *this = std::move(replacement);
  return true;
}
Secret::Secret(Secret &&other) noexcept { *this = std::move(other); }
Secret &Secret::operator=(Secret &&other) noexcept {
  if (this != &other) {
    Clear();
    bytes_ = other.bytes_;
    size_ = other.size_;
    other.Clear();
  }
  return *this;
}
} // namespace opennav::ais

#include "ais/Credentials.h"
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <type_traits>

using namespace opennav::ais;
int checks = 0;
#define CHECK(v)                                                               \
  do {                                                                         \
    ++checks;                                                                  \
    if (!(v))                                                                  \
      throw std::runtime_error("Credential check failed at line " +            \
                               std::to_string(__LINE__));                      \
  } while (false)
static_assert(!std::is_copy_constructible_v<Secret>);
static_assert(!std::is_copy_assignable_v<Secret>);
static_assert(std::is_nothrow_move_constructible_v<Secret>);

int main() {
  try {
    Secret first;
    CHECK(first.Empty());
    CHECK(first.Assign("test-secret-not-a-service-key"));
    Secret second(std::move(first));
    CHECK(first.Empty());
    CHECK(!second.Empty());
    first = std::move(second);
    CHECK(second.Empty());
    CHECK(first.View() == "test-secret-not-a-service-key");
    CHECK(first.Assign(first.View()));
    CHECK(first.View() == "test-secret-not-a-service-key");
    CHECK(!second.Assign("bad\nkey"));
    CHECK(second.Empty());
    CHECK(!second.Assign(std::string(513, 'a')));
    CHECK(second.Empty());
    CHECK(second.Assign(std::string(512, 'a')));
    second.Clear();
    CHECK(second.Empty());
    CHECK(!second.Assign(std::string_view("x\0y", 3)));
    CHECK(second.Empty());
#ifdef _WIN32
    const auto id = std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count());
    auto store = CreateTestAisCredentials("credential-" + id);
    CHECK(store);
    CHECK(store->Read().status == CredentialStatus::Missing);
    struct Cleanup {
      IAisCredentials *store;
      ~Cleanup() { store->Remove(); }
    } cleanup{store.get()};
    CHECK(store->Store(first) == CredentialStatus::Ready);
    auto saved = store->Read();
    CHECK(saved.status == CredentialStatus::Ready);
    CHECK(saved.key.View() == first.View());
    auto reopened = CreateTestAisCredentials("credential-" + id);
    CHECK(reopened->Read().key.View() == first.View());
    Secret replacement;
    CHECK(replacement.Assign("replacement-test-secret"));
    CHECK(reopened->Store(replacement) == CredentialStatus::Ready);
    CHECK(store->Read().key.View() == replacement.View());
    CHECK(store->Store(Secret{}) == CredentialStatus::Invalid);
    CHECK(store->Read().key.View() == replacement.View());
    CHECK(store->Remove() == CredentialStatus::Removed);
    CHECK(reopened->Read().status == CredentialStatus::Missing);
    CHECK(store->Remove() == CredentialStatus::Removed);
    CHECK(!CreateTestAisCredentials("../AISStream/v1"));
    CHECK(!CreateTestAisCredentials(""));
#else
    // Environment changes affect this test process only, never its parent.
    unsetenv("AISSTREAM_API_KEY");
    auto store = CreateAisCredentials();
    CHECK(store->Read().status == CredentialStatus::Missing);
    setenv("AISSTREAM_API_KEY", "test-environment-secret", 1);
    auto read = store->Read();
    CHECK(read.status == CredentialStatus::Ready);
    CHECK(read.key.View() == "test-environment-secret");
    CHECK(store->Store(first) == CredentialStatus::ReadOnly);
    CHECK(store->Remove() == CredentialStatus::ReadOnly);
    setenv("AISSTREAM_API_KEY", std::string(513, 'a').c_str(), 1);
    CHECK(store->Read().status == CredentialStatus::Invalid);
    setenv("AISSTREAM_API_KEY", "bad\nkey", 1);
    CHECK(store->Read().status == CredentialStatus::Invalid);
    unsetenv("AISSTREAM_API_KEY");
    CHECK(store->Read().status == CredentialStatus::Missing);
#endif
    first.Clear();
    CHECK(first.Empty());
    std::cout << checks
              << " credential checks passed; no credential bytes logged\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}

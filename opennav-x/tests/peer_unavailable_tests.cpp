// Calls the real pinned model entrypoints linked into the integrated product.
// No peer, socket, credential store or vessel object is needed by an unavailable
// operation, even if a stale/programmatic caller bypasses the UI.
#include "model/peer_client.h"
#include <gtest/gtest.h>

// This upstream symbol is externally linked although not declared in its header.
bool CheckKey(const std::string& key, PeerData peer_data);

namespace {
struct PeerAttempt {
  EventVar progress;
  PeerData data{progress};
  unsigned status_calls = 0;
  unsigned pairing_calls = 0;

  PeerAttempt() {
    data.dest_ip_address = "127.0.0.1:1";
    data.server_name = "untrusted-peer.invalid";
    data.api_version = SemanticVersion(5, 12);
    data.run_status_dlg = [this](PeerDlg, int) {
      ++status_calls;
      return PeerDlgResult::Cancel;
    };
    data.run_pincode_dlg = [this]() {
      ++pairing_calls;
      return std::make_pair(PeerDlgResult::Cancel, std::string{});
    };
  }
  void ExpectNoDialogs() const {
    EXPECT_EQ(status_calls, 0u);
    EXPECT_EQ(pairing_calls, 0u);
  }
};
}

TEST(OpenNavPeerUnavailable, RejectsEmptyTransfersInsteadOfClaimingSuccess) {
  PeerAttempt attempt;
  EXPECT_FALSE(CheckNavObjects(attempt.data));
  EXPECT_FALSE(SendNavobjects(attempt.data));
  attempt.ExpectNoDialogs();
}

TEST(OpenNavPeerUnavailable, ForcedSendDoesNotInspectOrSerializeObjects) {
  PeerAttempt attempt;
  // Deliberately unusable objects: containment must run before dereferencing
  // navigation state, acquiring credentials, asking for a PIN or sending data.
  attempt.data.routes.push_back(nullptr);
  attempt.data.routepoints.push_back(nullptr);
  attempt.data.tracks.push_back(nullptr);
  attempt.data.activate = true;
  attempt.data.overwrite = true;
  EXPECT_FALSE(CheckNavObjects(attempt.data));
  EXPECT_FALSE(SendNavobjects(attempt.data));
  attempt.ExpectNoDialogs();
  ASSERT_EQ(attempt.data.routes.size(), 1u);
  ASSERT_EQ(attempt.data.routepoints.size(), 1u);
  ASSERT_EQ(attempt.data.tracks.size(), 1u);
  EXPECT_EQ(attempt.data.routes.front(), nullptr);
  EXPECT_EQ(attempt.data.routepoints.front(), nullptr);
  EXPECT_EQ(attempt.data.tracks.front(), nullptr);
  EXPECT_TRUE(attempt.data.activate);
  EXPECT_TRUE(attempt.data.overwrite);
}

TEST(OpenNavPeerUnavailable, ClearsRetainedVersionWithoutPairingOrFallback) {
  PeerAttempt attempt;
  GetApiVersion(attempt.data);
  EXPECT_TRUE(attempt.data.api_version == SemanticVersion(0, 0));
  GetApiVersion(attempt.data);
  EXPECT_TRUE(attempt.data.api_version == SemanticVersion(0, 0));
  attempt.ExpectNoDialogs();
}

TEST(OpenNavPeerUnavailable, DirectKeyCheckRejectsWithoutSendingCredential) {
  PeerAttempt attempt;
  EXPECT_FALSE(CheckKey("fake-never-transmit", attempt.data));
  attempt.ExpectNoDialogs();
}

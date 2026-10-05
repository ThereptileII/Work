// Only framework/driver collaborators are stubs. The writer and serial encoder
// below are extracted verbatim from production; tN2kMsg is linked unchanged.
#include <N2kMsg.h>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

extern "C" uint32_t millis() { return 0; }
extern "C" void delay(uint32_t) {}

struct N2kName {
  union { uint64_t Name; } value;
  explicit N2kName(uint64_t name) { value.Name = name; }
};
struct NavAddr {
  enum class Bus { N2000, Undef };
  Bus bus = Bus::Undef;
  std::string iface;
  virtual ~NavAddr() = default;
};
struct NavAddr2000 : NavAddr {
  N2kName name{0};
  unsigned char address;
  NavAddr2000(const std::string& port, unsigned char addr) : address(addr) {
    bus = Bus::N2000;
    iface = port;
  }
  // Like upstream, a NAME-only address does not initialize the numeric address.
  NavAddr2000(const std::string& port, N2kName identity) : name(identity) {
    bus = Bus::N2000;
    iface = port;
  }
  NavAddr2000() = default;
};
struct NavMsg { virtual ~NavMsg() = default; };
struct Nmea2000Msg : NavMsg {
  struct { uint64_t pgn; } PGN;
  std::vector<unsigned char> payload;
  std::shared_ptr<const NavAddr2000> source;
  int priority;
  Nmea2000Msg(uint64_t pgn, const std::vector<unsigned char>& data,
              std::shared_ptr<const NavAddr2000> from, int prio = 6)
      : PGN{pgn}, payload(data), source(from), priority(prio) {}
};
struct FakeQueue {
  bool accepts = true;
  unsigned attempts = 0;
  std::vector<std::vector<unsigned char>> frames;
  bool SetOutMsg(const std::vector<unsigned char>& frame) {
    ++attempts;
    if (accepts) frames.push_back(frame);
    return accepts;
  }
};
struct FakeListener {
  std::vector<std::shared_ptr<const Nmea2000Msg>> messages;
  void Notify(std::shared_ptr<const Nmea2000Msg> message) {
    messages.push_back(std::move(message));
  }
};
struct CommDriverN2KSerial {
  std::string iface = "FAKE-NO-PHYSICAL-PORT";
  bool m_closing = false;
  bool active = true;
  FakeQueue queue;
  FakeQueue* worker = &queue;
  FakeListener m_listener;
  FakeQueue* GetSecondaryThread() { return worker; }
  bool IsSecThreadActive() const { return active; }
  std::shared_ptr<NavAddr2000> GetAddress(uint64_t name) {
    return std::make_shared<NavAddr2000>(iface, N2kName(name));
  }
  bool SendMessage(std::shared_ptr<const NavMsg>, std::shared_ptr<const NavAddr>);
};

#define ESCAPE 0x10
#define STARTOFTEXT 0x02
#define ENDOFTEXT 0x03
#include "actual_writer.inc"

void Check(bool condition, const char* reason) {
  if (!condition) throw std::runtime_error(reason);
}
auto Destination(unsigned char address = 255) {
  return std::make_shared<NavAddr2000>("FAKE-NO-PHYSICAL-PORT", address);
}
auto Message(const std::vector<unsigned char>& bytes, uint64_t pgn = 59904,
             int priority = 6) {
  return std::make_shared<Nmea2000Msg>(pgn, bytes, nullptr, priority);
}

// Independent wire decoder, checking DLE stuffing, all declared fields and CRC.
void Wire(const std::vector<unsigned char>& wire, uint64_t pgn, int priority,
          unsigned char destination, const std::vector<unsigned char>& payload) {
  Check(wire.size() >= 13 && wire[0] == 0x10 && wire[1] == 2 &&
            wire[wire.size() - 2] == 0x10 && wire.back() == 3, "DLE framing");
  std::vector<unsigned char> body;
  for (size_t i = 2; i < wire.size() - 2; ++i) {
    const auto byte = wire[i];
    if (byte == 0x10)
      Check(++i < wire.size() - 2 && wire[i] == 0x10, "DLE escape pairing");
    body.push_back(byte);
  }
  Check(body.size() == payload.size() + 9, "complete payload, no index wrap");
  Check(body[0] == 0x94 && body[1] == payload.size() + 6 &&
            body[2] == priority && body[6] == destination &&
            body[7] == payload.size(), "TX code, lengths, priority, destination");
  Check((uint64_t(body[3]) | (uint64_t(body[4]) << 8) |
         (uint64_t(body[5]) << 16)) == pgn, "PGN");
  Check(std::vector<unsigned char>(body.begin() + 8, body.end() - 1) == payload,
        "payload content");
  unsigned sum = 0;
  for (auto byte : body) sum += byte;
  Check(sum % 256 == 0, "checksum");
}
void Notifications(const CommDriverN2KSerial& driver, uint64_t pgn) {
  const auto& notices = driver.m_listener.messages;
  Check(notices.size() == 2 && notices[0]->PGN.pgn == pgn &&
            notices[1]->PGN.pgn == 1, "legacy specific and all-PGN notifications");
  for (const auto& notice : notices) {
    Check(!notice->payload.empty() && notice->payload[0] == 0x94,
          "transmit echo must never masquerade as 0x93 physical feedback");
    Check(notice->source && notice->source->iface == driver.iface &&
              notice->source->name.value.Name == 0 && notice->source->address == 254,
          "payload must not fabricate source NAME/address");
  }
}
void Rejected(std::shared_ptr<const NavMsg> msg,
              std::shared_ptr<const NavAddr> address) {
  CommDriverN2KSerial driver;
  Check(!driver.SendMessage(msg, address), "invalid input rejected");
  Check(driver.queue.attempts == 0 && driver.m_listener.messages.empty(),
        "invalid input has no queue/listener effects");
}
int main() {
  try {
    const std::vector<unsigned char> request{0, 0xee, 0};
    {
      CommDriverN2KSerial driver;
      Check(driver.SendMessage(Message(request), Destination()), "3-byte ISO request");
      Check(driver.queue.frames.size() == 1, "one queued request");
      Wire(driver.queue.frames.front(), 59904, 6, 255, request);
      Notifications(driver, 59904);
    }
    // A complete-PGN command payload, including a byte requiring DLE escaping.
    const std::vector<unsigned char> command{
        1, 0x63, 0xff, 0, 0xf8, 3, 1, 0x3b, 0x9f, 0x10, 0x40, 0, 0xff};
    {
      CommDriverN2KSerial driver;
      Check(driver.SendMessage(Message(command, 126208, 3), Destination(42)),
            "13-byte addressed command");
      Wire(driver.queue.frames.front(), 126208, 3, 42, command);
      Notifications(driver, 126208);
    }
    for (const unsigned char byte : {0x10, 0xff}) {
      CommDriverN2KSerial driver;
      std::vector<unsigned char> maximum(tN2kMsg::MaxDataLen, byte);
      Check(driver.SendMessage(Message(maximum, 126208), Destination(0x10)),
            "legal maximum payload remains supported");
      Wire(driver.queue.frames.front(), 126208, 6, 0x10, maximum);
    }
    {
      CommDriverN2KSerial driver;
      Check(driver.SendMessage(Message(request), nullptr), "legacy null means broadcast");
      Wire(driver.queue.frames.front(), 59904, 6, 255, request);
    }
    Rejected(nullptr, Destination());
    Rejected(std::make_shared<NavMsg>(), Destination());
    Rejected(Message(request), std::make_shared<NavAddr>());
    Rejected(Message(request), std::make_shared<NavAddr2000>());
    Rejected(Message(request), std::make_shared<NavAddr2000>("fake", N2kName(42)));
    Rejected(Message({}), Destination());
    Rejected(Message(std::vector<unsigned char>(tN2kMsg::MaxDataLen + 1, 0)), Destination());
    Rejected(Message(request, 0x40000), Destination());
    Rejected(Message(request, UINT64_MAX), Destination());
    Rejected(Message(request, 59904, -1), Destination());
    Rejected(Message(request, 59904, 8), Destination());
    {
      CommDriverN2KSerial driver;
      driver.queue.accepts = false;
      Check(!driver.SendMessage(Message(request), Destination()) &&
                driver.queue.attempts == 10 && driver.queue.frames.empty(),
            "queue rejection is false; no physical success inferred");
      Notifications(driver, 59904); // Historical TX attempt notifications retained.
    }
    {
      CommDriverN2KSerial driver;
      driver.worker = nullptr;
      Check(!driver.SendMessage(Message(request), Destination()), "no worker is false");
      driver.worker = &driver.queue;
      driver.active = false;
      Check(!driver.SendMessage(Message(request), Destination()), "inactive worker is false");
      Check(driver.queue.attempts == 0, "inactive worker never enqueues");
    }
    {
      CommDriverN2KSerial driver;
      driver.m_closing = true;
      Check(!driver.SendMessage(Message(request), Destination()) &&
                driver.queue.attempts == 0 && driver.m_listener.messages.empty(),
            "closing driver rejects without effects");
    }
    tN2kMsg malformed;
    malformed.DataLen = -1;
    Check(BufferToActisenseFormat(malformed).empty(), "negative serializer length rejected");
    malformed.DataLen = tN2kMsg::MaxDataLen + 1;
    Check(BufferToActisenseFormat(malformed).empty(), "excess serializer length rejected");
    std::cout << "PASS actual serial writer: short request, addressed command, maximum escaped payload, input rejection, TX provenance and queue acceptance\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL " << error.what() << '\n';
    return 1;
  }
}

// Only framework/driver collaborators are stubs. The writer and serial encoder
// below are extracted verbatim from production; tN2kMsg is linked unchanged.
#include <N2kMsg.h>
// Simulate Win32 headers without NOMINMAX. Production helpers must coexist
// with these macros; the portable harness itself otherwise has no SDK headers.
#include <chrono>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <vector>
#include <utility>
#define max(a,b) WINDOWS_MAX_MACRO_MUST_NOT_EXPAND(a,b)
#define min(a,b) WINDOWS_MIN_MACRO_MUST_NOT_EXPAND(a,b)
#include "model/comm_drv_n2k_serial_state.h"
#undef max
#undef min
#include "model/comm_drv_n2k_serial_framer.h"
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
struct Nmea2000SerialMsg : Nmea2000Msg {
  using Nmea2000Msg::Nmea2000Msg;
  N2kSerialState::Clock::time_point received_at{};
  uint64_t connection_generation = 0;
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
enum { DS_TYPE_INPUT, DS_TYPE_OUTPUT, DS_TYPE_INPUT_OUTPUT };
struct CommDriverN2KSerialEvent {
  std::shared_ptr<std::vector<unsigned char>> payload;
  N2kSerialState::Clock::time_point received_at{};
  uint64_t connection_generation = 0;
  auto GetPayload() { return payload; }
};
struct CommDriverN2KSerial {
  std::string iface = "FAKE-NO-PHYSICAL-PORT";
  bool m_closing = false;
  bool active = true;
  N2kSerialState m_serial_state;
  struct { bool bEnabled = true; int IOSelect = DS_TYPE_INPUT_OUTPUT; } m_params;
  void ProcessManagementPacket(std::vector<unsigned char>*) {}
  void handle_N2K_SERIAL_RAW(CommDriverN2KSerialEvent&);
  bool SendPilotMessage(std::shared_ptr<const NavMsg>, std::shared_ptr<const NavAddr>, uint64_t, uint64_t* = nullptr);
  FakeQueue queue;
  FakeQueue* worker = &queue;
  FakeListener m_listener;
  FakeQueue* GetSecondaryThread() { return worker; }
  bool IsSecThreadActive() const { return active; }
  std::shared_ptr<NavAddr2000> GetAddress(uint64_t name) {
    return std::make_shared<NavAddr2000>(iface, N2kName(name));
  }
  int SendMgmtMsg(unsigned char*,size_t,unsigned char,int,bool*);
  bool SendMessage(std::shared_ptr<const NavMsg>, std::shared_ptr<const NavAddr>);
};

struct CommDriverN2KSerialThread {
  struct Port {
    bool open = true, throws = false;
    size_t count = 0, purges = 0;
    bool isOpen() { return open; }
    size_t write(uint8_t*, size_t size) {
      if (throws) throw std::runtime_error("fake write error");
      return std::min(size, count);
    }
    void flushOutput() { ++purges; }
  } m_serial;
  size_t WriteComPortPhysical(std::vector<unsigned char>);
  size_t WriteComPortPhysical(unsigned char*, size_t);
};
#define DEBUG_LOG std::cerr
unsigned sleeps=0;
void wxMilliSleep(int){++sleeps;}
void wxYieldIfNeeded(){}
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
void LifecycleTests();
int main() {
  try {
    LifecycleTests();
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


void LifecycleTests() {
  using Clock = N2kSerialState::Clock;
  const std::vector<unsigned char> bytes{1,2,3};
  N2kSerialState state;
  Check(!state.Enqueue(bytes), "closed queue rejects all output");
  state.Connection(true);
  auto epoch = state.Get().epoch;
  auto now = Clock::now();
  Check(!state.Enqueue(bytes, true, epoch, now), "serial session starts OFF");
  Check(state.Enqueue(bytes, true, epoch, now, false), "explicit discovery allowed without steering session");
  int writes = 0;
  Check(state.WriteOne([&](auto& b) { ++writes; return b.size(); }, now) == 1, "nonsteering request written once");
  Check(state.Get().pilot_writes == 0, "discovery is not command write provenance");
  state.EnablePilot(true);
  Check(state.Enqueue(bytes, true, epoch, now), "session accepts one pilot item");
  Check(!state.Enqueue(bytes, true, epoch, now), "second pilot item refused");
  state.EnablePilot(false);
  Check(state.WriteOne([&](auto& b) { ++writes; return b.size(); }, now) == 0 && writes == 1, "disable cancels unsent pilot");
  state.EnablePilot(true);
  Check(state.Enqueue(bytes, true, epoch, now), "queue before disconnect");
  state.Connection(false); state.Connection(true);
  Check(state.Get().epoch != epoch && state.WriteOne([&](auto& b) { ++writes; return b.size(); }) == 0, "reconnect purges and advances epoch");
  Check(!state.Enqueue(bytes, true, epoch, now), "stale epoch rejected");
  epoch = state.Get().epoch;
  Check(!state.Enqueue(bytes, true, epoch, now), "reconnect never restores session");
  state.EnablePilot(true);
  Check(state.Enqueue(bytes, true, epoch, now), "queue timeout item");
  Check(state.WriteOne([&](auto& b) { ++writes; return b.size(); }, now + std::chrono::milliseconds(500)) == 0, "500ms expired item never transmitted");
  Check(writes == 1, "expired and cancelled items never wrote");
  Check(state.Enqueue(bytes, true, epoch), "queue short-write item");
  Check(state.Enqueue(bytes), "queue unrelated output behind pilot");
  Check(state.WriteOne([](auto&) { return size_t(1); }) == -1 && !state.Get().connected, "partial write invalidates connection");
  state.Connection(true); state.EnablePilot(true);
  Check(state.WriteOne([](auto&) { throw std::runtime_error("must not replay"); return size_t(3); }) == 0, "failure purges all queued output");
  epoch = state.Get().epoch;
  Check(state.Enqueue(bytes, true, epoch), "queue successful write");
  Check(state.WriteOne([](auto& b) { return b.size(); }) == 1 && state.Get().pilot_writes == 1 && state.Get().pilot_written_at >= now, "completed write has monotonic provenance");
  Check(state.Enqueue(bytes, true, epoch), "queue exception item");
  Check(state.WriteOne([](auto&) -> size_t { throw std::runtime_error("write callback failure"); }) == -1 && !state.Get().connected, "throwing writer invalidates/purges without retry");
  state.Connection(true);
  for (unsigned i=0; i<20; ++i) Check(state.Enqueue(bytes), "bounded legacy queue capacity");
  Check(!state.Enqueue(bytes), "queue capacity refuses overflow");
  state.Connection(false);

  CommDriverN2KSerial driver;
  driver.m_serial_state.Connection(true);
  epoch = driver.m_serial_state.Get().epoch;
  auto dest = Destination(42);
  std::vector<unsigned char> cmd{1,0x63,0xff,0,0xff,3,1,0x3b,7,3,4,6,0x40};
  uint64_t command_ticket = 99;
  Check(!driver.SendPilotMessage(Message(cmd,126208,3),dest,epoch,&command_ticket) && command_ticket == 0, "actual pilot sink disabled by default and returns no ticket");
  driver.m_serial_state.EnablePilot(true);
  driver.m_params.IOSelect = DS_TYPE_INPUT;
  Check(!driver.SendPilotMessage(Message(cmd,126208,3),dest,epoch), "actual sink refuses input-only connection");
  driver.m_params.IOSelect = DS_TYPE_INPUT_OUTPUT;
  Check(driver.SendPilotMessage(Message(cmd,126208,3),dest,epoch,&command_ticket) && command_ticket != 0, "actual sink returns this addressed command ticket");
  Check(!driver.SendPilotMessage(Message(cmd,126208,3),dest,epoch), "actual sink pending limit");
  Check(driver.m_serial_state.WriteOne([&](auto& b) { Wire(b,126208,3,42,cmd); return b.size(); }) == 1 && driver.m_serial_state.Get().pilot_written_ticket == command_ticket, "actual serial command encoding and exact completed ticket");
  Check(driver.SendPilotMessage(Message(cmd,126208,3),dest,epoch), "queue AUTO before STANDBY preemption");
  auto standby = cmd; standby.back() = 0;
  Check(driver.SendPilotMessage(Message(standby,126208,3),dest,epoch), "STANDBY replaces unsent AUTO atomically");
  Check(driver.m_serial_state.WriteOne([&](auto& b) { Wire(b,126208,3,42,standby); return b.size(); }) == 1, "only replacement STANDBY is written");
  Check(driver.m_serial_state.WriteOne([](auto& b) { return b.size(); }) == 0, "preempted AUTO never replayed");
  driver.m_serial_state.EnablePilot(false);
  Check(driver.SendPilotMessage(Message({0,0xee,0}),Destination(),epoch), "actual ISO identity request while steering OFF");

  // Actual receive handler: precise type, size, checksum and worker provenance.
  std::vector<unsigned char> rx{0x93,19,3,0x63,0xff,0,255,42,0,0,0,0,8,0x3b,0x9f,0x40,0,0xff,0xff,0xff,0xff,0};
  unsigned sum=0; for(auto b:rx)sum+=b; rx.back()=static_cast<unsigned char>(-sum);
  CommDriverN2KSerialEvent event{std::make_shared<std::vector<unsigned char>>(rx),Clock::now(),epoch};
  driver.handle_N2K_SERIAL_RAW(event);
  auto received = std::dynamic_pointer_cast<const Nmea2000SerialMsg>(driver.m_listener.messages.at(0));
  Check(driver.m_listener.messages.size()==2 && received && received->received_at==event.received_at && received->connection_generation==epoch, "actual RX preserves worker timestamp and epoch");
  driver.m_listener.messages.clear();
  auto valid_at=event.received_at;
  event.received_at=Clock::now()-std::chrono::seconds(4);driver.handle_N2K_SERIAL_RAW(event);
  event.received_at=valid_at;event.connection_generation=epoch+1;driver.handle_N2K_SERIAL_RAW(event);
  event.connection_generation=epoch;event.payload->at(0)=0x94;driver.handle_N2K_SERIAL_RAW(event);
  event.payload=std::make_shared<std::vector<unsigned char>>(std::vector<unsigned char>{0x93});driver.handle_N2K_SERIAL_RAW(event);
  Check(driver.m_listener.messages.empty(), "delayed, wrong epoch, TX echo and short frame never feedback");

  N2kSerialFramer framer;
  std::vector<unsigned char> wire{0x10,2,0x93,0x10,0x10,5,0x10,3};
  int frames=0;
  auto emit=[&](auto& body, auto at, auto generation) { ++frames; Check(body==std::vector<unsigned char>({0x93,0x10,5}) && at==now && generation==9,"framer preserves first read provenance across partial chunks and DLE escaping"); };
  framer.Feed(wire.data(),4,now,9,emit);
  framer.Feed(wire.data()+4,4,now+std::chrono::seconds(1),9,emit);
  Check(frames==1,"one complete frame");
  framer.Feed(wire.data(),4,now,9,emit);framer.Reset();
  framer.Feed(wire.data()+4,4,now,10,emit);
  Check(frames==1,"reconnect drops partial frame");
  std::vector<unsigned char> oversized(520,7);oversized[0]=0x10;oversized[1]=2;
  oversized.push_back(0x10);oversized.push_back(3);
  framer.Feed(oversized.data(),oversized.size(),now,9,emit);
  Check(frames==1,"oversized frame discarded");

  CommDriverN2KSerial management;
  management.queue.accepts=false;
  unsigned char mgmt[]{0x42};
  Check(management.SendMgmtMsg(mgmt,1,0x41,0,nullptr)==1 && management.queue.attempts==10 && sleeps==10,"actual management enqueue terminates on disconnected/full active worker");
  CommDriverN2KSerialThread worker;
  worker.m_serial.count=3;
  Check(worker.WriteComPortPhysical(bytes)==3 && worker.m_serial.purges==0,"actual write never discards output using flushOutput");
  worker.m_serial.count=1;Check(worker.WriteComPortPhysical(bytes)==1,"actual short write preserved for fail-closed queue");
  worker.m_serial.throws=true;Check(worker.WriteComPortPhysical(bytes)==0,"write exception is failure");
}

#include "helm/HelmHost.h"
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

namespace opennav::helm {
namespace {

std::string Str(const rapidjson::Value &v, const char *key) {
  const auto it = v.FindMember(key);
  return it != v.MemberEnd() && it->value.IsString() ? std::string(it->value.GetString(), it->value.GetStringLength()) : "";
}

// Reads `count` digits at `pos`, or -1.
int Digits(const std::string &s, size_t pos, size_t count) {
  if (pos + count > s.size()) return -1;
  int value = 0;
  for (size_t i = pos; i < pos + count; ++i) {
    if (s[i] < '0' || s[i] > '9') return -1;
    value = value * 10 + (s[i] - '0');
  }
  return value;
}

// "2026-10-26T21:14:05.123+01:00" → system_clock (seconds precision).
std::optional<std::chrono::system_clock::time_point> ParseTime(const std::string &iso) {
  const int y = Digits(iso, 0, 4), mo = Digits(iso, 5, 2), d = Digits(iso, 8, 2);
  const int h = Digits(iso, 11, 2), mi = Digits(iso, 14, 2), s = Digits(iso, 17, 2);
  if (y < 0 || mo < 1 || mo > 12 || d < 1 || d > 31 || h < 0 || mi < 0 || s < 0 || iso[10] != 'T')
    return std::nullopt;
  // Days from civil (Howard Hinnant), then apply the zone offset.
  const long long yy = y - (mo <= 2 ? 1 : 0);
  const long long era = (yy >= 0 ? yy : yy - 399) / 400;
  const long long yoe = yy - era * 400;
  const long long doy = (153LL * (mo + (mo > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const long long doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  const long long days = era * 146097 + doe - 719468;
  long long secs = days * 86400 + h * 3600LL + mi * 60LL + s;
  const auto zone = iso.find_first_of("Z+-", 19);
  if (zone != std::string::npos && iso[zone] != 'Z') {
    const int zh = Digits(iso, zone + 1, 2), zm = Digits(iso, zone + 4, 2);
    if (zh >= 0 && zm >= 0) secs -= (iso[zone] == '-' ? -1 : 1) * (zh * 3600LL + zm * 60LL);
  }
  return std::chrono::system_clock::time_point(std::chrono::seconds(secs));
}

std::string Quote(const std::string &text) {
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  writer.String(text.c_str(), static_cast<rapidjson::SizeType>(text.size()));
  return buffer.GetString();
}

void MarkConnected(HelmSnapshot &s, const std::string &id) {
  s.connected_name.clear();
  for (auto &d : s.devices) {
    d.connected = !id.empty() && d.id == id;
    if (d.connected) s.connected_name = d.name;
  }
}
}  // namespace

bool ApplyEvent(HelmSnapshot &s, const std::string &line, std::chrono::steady_clock::time_point now,
                std::atomic<std::int64_t> *input_at) {
  rapidjson::Document doc;
  if (line.size() > 65536 || doc.Parse(line.c_str(), line.size()).HasParseError() || !doc.IsObject()) return false;
  const std::string event = Str(doc, "event");
  if (event == "input") {
    // Not a visible change: no revision bump, so the UI does not rebuild per touch.
    if (input_at) input_at->store(now.time_since_epoch().count());
    return true;
  }
  if (event == "ready") {
    s.state = HostState::Ready;
    s.message.clear();
    s.addresses.clear();
    if (doc.HasMember("port") && doc["port"].IsInt()) s.port = doc["port"].GetInt();
    if (doc.HasMember("addresses") && doc["addresses"].IsArray())
      for (const auto &a : doc["addresses"].GetArray())
        if (a.IsString() && s.addresses.size() < 6) s.addresses.emplace_back(a.GetString());
  } else if (event == "pairing") {
    HelmPairing p;
    p.key = Str(doc, "key");
    p.link = Str(doc, "link");
    if (const auto at = ParseTime(Str(doc, "expires"))) p.expires = *at;
    if (doc.HasMember("qr") && doc["qr"].IsObject()) {
      const auto &qr = doc["qr"];
      if (qr.HasMember("size") && qr["size"].IsInt()) p.qr_size = qr["size"].GetInt();
      if (qr.HasMember("rows") && qr["rows"].IsArray())
        for (const auto &r : qr["rows"].GetArray())
          if (r.IsString() && static_cast<int>(r.GetStringLength()) == p.qr_size) p.qr_rows.emplace_back(r.GetString());
    }
    if (p.key.empty() || p.qr_size < 21 || static_cast<int>(p.qr_rows.size()) != p.qr_size) return false;
    if (doc.HasMember("addresses") && doc["addresses"].IsArray()) {
      s.addresses.clear();
      for (const auto &a : doc["addresses"].GetArray())
        if (a.IsString() && s.addresses.size() < 6) s.addresses.emplace_back(a.GetString());
    }
    s.pairing = std::move(p);
    s.message.clear();
  } else if (event == "pairing_closed") {
    s.pairing.reset();
    const auto reason = Str(doc, "reason");
    s.message = reason == "paired" ? "" : reason == "cancelled" ? "" : "Pairing closed.";
  } else if (event == "paired") {
    if (doc.HasMember("device") && doc["device"].IsObject())
      s.message = "Paired with " + Str(doc["device"], "name") + ".";
    s.pairing.reset();
  } else if (event == "devices") {
    s.devices.clear();
    if (doc.HasMember("devices") && doc["devices"].IsArray())
      for (const auto &d : doc["devices"].GetArray()) {
        if (!d.IsObject()) continue;
        s.devices.push_back({Str(d, "id"), Str(d, "name"), Str(d, "paired_at"), Str(d, "last_seen"), false});
      }
    MarkConnected(s, Str(doc, "connected"));
  } else if (event == "session") {
    const auto state = Str(doc, "state");
    std::string id;
    if (doc.HasMember("device") && doc["device"].IsObject()) id = Str(doc["device"], "id");
    if (state == "connected") MarkConnected(s, id);
    else if (state == "ended") {
      bool was = false;
      for (const auto &d : s.devices) was = was || (d.connected && d.id == id);
      if (was) MarkConnected(s, "");
    }
    s.screen_available = true;
  } else if (event == "screen") {
    s.screen_available = doc.HasMember("available") && doc["available"].IsBool() && doc["available"].GetBool();
  } else if (event == "error") {
    s.message = Str(doc, "message");
  } else {
    return false;
  }
  ++s.revision;
  return true;
}

std::string PairCommand() { return R"({"cmd":"pair"})"; }
std::string CancelPairCommand() { return R"({"cmd":"pair_cancel"})"; }
std::string RevokeCommand(const std::string &id) { return R"({"cmd":"revoke","device":)" + Quote(id) + "}"; }
std::string DisconnectCommand() { return R"({"cmd":"disconnect"})"; }
std::string NameCommand(const std::string &name) { return R"({"cmd":"name","name":)" + Quote(name) + "}"; }
std::string PilotCommand(const std::string &status_json) { return R"({"cmd":"pilot","status":)" + status_json + "}"; }

// ---------------------------------------------------------------- host

HelmHost::HelmHost(Factory factory) : factory_(std::move(factory)) {}

HelmHost::~HelmHost() { Stop(); }

void HelmHost::Update(const std::function<void(HelmSnapshot &)> &change) {
  std::lock_guard<std::mutex> lock(mutex_);
  change(snapshot_);
  ++snapshot_.revision;
}

void HelmHost::Start(const HelmOptions &options) {
  Stop();
  std::lock_guard<std::mutex> lock(mutex_);
  stopping_ = false;
  snapshot_ = HelmSnapshot{HostState::Starting, "", 0, {}, std::nullopt, {}, "", true, "", snapshot_.revision + 1};
  worker_ = std::thread([this, options] { Supervise(options); });
}

void HelmHost::Stop() {
  std::shared_ptr<HelmProcess> process;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
    process = process_;
  }
  wake_.notify_all();
  if (process) process->Stop();  // Closes stdin: the server ends its sessions and exits.
  if (worker_.joinable()) worker_.join();
  std::lock_guard<std::mutex> lock(mutex_);
  process_.reset();
  snapshot_.state = HostState::Off;
  snapshot_.pairing.reset();
  snapshot_.connected_name.clear();
  for (auto &d : snapshot_.devices) d.connected = false;
  ++snapshot_.revision;
}

void HelmHost::Supervise(HelmOptions o) {
  int failures = 0;
  for (;;) {
    std::vector<std::string> args{"-control", "stdio", "-state-dir", o.state_dir, "-name", o.name};
    if (!o.window.empty()) args.insert(args.end(), {"-window", o.window});
    args.insert(args.end(), o.extra_args.begin(), o.extra_args.end());
    std::shared_ptr<HelmProcess> process = factory_();
    std::string error;
    bool ready = false;
    if (!process->Start(o.executable, args, o.log_file, error)) {
      Update([&](HelmSnapshot &s) { s.state = HostState::Failed; s.message = "SKAGER Helm could not start: " + error; });
    } else {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stopping_) { process->Stop(); return; }
        process_ = process;
      }
      HelmProcess *raw = process.get();
      std::string line, pilot;
      while (raw->ReadLine(line)) {
        bool now_ready = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          ApplyEvent(snapshot_, line, std::chrono::steady_clock::now(), &input_at_);
          now_ready = !ready && snapshot_.state == HostState::Ready;
          pilot = pilot_;
        }
        if (now_ready) {
          ready = true;
          failures = 0;
          if (!pilot.empty()) raw->WriteLine(PilotCommand(pilot));
        }
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        process_.reset();
        if (stopping_) return;  // Stop() finishes the shutdown.
      }
      process->Stop();
      Update([&](HelmSnapshot &s) {
        s.state = HostState::Failed;
        s.pairing.reset();
        s.connected_name.clear();
        for (auto &d : s.devices) d.connected = false;
        if (s.message.empty() || ready) s.message = "SKAGER Helm stopped unexpectedly. Restarting...";
      });
    }
    if (++failures > o.max_restarts) {
      Update([](HelmSnapshot &s) {
        s.message = "SKAGER Helm could not start. Its log is in the SKAGER profile (helm folder).";
      });
      std::unique_lock<std::mutex> lock(mutex_);
      wake_.wait(lock, [this] { return stopping_; });
      return;
    }
    std::unique_lock<std::mutex> lock(mutex_);
    if (wake_.wait_for(lock, o.restart_delay, [this] { return stopping_; })) return;
    snapshot_.state = HostState::Starting;
    ++snapshot_.revision;
  }
}

HelmSnapshot HelmHost::Read() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_;
}

bool HelmHost::Running() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_.state == HostState::Ready;
}

bool HelmHost::Send(const std::string &line) {
  // Written outside the lock: a busy pipe must never block the event reader.
  // Writes are serialized inside the process; a write to an exited server fails quietly.
  std::shared_ptr<HelmProcess> process;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (snapshot_.state != HostState::Ready) return false;
    process = process_;
  }
  return process && process->WriteLine(line);
}

void HelmHost::Pair() {
  if (!Send(PairCommand())) Update([](HelmSnapshot &s) { s.message = "SKAGER Helm is not running."; });
}
void HelmHost::CancelPair() { Send(CancelPairCommand()); }
void HelmHost::Revoke(const std::string &id) { Send(RevokeCommand(id)); }
void HelmHost::Disconnect() { Send(DisconnectCommand()); }
void HelmHost::SetName(const std::string &name) { Send(NameCommand(name)); }

void HelmHost::SetPilot(const std::string &status_json) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (status_json == pilot_) return;
    pilot_ = status_json;
  }
  Send(PilotCommand(status_json));
}

bool HelmHost::RecentInput(std::chrono::milliseconds window) const {
  const auto at = input_at_.load();
  if (at == 0) return false;
  const auto then = std::chrono::steady_clock::time_point(std::chrono::steady_clock::duration(at));
  return std::chrono::steady_clock::now() - then <= window;
}

}  // namespace opennav::helm

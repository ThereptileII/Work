#include "observable.h"
#include <wx/app.h>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <future>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

wxDEFINE_EVENT(obsNOTIFY, ObservedEvt);
wxDEFINE_EVENT(EVT_RECEIVED, ObservedEvt);

class FixtureApp : public wxAppConsole {
public:
  bool OnInit() override { return true; }
};
wxIMPLEMENT_APP_NO_MAIN(FixtureApp);

namespace {
using namespace std::chrono_literals;
const auto payload = std::make_shared<const int>(37);

void Check(bool ok, const char* message) {
  if (ok) return;
  std::cerr << "FAIL: " << message << std::endl;
  // A negative control can fail while another thread is deliberately blocked
  // in QueueEvent. Do not destroy its mutex/handler during failure reporting.
  std::_Exit(1);
}

class Receiver : public wxEvtHandler {
public:
  Receiver() {
    Bind(EVT_RECEIVED, [this](ObservedEvt& event) {
      Check(event.GetSharedPtr() == payload, "owned payload survives queue delivery");
      ++delivered;
    });
  }
  void QueueEvent(wxEvent* event) override {
    ++queued;
    wxEvtHandler::QueueEvent(event);
  }
  std::atomic<unsigned> queued{0};
  unsigned delivered = 0;
};

class BlockingReceiver : public Receiver {
public:
  void QueueEvent(wxEvent* event) override {
    {
      std::unique_lock<std::mutex> lock(mutex);
      entered = true;
      condition.notify_all();
      Check(condition.wait_for(lock, 3s, [this] { return released; }),
            "bounded queue barrier released");
    }
    Receiver::QueueEvent(event);
    returned = true;
  }
  void WaitForEnqueue() {
    std::unique_lock<std::mutex> lock(mutex);
    Check(condition.wait_for(lock, 3s, [this] { return entered; }),
          "producer entered the actual wxQueueEvent dispatch");
  }
  void Release() {
    std::lock_guard<std::mutex> lock(mutex);
    released = true;
    condition.notify_all();
  }
  std::atomic<bool> returned{false};
private:
  std::mutex mutex;
  std::condition_variable condition;
  bool entered = false, released = false;
};

void UnregisterDuringEnqueue() {
  const std::string key = "n0183-ALL-unregister";
  auto receiver = std::make_unique<BlockingReceiver>();
  auto listener = std::make_unique<ObservableListener>(key, receiver.get(), EVT_RECEIVED);
  std::thread producer([&] { Observable(key).Notify(payload); });
  receiver->WaitForEnqueue();
  std::promise<void> started, removed;
  auto started_future = started.get_future();
  auto removed_future = removed.get_future();
  std::thread remover([&] {
    started.set_value();
    listener.reset();
    Check(receiver->returned, "unregister waits until queue delivery returns");
    removed.set_value();
  });
  started_future.wait();
  Check(removed_future.wait_for(100ms) == std::future_status::timeout,
        "unregister cannot overtake an in-flight enqueue");
  receiver->Release();
  producer.join();
  remover.join();
  wxAppConsole::GetInstance()->ProcessPendingEvents();
  Check(receiver->queued == 1 && receiver->delivered == 1,
        "one enqueued event remains valid after unregistration");
  Observable(key).Notify(payload);
  Check(receiver->queued == 1, "removed handler receives no further events");
  receiver.reset();
  // Exercise the lifetime boundary with the destination already destroyed.
  for (unsigned i = 0; i < 100; ++i) Observable(key).Notify(payload);
}

void RegisterDuringEnqueue() {
  const std::string key = "n0183-ALL-register";
  BlockingReceiver first;
  Receiver second;
  ObservableListener first_listener(key, &first, EVT_RECEIVED);
  std::unique_ptr<ObservableListener> second_listener;
  std::thread producer([&] { Observable(key).Notify(payload); });
  first.WaitForEnqueue();
  std::promise<void> started, registered;
  auto started_future = started.get_future();
  auto registered_future = registered.get_future();
  std::thread registrar([&] {
    started.set_value();
    second_listener = std::make_unique<ObservableListener>(key, &second, EVT_RECEIVED);
    registered.set_value();
  });
  started_future.wait();
  Check(registered_future.wait_for(100ms) == std::future_status::timeout,
        "registration cannot invalidate an in-flight listener traversal");
  first.Release();
  producer.join();
  registrar.join();
  Check(first.queued == 1 && second.queued == 0,
        "new registration begins after the existing delivery");
  Observable(key).Notify(payload);
  wxAppConsole::GetInstance()->ProcessPendingEvents();
  Check(first.delivered == 2 && second.delivered == 1,
        "all current listeners receive the subsequent event once");
}

void MoveRegistration() {
  Receiver old_receiver, new_receiver;
  {
    ObservableListener destination("move-old", &old_receiver, EVT_RECEIVED);
    ObservableListener source("move-new", &new_receiver, EVT_RECEIVED);
    destination = std::move(source);
    Observable("move-old").Notify(payload);
    Check(old_receiver.queued == 0, "move assignment removes the previous registration");
    Observable("move-new").Notify(payload);
    Check(new_receiver.queued == 1, "move assignment retains one transferred registration");
    auto* same = &destination;
    destination = std::move(*same);
    Observable("move-new").Notify(payload);
    Check(new_receiver.queued == 2, "self move retains its registration");
  }
  Observable("move-old").Notify(payload);
  Observable("move-new").Notify(payload);
  Check(old_receiver.queued == 0 && new_receiver.queued == 2,
        "destroying moved listeners leaves no dangling registration");
  wxAppConsole::GetInstance()->ProcessPendingEvents();
  Check(new_receiver.delivered == 2, "transferred queued payloads remain owned");
}

void ConcurrentChurn() {
  const std::string key = "n0183-ALL-stress";
  Receiver persistent;
  ObservableListener persistent_listener(key, &persistent, EVT_RECEIVED);
  std::atomic<bool> start{false};
  std::vector<std::thread> producers;
  constexpr unsigned producer_count = 4, messages = 2000;
  for (unsigned i = 0; i < producer_count; ++i) {
    producers.emplace_back([&] {
      while (!start.load()) std::this_thread::yield();
      for (unsigned n = 0; n < messages; ++n) Observable(key).Notify(payload);
    });
  }
  start = true;
  for (unsigned n = 0; n < 500; ++n) {
    auto receiver = std::make_unique<Receiver>();
    auto listener = std::make_unique<ObservableListener>(key, receiver.get(), EVT_RECEIVED);
    std::this_thread::yield();
    listener.reset();
    const auto count = receiver->queued.load();
    Observable(key).Notify(payload);
    Check(receiver->queued == count, "retired handler stays retired during producer churn");
    receiver.reset();
  }
  for (auto& producer : producers) producer.join();
  wxAppConsole::GetInstance()->ProcessPendingEvents();
  Check(persistent.queued == producer_count * messages + 500 &&
        persistent.delivered == persistent.queued,
        "concurrent producers deliver each owned event exactly once");
}
}  // namespace

int main(int argc, char** argv) {
  Check(argc == 2, "one bounded scenario is required");
  const std::string scenario = argv[1];
  Check(wxEntryStart(argc, argv), "wx console initialization");
  Check(wxAppConsole::GetInstance() && wxAppConsole::GetInstance()->CallOnInit(), "wx console application initialization");
  if (scenario == "unregister") UnregisterDuringEnqueue();
  else if (scenario == "register") RegisterDuringEnqueue();
  else if (scenario == "move") MoveRegistration();
  else if (scenario == "stress") ConcurrentChurn();
  else Check(false, "known scenario");
  wxAppConsole::GetInstance()->OnExit();
  wxEntryCleanup();
  std::cout << "PASS: observable " << scenario << std::endl;
}

#include "diagnostics/Recorder.h"
#include <random>
#include <sstream>
#include <stdexcept>
#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif
namespace opennav::diagnostics {
namespace fs = std::filesystem;
namespace {
void Publish(const fs::path &file, const std::string &bytes) {
  auto temp = file;
  temp += ".pending";
  if (fs::is_symlink(fs::symlink_status(file)) ||
      fs::is_symlink(fs::symlink_status(temp)))
    throw std::runtime_error("Recording file was replaced by a link");
  {
    std::ofstream out(temp, std::ios::binary | std::ios::trunc);
    out.exceptions(std::ios::failbit | std::ios::badbit);
    out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    out.flush();
    out.close();
  }
#ifdef _WIN32
  if (!MoveFileExW(temp.c_str(), file.c_str(),
                   MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH))
    throw std::runtime_error("Cannot commit recording checkpoint");
#else
  fs::rename(temp, file);
#endif
}
} // namespace
Recorder::Recorder(const fs::path &root, application::Settings assumptions,
                   bool nav, vessel::Time start, RecorderLimits limits)
    : assumptions_(RecordingAssumptions(assumptions)), limits_(limits),
      navigation_(nav), start_(start) {
  application::ValidateSettings(assumptions_);
  if (!limits.frames_per_segment ||
      limits.frames_per_segment > RecordingFrameLimit ||
      !limits.retained_segments || limits.retained_segments > 3 ||
      !limits.checkpoint_frames || limits.checkpoint_frames > 10)
    throw std::invalid_argument("Invalid recording limits");
  // A fixed application-owned root, no path ever supplied by file contents.
  // Resolve the chosen application log root once. Windows may express TEMP
  // using an 8.3 alias or different case: string inequality is not proof of a
  // link escape. All subsequent IO uses the resolved owned directory only.
  const auto resolved = fs::weakly_canonical(fs::absolute(root));
  fs::create_directories(resolved);
  const auto absolute = fs::canonical(resolved);
  std::random_device random;
  bool created = false;
  for (unsigned i = 0; i < 10 && !created; ++i) {
    std::ostringstream name;
    name << "session-" << std::hex << random() << '-' << random();
    status_.directory = absolute / name.str();
    created = fs::create_directory(status_.directory);
  }
  if (!created)
    throw std::runtime_error("Cannot create unique recording session");
  status_.directory = fs::canonical(status_.directory);
  status_.active = true;
  worker_ = std::thread(&Recorder::Work, this);
}
Recorder::~Recorder() { Stop(); }
bool Recorder::Capture(const vessel::VesselState &state, vessel::Time now) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!status_.active || stopping_)
    return false;
  if (have_last_ && now - last_ < std::chrono::seconds(1))
    return false;
  try {
    if (pending_.size() >= 8)
      throw std::runtime_error(
          "Recording disk cannot keep up; capture stopped");
    pending_.push_back(CaptureFrame(state, now, start_, navigation_));
    have_last_ = true;
    last_ = now;
    ++status_.captured;
    wake_.notify_one();
    return true;
  } catch (const std::exception &e) {
    status_.error = e.what();
    status_.active = false;
    stopping_ = true;
    wake_.notify_one();
    return false;
  }
}
void Recorder::Stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
    status_.active = false;
    wake_.notify_one();
  }
  if (worker_.joinable())
    worker_.join();
}
RecorderStatus Recorder::Status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return status_;
}
void Recorder::Work() {
  try {
    Recording current;
    current.navigation_included = navigation_;
    current.assumptions = assumptions_;
    std::size_t index = 0, estimated = 0, committed = 0;
    std::deque<fs::path> retained;
    auto path = [&] {
      return status_.directory / ("segment-" + std::to_string(index) + ".onxr");
    };
    auto checkpoint = [&] {
      if (current.frames.empty())
        return;
      if (fs::weakly_canonical(status_.directory) != status_.directory)
        throw std::runtime_error(
            "Recording directory moved or linked during capture");
      const auto bytes = EncodeRecording(current);
      Publish(path(), bytes);
      if (retained.empty() || retained.back() != path())
        retained.push_back(path());
      // Delete only files created by this instance, never enumerate/delete
      // unrelated recordings, chart files, profiles or previous sessions.
      while (retained.size() > limits_.retained_segments) {
        fs::remove(retained.front());
        retained.pop_front();
      }
      std::lock_guard<std::mutex> lock(mutex_);
      status_.published = committed + current.frames.size();
      status_.segments = retained.size();
    };
    for (;;) {
      RecordedFrame frame;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait(lock, [&] { return stopping_ || !pending_.empty(); });
        if (pending_.empty()) {
          break;
        }
        frame = std::move(pending_.front());
        pending_.pop_front();
      }
      Recording single;
      single.navigation_included = navigation_;
      single.assumptions = assumptions_;
      single.frames.push_back(frame);
      const auto size = EncodeRecording(single).size();
      // Conservative bound includes the configuration/header on every frame.
      // Thus checkpoint serialization cannot exceed the configured segment cap.
      if (!current.frames.empty() &&
          (estimated + size > RecordingByteLimit ||
           current.frames.size() >= limits_.frames_per_segment)) {
        checkpoint();
        committed += current.frames.size();
        current.frames.clear();
        estimated = 0;
        ++index;
      }
      estimated += size;
      current.frames.push_back(std::move(frame));
      if (current.frames.size() % limits_.checkpoint_frames == 0)
        checkpoint();
    }
    checkpoint();
  } catch (const std::exception &e) {
    std::lock_guard<std::mutex> lock(mutex_);
    status_.error = e.what();
    status_.active = false;
    stopping_ = true;
    pending_.clear();
  }
}
Recording LoadRecording(const fs::path &file) {
  if (!fs::is_regular_file(file) || fs::file_size(file) > RecordingByteLimit)
    throw std::invalid_argument("Not a bounded OpenNav recording file");
  std::ifstream in(file, std::ios::binary);
  if (!in)
    throw std::runtime_error("Cannot open recording");
  // Check the read as well as the preliminary size; the file might grow/change.
  std::string bytes(RecordingByteLimit + 1, '\0');
  in.read(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  if (in.bad() || static_cast<std::size_t>(in.gcount()) > RecordingByteLimit)
    throw std::invalid_argument("Unreadable or oversized recording");
  bytes.resize(static_cast<std::size_t>(in.gcount()));
  return DecodeRecording(bytes);
}
} // namespace opennav::diagnostics

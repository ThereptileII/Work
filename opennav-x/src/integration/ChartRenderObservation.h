#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

namespace opennav::integration {
// Application-thread-only, per-chart diagnostic data. No chart/object address,
// attribute storage or renderer callback escapes the owning chart. An observation
// means a raster paint was submitted, not that GPU output was read back.
class ChartRenderObservation {
 public:
  static constexpr unsigned max_objects = 256, max_symbols = 4;
  struct View {
    std::array<double, 8> numbers{};
    std::array<int, 9> integers{};
    long state = 0;
    bool operator==(const View& other) const {
      return numbers == other.numbers && integers == other.integers && state == other.state;
    }
    template<class VP>
    static View Capture(const VP& vp, long state, double tilt = 0,
                        double ref_scale = 0, bool fullscreen = false,
                        bool mercator_override = false) {
      return {{{vp.clat, vp.clon, vp.view_scale_ppm, vp.skew, vp.rotation,
                 double(vp.chart_scale), tilt, ref_scale}},
              {{vp.pix_width, vp.pix_height, vp.m_projection_type, int(vp.b_quilt),
                vp.rv_rect.x, vp.rv_rect.y, vp.rv_rect.width, vp.rv_rect.height,
                int(fullscreen) | (int(mercator_override) << 1)}}, state};
    }
    bool Valid() const {
      for (auto value : numbers) if (!std::isfinite(value)) return false;
      return integers[0] > 0 && integers[1] > 0 && numbers[2] > 0;
    }
  };
  uint64_t Begin(const View& view, bool enabled) {
    const bool nested = active_;
    view_ = view;
    count_ = query_count_ = 0;
    complete_ = query_valid_ = false;
    refused_ = nested || !enabled || !view.Valid() || generation_ == UINT64_MAX;
    if (generation_ != UINT64_MAX) ++generation_;
    active_ = true;
    return generation_;
  }
  void Complete(long state, uint64_t generation) {
    complete_ = active_ && generation == generation_ && !refused_ && state == view_.state;
  }
  void End(uint64_t generation) { if (generation == generation_) active_ = false; }
  template<class Object> void Unavailable(const Object* object) {
    if (!active_ || refused_ || !Relevant(object)) return;
    auto* record = FindOrAdd(object);
    if (record) record->incomplete = true;
  }
  template<class Object>
  void Record(const Object* object, int rcid, int style, const char* original,
              const char* painted, bool ca) {
    if (!active_ || refused_ || !Relevant(object)) return;
    if (!Symbol(original) || !Symbol(painted)) { Refuse(); return; }
    auto* record = FindOrAdd(object);
    if (!record) return;
    Paint paint{};
    paint.rcid = rcid; paint.style = style; paint.ca = ca;
    std::memcpy(paint.original.data(), original, 8);
    std::memcpy(paint.painted.data(), painted, 8);
    for (unsigned i = 0; i < record->count; ++i) if (record->paints[i] == paint) return;
    if (record->count == max_symbols) { Refuse(); return; }
    record->paints[record->count++] = paint;
  }
  void BeginQuery(const View& view, bool enabled) {
    query_count_ = 0;
    query_valid_ = enabled && !active_ && complete_ && !refused_ && view.Valid() && view == view_;
  }
  // Bind only where the real object query selects an object. Private adapters
  // bind their newly allocated PI clone here; copied values, never S52_Context,
  // travel into CreateObjDescriptions. A later query/render revokes this map.
  template<class Object>
  void Select(const void* query_object, const Object* actual) {
    if (!query_valid_ || !Relevant(actual) || !query_object) return;
    if (query_count_ == max_objects) { query_valid_ = false; query_count_ = 0; return; }
    for (unsigned i = 0; i < count_; ++i) {
      const auto& record = records_[i];
      if (record.object == actual && record.index == actual->Index &&
          !std::memcmp(record.feature.data(), actual->FeatureName, 6)) {
        queries_[query_count_++] = {query_object, record};
        return;
      }
    }
  }
  template<class Object>
  std::string Describe(const Object* query_object) const {
    if (!Relevant(query_object)) return {};
    std::string text = "<hr><b>Renderer observation (";
    text.append(query_object->FeatureName, 6);
    text += " #" + std::to_string(query_object->Index) + ")</b><br>";
    const RecordValue* record = nullptr;
    if (query_valid_) for (unsigned i = 0; i < query_count_; ++i)
      if (queries_[i].query_object == query_object &&
          queries_[i].value.index == query_object->Index &&
          !std::memcmp(queries_[i].value.feature.data(), query_object->FeatureName, 6))
        record = &queries_[i].value;
    if (!record || record->incomplete || !record->count)
      return text + "Raster paint observation unavailable for the matching render.<br>";
    text += "Completed render generation: " + std::to_string(generation_) + "<br>";
    for (unsigned i = 0; i < record->count; ++i) {
      const auto& paint = record->paints[i];
      text += paint.ca ? "CA light point: " : "Resolved SY: ";
      text.append(paint.original.data(), 8);
      text += " / raster paint submitted: ";
      text.append(paint.painted.data(), 8);
      text += " / LUP RCID: " + std::to_string(paint.rcid) +
              " / point table: " + std::to_string(paint.style) + "<br>";
    }
    return text;
  }
 private:
  template<class Object> static bool Relevant(const Object* object) {
    return object && (!std::memcmp(object->FeatureName, "LIGHTS", 6) ||
                      !std::memcmp(object->FeatureName, "LNDMRK", 6));
  }
  static bool Symbol(const char* name) {
    if (!name) return false;
    for (unsigned i = 0; i < 8; ++i)
      if (!((name[i] >= 'A' && name[i] <= 'Z') ||
            (name[i] >= '0' && name[i] <= '9') || name[i] == '_')) return false;
    return true;
  }
  struct Paint {
    std::array<char, 8> original{}, painted{};
    int rcid = 0, style = 0;
    bool ca = false;
    bool operator==(const Paint& other) const {
      return original == other.original && painted == other.painted &&
             rcid == other.rcid && style == other.style && ca == other.ca;
    }
  };
  struct RecordValue {
    const void* object = nullptr;
    std::array<char, 6> feature{};
    int index = 0;
    unsigned count = 0;
    bool incomplete = false;
    std::array<Paint, max_symbols> paints{};
  };
  struct Query { const void* query_object = nullptr; RecordValue value{}; };
  template<class Object> RecordValue* FindOrAdd(const Object* object) {
    for (unsigned i = 0; i < count_; ++i)
      if (records_[i].object == object) return &records_[i];
    if (count_ == max_objects) { Refuse(); return nullptr; }
    auto* record = &records_[count_++];
    *record = {};
    record->object = object;
    record->index = object->Index;
    std::memcpy(record->feature.data(), object->FeatureName, 6);
    return record;
  }
  void Refuse() { refused_ = true; complete_ = query_valid_ = false; count_ = query_count_ = 0; }
  View view_{};
  uint64_t generation_ = 0;
  bool refused_ = true, complete_ = false, query_valid_ = false, active_ = false;
  unsigned count_ = 0, query_count_ = 0;
  std::array<RecordValue, max_objects> records_{};
  std::array<Query, max_objects> queries_{};
};

template<class Library> class ChartRenderObservationScope {
 public:
  ChartRenderObservationScope(ChartRenderObservation& observation, Library* library,
      const ChartRenderObservation::View& view, bool enabled)
      : observation_(observation), library_(library) {
    generation_ = observation_.Begin(view, enabled);
    if (library_) previous_ = library_->SetPresentationObservation(enabled ? &observation_ : nullptr);
  }
  ~ChartRenderObservationScope() {
    if (library_) library_->SetPresentationObservation(previous_);
    observation_.End(generation_);
  }
  void Complete() { if (library_) observation_.Complete(library_->GetStateHash(), generation_); }
  ChartRenderObservationScope(const ChartRenderObservationScope&) = delete;
  ChartRenderObservationScope& operator=(const ChartRenderObservationScope&) = delete;
 private:
  ChartRenderObservation& observation_;
  Library* library_;
  ChartRenderObservation* previous_ = nullptr;
  uint64_t generation_ = 0;
};
}  // namespace opennav::integration

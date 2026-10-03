#pragma once
#include <stdint.h>

/* Private SKAGER adapter ABI. Not an OpenCPN plugin API extension. The host
 * resolves these exports only from an independently hash-verified adapter,
 * before its create_pi factory. All strings are copied, bounded UTF-8; no
 * renderer, chart, wx object, callback or allocator crosses this boundary. */
#define SKAGER_CHART_BINDING_VERSION 1u
#define SKAGER_CHART_BINDING_PATH_CAPACITY 4096u
#define SKAGER_CHART_BINDING_EXPORT "skager_bind_chart_presentation_v1"
#define SKAGER_CHART_STATUS_EXPORT "skager_chart_presentation_status_v1"

#ifdef _WIN32
#define SKAGER_CHART_CALL __cdecl
#else
#define SKAGER_CHART_CALL
#endif

typedef struct SkagerChartBindingV1 {
  uint32_t structBytes;
  uint32_t version;
  /* Absolute directory containing the exact compiled presentation resources.
   * Must contain one terminator; every byte after it and reserved[] is zero. */
  char resourceDirectory[SKAGER_CHART_BINDING_PATH_CAPACITY];
  uint32_t reserved[8];
} SkagerChartBindingV1;

enum SkagerChartPresentationStateV1 {
  SKAGER_CHART_UNBOUND = 0,
  SKAGER_CHART_BOUND_PENDING_INITIALIZATION = 1,
  SKAGER_CHART_SELECTED = 2,
  SKAGER_CHART_STANDARD_FALLBACK = 3
};
enum SkagerChartPresentationReasonV1 {
  SKAGER_CHART_REASON_NONE = 0,
  SKAGER_CHART_REASON_UNBOUND = 1,
  SKAGER_CHART_REASON_RESOURCE_VERIFICATION = 2,
  SKAGER_CHART_REASON_RENDERER_INITIALIZATION = 3
};
typedef struct SkagerChartPresentationStatusV1 {
  uint32_t structBytes;
  uint32_t version;
  uint32_t state;
  uint32_t reason;
  uint32_t reserved[8];
} SkagerChartPresentationStatusV1;

/* Return 1 only when the complete request was accepted, otherwise 0.
 * Binding is one-shot and must fail after renderer initialization. Rejection
 * does not make existing state fresh or successful. The host unloads a module
 * which rejects the bind before invoking its plugin factory.
 * Status is a copied observation; the query never initializes/refreshes the
 * renderer. Caller provides exact structBytes/version and zero other bytes. */
typedef int32_t (SKAGER_CHART_CALL *SkagerBindChartPresentationV1)(
    const SkagerChartBindingV1*);
typedef int32_t (SKAGER_CHART_CALL *SkagerGetChartPresentationStatusV1)(
    SkagerChartPresentationStatusV1*);

/* Separate v1 observation ABI: existing binding/status layout and reserved
 * fields remain unchanged. Caller provides exact size/version and zero other
 * fields. Main-thread only; rejected queries leave caller bytes unchanged.
 * Zero is unavailable, never a guessed chart table. No query initializes the
 * renderer or changes its policy. */
#define SKAGER_CHART_POINT_STYLE_VERSION 1u
#define SKAGER_CHART_POINT_STYLE_EXPORT "skager_chart_point_style_v1"
#define SKAGER_CHART_POINT_STYLE_SIMPLIFIED 76u
#define SKAGER_CHART_POINT_STYLE_PAPER 82u
typedef struct SkagerChartPointStyleV1 {
  uint32_t structBytes;
  uint32_t version;
  uint32_t available;
  uint32_t effectivePointStyle;
  uint32_t reserved[8];
} SkagerChartPointStyleV1;
typedef int32_t (SKAGER_CHART_CALL *SkagerGetChartPointStyleV1)(
    SkagerChartPointStyleV1*);

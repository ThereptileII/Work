#pragma once

class wxWindow;
// Private, additive bridge for the bundled Dashboard only. Existing OpenCPN
// plugin ABI/vtables stay unchanged; third-party plugins do not call this API.
#ifdef _WIN32
#ifdef OPENNAV_DASHBOARD_PLUGIN
#define OPENNAV_DASHBOARD_API __declspec(dllimport)
#else
#define OPENNAV_DASHBOARD_API __declspec(dllexport)
#endif
#else
#define OPENNAV_DASHBOARD_API __attribute__((visibility("default")))
#endif
extern "C" OPENNAV_DASHBOARD_API void OpenNavDashboardWindow(wxWindow*, bool add);
extern "C" OPENNAV_DASHBOARD_API void OpenNavDashboardLayout(bool begin);
class OpenNavDashboardLayoutScope {
 public:
  OpenNavDashboardLayoutScope() { OpenNavDashboardLayout(true); }
  ~OpenNavDashboardLayoutScope() { OpenNavDashboardLayout(false); }
  OpenNavDashboardLayoutScope(const OpenNavDashboardLayoutScope&) = delete;
  OpenNavDashboardLayoutScope& operator=(const OpenNavDashboardLayoutScope&) = delete;
};

#include <iostream>

#include <wx/curl/http.h>
#include <wx/init.h>
#include <wx/mstream.h>

struct CurlSession {
  CurlSession() { wxCurlBase::Init(); }
  ~CurlSession() { wxCurlBase::Shutdown(); }
};

class ProbeHTTP : public wxCurlHTTP {
 public:
  bool ConfigureOnly(const wxString& url) {
    SetCurlHandleToDefaults(url);
    return SetOpt(CURLOPT_NOSIGNAL, 1L);
  }

  bool BadOptionBlocksPerform(const wxString& url) {
    SetCurlHandleToDefaults(url);
    const bool rejected = !SetOpt(static_cast<CURLoption>(999999),
                                  static_cast<curl_off_t>(1));
    return rejected && !Perform();
  }
};

int main(int argc, char** argv) {
  if (argc != 2 && argc != 3) return 2;
  wxInitializer initializer;
  if (!initializer.IsOk()) return 3;
  CurlSession curl_session;
  const wxString url = wxString::FromUTF8(argv[argc - 1]);
  if (argc == 3) {
    if (std::string(argv[1]) != "--configure") return 2;
    ProbeHTTP configured;
    const bool ok = configured.ConfigureOnly(url);
    std::cout << "configure_ok=" << (ok ? "true" : "false") << "\n";
    return ok ? 0 : 1;
  }
  ProbeHTTP guard;
  const bool bad_option_blocked = guard.BadOptionBlocksPerform(url);
  wxCurlHTTP http;
  wxMemoryOutputStream output;
  const bool get_ok = http.Get(output, url);
  const std::string get_error = http.GetErrorString();
  const std::string get_detail = http.GetDetailedErrorString();
  const bool head_ok = http.Head(url);
  std::cout << "bad_option_blocked="
            << (bad_option_blocked ? "true" : "false") << "\n"
            << "get_ok=" << (get_ok ? "true" : "false") << "\n"
            << "get_bytes=" << output.GetSize() << "\n"
            << "get_error=" << get_error << "\n"
            << "get_detail=" << get_detail << "\n"
            << "head_ok=" << (head_ok ? "true" : "false") << "\n"
            << "head_error=" << http.GetErrorString() << "\n"
            << "head_detail=" << http.GetDetailedErrorString() << "\n";
  return bad_option_blocked && get_ok && head_ok ? 0 : 1;
}

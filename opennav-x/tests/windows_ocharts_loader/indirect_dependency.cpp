// Locally built test-only intermediate DLL; no OpenSSL implementation.
#ifdef INDIRECT_LEGACY
extern "C" __declspec(dllimport) int __cdecl legacy_tls_fixture();
#endif
extern "C" __declspec(dllexport) int __cdecl indirect_fixture() {
#ifdef INDIRECT_LEGACY
  return legacy_tls_fixture();
#else
  return 209;
#endif
}

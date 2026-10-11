// Test-only stand-in for a denied basename; contains no TLS implementation.
extern "C" __declspec(dllexport) int __cdecl legacy_tls_fixture() { return 209; }

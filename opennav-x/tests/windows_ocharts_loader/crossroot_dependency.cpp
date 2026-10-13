// Harmless plugin-directory intermediate importing an application-only helper.
extern "C" __declspec(dllimport) int __cdecl external_child_fixture();
extern "C" __declspec(dllexport) int __cdecl indirect_fixture() {
  return external_child_fixture();
}

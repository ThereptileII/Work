#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <string>

struct CURL {};
enum CURLcode {
  CURLE_OK = 0,
  CURLE_FAILED_INIT = 2,
  CURLE_OUT_OF_MEMORY = 27,
  CURLE_BAD_FUNCTION_ARGUMENT = 43
};
bool fail_curl_init = false;
CURL curl_handle;
CURL* curl_easy_init() { return fail_curl_init ? nullptr : &curl_handle; }

#include "peer_buffer_under_test.inc"

namespace {
bool fail_allocate = false;
bool fail_reallocate = false;

void* TestAllocate(size_t bytes) {
  return fail_allocate ? nullptr : std::malloc(bytes);
}

void* TestReallocate(void* pointer, size_t bytes) {
  return fail_reallocate ? nullptr : std::realloc(pointer, bytes);
}

void TestRelease(void* pointer) { std::free(pointer); }

void Require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}
}  // namespace

int main() {
  {
    MemoryStruct memory(TestAllocate, TestReallocate, TestRelease);
    Require(memory.memory && memory.size == 0 && memory.memory[0] == '\0',
            "empty buffer is not a valid NUL-terminated string");
  }
  {
    fail_allocate = true;
    MemoryStruct memory(TestAllocate, TestReallocate, TestRelease);
    fail_allocate = false;
    const char byte = 'x';
    Require(!memory.memory &&
                WriteMemoryCallback((void*)&byte, 1, 1, &memory) == 0,
            "initial allocation failure was not retained and rejected");
    CURLcode result = CURLE_OK;
    Require(InitPeerRequest(&memory, result) == nullptr &&
                result == CURLE_OUT_OF_MEMORY,
            "empty response accepted failed initial allocation");
  }
  {
    CURLcode result = CURLE_OK;
    Require(InitPeerRequest(nullptr, result) == nullptr &&
                result == CURLE_BAD_FUNCTION_ARGUMENT,
            "null response state was accepted");
    MemoryStruct memory(TestAllocate, TestReallocate, TestRelease);
    fail_curl_init = true;
    Require(InitPeerRequest(&memory, result) == nullptr &&
                result == CURLE_FAILED_INIT,
            "curl initialization failure was accepted");
    fail_curl_init = false;
    Require(InitPeerRequest(&memory, result) == &curl_handle &&
                result == CURLE_OK,
            "valid initialized response was rejected");
  }
  {
    MemoryStruct memory(TestAllocate, TestReallocate, TestRelease);
    const char byte = 'x';
    Require(WriteMemoryCallback((void*)&byte, 1, 1, nullptr) == 0,
            "null state was accepted");
    Require(WriteMemoryCallback(nullptr, 1, 1, &memory) == 0,
            "null input was accepted");
    Require(WriteMemoryCallback((void*)&byte,
                                std::numeric_limits<size_t>::max(), 2,
                                &memory) == 0,
            "multiplication overflow was accepted");
    Require(memory.size == 0 && memory.memory[0] == '\0',
            "invalid input changed the buffer");
  }
  {
    MemoryStruct memory(TestAllocate, TestReallocate, TestRelease);
    const std::string first = "{\"result\":";
    const std::string second = "0,\"version\":\"5.12.4\"}";
    Require(WriteMemoryCallback((void*)first.data(), 1, first.size(), &memory) ==
                first.size() &&
                WriteMemoryCallback((void*)second.data(), 1, second.size(),
                                    &memory) == second.size(),
            "normal split chunks were rejected");
    Require(memory.size == first.size() + second.size() &&
                std::string(memory.memory) == first + second,
            "normal split chunks were not combined and terminated");
  }
  {
    MemoryStruct memory(TestAllocate, TestReallocate, TestRelease);
    const std::string retained = "retained";
    Require(WriteMemoryCallback((void*)retained.data(), 1, retained.size(),
                                &memory) == retained.size(),
            "test setup failed");
    char* original = memory.memory;
    fail_reallocate = true;
    const char byte = 'x';
    Require(WriteMemoryCallback((void*)&byte, 1, 1, &memory) == 0,
            "reallocation failure was accepted");
    fail_reallocate = false;
    Require(memory.memory == original && memory.size == retained.size() &&
                std::string(memory.memory) == retained,
            "reallocation failure discarded the retained buffer");
    Require(WriteMemoryCallback((void*)&byte, 1,
                                kMaxPeerResponseBytes - retained.size() + 1,
                                &memory) == 0,
            "response cap overflow was accepted");
    Require(memory.memory == original && memory.size == retained.size() &&
                std::string(memory.memory) == retained,
            "cap rejection changed the retained buffer");
    memory.size = std::numeric_limits<size_t>::max();
    Require(WriteMemoryCallback((void*)&byte, 1, 1, &memory) == 0,
            "addition overflow state was accepted");
    memory.size = retained.size();
  }
  {
    MemoryStruct memory(TestAllocate, TestReallocate, TestRelease);
    std::string exact(kMaxPeerResponseBytes, 'a');
    Require(WriteMemoryCallback((void*)exact.data(), 1, exact.size(), &memory) ==
                exact.size() &&
                memory.size == kMaxPeerResponseBytes &&
                memory.memory[kMaxPeerResponseBytes] == '\0',
            "exact response cap was not accepted and terminated");
    const char byte = 'b';
    Require(WriteMemoryCallback((void*)&byte, 1, 1, &memory) == 0 &&
                memory.size == kMaxPeerResponseBytes,
            "byte beyond response cap was accepted");
  }
  std::cout << "peer response buffer tests passed\n";
}

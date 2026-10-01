#include <fstream>
#include <iostream>
#include <string>

#include <curl/curl.h>

#include "model/downloader.h"

class RefusingBuffer : public std::streambuf {
 protected:
  std::streamsize xsputn(const char*, std::streamsize) override { return 0; }
};

class ProbeDownloader : public Downloader {
 public:
  using Downloader::Downloader;
  long filesize() { return get_filesize(); }
};

int main(int argc, char** argv) {
  if (argc != 3 && argc != 4) return 2;
  if (curl_global_init(CURL_GLOBAL_DEFAULT) != CURLE_OK) return 3;
  std::string path(argv[2]);
  ProbeDownloader downloader(argv[1]);
  bool ok;
  if (argc == 4 && std::string(argv[3]) == "--reject-stream") {
    RefusingBuffer buffer;
    std::ostream refusing(&buffer);
    refusing.exceptions(std::ios::failbit | std::ios::badbit);
    ok = downloader.download(&refusing);
  } else {
    ok = downloader.download(path);
  }
  const int download_error = downloader.last_errorcode();
  const std::string download_message = downloader.last_error();
  const long size = downloader.filesize();
  std::cout << "download_ok=" << (ok ? "true" : "false") << "\n"
            << "download_error=" << download_error << "\n"
            << "download_message=" << download_message << "\n"
            << "head_size=" << size << "\n"
            << "head_error=" << downloader.last_errorcode() << "\n"
            << "head_message=" << downloader.last_error() << "\n";
  curl_global_cleanup();
  return ok ? 0 : 1;
}

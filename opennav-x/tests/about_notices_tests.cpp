#include "application/AboutNotices.h"
#include <cassert>
#include <chrono>
#include <iostream>

int main() {
  namespace fs=std::filesystem;
  using namespace opennav::application;
  const auto root=fs::temp_directory_path()/
      ("skager-about-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  const auto executable=root/fs::u8path("Båt installation")/"app/opencpn.exe";
  const auto docs=AboutDocsDirectory(executable);
  fs::create_directories(docs/"licenses/OpenCPN");
  assert(docs==executable.parent_path().parent_path()/"docs");
  assert(AboutDocsDirectory("app/opencpn.exe").empty());
  assert(!ReadAboutNotice(executable,AboutNotice::Sources).available);
  // The same package layout is retained by the installer and portable ZIP.
  for(auto notice:{AboutNotice::Sources,AboutNotice::Components,
                   AboutNotice::SkagerLicense,AboutNotice::OpenCpnLicense}) {
    const auto file=docs/AboutNoticeFile(notice);
    {std::ofstream out(file,std::ios::binary);out<<"Packaged UTF-8 notice: båt\n";}
    const auto read=ReadAboutNotice(executable,notice);
    assert(read.available && read.text=="Packaged UTF-8 notice: båt\n");
    fs::remove(file);
    const auto missing=ReadAboutNotice(executable,notice);
    assert(!missing.available && missing.text.find(AboutNoticeFile(notice))!=std::string::npos);
  }
  const auto file=docs/AboutNoticeFile(AboutNotice::Sources);
  {std::ofstream out(file,std::ios::binary);out<<std::string(2*1024*1024+1,'x');}
  assert(!ReadAboutNotice(executable,AboutNotice::Sources).available);
  {std::ofstream out(file,std::ios::binary);out<<std::string("bad\0text",8);}
  assert(!ReadAboutNotice(executable,AboutNotice::Sources).available);
  {std::ofstream out(file);}
  assert(!ReadAboutNotice(executable,AboutNotice::Sources).available);
  fs::remove(file);fs::create_directory(file);
  assert(!ReadAboutNotice(executable,AboutNotice::Sources).available);
  // No CWD fallback when this installation is missing its own notices.
  const auto cwd=fs::current_path();fs::current_path(docs);
  assert(!ReadAboutNotice("app/opencpn.exe",AboutNotice::Sources).available);
  fs::current_path(cwd);
  fs::remove_all(root);
  std::cout<<"About notice package-path, UTF-8, missing-file and bounded-read checks passed\n";
}

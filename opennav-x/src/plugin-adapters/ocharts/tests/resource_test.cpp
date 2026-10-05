#include "plugin-adapters/ocharts/ResourceVerification.h"
#include "plugin-adapters/ocharts/OwnedPresentationValidation.h"
#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>
namespace fs=std::filesystem;
using namespace skager::ocharts;
int main(int argc,char**argv) {
  if(argc!=3) return 2; // generated resource directory; private scratch directory
  const fs::path source(argv[1]),scratch(argv[2]);
  if(fs::exists(scratch)) return 2;
  fs::create_directories(scratch);
  for(const auto& resource:opennav::chart_style::generated::resources)
    fs::copy_file(source/resource.name,scratch/resource.name);
  wxInitAllImageHandlers();
  const auto dir=wxString::FromUTF8(scratch.string());
  assert(VerifyCompiledResources(dir));
  pugi::xml_document doc;
  assert(doc.load_file((scratch/"chartsymbols.xml").string().c_str()));
  assert(ValidateOwnedPresentation(doc,dir));
  // A single changed byte invalidates the compiled closure, independently of XML.
  const auto atlas=scratch/"rastersymbols-dusk.png";
  {std::fstream f(atlas,std::ios::binary|std::ios::in|std::ios::out);char c;f.get(c);f.seekp(0);f.put(c^1);}
  assert(!VerifyCompiledResources(dir));
  assert(!ValidateOwnedPresentation(doc,dir));
  fs::copy_file(source/atlas.filename(),atlas,fs::copy_options::overwrite_existing);
  assert(VerifyCompiledResources(dir));
  fs::rename(atlas,scratch/"missing.png");
  assert(!VerifyCompiledResources(dir));assert(!ValidateOwnedPresentation(doc,dir));
  fs::rename(scratch/"missing.png",atlas);
  // Valid XML with a partial library cannot be accepted as usable.
  doc.child("chartsymbols").remove_child("lookups");
  assert(!ValidateOwnedPresentation(doc,dir));
  assert(doc.load_file((scratch/"chartsymbols.xml").string().c_str()));
  doc.child("chartsymbols").child("color-tables").child("color-table")
      .child("graphics-file").attribute("name").set_value("../outside.png");
  assert(!ValidateOwnedPresentation(doc,dir));
  doc.reset(); assert(!ValidateOwnedPresentation(doc,dir));
  fs::remove_all(scratch);
  std::cout<<"PASS exact resources, changed byte, broken/missing atlas, partial XML and foreign atlas refusal\n";
}

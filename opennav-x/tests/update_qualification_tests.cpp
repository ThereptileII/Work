#include "integration/UpdateQualification.h"
#include <iostream>
#include <stdexcept>
using namespace opennav::integration;
void Require(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
int main() {
  try {
    UpdateQualificationFlag absent;
    absent.Capture({});
    Require(!absent.Required() && !absent.Valid(),"Absent qualification preserves ordinary behavior");
    absent.Capture(std::string("1"));
    Require(!absent.Required(),"Later injection cannot qualify ordinary capture");
    for (const auto& value : {"", "0", "true", "01", "1 ", "1\n", "2"}) {
      UpdateQualificationFlag invalid;
      invalid.Capture(std::string(value));
      invalid.Capture(std::string("1"));
      Require(invalid.Required() && !invalid.Valid(),"Every malformed presence remains restrictive");
    }
    for (const auto& value : {std::optional<std::string>{}, std::optional<std::string>{"1"}}) {
      UpdateQualificationFlag failed_removal;
      failed_removal.Capture(value,false);
      Require(failed_removal.Required() && !failed_removal.Valid(),"Uncertain removal remains restrictive");
    }
    UpdateQualificationFlag valid;
    valid.Capture(std::string("1"));
    valid.Capture(std::string("0"));
    Require(valid.Required() && valid.Valid(),"Exact valid initial flag is immutable");
    const QualifiedConfigObservation original{std::string(16,'1'),std::string(32,'2'),
      std::string(16,'3'),std::string(16,'4'),std::string(32,'5'),std::string(16,'6'),std::string(64,'7')};
    Require(bool(EncodeQualifiedConfigObservation(original)),"Complete original identity encodes");
    for (int field=0;field<7;++field) {
      for(int damage=0;damage<4;++damage) {
        auto copy=original;
        std::string* fields[]={&copy.directory_volume,&copy.directory_id,&copy.directory_created,
          &copy.file_volume,&copy.file_id,&copy.file_created,&copy.ini_sha256};
        auto& selected=*fields[field];
        if(damage==0) selected.pop_back();
        if(damage==1) selected+='0';
        if(damage==2) selected[0]='A';
        if(damage==3) selected[0]='\n';
        Require(!EncodeQualifiedConfigObservation(copy),"Identity widths/case/framing fail closed");
      }
    }
#ifndef _WIN32
    CaptureUpdateQualificationFlag();
    Require(!QualificationOutputSuppressed() && !QualificationProfileRequired() && QualifiedProfileObserved(),
      "Non-Windows ordinary startup preserved");
    auto owner=BeginQualifiedConfigLoad(L"ordinary-path");
    ObserveQualifiedConfigConstructor(L"ordinary-path");
    Require(!owner && CompleteQualifiedConfigLoad(std::move(owner),L"ordinary-path"),
      "Ordinary config path neither pinned nor rewritten");
#endif
    std::cout << "PASS qualification flag, exact CONFIG identity encoding and ordinary startup\n";
  } catch(const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}

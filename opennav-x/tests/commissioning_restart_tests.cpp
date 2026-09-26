#include "platform/windows/CommissioningRestartProtocol.h"
#include <iostream>
#include <stdexcept>

using namespace opennav::platform::commissioning;
namespace {
int checks=0;
void Check(bool value) {if(!value)throw std::runtime_error("Contract "+std::to_string(checks+1)+" failed");++checks;}
std::string H(char c) {return std::string(64,c);}
Request Sample() {
  Request r;
  r.binding=ReadBinding(H('a'),H('b'));r.nonce=H('c');
  r.parent_pid=31;r.parent_created=100;r.helper_pid=32;r.helper_created=200;r.windows_session=1;
  r.executable="C:\\Marine user & boat\\opencpn.exe";r.executable_sha256=H('d');
  r.helper="C:\\Marine user & boat\\opennav-restart.exe";r.helper_sha256=H('e');
  r.working_directory="C:\\Marine user & boat";r.path="C:\\Marine user & boat;C:\\Windows\\System32;C:\\Windows";
  r.arguments={"--legacy"};return r;
}
std::vector<std::string> Fields(const Request& r) {
  return {Protocol,"ALLOW",r.binding.session,r.binding.record_sha256,r.nonce,H('f'),
          "1000000000","1010000000",r.executable,r.executable_sha256,r.helper,r.helper_sha256,
          "C:\\ProgramData\\opencpn\\opencpn.ini",H('1'),r.working_directory,r.path,H('2')};
}
}
int main() {
  Check(ReadBinding({},{}).state==GuardState::Unarmed);
  for(const auto& x: {std::string{},H('A'),std::string("missing"),H('a')+"0"}) {
    Check(ReadBinding(x,{}).state==GuardState::Invalid);
    Check(ReadBinding({},x).state==GuardState::Invalid);
    Check(ReadBinding(x,H('b')).state==GuardState::Invalid);
  }
  auto request=Sample();const auto payload=EncodeRequest(request);Check(payload.has_value());
  Check(payload->find("Marine user & boat")!=std::string::npos);
  Check(payload->find("\"arguments\":[\"--legacy\"]")!=std::string::npos);
  for(const auto& mode:{"--xnav","--legacy","--safe-mode"}) {
    auto r=request;r.arguments={mode};Check(EncodeRequest(r).has_value());
  }
  for(const auto& mode:{"--xnav-demo","--safe-mode --xnav","","--configdir"}) {
    auto r=request;r.arguments={mode};Check(!EncodeRequest(r));
  }
  auto r=request;r.arguments.push_back("--xnav");Check(!EncodeRequest(r));
  r=request;r.binding.state=GuardState::Unarmed;Check(!EncodeRequest(r));
  r=request;r.parent_pid=0;Check(!EncodeRequest(r));
  r=request;r.path="C:\\Windows\nInjected";Check(!EncodeRequest(r));
  r=request;r.helper_sha256="unknown";Check(!EncodeRequest(r));
  const auto fields=Fields(request);const auto encoded=EncodeFields(fields);Check(encoded.has_value());
  const auto decoded=DecodeFields(*encoded);Check(decoded && *decoded==fields);
  const auto permit=DecodePermit(*encoded);Check(permit.has_value());
  Check(ValidatePermit(request,*permit,H('f'),1000000000));
  Check(ValidatePermit(request,*permit,H('f'),1009999999));
  Check(!ValidatePermit(request,*permit,H('f'),999999999));
  Check(!ValidatePermit(request,*permit,H('f'),1010000000));
  Check(!ValidatePermit(request,*permit,H('0'),1000000001));
  for(const auto index:{2,3,4,5,8,9,10,11,13,14,16}) {
    auto changed=fields;changed[index]="tampered";
    const auto p=DecodePermit(*EncodeFields(changed));Check(!p || !ValidatePermit(request,*p,H('f'),1000000001));
  }
  for(const auto time:{"01000000000","-1","18446744073709551616","NaN"}) {
    auto changed=fields;changed[6]=time;Check(!DecodePermit(*EncodeFields(changed)));
  }
  auto changed=fields;changed[7]="1100000001";
  Check(!ValidatePermit(request,*DecodePermit(*EncodeFields(changed)),H('f'),1000000001));
  Check(!DecodePermit(*EncodeFields({Protocol,"DENY"})));
  changed=fields;changed.push_back("ignored");Check(!DecodePermit(*EncodeFields(changed)));
  Check(!DecodeFields(*encoded+"ignored"));
  for(std::size_t n=0;n<encoded->size();++n)Check(!DecodeFields(encoded->substr(0,n)));
  Check(!DecodeFields(std::string(MaximumFrame+1,'x')));
  Check(!EncodeFields({""}));
  Check(!EncodeFields({std::string("embedded\0null",13)}));
  Check(!EncodeFields({"\xc0\xaf"}));
  Check(!EncodeFields({"\xed\xa0\x80"}));
  Check(!EncodeFields({"\xf4\x90\x80\x80"}));
  Check(!EncodeFields({"\xe2\x82"}));
  const std::string text="Ark\xc3\xb6sund / \xe2\x9b\xb5";
  const auto unicode=EncodeFields({text});Check(unicode && (*DecodeFields(*unicode))[0]==text);
  Check(EncodeReceipt(request,*permit,H('f'),50,500,0).find("\"status\":\"started\"")!=std::string::npos);
  Check(EncodeReceipt(request,*permit,H('f'),0,0,5).find("\"status\":\"failed\"")!=std::string::npos);
  std::cout<<checks<<" commissioning restart protocol checks passed\n";
}

// User-mode dynamic WFP proof. No persistent rules, permits, driver or product code.
#include "guard.h"
#include <bcrypt.h>
#include <fwpmu.h>
#include <rpc.h>
#include <array>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <vector>

namespace {
void Check(DWORD result,const char *operation) {
  if(result!=ERROR_SUCCESS) {
    std::cerr<<operation<<" status="<<result<<'\n';
    throw std::runtime_error(operation);
  }
}
struct Handle {
  HANDLE value=nullptr;
  explicit Handle(HANDLE h=nullptr):value(h) {}
  ~Handle(){if(value && value!=INVALID_HANDLE_VALUE)CloseHandle(value);}
  Handle(const Handle &)=delete;
};
struct Engine {
  HANDLE value=nullptr;
  ~Engine(){if(value)FwpmEngineClose0(value);}
};
GUID Unique() {GUID value{};Check(UuidCreate(&value),"create object GUID");return value;}
std::string Text(const GUID &guid) {
  RPC_CSTR text=nullptr;Check(UuidToStringA(&guid,&text),"format object GUID");
  std::string result(reinterpret_cast<char *>(text));RpcStringFreeA(&text);return result;
}
GUID Parse(wchar_t *text) {GUID guid{};Check(UuidFromStringW(reinterpret_cast<RPC_WSTR>(text),&guid),"parse audit GUID");return guid;}
bool Equal(const GUID &a,const GUID &b){return InlineIsEqualGUID(a,b)!=0;}
std::string Hash(HANDLE file) {
  BCRYPT_ALG_HANDLE algorithm=nullptr;BCRYPT_HASH_HANDLE hash=nullptr;
  outage::Require(BCryptOpenAlgorithmProvider(&algorithm,BCRYPT_SHA256_ALGORITHM,nullptr,0)>=0,"open SHA256");
  std::vector<unsigned char> object;std::array<unsigned char,32> digest{};
  try {
    DWORD size=0,received=0;
    outage::Require(BCryptGetProperty(algorithm,BCRYPT_OBJECT_LENGTH,reinterpret_cast<PUCHAR>(&size),sizeof(size),&received,0)>=0,"SHA256 object length");
    object.resize(size);
    outage::Require(BCryptCreateHash(algorithm,&hash,object.data(),size,nullptr,0,0)>=0,"create SHA256");
    std::array<unsigned char,16384> data{};DWORD count=0;
    for(;;) {
      outage::Require(ReadFile(file,data.data(),static_cast<DWORD>(data.size()),&count,nullptr)!=0,"read marker file");
      if(!count)break;
      outage::Require(BCryptHashData(hash,data.data(),count,0)>=0,"hash marker file");
    }
    outage::Require(BCryptFinishHash(hash,digest.data(),static_cast<ULONG>(digest.size()),0)>=0,"finish SHA256");
  } catch(...) {if(hash)BCryptDestroyHash(hash);BCryptCloseAlgorithmProvider(algorithm,0);throw;}
  BCryptDestroyHash(hash);BCryptCloseAlgorithmProvider(algorithm,0);
  std::ostringstream result;result<<std::hex<<std::setfill('0');
  for(auto byte:digest)result<<std::setw(2)<<static_cast<unsigned>(byte);
  return result.str();
}
HANDLE Process(unsigned pid,const std::filesystem::path &marker) {
  HANDLE process=OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION|SYNCHRONIZE,FALSE,pid);
  outage::Require(process!=nullptr,"open proof marker process");
  try {
    wchar_t path[32768]{};DWORD size=32768;
    outage::Require(QueryFullProcessImageNameW(process,0,path,&size)!=0,"query marker process image");
    outage::Require(outage::Lower(std::filesystem::canonical(path).wstring())==outage::Lower(marker.wstring()),"PID is not the exact sibling marker executable");
    outage::Require(WaitForSingleObject(process,0)==WAIT_TIMEOUT,"marker process is not alive");
    FILETIME created{},exit{},kernel{},user{};
    outage::Require(GetProcessTimes(process,&created,&exit,&kernel,&user)!=0,"query marker creation time");
    ULARGE_INTEGER stamp{};stamp.LowPart=created.dwLowDateTime;stamp.HighPart=created.dwHighDateTime;
    std::cout<<"{\"event\":\"marker_identity\",\"pid\":"<<pid
             <<",\"creation_filetime\":"<<stamp.QuadPart<<"}"<<std::endl;
  } catch(...) {CloseHandle(process);throw;}
  return process;
}
void Preconditions() {
  HANDLE raw=nullptr;outage::Require(OpenProcessToken(GetCurrentProcess(),TOKEN_QUERY,&raw)!=0,"query elevated token");
  Handle token(raw);TOKEN_ELEVATION elevation{};DWORD bytes=0;
  outage::Require(GetTokenInformation(token.value,TokenElevation,&elevation,sizeof(elevation),&bytes)!=0 && elevation.TokenIsElevated,"elevated disposable runner required");
  SC_HANDLE manager=OpenSCManagerW(nullptr,nullptr,SC_MANAGER_CONNECT);
  outage::Require(manager!=nullptr,"query service manager");
  SC_HANDLE service=OpenServiceW(manager,L"BFE",SERVICE_QUERY_STATUS);
  if(!service){CloseServiceHandle(manager);throw std::runtime_error("BFE unavailable");}
  SERVICE_STATUS_PROCESS state{};
  const bool ok=QueryServiceStatusEx(service,SC_STATUS_PROCESS_INFO,reinterpret_cast<LPBYTE>(&state),sizeof(state),&bytes)!=0;
  CloseServiceHandle(service);CloseServiceHandle(manager);
  outage::Require(ok && state.dwCurrentState==SERVICE_RUNNING,"BFE must already be running");
  std::cout<<"{\"event\":\"environment\",\"admin\":true,\"bfe_state\":"<<state.dwCurrentState<<"}"<<std::endl;
}
std::array<FWPM_FILTER_CONDITION0,6> Conditions(FWP_BYTE_BLOB *app,unsigned port,bool ipv6,FWP_BYTE_ARRAY16 &loop6) {
  std::array<FWPM_FILTER_CONDITION0,6> c{};
  for(auto &condition:c)condition.matchType=FWP_MATCH_EQUAL;
  c[0].fieldKey=FWPM_CONDITION_ALE_APP_ID;c[0].conditionValue.type=FWP_BYTE_BLOB_TYPE;c[0].conditionValue.byteBlob=app;
  c[1].fieldKey=FWPM_CONDITION_IP_PROTOCOL;c[1].conditionValue.type=FWP_UINT8;c[1].conditionValue.uint8=IPPROTO_TCP;
  c[2].fieldKey=FWPM_CONDITION_IP_REMOTE_PORT;c[2].conditionValue.type=FWP_UINT16;c[2].conditionValue.uint16=static_cast<UINT16>(port);
  c[3].fieldKey=FWPM_CONDITION_IP_REMOTE_ADDRESS;c[4].fieldKey=FWPM_CONDITION_IP_LOCAL_ADDRESS;
  for(int index:{3,4}) {
    c[index].conditionValue.type=ipv6?FWP_BYTE_ARRAY16_TYPE:FWP_UINT32;
    if(ipv6)c[index].conditionValue.byteArray16=&loop6;
    else c[index].conditionValue.uint32=0x7f000001; // WFP address/port are host order.
  }
  c[5].fieldKey=FWPM_CONDITION_FLAGS;c[5].matchType=FWP_MATCH_FLAGS_ALL_SET;
  c[5].conditionValue.type=FWP_UINT32;c[5].conditionValue.uint32=FWP_CONDITION_FLAG_IS_LOOPBACK;
  return c;
}
bool SameValue(const FWP_CONDITION_VALUE0 &a,const FWP_CONDITION_VALUE0 &b) {
  if(a.type!=b.type)return false;
  switch(a.type) {
  case FWP_UINT8:return a.uint8==b.uint8;
  case FWP_UINT16:return a.uint16==b.uint16;
  case FWP_UINT32:return a.uint32==b.uint32;
  case FWP_BYTE_ARRAY16_TYPE:return std::memcmp(a.byteArray16,b.byteArray16,16)==0;
  case FWP_BYTE_BLOB_TYPE:return a.byteBlob->size==b.byteBlob->size &&
      std::memcmp(a.byteBlob->data,b.byteBlob->data,a.byteBlob->size)==0;
  default:return false;
  }
}
void Hex(std::ostream &out,const unsigned char *data,std::size_t size) {
  if(!data){out<<"null";return;}
  // App ID is the inert marker path. Cap even an unexpected BFE blob at 64KiB.
  const auto count=std::min<std::size_t>(size,65536);
  const char digits[]="0123456789abcdef";out<<'"';
  for(std::size_t i=0;i<count;++i)out<<digits[data[i]>>4]<<digits[data[i]&15];
  out<<'"';
}
void Value(std::ostream &out,const FWP_CONDITION_VALUE0 &value) {
  out<<"{\"type\":"<<value.type<<",\"value\":";
  switch(value.type) {
  case FWP_EMPTY:out<<"null";break;
  case FWP_UINT8:out<<static_cast<unsigned>(value.uint8);break;
  case FWP_UINT16:out<<value.uint16;break;
  case FWP_UINT32:out<<value.uint32;break;
  case FWP_UINT64:if(value.uint64)out<<*value.uint64;else out<<"null";break;
  case FWP_BYTE_ARRAY16_TYPE:
    Hex(out,value.byteArray16?value.byteArray16->byteArray16:nullptr,16);break;
  case FWP_BYTE_BLOB_TYPE:
    if(!value.byteBlob){out<<"null";break;}
    out<<"{\"size\":"<<value.byteBlob->size<<",\"hex\":";
    Hex(out,value.byteBlob->data,value.byteBlob->size);
    out<<",\"truncated\":"<<(value.byteBlob->size>65536?"true":"false")<<'}';break;
  case FWP_V4_ADDR_MASK:
    if(!value.v4AddrMask){out<<"null";break;}
    out<<"{\"address\":"<<value.v4AddrMask->addr<<",\"mask\":"<<value.v4AddrMask->mask<<'}';break;
  case FWP_V6_ADDR_MASK:
    if(!value.v6AddrMask){out<<"null";break;}
    out<<"{\"address_hex\":";Hex(out,value.v6AddrMask->addr,16);
    out<<",\"prefix_length\":"<<static_cast<unsigned>(value.v6AddrMask->prefixLength)<<'}';break;
  default:out<<"null,\"unsupported_diagnostic_type\":true";break;
  }
  out<<'}';
}
void Filter(std::ostream &out,const FWPM_FILTER0 &filter) {
  out<<"{\"key\":\""<<Text(filter.filterKey)<<"\",\"layer\":\""<<Text(filter.layerKey)
     <<"\",\"sublayer\":\""<<Text(filter.subLayerKey)<<"\",\"action\":"<<filter.action.type
     <<",\"flags\":"<<filter.flags<<",\"condition_count\":"<<filter.numFilterConditions
     <<",\"filter_id\":"<<filter.filterId<<",\"weight_type\":"<<filter.weight.type
     <<",\"effective_weight_type\":"<<filter.effectiveWeight.type<<",\"conditions\":[";
  for(unsigned i=0;i<filter.numFilterConditions;++i) {
    if(i)out<<',';const auto &condition=filter.filterCondition[i];
    out<<"{\"field\":\""<<Text(condition.fieldKey)<<"\",\"match_type\":"<<condition.matchType<<",\"condition_value\":";
    Value(out,condition.conditionValue);out<<'}';
  }
  out<<"]}";
}
bool Verify(HANDLE engine,const FWPM_FILTER0 &expected) {
  FWPM_FILTER0 *actual=nullptr;Check(FwpmFilterGetByKey0(engine,&expected.filterKey,&actual),"read back filter");
  bool same=Equal(actual->subLayerKey,expected.subLayerKey) && Equal(actual->layerKey,expected.layerKey) &&
      actual->action.type==FWP_ACTION_BLOCK && actual->flags==0 && actual->numFilterConditions==expected.numFilterConditions;
  // BFE may sort conditions; compare by field instead of relying on input order.
  for(unsigned i=0;i<expected.numFilterConditions;++i) {
    bool found=false;
    for(unsigned j=0;j<actual->numFilterConditions;++j)
      if(Equal(expected.filterCondition[i].fieldKey,actual->filterCondition[j].fieldKey))
        found=expected.filterCondition[i].matchType==actual->filterCondition[j].matchType &&
              SameValue(expected.filterCondition[i].conditionValue,actual->filterCondition[j].conditionValue);
    same=same && found;
  }
  // Log both full guarded representations before accepting any normalization.
  // BFE-assigned IDs/weight types are diagnostic metadata, not compared fields.
  std::cout<<"{\"event\":\"filter_readback\",\"matches\":"<<(same?"true":"false")<<",\"expected\":";
  Filter(std::cout,expected);std::cout<<",\"actual\":";Filter(std::cout,*actual);std::cout<<'}'<<std::endl;
  FwpmFreeMemory0(reinterpret_cast<void **>(&actual));return same;
}
int Audit(int argc,wchar_t **argv) {
  outage::Require(argc==5,"audit requires exactly three owned GUIDs");
  const GUID sub=Parse(argv[2]),four=Parse(argv[3]),six=Parse(argv[4]);
  Engine engine;Check(FwpmEngineOpen0(nullptr,RPC_C_AUTHN_WINNT,nullptr,nullptr,&engine.value),"open independent audit engine");
  bool absent=true;
  for(const auto &key:{four,six}) {
    FWPM_FILTER0 *filter=nullptr;const DWORD status=FwpmFilterGetByKey0(engine.value,&key,&filter);
    if(filter)FwpmFreeMemory0(reinterpret_cast<void **>(&filter));
    absent=absent && status==FWP_E_FILTER_NOT_FOUND;
  }
  FWPM_SUBLAYER0 *sublayer=nullptr;const DWORD status=FwpmSubLayerGetByKey0(engine.value,&sub,&sublayer);
  if(sublayer)FwpmFreeMemory0(reinterpret_cast<void **>(&sublayer));
  absent=absent && status==FWP_E_SUBLAYER_NOT_FOUND;
  std::cout<<"{\"event\":\"audit\",\"owned_objects_absent\":"<<(absent?"true":"false")<<"}"<<std::endl;
  return absent?0:2;
}
}
int wmain(int argc,wchar_t **argv) {
  try {
    outage::Guard(L"xnav-ais-outage-filter.exe");outage::Deadline(15000);Preconditions();
    if(argc>1 && std::wstring(argv[1])==L"--audit")return Audit(argc,argv);
    outage::Require(argc==5,"expected ephemeral port, two marker PIDs and marker SHA256 only");
    const unsigned port=outage::Number(argv[1],49152,65535);
    const unsigned pid4=outage::Number(argv[2],1,MAXDWORD),pid6=outage::Number(argv[3],1,MAXDWORD);
    outage::Require(pid4!=pid6,"separate family marker processes required");
    const std::wstring supplied(argv[4]);
    outage::Require(supplied.size()==64 && std::all_of(supplied.begin(),supplied.end(),[](wchar_t c) {
      return (c>=L'0' && c<=L'9') || (c>=L'a' && c<=L'f');
    }),"exact marker SHA256 required");
    const auto marker=outage::Self().parent_path()/L"xnav-ais-outage-marker.exe";
    Handle file(CreateFileW(marker.c_str(),GENERIC_READ,FILE_SHARE_READ,nullptr,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,nullptr));
    outage::Require(file.value!=INVALID_HANDLE_VALUE,"lock marker image against replacement");
    outage::Require(Hash(file.value)==std::string(supplied.begin(),supplied.end()),"marker executable hash mismatch");
    Handle four(Process(pid4,marker)),six(Process(pid6,marker));
    const GUID subkey=Unique(),key4=Unique(),key6=Unique();
    std::cout<<"{\"event\":\"prepared\",\"sublayer\":\""<<Text(subkey)
             <<"\",\"filter4\":\""<<Text(key4)<<"\",\"filter6\":\""<<Text(key6)
             <<"\",\"port\":"<<port<<",\"maximum_lifetime_ms\":15000}"<<std::endl;
    // No filtering objects exist before the runner assigns its kill-on-close job.
    std::string command;outage::Require(static_cast<bool>(std::getline(std::cin,command)) && command=="arm","explicit job-owned arm required");
    outage::Require(WaitForSingleObject(four.value,0)==WAIT_TIMEOUT && WaitForSingleObject(six.value,0)==WAIT_TIMEOUT,"marker exited before arming");
    FWPM_SESSION0 session{};session.flags=FWPM_SESSION_FLAG_DYNAMIC;session.txnWaitTimeoutInMSec=1000;
    Engine engine;Check(FwpmEngineOpen0(nullptr,RPC_C_AUTHN_WINNT,nullptr,&session,&engine.value),"open dynamic WFP session");
    FWP_BYTE_BLOB *appid=nullptr;Check(FwpmGetAppIdFromFileName0(marker.c_str(),&appid),"derive exact marker app ID");
    FWP_BYTE_ARRAY16 loop6{};loop6.byteArray16[15]=1;
    auto c4=Conditions(appid,port,false,loop6),c6=Conditions(appid,port,true,loop6);
    FWPM_FILTER0 f4{},f6{};
    f4.filterKey=key4;f4.subLayerKey=subkey;f4.layerKey=FWPM_LAYER_ALE_AUTH_CONNECT_V4;
    f4.displayData.name=const_cast<wchar_t *>(L"Disposable loopback marker IPv4");
    f4.action.type=FWP_ACTION_BLOCK;f4.weight.type=FWP_EMPTY;f4.numFilterConditions=static_cast<UINT32>(c4.size());f4.filterCondition=c4.data();
    f6=f4;f6.filterKey=key6;f6.layerKey=FWPM_LAYER_ALE_AUTH_CONNECT_V6;
    f6.displayData.name=const_cast<wchar_t *>(L"Disposable loopback marker IPv6");f6.filterCondition=c6.data();
    Check(FwpmTransactionBegin0(engine.value,0),"begin atomic dual-family filters");
    ULONGLONG commit_before=0,commit_after=0;
    try {
      FWPM_SUBLAYER0 sub{};sub.subLayerKey=subkey;sub.displayData.name=const_cast<wchar_t *>(L"Disposable AIS outage proof");sub.weight=0x100;
      Check(FwpmSubLayerAdd0(engine.value,&sub,nullptr),"add dynamic sublayer");
      Check(FwpmFilterAdd0(engine.value,&f4,nullptr,nullptr),"add IPv4 block");
      Check(FwpmFilterAdd0(engine.value,&f6,nullptr,nullptr),"add IPv6 block");
      commit_before=GetTickCount64();
      Check(FwpmTransactionCommit0(engine.value),"commit dual-family filters");
      commit_after=GetTickCount64();
    } catch(...) {FwpmTransactionAbort0(engine.value);FwpmFreeMemory0(reinterpret_cast<void **>(&appid));throw;}
    const bool match4=Verify(engine.value,f4),match6=Verify(engine.value,f6);
    FwpmFreeMemory0(reinterpret_cast<void **>(&appid));
    outage::Require(match4 && match6,"filter readback differs from exact guarded scope");
    std::cout<<"{\"event\":\"active\",\"scope_verified\":true,\"families\":[4,6],\"action\":\"block\",\"dynamic\":true,\"commit_before_tick\":"
             <<commit_before<<",\"commit_after_tick\":"<<commit_after<<"}"<<std::endl;
    outage::Require(static_cast<bool>(std::getline(std::cin,command)) && command=="stop","bounded stop command required");
    Check(FwpmEngineClose0(engine.value),"close owned dynamic session");engine.value=nullptr;
    std::cout<<"{\"event\":\"closed\"}"<<std::endl;return 0;
  } catch(const std::exception &e) {return outage::Error(e);}
}

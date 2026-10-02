// Inert fixed-loopback echo client, compiled under two distinct app identities.
#include "guard.h"
#include <cstdint>

namespace {
SOCKET Connect(int family,unsigned port) {
  SOCKET socket=::socket(family,SOCK_STREAM,IPPROTO_TCP);
  if(socket==INVALID_SOCKET)return socket;
  u_long nonblocking=1;
  if(ioctlsocket(socket,FIONBIO,&nonblocking)) {closesocket(socket);return INVALID_SOCKET;}
  sockaddr_storage address{};int size=0;
  if(family==AF_INET) {
    auto &v4=reinterpret_cast<sockaddr_in &>(address);
    v4.sin_family=AF_INET;v4.sin_port=htons(static_cast<u_short>(port));
    v4.sin_addr.s_addr=htonl(INADDR_LOOPBACK);size=sizeof(v4);
  } else {
    auto &v6=reinterpret_cast<sockaddr_in6 &>(address);
    v6.sin6_family=AF_INET6;v6.sin6_port=htons(static_cast<u_short>(port));
    v6.sin6_addr=in6addr_loopback;size=sizeof(v6);
  }
  const int status=connect(socket,reinterpret_cast<sockaddr *>(&address),size);
  if(status==SOCKET_ERROR && WSAGetLastError()!=WSAEWOULDBLOCK) {
    closesocket(socket);return INVALID_SOCKET;
  }
  fd_set write,error;FD_ZERO(&write);FD_ZERO(&error);
  FD_SET(socket,&write);FD_SET(socket,&error);timeval timeout{0,500000};
  int socket_error=0;int length=sizeof(socket_error);
  if(select(0,nullptr,&write,&error,&timeout)<=0 || !FD_ISSET(socket,&write) ||
     getsockopt(socket,SOL_SOCKET,SO_ERROR,reinterpret_cast<char *>(&socket_error),&length) || socket_error) {
    closesocket(socket);return INVALID_SOCKET;
  }
  nonblocking=0;
  if(ioctlsocket(socket,FIONBIO,&nonblocking)) {closesocket(socket);return INVALID_SOCKET;}
  DWORD wait=300;
  if(setsockopt(socket,SOL_SOCKET,SO_RCVTIMEO,reinterpret_cast<char *>(&wait),sizeof(wait)) ||
     setsockopt(socket,SOL_SOCKET,SO_SNDTIMEO,reinterpret_cast<char *>(&wait),sizeof(wait))) {
    closesocket(socket);return INVALID_SOCKET;
  }
  return socket;
}
bool Exchange(SOCKET socket,std::uint64_t sequence) {
  const auto *data=reinterpret_cast<const char *>(&sequence);
  int sent=0;
  while(sent<sizeof(sequence)) {
    const int n=send(socket,data+sent,static_cast<int>(sizeof(sequence))-sent,0);
    if(n<=0)return false;sent+=n;
  }
  std::uint64_t echoed=0;auto *received=reinterpret_cast<char *>(&echoed);int count=0;
  while(count<sizeof(echoed)) {
    const int n=recv(socket,received+count,static_cast<int>(sizeof(echoed))-count,0);
    if(n<=0)return false;count+=n;
  }
  return echoed==sequence;
}
}
int wmain(int argc,wchar_t **argv) {
  try {
#ifdef OUTAGE_CONTROL
    outage::Guard(L"xnav-ais-outage-control.exe");
#else
    outage::Guard(L"xnav-ais-outage-marker.exe");
#endif
    outage::Require(argc==3,"expected only family and ephemeral port");
    const unsigned family=outage::Number(argv[1],4,6);
    outage::Require(family==4 || family==6,"only literal loopback families allowed");
    const unsigned port=outage::Number(argv[2],49152,65535);
    outage::Deadline(60000);
    WSADATA data{};outage::Require(WSAStartup(MAKEWORD(2,2),&data)==0,"Winsock startup failed");
    SOCKET socket=INVALID_SOCKET;unsigned connection=0;std::uint64_t sequence=0;
    for(;;) {
      if(socket==INVALID_SOCKET) {
        socket=Connect(family==4?AF_INET:AF_INET6,port);
        if(socket==INVALID_SOCKET) {
          std::cout<<"{\"event\":\"connect_fail\",\"tick\":"<<GetTickCount64()<<"}"<<std::endl;
          Sleep(100);continue;
        }
        ++connection;
        std::cout<<"{\"event\":\"connected\",\"connection\":"<<connection
                 <<",\"tick\":"<<GetTickCount64()<<"}"<<std::endl;
      }
      if(Exchange(socket,++sequence))
        std::cout<<"{\"event\":\"echo\",\"sequence\":"<<sequence
                 <<",\"connection\":"<<connection<<",\"tick\":"<<GetTickCount64()<<"}"<<std::endl;
      else {
        std::cout<<"{\"event\":\"flow_error\",\"connection\":"<<connection
                 <<",\"tick\":"<<GetTickCount64()<<"}"<<std::endl;
        closesocket(socket);socket=INVALID_SOCKET;
      }
      Sleep(100);
    }
  } catch(const std::exception &e) {return outage::Error(e);}
}
